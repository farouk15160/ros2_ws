#!/usr/bin/env python3
"""
Segmentation Node – Visiona Jarvis Pipeline

Uses MobileSAM to generate precise object masks from detected pixel points.

Subscribes:
    /jarvis/detected_objects  (std_msgs/String – JSON from object_detector_node)
    /ascamera_hp60c/camera_publisher/rgb0/image_raw (sensor_msgs/Image)

Publishes:
    /jarvis/object_masks     (std_msgs/String – JSON with bbox + mask info)
    /jarvis/segmentation_debug (sensor_msgs/Image – debug visualization)

Author: Farouk / Antigravity AI
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False


class SegmentationNode(Node):
    """
    Receives object detections (pixel coords), runs MobileSAM to get
    precise masks, extracts bounding boxes, and publishes mask metadata.
    Falls back gracefully if MobileSAM is not installed.
    """

    DEFAULT_SAM_CHECKPOINT = '/home/farouk/mobile_sam.pt'
    DEFAULT_IMG_TOPIC      = '/ascamera_hp60c/camera_publisher/rgb0/image_raw'

    def __init__(self):
        super().__init__('jarvis_segmentation')

        self.declare_parameter('sam_checkpoint', self.DEFAULT_SAM_CHECKPOINT)
        self.declare_parameter('image_topic',    self.DEFAULT_IMG_TOPIC)
        self.declare_parameter('device', 'cpu')

        self.sam_ckpt    = self.get_parameter('sam_checkpoint').value
        self.image_topic = self.get_parameter('image_topic').value
        self.device      = self.get_parameter('device').value

        self.latest_image = None
        self.predictor    = None
        self.bridge       = CvBridge() if HAS_CV else None

        # ── Publishers ─────────────────────────────────────────────────── #
        self.masks_pub = self.create_publisher(
            String, '/jarvis/object_masks', 10)
        self.debug_pub = self.create_publisher(
            Image,  '/jarvis/segmentation_debug', 10)

        # ── Subscribers ────────────────────────────────────────────────── #
        self.create_subscription(
            Image,  self.image_topic,
            self._on_image, 10)
        self.create_subscription(
            String, '/jarvis/detected_objects',
            self._on_detections, 10)

        # ── Load SAM ── #
        self._load_sam()

        self.get_logger().info(
            f'✅ [JARVIS Segmentation] Ready.'
            f'\n   SAM checkpoint: {self.sam_ckpt}'
            f'\n   Device        : {self.device}')

    # ------------------------------------------------------------------ #
    # SAM loading
    # ------------------------------------------------------------------ #

    def _load_sam(self):
        """Load MobileSAM model (graceful fallback if not installed)."""
        import os
        if not os.path.exists(self.sam_ckpt):
            self.get_logger().warn(
                f'⚠️  [Segmentation] SAM checkpoint not found: {self.sam_ckpt}\n'
                f'   Download: wget -O {self.sam_ckpt} '
                f'https://github.com/ChaoningZhang/MobileSAM/releases/download/v1.0/mobile_sam.pt\n'
                f'   Running WITHOUT segmentation (bbox-only fallback).')
            return
        try:
            from mobile_sam import sam_model_registry, SamPredictor
            sam_type = 'vit_t'
            sam      = sam_model_registry[sam_type](checkpoint=self.sam_ckpt)
            sam.to(self.device)
            sam.eval()
            self.predictor = SamPredictor(sam)
            self.get_logger().info(
                f'✅  [Segmentation] MobileSAM loaded from {self.sam_ckpt}')
        except Exception as e:
            self.get_logger().warn(
                f'⚠️  [Segmentation] MobileSAM load failed: {e}\n'
                f'   Install: pip3 install git+https://github.com/ChaoningZhang/MobileSAM.git')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_image(self, msg: Image):
        self.latest_image = msg

    def _on_detections(self, msg: String):
        """Process detections and generate masks."""
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not detections:
            return
        if not HAS_CV or self.bridge is None:
            return
        if self.latest_image is None:
            self.get_logger().warn('⚠️  [Segmentation] No camera frame yet.')
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'❌  cv_bridge error: {e}')
            return

        results = []
        debug_img = cv_img.copy()

        for obj in detections:
            name  = obj.get('name', 'unknown')
            x_px  = int(obj.get('x_px', 0))
            y_px  = int(obj.get('y_px', 0))

            # Clamp to image bounds
            h, w = cv_img.shape[:2]
            x_px = max(0, min(x_px, w - 1))
            y_px = max(0, min(y_px, h - 1))

            bbox = None

            if self.predictor is not None:
                try:
                    self.predictor.set_image(cv_img)
                    pts   = np.array([[x_px, y_px]], dtype=np.float32)
                    lbls  = np.array([1], dtype=np.int32)
                    masks, scores, _ = self.predictor.predict(
                        point_coords=pts,
                        point_labels=lbls,
                        multimask_output=True,
                    )
                    # Pick highest-score mask
                    best_idx  = int(np.argmax(scores))
                    best_mask = masks[best_idx]
                    ys, xs    = np.where(best_mask)
                    if len(xs) > 0:
                        bbox = [int(xs.min()), int(ys.min()),
                                int(xs.max()), int(ys.max())]
                        # Draw on debug image
                        cv2.rectangle(debug_img,
                                      (bbox[0], bbox[1]),
                                      (bbox[2], bbox[3]),
                                      (0, 255, 0), 2)
                        cv2.putText(debug_img, name,
                                    (bbox[0], max(0, bbox[1] - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 1)
                except Exception as e:
                    self.get_logger().debug(f'SAM predict error: {e}')
            else:
                # Fallback: 60×60 px bounding box around click point
                r = 30
                bbox = [max(0, x_px - r), max(0, y_px - r),
                        min(w - 1, x_px + r), min(h - 1, y_px + r)]

            self.get_logger().info(
                f'✂️   [{name}] cx={x_px} cy={y_px}  bbox={bbox}')

            results.append({
                'name':  name,
                'x_px':  x_px,
                'y_px':  y_px,
                'bbox':  bbox,
            })

        # Publish mask metadata
        out_msg      = String()
        out_msg.data = json.dumps(results)
        self.masks_pub.publish(out_msg)

        # Publish debug image
        try:
            debug_ros = self.bridge.cv2_to_imgmsg(debug_img, encoding='rgb8')
            debug_ros.header = self.latest_image.header
            self.debug_pub.publish(debug_ros)
        except Exception:
            pass

        self.get_logger().info(
            f'✅  [Segmentation] Published {len(results)} mask(s)')


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
