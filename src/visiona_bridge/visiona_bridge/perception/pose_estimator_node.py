#!/usr/bin/env python3
"""
Pose Estimator Node – Visiona Jarvis Pipeline

Fuses segmentation mask bboxes with aligned depth data to compute 3D object
positions in the robot's world frame via TF2.

Subscribes:
    /jarvis/object_masks                                   (std_msgs/String)
    /ascamera_hp60c/camera_publisher/depth0/image_raw      (sensor_msgs/Image)
    /ascamera_hp60c/camera_publisher/rgb0/camera_info      (sensor_msgs/CameraInfo)

Publishes:
    /jarvis/object_poses       (geometry_msgs/PoseArray – world frame)
    /jarvis/world_markers      (visualization_msgs/MarkerArray – RViz overlay)
    /jarvis/pose_status        (std_msgs/String – JSON for world model)

Broadcasts TF frames: detected_<object_name>  (child of 'world')

Author: Farouk / Antigravity AI
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

try:
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class PoseEstimatorNode(Node):
    """
    Converts 2D mask bboxes + aligned depth into 3D object poses in the
    robot world frame. Broadcasts TF frames and generates RViz markers
    overlaid on top of the RTAB-Map colored point cloud.
    """

    CAMERA_FRAME = 'ascamera_hp60c_ascamera_0'
    WORLD_FRAME  = 'world'

    def __init__(self):
        super().__init__('jarvis_pose_estimator')

        self.declare_parameter('world_frame',  self.WORLD_FRAME)
        self.declare_parameter('camera_frame', self.CAMERA_FRAME)
        self.declare_parameter('depth_scale',  0.001)   # mm → m
        self.declare_parameter('max_depth',    2.0)
        self.declare_parameter('min_depth',    0.05)
        self.declare_parameter(
            'depth_topic',
            '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/ascamera_hp60c/camera_publisher/rgb0/camera_info')

        self.world_frame       = self.get_parameter('world_frame').value
        self.camera_frame      = self.get_parameter('camera_frame').value
        self.depth_scale       = self.get_parameter('depth_scale').value
        self.max_depth         = self.get_parameter('max_depth').value
        self.min_depth         = self.get_parameter('min_depth').value
        self.depth_topic       = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth  = None
        self.bridge        = CvBridge() if HAS_CV else None

        # TF2
        try:
            import tf2_ros
            self.tf_broadcaster = TransformBroadcaster(self)
            self.tf_buffer      = tf2_ros.Buffer()
            self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer, self)
            self.has_tf = True
        except Exception as e:
            self.get_logger().warn(f'⚠️  TF2 not available: {e}')
            self.has_tf = False

        # Publishers
        self.pose_array_pub = self.create_publisher(PoseArray,   '/jarvis/object_poses',  10)
        self.markers_pub    = self.create_publisher(MarkerArray, '/jarvis/world_markers',  10)
        self.status_pub     = self.create_publisher(String,      '/jarvis/pose_status',    10)

        # Subscribers
        self.create_subscription(String,     '/jarvis/object_masks',    self._on_masks, 10)
        self.create_subscription(Image,      self.depth_topic,           self._on_depth, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic,     self._on_camera_info, 1)

        self.get_logger().info(
            f'✅ [JARVIS Pose Estimator] Ready.'
            f'\n   Depth  : {self.depth_topic}'
            f'\n   Frames : {self.camera_frame} → {self.world_frame}')

    def _on_camera_info(self, msg: CameraInfo):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f'📷  Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _on_depth(self, msg: Image):
        if not HAS_CV or self.bridge is None:
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(msg)
            if raw.dtype == np.uint16:
                self.latest_depth = raw.astype(np.float32) * self.depth_scale
            else:
                self.latest_depth = raw.astype(np.float32)
        except Exception as e:
            self.get_logger().debug(f'Depth conv: {e}')

    def _on_masks(self, msg: String):
        try:
            masks = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not masks:
            return
        if self.fx is None:
            self.get_logger().warn('⚠️  Camera intrinsics not received yet.')
            return

        poses_3d  = []
        pose_arr  = PoseArray()
        marker_arr = MarkerArray()
        pose_arr.header.frame_id = self.world_frame
        pose_arr.header.stamp    = self.get_clock().now().to_msg()

        for i, obj in enumerate(masks):
            name = obj.get('name', f'object_{i}')
            bbox = obj.get('bbox')
            x_px = obj.get('x_px', 0)
            y_px = obj.get('y_px', 0)

            u = ((bbox[0] + bbox[2]) / 2.0) if bbox else float(x_px)
            v = ((bbox[1] + bbox[3]) / 2.0) if bbox else float(y_px)

            depth_m = self._sample_depth(u, v, bbox)
            if depth_m is None:
                self.get_logger().warn(f'⚠️  No valid depth for "{name}"')
                continue

            x_cam = (u - self.cx) * depth_m / self.fx
            y_cam = (v - self.cy) * depth_m / self.fy
            z_cam = depth_m

            pos_world = self._camera_to_world(x_cam, y_cam, z_cam)
            if pos_world is None:
                pos_world = (x_cam, y_cam, z_cam)
            wx, wy, wz = pos_world

            self.get_logger().info(
                f'📍  [{name}] depth={depth_m:.3f}m '
                f'world=({wx:.3f},{wy:.3f},{wz:.3f})')

            safe_name = name.replace(' ', '_').lower()
            self._broadcast_tf(safe_name, wx, wy, wz)

            p = Pose()
            p.position.x = wx; p.position.y = wy; p.position.z = wz
            p.orientation.w = 1.0
            pose_arr.poses.append(p)

            now = self.get_clock().now().to_msg()
            sphere = Marker()
            sphere.header.frame_id = self.world_frame
            sphere.header.stamp    = now
            sphere.ns = 'jarvis_objects'; sphere.id = i * 2
            sphere.type   = Marker.SPHERE; sphere.action = Marker.ADD
            sphere.pose.position.x = wx; sphere.pose.position.y = wy; sphere.pose.position.z = wz
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
            sphere.color.r = 1.0; sphere.color.g = 0.6; sphere.color.a = 0.9
            sphere.lifetime.sec = 10

            label = Marker()
            label.header = sphere.header
            label.ns = 'jarvis_labels'; label.id = i * 2 + 1
            label.type   = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD
            label.pose.position.x = wx; label.pose.position.y = wy
            label.pose.position.z = wz + 0.06
            label.pose.orientation.w = 1.0
            label.scale.z = 0.03
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = name; label.lifetime.sec = 10

            marker_arr.markers.extend([sphere, label])
            poses_3d.append({'name': name, 'safe_name': safe_name,
                             'x': wx, 'y': wy, 'z': wz, 'depth_m': depth_m})

        self.pose_array_pub.publish(pose_arr)
        self.markers_pub.publish(marker_arr)

        s = String(); s.data = json.dumps(poses_3d)
        self.status_pub.publish(s)

        self.get_logger().info(
            f'✅  [Pose] Published {len(poses_3d)} pose(s) → RViz + TF')

    def _sample_depth(self, u, v, bbox):
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape
        if bbox:
            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            roi = self.latest_depth[y1:y2, x1:x2]
        else:
            r  = 15
            iu, iv = max(0, min(int(u), w - 1)), max(0, min(int(v), h - 1))
            roi = self.latest_depth[max(0, iv - r):min(h - 1, iv + r),
                                    max(0, iu - r):min(w - 1, iu + r)]
        if roi.size == 0:
            return None
        valid = roi[(roi > self.min_depth) & (roi < self.max_depth)]
        return float(np.median(valid)) if valid.size > 0 else None

    def _camera_to_world(self, x_cam, y_cam, z_cam):
        if not self.has_tf:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            p  = np.array([x_cam, y_cam, z_cam])
            pw = self._quat_rotate(p, qx, qy, qz, qw)
            return (pw[0] + tx, pw[1] + ty, pw[2] + tz)
        except Exception as e:
            self.get_logger().debug(f'TF2 lookup: {e}')
            return None

    @staticmethod
    def _quat_rotate(v, qx, qy, qz, qw):
        qvec = np.array([qx, qy, qz])
        uv   = np.cross(qvec, v)
        uuv  = np.cross(qvec, uv)
        return v + 2.0 * (qw * uv + uuv)

    def _broadcast_tf(self, name, x, y, z):
        if not self.has_tf:
            return
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id  = f'detected_{name}'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w    = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
