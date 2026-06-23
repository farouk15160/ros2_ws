#!/usr/bin/env python3
"""
Object Detector Node – Visiona Jarvis Pipeline

Uses a local Vision-Language Model (LLaVA via Ollama) to detect objects
in the camera feed based on a natural-language command.

Subscribes:
    /jarvis/command                                         (std_msgs/String)
    /ascamera_hp60c/camera_publisher/rgb0/image_raw        (sensor_msgs/Image)

Publishes:
    /jarvis/detected_objects  (std_msgs/String – JSON list of detected objects)
    /jarvis/detection_status  (std_msgs/String)

Author: Farouk / Antigravity AI
"""

import base64
import json
import threading
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

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False


class ObjectDetectorNode(Node):
    """
    Subscribes to camera images and /jarvis/command.
    When a command arrives, the latest frame is encoded to JPEG/base64
    and sent to the Ollama VLM API. The response is parsed for object
    names + pixel coordinates and published as JSON.
    """

    DEFAULT_OLLAMA_URL   = 'http://localhost:11434'
    DEFAULT_VISION_MODEL = 'llava'
    DEFAULT_IMG_TOPIC    = '/ascamera_hp60c/camera_publisher/rgb0/image_raw'

    def __init__(self):
        super().__init__('jarvis_object_detector')

        # ── Parameters ────────────────────────────────────────────────── #
        self.declare_parameter('ollama_url',    self.DEFAULT_OLLAMA_URL)
        self.declare_parameter('vision_model',  self.DEFAULT_VISION_MODEL)
        self.declare_parameter('image_topic',   self.DEFAULT_IMG_TOPIC)
        self.declare_parameter('request_timeout', 60)
        self.declare_parameter('jpeg_quality', 75)

        self.ollama_url    = self.get_parameter('ollama_url').value
        self.vision_model  = self.get_parameter('vision_model').value
        self.image_topic   = self.get_parameter('image_topic').value
        self.req_timeout   = self.get_parameter('request_timeout').value
        self.jpeg_quality  = self.get_parameter('jpeg_quality').value

        # ── State ──────────────────────────────────────────────────────── #
        self.latest_image    = None
        self.current_command = ''
        self.is_detecting    = False
        self.bridge          = CvBridge() if HAS_CV else None

        # ── Publishers ─────────────────────────────────────────────────── #
        self.detections_pub = self.create_publisher(
            String, '/jarvis/detected_objects', 10)
        self.status_pub     = self.create_publisher(
            String, '/jarvis/detection_status', 10)

        # ── Subscribers ────────────────────────────────────────────────── #
        self.create_subscription(
            Image, self.image_topic,
            self._on_image, 10)
        self.create_subscription(
            String, '/jarvis/command',
            self._on_command, 10)
        self.create_subscription(
            String, '/jarvis/scan_trigger',
            self._on_scan_trigger, 10)

        self._check_dependencies()

        self.get_logger().info(
            f'\n✅ [JARVIS Object Detector] Ready.'
            f'\n   Model   : {self.vision_model}'
            f'\n   Server  : {self.ollama_url}'
            f'\n   Image   : {self.image_topic}')

    # ------------------------------------------------------------------ #
    # Dependency check
    # ------------------------------------------------------------------ #

    def _check_dependencies(self):
        if not HAS_CV:
            self.get_logger().error(
                '❌  cv_bridge / opencv not found. '
                'Install: sudo apt install ros-humble-cv-bridge python3-opencv')
        if not HAS_REQUESTS:
            self.get_logger().error(
                '❌  requests not found. Install: pip3 install requests')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_image(self, msg: Image):
        """Buffer latest camera frame."""
        self.latest_image = msg

    def _on_command(self, msg: String):
        """Run VLM only for vision-related commands (not every NL plan request)."""
        cmd = msg.data.strip().lower()
        if not cmd:
            return
        vision_keywords = (
            "scan", "see", "look", "detect", "find", "identify",
            "what do you see", "what can you see", "show me", "spot",
        )
        if not any(k in cmd for k in vision_keywords):
            return
        self.current_command = msg.data.strip()
        self._run_detection_async()

    def _on_scan_trigger(self, msg: String):
        """Trigger a generic scene scan."""
        self.current_command = 'List all objects visible on the table or surface.'
        self._run_detection_async()

    # ------------------------------------------------------------------ #
    # Detection
    # ------------------------------------------------------------------ #

    def _run_detection_async(self):
        if self.is_detecting:
            return
        t = threading.Thread(target=self._run_detection, daemon=True)
        t.start()

    def _run_detection(self):
        """Core detection logic (runs in background thread)."""
        if not HAS_CV or not HAS_REQUESTS or self.bridge is None:
            return
        if self.latest_image is None:
            self.get_logger().warn(
                '⚠️  [Detector] No camera frame available yet.')
            return

        self.is_detecting = True
        self._publish_status('detecting')

        try:
            # Convert ROS Image → JPEG base64
            cv_img = self.bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='rgb8')
            ok, buf = cv2.imencode(
                '.jpg', cv_img,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                self.get_logger().warn('⚠️  JPEG encoding failed.')
                return
            img_b64 = base64.b64encode(buf.tobytes()).decode('utf-8')

            h, w = cv_img.shape[:2]
            prompt = (
                f'Command: "{self.current_command}"\n'
                f'Image size: {w}x{h} pixels.\n\n'
                'Identify all relevant objects in the image that relate to the command. '
                'For each object, respond ONLY with a valid JSON array, for example:\n'
                '[{"name":"red cup","x_px":320,"y_px":240},{"name":"bottle","x_px":100,"y_px":150}]\n'
                'Only output the JSON array. No other text.'
            )

            self.get_logger().info(
                f'🔍  [Detector] Querying {self.vision_model} for: "{self.current_command}"')

            resp = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model':  self.vision_model,
                    'prompt': prompt,
                    'images': [img_b64],
                    'stream': False,
                    'options': {'temperature': 0.0},
                },
                timeout=self.req_timeout,
            )
            resp.raise_for_status()
            raw = resp.json().get('response', '').strip()

            self.get_logger().info(
                f'📥  [Detector] Raw response: {raw[:200]}')

            objects = self._parse_json_objects(raw)
            if objects is None:
                self.get_logger().warn(
                    '⚠️  [Detector] Could not parse JSON from VLM response.')
                objects = []

            self.get_logger().info(
                f'✅  [Detector] Detected {len(objects)} object(s):')
            for o in objects:
                self.get_logger().info(
                    f'   • {o.get("name","?")} @ '
                    f'({o.get("x_px",0)}, {o.get("y_px",0)})')

            msg_out = String()
            msg_out.data = json.dumps(objects)
            self.detections_pub.publish(msg_out)
            self._publish_status('idle')

        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                f'❌  [Detector] Cannot connect to Ollama at {self.ollama_url}')
            self._publish_status('error: ollama offline')
        except Exception as e:
            self.get_logger().error(f'❌  [Detector] Error: {e}')
            self._publish_status(f'error: {e}')
        finally:
            self.is_detecting = False

    def _parse_json_objects(self, text: str):
        """Extract JSON array from VLM response."""
        text = text.strip()
        if '```' in text:
            lines = [l for l in text.split('\n') if not l.strip().startswith('```')]
            text = '\n'.join(lines)
        start = text.find('[')
        end   = text.rfind(']') + 1
        if start == -1 or end == 0:
            return None
        try:
            data = json.loads(text[start:end])
            if isinstance(data, list):
                return data
        except json.JSONDecodeError:
            pass
        return None

    def _publish_status(self, s: str):
        m = String(); m.data = s
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
