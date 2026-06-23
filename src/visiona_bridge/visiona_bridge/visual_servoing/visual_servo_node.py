#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseArray

class VisualServoController(Node):
    def __init__(self):
        super().__init__("visual_servo_controller")
        self.declare_parameter("drift_threshold", 0.02)
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("correction_z_offset", 0.05)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("max_corrections", 10)
        self.drift_threshold = self.get_parameter("drift_threshold").value
        self.check_rate_hz = self.get_parameter("check_rate_hz").value
        self.correction_offset = self.get_parameter("correction_z_offset").value
        self.world_frame = self.get_parameter("world_frame").value
        self.max_corrections = self.get_parameter("max_corrections").value
        self.target_pose = None; self.is_active = False
        self.correction_count = 0; self._poses = []
        self.cartesian_pub = self.create_publisher(PoseStamped, "/visiona/cartesian_command", 10)
        self.status_pub = self.create_publisher(String, "/visual_servo/status", 10)
        self.feedback_pub = self.create_publisher(String, "/jarvis/feedback", 10)
        self.create_subscription(PoseArray, "/jarvis/object_poses", self._on_poses, 10)
        self.create_subscription(String, "/jarvis/execution_status", self._on_exec_status, 10)
        self.create_subscription(String, "/jarvis/scan_trigger", self._on_target_set, 10)
        self.create_subscription(PoseStamped, "/visiona/cartesian_command", self._on_cartesian_cmd, 10)
        self.create_timer(1.0/self.check_rate_hz, self._servo_loop)
        self.get_logger().info(f"[Visual Servo] Ready.\n   Drift: {self.drift_threshold*100:.0f}cm\n   Rate: {self.check_rate_hz}Hz\n   MaxCorr: {self.max_corrections}")

    def _on_exec_status(self, msg: String):
        na = msg.data.lower() == "executing"
        if na and not self.is_active:
            self.correction_count = 0; self.get_logger().info("[Servo] Active."); self._ps("monitoring")
        elif not na and self.is_active:
            self.get_logger().info("[Servo] Stopped."); self._ps("idle"); self.target_pose = None
        self.is_active = na

    def _on_target_set(self, msg: String):
        if msg.data != "__scan__":
            self.correction_count = 0

    def _on_cartesian_cmd(self, msg: PoseStamped):
        if not self.is_active:
            return
        p = msg.pose.position
        self.target_pose = (p.x, p.y, p.z)
        self.correction_count = 0

    def _on_poses(self, msg: PoseArray):
        self._poses = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]

    def _servo_loop(self):
        if not self.is_active or not self._poses or self.target_pose is None: return
        if self.correction_count >= self.max_corrections: return
        tx, ty, tz = self.target_pose; bd = float("inf"); bp = None
        for (px, py, pz) in self._poses:
            d = math.sqrt((px-tx)**2+(py-ty)**2+(pz-tz)**2)
            if d < bd: bd, bp = d, (px, py, pz)
        if bp is None: return
        if bd > self.drift_threshold:
            npx, npy, npz = bp; n = self.correction_count+1
            self.get_logger().warn(f"[Servo] Drift {bd*100:.1f}cm correction #{n}")
            self._fb(f"Visual servo: correcting {bd*100:.1f}cm drift")
            c = PoseStamped(); c.header.stamp = self.get_clock().now().to_msg()
            c.header.frame_id = self.world_frame
            c.pose.position.x = npx; c.pose.position.y = npy; c.pose.position.z = npz+self.correction_offset
            c.pose.orientation.w = 1.0; self.cartesian_pub.publish(c)
            self.target_pose = (npx, npy, npz); self.correction_count = n
            self._ps(f"correcting ({bd*100:.1f}cm,n={n})")
        else: self._ps(f"ok ({bd*100:.1f}cm)")

    def set_target(self, x, y, z): self.target_pose = (x, y, z); self.correction_count = 0
    def _ps(self, s): m = String(); m.data = s; self.status_pub.publish(m)
    def _fb(self, s): m = String(); m.data = s; self.feedback_pub.publish(m)

def main(args=None):
    rclpy.init(args=args); node = VisualServoController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
