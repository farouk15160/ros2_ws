#!/usr/bin/env python3
import json
import time
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .motion_planner import HybridMotionPlanner


class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__("jarvis_action_executor")
        self.declare_parameter("step_delay_sec", 0.3)
        self.declare_parameter("approach_timeout", 8.0)
        self.declare_parameter("grasp_height_offset", 0.05)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("planning_mode", "auto")
        self.declare_parameter("position_tolerance", 0.015)
        self.declare_parameter("move_timeout", 8.0)
        self.declare_parameter("moveit_group", "arm")
        self.declare_parameter("moveit_ee_link", "gripper_base")
        self.declare_parameter("use_twin_validation", True)
        self.declare_parameter("use_safety_gate", True)

        self.step_delay = self.get_parameter("step_delay_sec").value
        self.approach_timeout = self.get_parameter("approach_timeout").value
        self.grasp_offset = self.get_parameter("grasp_height_offset").value
        self.world_frame = self.get_parameter("world_frame").value

        self.world_state = {}
        self.is_executing = False
        self.motion = HybridMotionPlanner(self)

        self.scan_pub = self.create_publisher(String, "/jarvis/scan_trigger", 10)
        self.feedback_pub = self.create_publisher(String, "/jarvis/feedback", 10)
        self.exec_status_pub = self.create_publisher(String, "/jarvis/execution_status", 10)
        self.event_pub = self.create_publisher(String, "/jarvis/execution_event", 10)
        self.gripper_pub = self.create_publisher(Float32MultiArray, "gripper_command", 10)

        self.declare_parameter("gripper_open_angle", 0.0)
        self.declare_parameter("gripper_close_angle", 90.0)
        self.declare_parameter("gripper_current_ma", 250.0)
        self.gripper_open = self.get_parameter("gripper_open_angle").value
        self.gripper_close = self.get_parameter("gripper_close_angle").value
        self.gripper_current = self.get_parameter("gripper_current_ma").value

        self.create_subscription(String, "/jarvis/action_plan", self._on_plan, 10)
        self.create_subscription(String, "/jarvis/world_state", self._on_world_state, 10)
        mode = self.get_parameter("planning_mode").value
        self.planning_mode = mode
        self.get_logger().info(f"[JARVIS Executor] Ready (planning_mode={mode}).")

    def _on_world_state(self, msg: String):
        try:
            self.world_state = json.loads(msg.data)
        except Exception:
            pass

    def _on_plan(self, msg: String):
        if self.is_executing:
            self._fb("Robot busy.")
            return
        try:
            plan = json.loads(msg.data)
        except Exception:
            return
        acts = plan.get("actions", [])
        if not acts:
            return
        self.get_logger().info(f"Plan: {len(acts)} steps")
        threading.Thread(target=self._execute_plan, args=(acts,), daemon=True).start()

    def _execute_plan(self, actions):
        self.is_executing = True
        self._es("executing")
        try:
            for i, a in enumerate(actions):
                t = a.get("type", "").lower()
                self.get_logger().info(f"Step {i+1}/{len(actions)}: {a}")
                self._fb(f"Step {i+1}/{len(actions)}: {t}")
                if t == "pick":
                    self._do_pick(a)
                elif t == "place":
                    self._do_place(a)
                elif t == "scan":
                    self._do_scan()
                elif t == "home":
                    self._do_home()
                elif t == "open_gripper":
                    self._do_gripper(True)
                elif t == "close_gripper":
                    self._do_gripper(False)
                elif t == "speak":
                    self._do_speak(a.get("message", ""))
                else:
                    self.get_logger().warn(f"Unknown: {t}")
                self._emit_event("step_complete", action_type=t, success=True)
                time.sleep(self.step_delay)
            self.get_logger().info("Plan complete.")
            self._fb("All actions done.")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self._fb(f"Error: {e}")
        finally:
            self.is_executing = False
            self._es("idle")

    def _do_pick(self, a):
        raw = a.get("object", "")
        safe = raw.lower().replace(" ", "_")
        zo = a.get("approach_z_offset", self.grasp_offset)
        obj = self._lookup(safe)
        if obj is None:
            self.get_logger().warn(f"{raw} not found. Available: {list(self.world_state.keys())}")
            self._fb(f"{raw} not found.")
            self._emit_event("pick_failed", action_type="pick", object=raw, success=False)
            return
        p = obj.get("position", {})
        x, y, z = p.get("x", 0), p.get("y", 0), p.get("z", 0)
        self.get_logger().info(f"PICK {raw} @ ({x:.3f},{y:.3f},{z:.3f})")
        self._fb("Opening gripper...")
        self._do_gripper(True)
        time.sleep(0.8)
        self._fb(f"Approaching {raw}...")
        self._mv(x, y, z + zo)
        self._fb("Descending...")
        self._mv(x, y, z)
        self._fb("Closing gripper...")
        self._do_gripper(False)
        time.sleep(0.8)
        self._fb("Lifting...")
        self._mv(x, y, z + zo + 0.05)
        self._fb(f"Picked up {raw}")
        self._emit_event("pick_complete", action_type="pick", object=raw, success=True)

    def _do_place(self, a):
        x, y, z = float(a.get("x", 0)), float(a.get("y", 0)), float(a.get("z", 0.08))
        self.get_logger().info(f"PLACE @ ({x:.3f},{y:.3f},{z:.3f})")
        self._fb(f"Moving to ({x:.2f},{y:.2f},{z:.2f})...")
        self._mv(x, y, z + 0.07)
        self._mv(x, y, z)
        self._fb("Releasing...")
        self._do_gripper(True)
        time.sleep(0.8)
        self._mv(x, y, z + 0.10)
        self._fb("Object placed.")
        self._emit_event("place_complete", action_type="place", success=True)

    def _do_scan(self):
        self.get_logger().info("Scanning...")
        self._fb("Scanning scene...")
        m = String()
        m.data = "__scan__"
        self.scan_pub.publish(m)
        time.sleep(3.0)

    def _do_home(self):
        self.get_logger().info("Home...")
        self._fb("Moving home...")
        if not self.motion.move_home(self.approach_timeout):
            self._fb("Home motion timed out.")

    def _do_gripper(self, open_g: bool):
        angle = self.gripper_open if open_g else self.gripper_close
        msg = Float32MultiArray()
        msg.data = [float(angle), float(self.gripper_current)]
        self.gripper_pub.publish(msg)
        self._fb(f"Gripper {'opened' if open_g else 'closed'}.")

    def _do_speak(self, message: str):
        self.get_logger().info(f"JARVIS: {message}")
        self._fb(f"JARVIS: {message}")

    def _mv(self, x, y, z):
        t0 = time.time()
        ok = self.motion.move_to(x, y, z, self.approach_timeout)
        if not ok:
            self._fb(f"Motion timeout at ({x:.2f},{y:.2f},{z:.2f})")
            self.get_logger().warn(f"Motion timeout: ({x:.3f},{y:.3f},{z:.3f})")
        self._emit_event(
            "motion",
            action_type="move",
            success=ok,
            duration_sec=round(time.time() - t0, 2),
            target={"x": x, "y": y, "z": z},
        )

    def _emit_event(self, event: str, **fields):
        payload = {
            "event": event,
            "planning_backend": self.planning_mode,
            **fields,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)

    def _lookup(self, name: str):
        if name in self.world_state:
            return self.world_state[name]
        for k, v in self.world_state.items():
            if name in k or k in name:
                return v
        return None

    def _fb(self, s):
        m = String()
        m.data = s
        self.feedback_pub.publish(m)

    def _es(self, s):
        m = String()
        m.data = s
        self.exec_status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
