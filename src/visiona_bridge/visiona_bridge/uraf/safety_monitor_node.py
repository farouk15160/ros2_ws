#!/usr/bin/env python3
"""URAF Safety Monitor — PLAN.md §19 safe startup + safety functions."""

from __future__ import annotations

import json
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

from .agent_bus import AgentBus
from .safety_engine import SafetyEngine, StartupPhase


class SafetyMonitorNode(Node):
    AGENT_ID = SafetyEngine.AGENT_ID

    def __init__(self):
        super().__init__("uraf_safety_monitor")
        self.declare_parameter("startup_auto", True)
        self.declare_parameter("monitor_rate_hz", 20.0)

        params = self._load_params()
        self.engine = SafetyEngine(params)
        self.bus = AgentBus(self)

        self.status_pub = self.create_publisher(String, "/uraf/safety/status", 10)
        self.violation_pub = self.create_publisher(String, "/uraf/safety/violation", 10)
        self.scale_pub = self.create_publisher(Float32, "/uraf/safety/motion_scale", 10)
        self.startup_pub = self.create_publisher(String, "/uraf/safety/startup", 10)
        self.estop_pub = self.create_publisher(String, "/uraf/safety/estop_command", 10)

        self._joints: list[float] = []
        self._last_joint_time: float | None = None
        self._system: dict = {}
        self._ee = (None, None, None)
        self._startup_step = 0
        self._startup_phases = [
            StartupPhase.POWER_ON,
            StartupPhase.FAL_CHECK,
            StartupPhase.POSITION_VALIDATE,
            StartupPhase.CONTROLLER_ENABLE,
            StartupPhase.MOVE_HOME,
            StartupPhase.OPERATIONAL,
        ]

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(String, "/visiona/system_status", self._on_system, 10)
        self.create_subscription(PoseStamped, "/visiona/current_pose", self._on_pose, 10)
        self.create_subscription(String, "/uraf/safety/command", self._on_command, 10)

        rate = self.get_parameter("monitor_rate_hz").value
        self.create_timer(1.0 / rate, self._tick)
        if self.get_parameter("startup_auto").value:
            self.create_timer(1.5, self._startup_tick)
        self.get_logger().info("[URAF] Safety Monitor active (§19)")

    def _load_params(self) -> dict:
        try:
            share = get_package_share_directory("visiona_bridge")
            path = f"{share}/config/safety_params.yaml"
            raw = yaml.safe_load(open(path, encoding="utf-8")) or {}
            return raw.get("safety_params", {}).get("ros__parameters", {})
        except Exception as exc:
            self.get_logger().warn(f"Using default safety params: {exc}")
            return {}

    def _on_joints(self, msg: JointState):
        if msg.position:
            self._joints = list(msg.position)
            self._last_joint_time = time.time()

    def _on_system(self, msg: String):
        try:
            self._system = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self._ee = (p.x, p.y, p.z)

    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "arm":
            self.engine.arm()
        elif cmd == "disarm":
            self.engine.disarm()
        elif cmd == "startup":
            self._startup_step = 0

    def _startup_tick(self):
        if self._startup_step >= len(self._startup_phases):
            return
        phase = self._startup_phases[self._startup_step]
        self.engine.advance_startup(phase)
        payload = {"phase": phase.value, "step": self._startup_step + 1, "total": len(self._startup_phases)}
        out = String()
        out.data = json.dumps(payload, indent=2)
        self.startup_pub.publish(out)
        self.get_logger().info(f"Safe startup: {phase.value}")
        self._startup_step += 1

    def _tick(self):
        now = time.time()
        joint_age = (now - self._last_joint_time) if self._last_joint_time else None
        ex, ey, ez = self._ee
        state = self.engine.evaluate(
            joints_rad=self._joints,
            ee_x=ex, ee_y=ey, ee_z=ez,
            main_current=float(self._system.get("main_current", 0.0)),
            estop=bool(self._system.get("emergency_stop", False)),
            serial_ok=bool(self._system.get("is_connected", True)),
            joint_age_sec=joint_age,
        )

        status = self.engine.status_dict()
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)

        scale = Float32()
        scale.data = float(state.motion_scale)
        self.scale_pub.publish(scale)

        critical = [v for v in state.violations if v.severity == "critical"]
        if critical:
            vmsg = String()
            vmsg.data = json.dumps({
                "violations": status["violations"],
                "estop_recommended": True,
            }, indent=2)
            self.violation_pub.publish(vmsg)
            estop = String()
            estop.data = "trigger"
            self.estop_pub.publish(estop)
            self.bus.emit_status(self.AGENT_ID, "safety_violation", status, 0.95)

        if not state.operational and state.armed:
            self.bus.emit_status(self.AGENT_ID, "safety_degraded", status, 0.7)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
