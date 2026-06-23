#!/usr/bin/env python3
"""URAF Self-Healing Agent  Phase 6 (enhanced health monitor + auto-recovery)."""

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .agent_bus import AgentBus
from .self_healing import FailureEvent, FailureType, RecoveryAction, SelfHealingEngine


class SelfHealingNode(Node):
    AGENT_ID = "self_healing_agent"

    def __init__(self):
        super().__init__("uraf_self_healing")
        self.declare_parameter("status_rate_hz", 0.5)
        self.declare_parameter("joint_stale_sec", 2.0)
        self.declare_parameter("recovery_cooldown_sec", 30.0)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.25)
        self.declare_parameter("home_z", 0.35)

        self.engine = SelfHealingEngine(
            cooldown_sec=self.get_parameter("recovery_cooldown_sec").value,
        )
        self.bus = AgentBus(self)
        self.health_pub = self.create_publisher(String, "/uraf/health", 10)
        self.recovery_pub = self.create_publisher(String, "/uraf/recovery/status", 10)
        self.discovery_pub = self.create_publisher(String, "/uraf/discovery/trigger", 10)
        self.scan_pub = self.create_publisher(String, "/jarvis/scan_trigger", 10)
        self.cartesian_pub = self.create_publisher(PoseStamped, "/visiona/cartesian_command", 10)

        self._last_joint_time = None
        self._system_status: dict = {}
        self._llm_status = "unknown"
        self._recent_feedback: list[str] = []
        self._active_failures: list[dict] = []
        self._recovery_count = 0
        self._recovery_success = 0
        self._failure_state: dict[str, bool] = {}

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(String, "/visiona/system_status", self._on_system_status, 10)
        self.create_subscription(String, "/jarvis/feedback", self._on_feedback, 10)
        self.create_subscription(String, "/jarvis/llm_status", self._on_llm_status, 10)

        rate = self.get_parameter("status_rate_hz").value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("[URAF] Self-Healing Agent active")

    def _on_joints(self, _msg: JointState):
        self._last_joint_time = time.time()

    def _on_system_status(self, msg: String):
        try:
            self._system_status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_llm_status(self, msg: String):
        self._llm_status = msg.data.strip().lower()

    def _on_feedback(self, msg: String):
        text = msg.data.strip()
        self._recent_feedback.append(text.lower())
        self._recent_feedback = self._recent_feedback[-20:]
        if "motion timeout" in text:
            self._handle_failure(FailureEvent(
                FailureType.MOTION_TIMEOUT,
                text,
                severity="warn",
            ))

    def _tick(self):
        now = time.time()
        stale_sec = self.get_parameter("joint_stale_sec").value
        joint_ok = self._last_joint_time is not None and (now - self._last_joint_time) < stale_sec
        serial_ok = self._system_status.get("is_connected", True)
        estop = self._system_status.get("emergency_stop", False)
        llm_ok = not self._llm_status.startswith("error")
        perception_ok = self._system_status.get("perception_ok", True)

        self._active_failures = []
        if not joint_ok:
            self._active_failures.append({"type": FailureType.JOINT_STATES_STALE.value, "message": "Joint states stale"})
        if not serial_ok:
            self._active_failures.append({"type": FailureType.SERIAL_DISCONNECTED.value, "message": "MCU serial disconnected"})
        if estop:
            self._active_failures.append({"type": FailureType.ESTOP_ACTIVE.value, "message": "Emergency stop active"})
        if not llm_ok and self._system_status.get("jarvis_enabled", False):
            self._active_failures.append({"type": FailureType.LLM_UNAVAILABLE.value, "message": self._llm_status})

        for failure in list(self._active_failures):
            ftype = failure["type"]
            was_active = self._failure_state.get(ftype, False)
            self._failure_state[ftype] = True
            if not was_active:
                self._handle_failure(FailureEvent(FailureType(ftype), failure["message"], severity="crit" if estop else "warn"))

        for ftype in list(self._failure_state.keys()):
            if not any(f["type"] == ftype for f in self._active_failures):
                self._failure_state[ftype] = False

        report = self.engine.build_health_report(
            joint_ok=joint_ok,
            serial_ok=serial_ok,
            llm_ok=llm_ok,
            perception_ok=perception_ok,
            estop=estop,
            active_failures=self._active_failures,
            recovery_stats={
                "attempts": self._recovery_count,
                "successes": self._recovery_success,
                "recent": [
                    {"action": r.action.value, "success": r.success, "message": r.message}
                    for r in self.engine.history[-5:]
                ],
            },
        )
        report["last_joint_age_sec"] = round(now - self._last_joint_time, 2) if self._last_joint_time else None

        msg = String()
        msg.data = json.dumps(report, indent=2)
        self.health_pub.publish(msg)

        if report["status"] != "healthy":
            self.bus.emit_status(self.AGENT_ID, "health_report", report, 0.85 if joint_ok else 0.5)

    def _handle_failure(self, event: FailureEvent):
        actions = self.engine.plan_recovery(event)
        for action in actions:
            result = self._execute_recovery(action, event)
            if result:
                self.engine.record_recovery(event, result)
                self._recovery_count += 1
                if result.success:
                    self._recovery_success += 1
                self._publish_recovery(result)
                if action == RecoveryAction.ALERT_USER:
                    self.bus.emit_status(
                        self.AGENT_ID,
                        "recovery_alert",
                        {"failure": event.failure_type.value, "message": event.message, "recovery": result.message},
                        confidence=0.9,
                    )
                break

    def _execute_recovery(self, action: RecoveryAction, event: FailureEvent):
        from .self_healing import RecoveryResult

        if action == RecoveryAction.REPUBLISH_HOME:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.pose.position.x = float(self.get_parameter("home_x").value)
            msg.pose.position.y = float(self.get_parameter("home_y").value)
            msg.pose.position.z = float(self.get_parameter("home_z").value)
            msg.pose.orientation.w = 1.0
            self.cartesian_pub.publish(msg)
            self.get_logger().info("Recovery: republish home pose")
            return RecoveryResult(action, True, "Sent home Cartesian command")

        if action == RecoveryAction.TRIGGER_DISCOVERY:
            out = String()
            out.data = "recovery"
            self.discovery_pub.publish(out)
            self.get_logger().info("Recovery: triggered hardware discovery")
            return RecoveryResult(action, True, "Hardware discovery triggered")

        if action == RecoveryAction.TRIGGER_RESCAN:
            out = String()
            out.data = "__scan__"
            self.scan_pub.publish(out)
            self.get_logger().info("Recovery: triggered scene rescan")
            return RecoveryResult(action, True, "Scene rescan triggered")

        if action == RecoveryAction.ALERT_USER:
            self.get_logger().warn(f"Recovery alert: {event.message}")
            return RecoveryResult(action, True, f"User alert: {event.message}")

        return None

    def _publish_recovery(self, result):
        payload = {
            "action": result.action.value,
            "success": result.success,
            "message": result.message,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        msg = String()
        msg.data = json.dumps(payload, indent=2)
        self.recovery_pub.publish(msg)
        self.bus.emit_result(self.AGENT_ID, "recovery_action", payload, 0.85 if result.success else 0.4)


def main(args=None):
    rclpy.init(args=args)
    node = SelfHealingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
