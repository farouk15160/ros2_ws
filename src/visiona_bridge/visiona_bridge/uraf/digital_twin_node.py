#!/usr/bin/env python3
"""URAF Digital Twin Agent  Phase 18 (PLAN.md 18)."""

from __future__ import annotations

import json
import uuid

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .agent_bus import AgentBus
from .digital_twin import DigitalTwinEngine


class DigitalTwinNode(Node):
    AGENT_ID = DigitalTwinEngine.AGENT_ID

    def __init__(self):
        super().__init__("uraf_digital_twin")
        self.declare_parameter("backend", "bridge_sim")
        self.declare_parameter("pre_validate", True)
        self.declare_parameter("gate_motion", False)
        self.declare_parameter("status_rate_hz", 1.0)

        workspace = self._load_workspace()
        backend = self.get_parameter("backend").value
        self.engine = DigitalTwinEngine(backend=backend, workspace=workspace)
        self.bus = AgentBus(self)

        self.state_pub = self.create_publisher(String, "/uraf/twin/state", 10)
        self.validation_pub = self.create_publisher(String, "/uraf/twin/validation", 10)
        self.cartesian_out = self.create_publisher(PoseStamped, "/visiona/cartesian_command", 10)

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(PoseStamped, "/visiona/current_pose", self._on_pose, 10)
        self.create_subscription(String, "/visiona/twin/validate_motion", self._on_validate, 10)

        if self.get_parameter("gate_motion").value:
            self.create_subscription(PoseStamped, "/visiona/cartesian_request", self._on_motion_request, 10)

        rate = self.get_parameter("status_rate_hz").value
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info(f"[URAF] Digital Twin active (backend={backend})")

    def _load_workspace(self) -> dict:
        try:
            share = get_package_share_directory("visiona_bridge")
            path = f"{share}/config/kinematic_params.yaml"
            raw = yaml.safe_load(open(path, encoding="utf-8")) or {}
            params = raw.get("kinematic_params", {}).get("ros__parameters", raw)
            return params.get("workspace", {})
        except Exception as exc:
            self.get_logger().warn(f"Using default workspace limits: {exc}")
            return {}

    def _on_joints(self, msg: JointState):
        if msg.position:
            self.engine.update_joints(list(msg.position))

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.engine.update_ee_pose(p.x, p.y, p.z)

    def _on_validate(self, msg: String):
        try:
            req = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        request_id = req.get("request_id", str(uuid.uuid4()))
        x, y, z = float(req["x"]), float(req["y"]), float(req["z"])
        result = self.engine.validate_motion(x, y, z)
        self._publish_validation(request_id, result.approved, result.reason, result.checks)

    def _on_motion_request(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        result = self.engine.validate_motion(x, y, z)
        if result.approved:
            self.cartesian_out.publish(msg)
            self.get_logger().info(f"Twin approved motion ? ({x:.3f},{y:.3f},{z:.3f})")
        else:
            self.get_logger().warn(result.reason)
            self._publish_validation(str(uuid.uuid4()), False, result.reason, result.checks)

    def _publish_validation(self, request_id: str, approved: bool, reason: str, checks: dict):
        payload = {
            "request_id": request_id,
            "approved": approved,
            "reason": reason,
            "checks": checks,
        }
        out = String()
        out.data = json.dumps(payload, indent=2)
        self.validation_pub.publish(out)
        if not approved:
            self.bus.emit_status(self.AGENT_ID, "motion_rejected", payload, 0.9)

    def _publish_state(self):
        status = self.engine.status()
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
