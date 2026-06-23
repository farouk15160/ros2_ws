#!/usr/bin/env python3
"""URAF Multi-Robot Coordinator — Phase 6 namespace and priority management."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .agent_bus import AgentBus
from .config_store import ConfigStore
from .multi_robot import MultiRobotCoordinator


class MultiRobotCoordinatorNode(Node):
    AGENT_ID = "multi_robot_coordinator"

    def __init__(self):
        super().__init__("uraf_multi_robot_coordinator")
        self.declare_parameter("status_rate_hz", 0.5)
        self.store = ConfigStore(self.get_logger())
        self.bus = AgentBus(self)
        self.status_pub = self.create_publisher(String, "/uraf/multi_robot/status", 10)
        self.coordinator = MultiRobotCoordinator(self.store.load())
        rate = self.get_parameter("status_rate_hz").value
        self.create_timer(1.0 / rate, self._publish_status)
        self.create_subscription(String, "/uraf/config/reload", self._on_reload, 10)
        self.get_logger().info("[URAF] Multi-Robot Coordinator active")

    def _on_reload(self, _msg: String):
        self.coordinator = MultiRobotCoordinator(self.store.load())
        self.get_logger().info("Multi-robot config reloaded")

    def _publish_status(self):
        status = self.coordinator.status()
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)
        if status["enabled"]:
            self.bus.emit_status(self.AGENT_ID, "multi_robot_status", status, 0.95)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
