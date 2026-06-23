#!/usr/bin/env python3
"""URAF Learning Agent — Phase 6 execution recording and grasp models."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .agent_bus import AgentBus
from .learning_store import LearningStore


class LearningAgentNode(Node):
    AGENT_ID = "learning_agent"

    def __init__(self):
        super().__init__("uraf_learning_agent")
        self.declare_parameter("stats_rate_hz", 0.2)
        self.store = LearningStore(self.get_logger())
        self.bus = AgentBus(self)
        self.stats_pub = self.create_publisher(String, "/uraf/learning/stats", 10)
        self._current_plan: dict | None = None
        self._plan_start = None

        self.create_subscription(String, "/jarvis/action_plan", self._on_plan, 10)
        self.create_subscription(String, "/jarvis/execution_event", self._on_event, 10)
        self.create_subscription(String, "/jarvis/execution_status", self._on_exec_status, 10)

        rate = self.get_parameter("stats_rate_hz").value
        self.create_timer(1.0 / rate, self._publish_stats)
        self.get_logger().info("[URAF] Learning Agent active")

    def _on_plan(self, msg: String):
        try:
            self._current_plan = json.loads(msg.data)
            self._plan_start = time.time()
        except (json.JSONDecodeError, TypeError):
            self._current_plan = None

    def _on_event(self, msg: String):
        try:
            event = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.store.record_execution(event)
        if event.get("event") == "plan_complete":
            self.bus.emit_result(self.AGENT_ID, "learning_update", self.store.get_stats(), 0.9)

    def _on_exec_status(self, msg: String):
        if msg.data == "executing":
            self._plan_start = time.time()
        elif msg.data == "idle" and self._current_plan and self._plan_start:
            duration = time.time() - self._plan_start
            self.store.record_execution({
                "event": "plan_complete",
                "plan_id": self._current_plan.get("plan_id", "unknown"),
                "actions": self._current_plan.get("actions", []),
                "success": True,
                "duration_sec": round(duration, 2),
                "planning_backend": self._current_plan.get("planning_backend", "unknown"),
            })
            self._current_plan = None
            self._plan_start = None

    def _publish_stats(self):
        stats = self.store.get_stats()
        msg = String()
        msg.data = json.dumps(stats, indent=2)
        self.stats_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LearningAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
