#!/usr/bin/env python3
"""URAF Hardware Discovery Agent  Phase 1 (Visiona-focused)."""

from __future__ import annotations

import glob
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .agent_bus import AgentBus
from .config_store import ConfigStore


class HardwareDiscoveryNode(Node):
    AGENT_ID = "hardware_discovery_agent"

    def __init__(self):
        super().__init__("uraf_hardware_discovery")
        self.declare_parameter("serial_probe_baud", 921600)
        self.store = ConfigStore(self.get_logger())
        self.bus = AgentBus(self)
        self.profile_pub = self.create_publisher(String, "/uraf/hardware_profile", 10)
        self.create_subscription(String, "/uraf/discovery/trigger", self._on_trigger, 10)
        self.get_logger().info("[URAF] Hardware Discovery Agent ready")

    def _on_trigger(self, _msg: String):
        profile = self.discover()
        raw = json.dumps(profile, indent=2)
        out = String()
        out.data = raw
        self.profile_pub.publish(out)
        self.bus.emit_result(self.AGENT_ID, "hardware_profile", profile, profile.get("confidence", 0.0))

    def discover(self) -> dict:
        self.get_logger().info("Starting hardware discovery")
        interfaces = self._scan_serial_ports()
        ros2 = self._scan_ros2_ecosystem()
        config = self.store.load()
        robot_name = config.get("robot", {}).get("name", "Visiona V1")

        confidence = 0.92 if interfaces else 0.55
        if ros2.get("joint_states_topic"):
            confidence = min(0.98, confidence + 0.05)

        profile = {
            "discovery_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "confidence": confidence,
            "robot_id": f"visiona:{robot_name.lower().replace(' ', '_')}",
            "robot_name": robot_name,
            "interfaces": interfaces,
            "ros2": ros2,
            "joints": {
                "count": 6,
                "type": "revolute",
                "encoders": True,
                "control_modes": ["position"],
            },
            "known_robot_family": "visiona_v1",
            "community_profile_match": {"found": True, "profile_id": "visiona_v1", "match_confidence": 0.95},
        }
        self.get_logger().info(f"Discovery complete  confidence {confidence:.0%}")
        return profile

    def _scan_serial_ports(self) -> list:
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        results = []
        for port in ports:
            entry = {
                "type": "serial",
                "device": port,
                "baudrate_candidates": [921600, 115200, 57600],
                "protocol_guess": "visiona_packet_v5" if "USB" in port else "unknown",
                "responsive": os.access(port, os.R_OK | os.W_OK),
            }
            results.append(entry)
        return results

    def _scan_ros2_ecosystem(self) -> dict:
        topics = [name for name, _ in self.get_topic_names_and_types()]
        nodes = self.get_node_names()
        return {
            "topics_count": len(topics),
            "nodes_count": len(nodes),
            "joint_states_topic": "/joint_states" if "/joint_states" in topics else None,
            "cartesian_topic": "/visiona/cartesian_command" if "/visiona/cartesian_command" in topics else None,
            "jarvis_active": "/jarvis/command" in topics,
            "octomap_active": any("octomap" in t for t in topics),
            "camera_active": any("ascamera" in t for t in topics),
        }


def main(args=None):
    rclpy.init(args=args)
    node = HardwareDiscoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
