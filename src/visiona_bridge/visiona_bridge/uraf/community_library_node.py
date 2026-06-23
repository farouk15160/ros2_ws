#!/usr/bin/env python3
"""URAF Community Robot Library agent — PLAN.md §21."""

from __future__ import annotations

import json

import rclpy
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import String

from .agent_bus import AgentBus
from .community_library import CommunityLibrary, compute_robot_fingerprint


class CommunityLibraryNode(Node):
    AGENT_ID = CommunityLibrary.AGENT_ID

    def __init__(self):
        super().__init__("uraf_community_library")
        share = Path(get_package_share_directory("visiona_bridge"))
        self.library = CommunityLibrary(bundled_dir=share / "community", logger=self.get_logger())
        self.bus = AgentBus(self)
        self.catalog_pub = self.create_publisher(String, "/uraf/community/catalog", 10)
        self.match_pub = self.create_publisher(String, "/uraf/community/match", 10)
        self.create_subscription(String, "/uraf/discovery/trigger", self._on_discovery, 10)
        self.create_subscription(String, "/uraf/hardware_profile", self._on_profile, 10)
        self.create_subscription(String, "/uraf/community/lookup", self._on_lookup, 10)
        self.create_timer(5.0, self._publish_catalog)
        self.get_logger().info("[URAF] Community Robot Library active")

    def _publish_catalog(self):
        catalog = {"profiles": self.library.list_profiles()}
        msg = String()
        msg.data = json.dumps(catalog, indent=2)
        self.catalog_pub.publish(msg)

    def _on_discovery(self, _msg: String):
        self._publish_catalog()

    def _on_profile(self, msg: String):
        try:
            profile = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        match = self.library.lookup_from_discovery(profile)
        if match:
            result = {
                "matched": True,
                "profile_id": match["crl"].get("profile_id"),
                "robot_name": match["crl"].get("robot_name"),
                "test_status": match["crl"].get("test_status"),
                "confidence": profile.get("confidence", 0.8),
            }
            out = String()
            out.data = json.dumps(result, indent=2)
            self.match_pub.publish(out)
            self.bus.emit_result(self.AGENT_ID, "profile_match", result, result.get("confidence", 0.8))

    def _on_lookup(self, msg: String):
        try:
            req = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        profile_id = req.get("profile_id")
        fingerprint = req.get("fingerprint")
        entry = None
        if profile_id:
            entry = self.library.lookup_by_id(profile_id)
        elif fingerprint:
            entry = self.library.lookup_by_fingerprint(fingerprint)
        result = {"matched": entry is not None}
        if entry:
            result.update({
                "profile_id": entry["crl"].get("profile_id"),
                "robot_name": entry["crl"].get("robot_name"),
                "fingerprint": entry["crl"].get("fingerprint"),
            })
        out = String()
        out.data = json.dumps(result, indent=2)
        self.match_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CommunityLibraryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
