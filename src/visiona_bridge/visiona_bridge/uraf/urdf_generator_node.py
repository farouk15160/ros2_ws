#!/usr/bin/env python3
"""URAF URDF Generator Node — Phase 2."""

from __future__ import annotations

import json

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from .agent_bus import AgentBus
from .urdf_generator import UrdfGenerator


class UrdfGeneratorNode(Node):
    def __init__(self):
        super().__init__("uraf_urdf_generator")
        self.generator = UrdfGenerator(self.get_logger())
        self.bus = AgentBus(self)
        self.result_pub = self.create_publisher(String, "/uraf/urdf_generated", 10)
        self.create_subscription(String, "/uraf/generate_urdf", self._on_trigger, 10)
        self.create_subscription(String, "/uraf/discovery/trigger", self._on_trigger, 10)
        self.get_logger().info("[URAF] URDF Generator ready")

    def _on_trigger(self, _msg: String):
        share = get_package_share_directory("visiona_bridge")
        profile = f"{share}/config/robots/visiona_v1.yaml"
        kinematic = f"{share}/config/kinematic_params.yaml"
        try:
            manifest = self.generator.generate_from_profile(
                __import__("pathlib").Path(profile),
                __import__("pathlib").Path(kinematic),
            )
            raw = json.dumps(manifest, indent=2)
            out = String()
            out.data = raw
            self.result_pub.publish(out)
            self.bus.emit_result(
                UrdfGenerator.AGENT_ID,
                "urdf_manifest",
                manifest,
                manifest.get("confidence", 0.8),
            )
            self.get_logger().info(f"URDF generated: {manifest.get('urdf_path')}")
        except Exception as exc:
            self.get_logger().error(f"URDF generation failed: {exc}")
            self.bus.emit_status(
                UrdfGenerator.AGENT_ID,
                "generation_error",
                {"error": str(exc)},
                0.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = UrdfGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
