#!/usr/bin/env python3
"""URAF Plugin Manager — PLAN.md §20."""

from __future__ import annotations

import json

import rclpy
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import String

from .agent_bus import AgentBus
from .plugin_loader import PluginLoader
from .plugin_registry import PluginRegistry


class PluginManagerNode(Node):
    AGENT_ID = "plugin_manager"

    def __init__(self):
        super().__init__("uraf_plugin_manager")
        self.declare_parameter("auto_install_bundled", True)
        self.registry = PluginRegistry(self.get_logger())
        self.loader = PluginLoader(self.registry, self.get_logger())
        self.bus = AgentBus(self)
        self.status_pub = self.create_publisher(String, "/uraf/plugins/status", 10)

        if self.get_parameter("auto_install_bundled").value:
            self._install_bundled()

        self.loader.load_all()
        self.create_timer(2.0, self._publish_status)
        self.create_subscription(String, "/uraf/plugins/reload", self._on_reload, 10)
        self.get_logger().info("[URAF] Plugin Manager active")

    def _install_bundled(self):
        try:
            share = Path(get_package_share_directory("visiona_bridge"))
            bundled = share / "plugins"
            if (bundled / "visiona_fal" / "plugin.yaml").is_file():
                self.registry.install_from_manifest(bundled / "visiona_fal" / "plugin.yaml")
        except Exception as exc:
            self.get_logger().warn(f"Bundled plugin install skipped: {exc}")

    def _on_reload(self, _msg: String):
        self.loader.load_all()
        self._publish_status()

    def _publish_status(self):
        status = self.loader.status()
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PluginManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
