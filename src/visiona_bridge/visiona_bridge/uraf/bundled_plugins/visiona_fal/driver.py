"""Reference FAL driver plugin for Visiona ESP32 (PLAN.md §20 example)."""

from __future__ import annotations

from typing import Any

from ...plugin_base import FALDriverPlugin


class VisionaFALDriver(FALDriverPlugin):
    """Stub FAL driver — production path uses bridge serial protocol."""

    def __init__(self):
        self._connected = False
        self._config: dict[str, Any] = {}

    def initialize(self, config: dict[str, Any]) -> bool:
        self._config = config
        return True

    def connect(self, config: dict[str, Any]) -> bool:
        self._config.update(config)
        port = config.get("serial_port", "/dev/ttyUSB0")
        self._connected = bool(port)
        return self._connected

    def write_command(self, joint_id: int, value: float) -> bool:
        return self._connected

    def status(self) -> dict[str, Any]:
        return {
            "connected": self._connected,
            "port": self._config.get("serial_port", "/dev/ttyUSB0"),
            "protocol": "visiona_packet_v5",
        }
