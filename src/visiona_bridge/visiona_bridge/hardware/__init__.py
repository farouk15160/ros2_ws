"""
Hardware communication layer for Visiona Bridge.

This package handles all low-level hardware communication including:
- Serial port management
- Packet protocol (encoding/decoding)
- Robot hardware abstraction
"""

from .packet_protocol import PacketProtocol
from .serial_interface import SerialInterface
from .robot_hardware import RobotHardware

__all__ = ['PacketProtocol', 'SerialInterface', 'RobotHardware']
