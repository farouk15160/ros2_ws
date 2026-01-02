"""
Packet protocol for Visiona robot communication.

Handles encoding and decoding of serial packets for communication with the robot MCU.
"""

import struct
from typing import Tuple, List


# ==============================================================================
# PACKET PROTOCOL CONSTANTS
# ==============================================================================
HEADER_BYTE = 0xA5

# Command packet format: Header, ID, 6 joint angles, speed, gripper current
COMMAND_PACKET_PAYLOAD_FORMAT = '<BB8f'

# Status packet format
STATUS_PACKET_ID = ord('S')
STATUS_PACKET_FORMAT = '<BBff6fBB'
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FORMAT)

# Config packet format
CONFIG_PACKET_ID = ord('R')
CONFIG_PACKET_FORMAT = '<BB6f6fffB'
CONFIG_PACKET_SIZE = struct.calcsize(CONFIG_PACKET_FORMAT)


class PacketProtocol:
    """
    Handles encoding and decoding of packets for robot communication.
    
    This class provides methods to build command packets and parse
    status/config packets from the robot MCU.
    """
    
    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """Calculate XOR checksum for packet data."""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    @staticmethod
    def build_command_packet(command_id: int, angles_deg: List[float], 
                           speed_factor: float, gripper_current: float) -> bytes:
        """
        Build a command packet to send to the robot.
        
        Args:
            command_id: ASCII command character (e.g., ord('M'))
            angles_deg: List of 6 joint angles in degrees
            speed_factor: Speed scaling factor
            gripper_current: Gripper current in mA
            
        Returns:
            Complete packet as bytes including checksum
        """
        # Ensure we have 6 angles
        payload_angles = list(angles_deg)
        while len(payload_angles) < 6:
            payload_angles.append(0.0)
        
        # Build payload: 6 angles + speed + gripper current
        payload = payload_angles[:6] + [speed_factor, gripper_current]
        
        # Pack packet data
        packet_data = struct.pack(
            COMMAND_PACKET_PAYLOAD_FORMAT,
            HEADER_BYTE,
            command_id,
            *payload
        )
        
        # Add checksum
        checksum = PacketProtocol.calculate_checksum(packet_data)
        return packet_data + struct.pack('<B', checksum)
    
    @staticmethod
    def parse_status_packet(data: bytes) -> Tuple[float, float, List[float], int]:
        """
        Parse a status packet from the robot.
        
        Args:
            data: Raw packet bytes
            
        Returns:
            Tuple of (main_current, gripper_current, joint_angles, collision_flag)
            
        Raises:
            struct.error: If packet format is invalid
        """
        _, _, main, grip, a0, a1, a2, a3, a4, a5, collision_flag, _ = struct.unpack(
            STATUS_PACKET_FORMAT, data
        )
        joint_angles = [a0, a1, a2, a3, a4, a5]
        return main, grip, joint_angles, collision_flag
    
    @staticmethod
    def parse_config_packet(data: bytes) -> Tuple[List[float], List[float], float, float]:
        """
        Parse a configuration packet from the robot.
        
        Args:
            data: Raw packet bytes
            
        Returns:
            Tuple of (servos_min, servos_max, collision_threshold, deviation_threshold)
            
        Raises:
            struct.error: If packet format is invalid
        """
        parts = struct.unpack(CONFIG_PACKET_FORMAT, data)
        servos_min = list(parts[2:8])
        servos_max = list(parts[8:14])
        collision_threshold = parts[14]
        collision_deviation_threshold = parts[15]
        return servos_min, servos_max, collision_threshold, collision_deviation_threshold
