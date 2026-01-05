"""
Main ROS2 Bridge Node for Visiona Robot - Restructured V5.0

This orchestrator coordinates between three clean layers:
- Hardware Layer: Serial communication, packet protocol, robot state
- ROS2 Layer: Publishers, subscribers, services, cartesian interface  
- GUI Layer: Flask routes, SocketIO handlers

File size: ~280 lines (target: < 300 lines)
"""

import rclpy
from rclpy.node import Node
import math

# Hardware layer
from .hardware import PacketProtocol, SerialInterface, RobotHardware

# State management
from .state import SequenceManager, PositionManager

# ROS2 interface
from .ros2_interface.publishers import RobotPublishers
from .ros2_interface.subscribers import RobotSubscribers
from .ros2_interface.services import RobotServices
from .ros2_interface.cartesian_interface import CartesianInterface

# GUI layer
from .gui.socketio_handlers import emit_status, emit_log_message, emit_cartesian_pose

# Constants
from .bridge_constants import STATUS_TIMEOUT_SEC


class RobotArmBridge(Node):
    """
    Main ROS2 node orchestrating the robot bridge.
    
    Delegates to specialized modules for hardware, ROS2, and GUI communication.
    """
    
    def __init__(self):
        super().__init__('visiona_bridge_with_gui')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('debug', True)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('sync_gazebo', False)
        self.declare_parameter('mode', 'real')  # sim, real, or fake
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.should_publish = self.get_parameter('publish_joint_states').get_parameter_value().bool_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.socketio = None  # Set by GUI
        self.is_connected = False
        self.last_status_time = self.get_clock().now()
        
        # Initialize Cartesian Interface FIRST (needed by routes)
        self.cartesian = CartesianInterface(self, self.get_logger())
        self.get_logger().info("✅ Cartesian Interface Initialized (Simple IK Mode)")
        
        # Initialize layers
        self._init_hardware_layer()
        self._init_state_managers()
        self._init_ros2_layer()
        
        # Now it's safe to activate simulation mode (after ROS2 layer is ready)
        if self.mode == 'sim':
            self.hardware.set_simulation_mode(True)
            self.is_connected = True  # Pretend we're connected in sim
        
        # Status timeout timer
        self.estop_timer = self.create_timer(STATUS_TIMEOUT_SEC + 0.1, self._check_timeout)
        
        # Load saved data - use runtime path functions
        from .bridge_constants import get_positions_file, get_sequence_file
        self.position_mgr.load_from_file(get_positions_file())
        self.sequence_mgr.load_from_file(get_sequence_file())
        
        self.get_logger().info("Robot Arm Bridge initialized (Restructured V5.0)")
    
    def _init_hardware_layer(self):
        """Initialize hardware communication layer."""
        self.hardware = RobotHardware(
            logger=self.get_logger(),
            on_state_update=self._on_hardware_update,
            get_clock=self.get_clock,
            socketio=self.socketio  # Pass socketio for MCU logging
        )
        
        # Only initialize serial in real mode
        if self.mode == 'real':
            self.serial = SerialInterface(
                port=self.serial_port,
                baudrate=self.baud_rate,
                logger=self.get_logger(),
                on_status_packet=self.hardware.process_status_packet,
                on_config_packet=self.hardware.process_config_packet,
                on_connection_changed=self._on_connection_changed
            )
        else:
            self.serial = None
            self.get_logger().info(f"Running in {self.mode} mode - serial connection disabled")
            # Don't activate simulation here - defer until after ROS2 layer init
    
    def _init_state_managers(self):
        """Initialize sequence and position managers."""
        self.sequence_mgr = SequenceManager(
            logger=self.get_logger(),
            get_clock=self.get_clock,
            send_command=self._send_command,
            get_current_state=self._get_state
        )
        
        self.position_mgr = PositionManager(
            logger=self.get_logger(),
            send_command=self._send_command,
            get_current_state=self._get_state
        )
    
    def _init_ros2_layer(self):
        """Initialize ROS2 publishers, subscribers, services."""
        self.robot_publishers = RobotPublishers(self, self.deg_to_rad)
        self.robot_subscribers = RobotSubscribers(self, self)
        self.robot_services = RobotServices(self, self)
        self.cartesian = CartesianInterface(self, self.get_logger())
        
        # Cartesian pose publisher timer
        self.create_timer(0.5, self._publish_cartesian_timer)
    
    # === Hardware callbacks ===
    
    def _on_hardware_update(self, state_dict, immediate=False):
        """Called when hardware state updates."""
        if self.should_publish:
            sync_gazebo = self.hardware.simulation_mode or \
                         self.get_parameter('sync_gazebo').get_parameter_value().bool_value
            self.robot_publishers.publish_joint_state(state_dict['joint_angles'], sync_gazebo)
        
        self.robot_publishers.publish_currents(state_dict['main_current'], state_dict['gripper_current'])
        self.last_status_time = self.get_clock().now()
        
        if immediate or self.socketio:
            self.emit_full_status()
    
    def _on_connection_changed(self, connected: bool):
        """Called when serial connection state changes."""
        self.is_connected = connected
        if connected:
            self._send_command(ord('R'), [], 0.0, 0.0)  # Request config
            self._send_command(ord('O'), [], 0.0, 0.0)  # Safety OFF on boot
        self.emit_full_status()
    
    def _send_command(self, cmd_id: int, angles_deg: list, speed: float, gripper_cur: float) -> bool:
        """Send command via simulation or serial."""
        if self.hardware.simulation_mode:
            return self.hardware.process_sim_command(cmd_id, angles_deg, speed, gripper_cur)
        
        packet = PacketProtocol.build_command_packet(cmd_id, angles_deg, speed, gripper_cur)
        success = self.serial.write_packet(packet)
        
        if self.debug and success:
            self.get_logger().info(f"SENT ► ID:'{chr(cmd_id)}'")
        
        return success
    
    def _get_state(self) -> dict:
        """Get complete robot state."""
        state = self.hardware.get_state_dict()
        state['is_connected'] = self.is_connected
        state['is_playing'] = self.sequence_mgr.is_playing()
        return state
    
    def _publish_cartesian_timer(self):
        """Timer callback to publish cartesian pose."""
        pose = self.robot_publishers.publish_cartesian_pose(
            self.cartesian.tf_buffer,
            self.cartesian.planning_frame,
            self.cartesian.ee_link
        )
        if pose and self.socketio:
            emit_cartesian_pose(self.socketio, *pose)
    
    def _check_timeout(self):
        """Check for communication timeout."""
        if self.hardware.simulation_mode or not self.is_connected:
            return
        
        duration = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
        if duration > STATUS_TIMEOUT_SEC and not self.hardware.emergency_stop_active:
            self.get_logger().error("Status timeout! MCU communication lost.")
            self.hardware.emergency_stop_active = True
            self.serial.trigger_reconnect()
            if self.socketio:
                emit_log_message(self.socketio, 'error', 'MCU timeout! E-Stop activated.')
            self.emit_full_status()
    
    def emit_full_status(self):
        """Emit complete status to GUI."""
        if self.socketio:
            emit_status(
                self.socketio,
                self._get_state(),
                self.sequence_mgr.get_sequence(),
                self.position_mgr.get_position_names()
            )
    
    # === Public API (called from GUI routes) ===
    
    def set_simulation_mode(self, enable: bool):
        self.hardware.set_simulation_mode(enable)
        if enable:
            self.serial.trigger_reconnect()
        self.emit_full_status()
    
    def send_joint_command(self, angles_deg: list):
        if len(angles_deg) == 6:
            self._send_command(ord('M'), angles_deg, self.hardware.speed_factor, 0.0)
    
    def send_home_command(self):
        self._send_command(ord('H'), [90.0]*6, 0.0, 0.0)
    
    def send_gripper_command(self, angle: float, current: float):
        state = self._get_state()
        angles = state['joint_angles'][:]
        angles[5] = angle
        self._send_command(ord('G'), angles, self.hardware.speed_factor, current)
    
    def set_speed_factor(self, speed: float):
        if speed > 0:
            self.hardware.speed_factor = speed
            self.emit_full_status()
    
    def set_min_limits(self, limits: list):
        if len(limits) == 6:
            self.hardware.servos_min = limits
            self._send_command(ord('N'), limits, 0.0, 0.0)
            self.emit_full_status()
    
    def set_max_limits(self, limits: list):
        if len(limits) == 6:
            self.hardware.servos_max = limits
            self._send_command(ord('X'), limits, 0.0, 0.0)
            self.emit_full_status()
    
    def set_collision_threshold(self, threshold: float):
        if 0.1 <= threshold <= 20.0:
            self.hardware.collision_threshold = threshold
            self._send_command(ord('T'), [], threshold, 0.0)
            self.emit_full_status()
    
    def set_deviation_threshold(self, threshold: float):
        if 0.05 <= threshold <= 10.0:
            self.hardware.collision_deviation_threshold = threshold
            self._send_command(ord('D'), [], threshold, 0.0)
            self.emit_full_status()
    
    def set_fan_speed(self, speed: int):
        speed = max(0, min(255, speed))
        self.hardware.fan_speed = speed
        self._send_command(ord('F'), [float(speed)], 0.0, 0.0)
        self.emit_full_status()
    
    def save_config(self):
        self._send_command(ord('C'), [], 0.0, 0.0)
    
    def release_estop(self):
        self._send_command(ord('E'), [], 0.0, 0.0)
    
    def kill_motors(self):
        self._send_command(ord('K'), [], 0.0, 0.0)
    
    def set_collision_safety(self, enable: bool):
        val = 1.0 if enable else 0.0
        self._send_command(ord('O'), [], val, 0.0)
        self.hardware.collision_detection_enabled = enable
        self.emit_full_status()
    
    def trigger_reconnect(self):
        self.serial.trigger_reconnect()
    
    def jog_cartesian(self, dx: float, dy: float, dz: float):
        self.cartesian.jog(dx, dy, dz, self.socketio)
    
    # Sequence management (delegate to manager)
    def add_sequence_point(self, delay_ms: int):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.add_point(delay_ms)
        self.emit_full_status()
    
    def clear_sequence(self):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.clear()
        self.emit_full_status()
    
    def delete_sequence_point(self, index: int):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.delete_point(index)
        self.emit_full_status()
    
    def save_sequence(self, filename: str):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.save_to_file(filename)
    
    def load_sequence(self, filename: str):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.load_from_file(filename)
        self.emit_full_status()
    
    def play_sequence(self):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.play()
        self.emit_full_status()
    
    def stop_sequence(self):
        self.sequence_mgr.socketio = self.socketio
        self.sequence_mgr.stop()
        self.emit_full_status()
    
    # Position management (delegate to manager)
    def save_position(self, name: str):
        self.position_mgr.socketio = self.socketio
        self.position_mgr.save(name)
        from .bridge_constants import get_positions_file
        self.position_mgr.save_to_file(get_positions_file())
        self.emit_full_status()
    
    def go_to_position(self, name: str):
        self.position_mgr.socketio = self.socketio
        self.position_mgr.go_to(name)
    
    def delete_position(self, name: str):
        self.position_mgr.socketio = self.socketio
        self.position_mgr.delete(name)
        from .bridge_constants import get_positions_file
        self.position_mgr.save_to_file(get_positions_file())
        self.emit_full_status()
    
    # Utility methods
    def deg_to_rad(self, deg: float) -> float:
        rad = deg * math.pi / 180.0
        while rad > math.pi:
            rad -= 2 * math.pi
        while rad < -math.pi:
            rad += 2 * math.pi
        return rad
    
    def rad_to_deg(self, rad: float) -> float:
        return rad * 180.0 / math.pi
    
    def cleanup(self):
        """Cleanup on shutdown."""
        self.get_logger().info("Cleaning up node...")
        self.sequence_mgr.stop()
        from .bridge_constants import get_positions_file, get_sequence_file
        self.position_mgr.save_to_file(get_positions_file())
        self.sequence_mgr.save_to_file(get_sequence_file())
        if self.serial:
            self.serial.shutdown()