import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy.duration
import rclpy.time # Import Time explicitly
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32, Bool, Header # Import Header
import serial
import struct
import time
import threading
import math
import json
import os
import signal # For graceful shutdown

# Import constants from our new file
from .bridge_constants import *

# --- Simulation Constants ---
SIM_LOOP_HZ = 100.0 # How often to update the simulation state
SIM_MAX_SPEED_DPS = 180.0 # Max speed in deg/sec (at speed factor 10)
SIM_MIN_SPEED_DPS = 20.0  # Min speed in deg/sec (at speed factor 500)


# ==============================================================================
# 2. ROS 2 AND FLASK BRIDGE NODE (VERSION 4.1.1 - Sim Fix)
# ==============================================================================
class RobotArmBridge(Node):
    def __init__(self):
        super().__init__('visiona_bridgeith_gui')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('debug', True)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.ser = None

        # Threading and State Management
        self.state_lock = threading.Lock()
        self.sequence_lock = threading.Lock() # Protects sequence data
        self.position_lock = threading.Lock() # Protects saved_positions data
        self.connection_thread = threading.Thread(target=self.manage_connection, daemon=True)
        self.playback_thread = None
        self.stop_playback_flag = threading.Event()
        self.socketio = None # This will be set by the main launcher
        self.reconnect_event = threading.Event()

        # --- New Simulation State ---
        self.simulation_mode = False
        self.simulation_thread = None
        self.sim_current_joint_angles_deg = [90.0] * 6
        self.sim_target_joint_angles_deg = [90.0] * 6
        self.sim_last_update_time = self.get_clock().now()

        # State Variables
        self.speed_factor = 150.0
        self.current_joint_angles_deg = [90.0] * 6
        self.main_current = 0.0
        self.gripper_current = 0.0
        self.is_connected = False
        self.emergency_stop_active = False
        self.sequence = [] 
        self.saved_positions = {} 
        self.servos_min = [15.0] * 6
        self.servos_max = [345.0] * 6
        self.collision_threshold = 5.0
        self.collision_deviation_threshold = 1.0
        self.fan_speed = 18

        self.last_emit_time = self.get_clock().now()
        self.emit_interval = rclpy.duration.Duration(seconds=1.0 / 30.0) # 30 Hz GUI updates

        # ROS Setup
        sensor_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', qos_profile=sensor_qos_profile)
        self.main_current_publisher = self.create_publisher(Float32, 'main_current', 10)
        self.gripper_current_publisher = self.create_publisher(Float32, 'gripper_current', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10) # change this to service maybe?
        self.joint_command_subscriber = self.create_subscription(JointState, 'joint_targets', self.joint_command_callback, 10)
        self.gripper_command_subscriber = self.create_subscription(Float32MultiArray, 'gripper_command', self.gripper_command_callback, 10)
        self.speed_factor_subscriber = self.create_subscription(Float32, 'set_speed_factor', self.set_speed_factor_callback, 10)
        self.homing_subscriber = self.create_subscription(Bool, 'home_robot', self.home_robot_callback, 10) # this one alos to service!
        self.set_min_limits_subscriber = self.create_subscription(Float32MultiArray, 'set_min_limits', self.set_min_limits_callback, 10)
        self.set_max_limits_subscriber = self.create_subscription(Float32MultiArray, 'set_max_limits', self.set_max_limits_callback, 10)
        self.set_threshold_subscriber = self.create_subscription(Float32, 'set_collision_threshold', self.set_threshold_callback, 10)
        self.set_dev_threshold_subscriber = self.create_subscription(Float32, 'set_collision_dev_threshold', self.set_dev_threshold_callback, 10)
        self.release_estop_subscriber = self.create_subscription(Bool, 'release_emergency_stop', self.release_estop_callback, 10) # chnage to service!
        self.fan_speed_subscriber = self.create_subscription(Float32, 'set_fan_speed', self.set_fan_speed_callback, 10)

        # E-Stop timer
        self.last_status_time = self.get_clock().now()
        self.estop_timer = self.create_timer(STATUS_TIMEOUT_SEC + 0.1, self.check_status_timeout)

        # Load saved data on startup
        self.load_positions_from_file(DEFAULT_POSITIONS_FILE)
        self.load_sequence_from_file(DEFAULT_SEQUENCE_FILE)

        self.get_logger().info("Robot Arm Bridge with Web GUI initialized (V4.1.1).")
        if self.debug: self.get_logger().info("Debug mode is ON.")
        self.connection_thread.start()

    # --- New Simulation Methods ---
    def set_simulation_mode(self, enable: bool):
        """Toggles simulation mode on or off."""
        with self.state_lock:
            if self.simulation_mode == enable:
                return # No change
            
            self.simulation_mode = enable
            
            if enable:
                # --- Enabling Simulation Mode ---
                self.get_logger().warn(">>> SIMULATION MODE ENABLED <<<")
                if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'Simulation Mode Enabled.'})
                
                # Force disconnect real serial port
                if self.ser and self.ser.is_open:
                    self.get_logger().info("Closing serial port for simulation.")
                    try: self.ser.close()
                    except Exception as e: self.get_logger().error(f"Error closing serial port: {e}")
                    self.ser = None
                
                # Set "connected" state
                self.is_connected = True
                self.emergency_stop_active = False # Reset E-Stop
                
                # Sync simulation state
                self.sim_current_joint_angles_deg = self.current_joint_angles_deg[:]
                self.sim_target_joint_angles_deg = self.current_joint_angles_deg[:]
                self.sim_last_update_time = self.get_clock().now()
                
                # Start simulation worker thread
                if self.simulation_thread is None or not self.simulation_thread.is_alive():
                    self.simulation_thread = threading.Thread(target=self._simulation_worker, daemon=True)
                    self.simulation_thread.start()
            
            else:
                # --- Disabling Simulation Mode ---
                self.get_logger().warn(">>> SIMULATION MODE DISABLED <<<")
                if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': 'Simulation Mode Disabled.'})
                
                self.is_connected = False # Will trigger reconnect
                
                # Simulation thread will stop itself
                # Trigger the connection manager to wake up and try connecting for real
                self.reconnect_event.set() 
        
        self.emit_status() # Update GUI immediately

    # =========================================================================
    # --- THIS IS THE CORRECTED FUNCTION ---
    # =========================================================================
    def _simulation_worker(self):
        """Worker thread for running the simulation loop."""
        self.get_logger().info("Simulation worker started.")
        sleep_duration = 1.0 / SIM_LOOP_HZ
        
        while rclpy.ok() and self.simulation_mode:
            now = self.get_clock().now()
            dt = (now - self.sim_last_update_time).nanoseconds / 1e9
            self.sim_last_update_time = now

            if dt <= 0:
                time.sleep(sleep_duration / 2.0)
                continue
            
            # --- FIX: Declare all variables *outside* the lock ---
            new_angles = []
            sim_main_current = 0.0
            sim_gripper_current = 0.0
            sim_estop = 0
            
            with self.state_lock:
                # Calculate speed
                speed_range = 500.0 - 10.0
                # 0.0=fast (factor 10), 1.0=slow (factor 500)
                speed_normalized = max(0.0, min(1.0, (self.speed_factor - 10.0) / speed_range)) 
                deg_per_sec = SIM_MAX_SPEED_DPS - (speed_normalized * (SIM_MAX_SPEED_DPS - SIM_MIN_SPEED_DPS))
                max_change_this_frame = deg_per_sec * dt
                
                is_moving = False
                # --- FIX: Use the 'new_angles' variable from the outer scope ---
                new_angles = self.sim_current_joint_angles_deg[:] 
                
                # Interpolate each joint
                for i in range(6):
                    target = self.sim_target_joint_angles_deg[i]
                    current = self.sim_current_joint_angles_deg[i]
                    diff = target - current
                    
                    if abs(diff) < 0.01:
                        continue 
                        
                    is_moving = True
                    change = max(-max_change_this_frame, min(max_change_this_frame, diff))
                    new_angles[i] = current + change
                
                # --- FIX: Update all variables for the outer scope ---
                self.sim_current_joint_angles_deg = new_angles[:]
                sim_main_current = 0.15 if is_moving else 0.05
                # Fake gripper current if the gripper joint is moving
                is_gripper_moving = abs(self.sim_current_joint_angles_deg[5] - self.sim_target_joint_angles_deg[5]) > 0.01
                sim_gripper_current = 5.0 if is_gripper_moving else 0.0 
                sim_estop = 1 if self.emergency_stop_active else 0
            
            # --- FIX: This call now has all the correct values ---
            try:
                # This function publishes to ROS and updates the GUI
                self._update_state_from_status(
                    sim_main_current, 
                    sim_gripper_current,
                    *new_angles, # This is now the list of 6 angles
                    sim_estop
                )
            except Exception as e:
                # Log any errors from the update function
                self.get_logger().error(f"Error in sim worker update: {e}", exc_info=True)
            
            # Sleep for the loop duration
            time.sleep(sleep_duration)
            
        self.get_logger().info("Simulation worker stopped.")
    # =========================================================================
    # --- END OF CORRECTED FUNCTION ---
    # =========================================================================

    def _process_sim_command(self, command_id: int, angles_deg: list, speed_factor: float, gripper_current: float):
        """Intercepts 'send_packet' calls during simulation mode."""
        cmd_char = chr(command_id)
        if self.debug: self.get_logger().info(f"SIM ► ID:'{cmd_char}'")
        
        with self.state_lock:
            if self.emergency_stop_active and cmd_char != 'E':
                self.get_logger().warn("Sim E-Stop active, ignoring command.")
                return True # Pretend it was "sent"
                
            if cmd_char == 'M':
                self.sim_target_joint_angles_deg = angles_deg[:]
            elif cmd_char == 'G':
                # In sim, gripper is just another joint
                self.sim_target_joint_angles_deg = angles_deg[:]
            elif cmd_char == 'H':
                # Set target to a "home" position (e.g., all 90)
                self.sim_target_joint_angles_deg = [90.0] * 6 
            elif cmd_char == 'E':
                self.get_logger().warn("Sim E-Stop Released!")
                self.emergency_stop_active = False
            elif cmd_char == 'N': 
                self.servos_min = angles_deg[:]
            elif cmd_char == 'X': 
                self.servos_max = angles_deg[:]
            elif cmd_char == 'T': 
                self.collision_threshold = speed_factor
            elif cmd_char == 'D': 
                self.collision_deviation_threshold = speed_factor
            elif cmd_char == 'F': 
                self.fan_speed = int(angles_deg[0])
            
            # Emit status immediately for config changes
            if cmd_char not in ['M', 'G', 'H']:
                self.emit_status()

        return True # Packet "sent" successfully


    # --- Connection and Serial Parsing (Modified) ---
    def manage_connection(self):
        """Handles serial connection, disconnection, and retries."""
        while rclpy.ok():
            # If in simulation mode, just sleep and bypass all serial logic
            if self.simulation_mode:
                time.sleep(0.5)
                continue
                
            if self.ser is None or not self.ser.is_open:
                with self.state_lock: self.is_connected = False
                self.emit_status()
                try:
                    self.get_logger().info(f"Attempting connection to {self.serial_port}...")
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                    with self.state_lock: self.is_connected = True
                    self.get_logger().info(f"Successfully connected to {self.serial_port}")
                    self.reconnect_event.clear()
                    if self.emergency_stop_active:
                        self.get_logger().info("Resetting timeout-based E-Stop flag on reconnect.")
                        self.last_status_time = self.get_clock().now()

                    time.sleep(0.1)
                    self.ser.reset_input_buffer()
                    self.request_config_from_mcu()
                    self.send_fan_command_from_gui(self.fan_speed)

                except serial.SerialException as e:
                    self.get_logger().warn(f"Connection failed: {e}. Waiting to retry...", throttle_duration_sec=5)
                    self.reconnect_event.wait(timeout=5.0)
                    if self.reconnect_event.is_set():
                        self.get_logger().info("Reconnect triggered.")
                        self.reconnect_event.clear()
                    continue

            if self.ser and self.ser.is_open:
                self.read_from_serial_robust()
            else:
                time.sleep(1.0)

    def trigger_reconnect(self):
        self.get_logger().info("Manual reconnect requested.")
        if self.ser and self.ser.is_open:
            self.get_logger().info("Closing existing connection.")
            try: self.ser.close()
            except Exception as e: self.get_logger().error(f"Error closing serial port: {e}")
            self.ser = None
        with self.state_lock: self.is_connected = False
        self.emit_status()
        self.reconnect_event.set()

    def read_from_serial_robust(self):
        read_buffer = bytearray()
        while rclpy.ok() and self.ser and self.ser.is_open:
            if self.simulation_mode:
                self.get_logger().info("Sim mode enabled, stopping serial read loop.")
                break
            try:
                bytes_to_read = self.ser.in_waiting
                if bytes_to_read > 0:
                    read_buffer.extend(self.ser.read(bytes_to_read))
                elif not read_buffer:
                    time.sleep(0.005)
                    continue
                processed_bytes = 0
                while len(read_buffer) - processed_bytes >= 2:
                    header_index = read_buffer.find(HEADER_BYTE, processed_bytes)
                    if header_index == -1:
                        processed_bytes = len(read_buffer)
                        break
                    if header_index > processed_bytes:
                        self.get_logger().warn(f"Discarding {header_index - processed_bytes} bytes before header.", throttle_duration_sec=2)
                        processed_bytes = header_index
                    if len(read_buffer) - processed_bytes < 2:
                        break 
                    packet_id = read_buffer[processed_bytes + 1]
                    packet_size, process_func = (0, None)
                    if packet_id == STATUS_PACKET_ID:
                        packet_size, process_func = STATUS_PACKET_SIZE, self.process_status_packet
                    elif packet_id == CONFIG_PACKET_ID:
                        packet_size, process_func = CONFIG_PACKET_SIZE, self.process_config_packet
                    else: 
                        self.get_logger().warn(f"Unknown packet ID: {hex(packet_id)}. Discarding header byte.", throttle_duration_sec=1)
                        if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': f'MCU packet sync error. ID: {hex(packet_id)}'})
                        processed_bytes += 1
                        continue
                    if len(read_buffer) - processed_bytes < packet_size:
                        break 
                    packet_data = read_buffer[processed_bytes : processed_bytes + packet_size]
                    received_checksum = packet_data[-1]
                    calculated_checksum = self.calculate_checksum(packet_data[:-1])
                    if received_checksum == calculated_checksum:
                        try:
                            process_func(packet_data)
                        except Exception as e:
                            self.get_logger().error(f"Error processing packet ID {chr(packet_id)}: {e}")
                    else: 
                        self.get_logger().warn(f"Checksum mismatch for ID {chr(packet_id)}! Discarding.", throttle_duration_sec=1)
                        if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': f'MCU checksum mismatch! (ID: {chr(packet_id)})'})
                    processed_bytes += packet_size
                if processed_bytes > 0:
                    read_buffer = read_buffer[processed_bytes:]
            except (serial.SerialException, OSError) as e:
                if not self.simulation_mode:
                    self.get_logger().error(f"Serial error: {e}. Closing connection.")
                if self.ser:
                    try: self.ser.close()
                    except Exception: pass
                    self.ser = None
                with self.state_lock: self.is_connected = False
                self.emit_status()
                break 
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read loop: {type(e).__name__} - {e}", exc_info=True)
                read_buffer.clear()
                time.sleep(0.1)

    def process_config_packet(self, data):
        try:
            parts = struct.unpack(CONFIG_PACKET_FORMAT, data)
            with self.state_lock:
                self.servos_min = list(parts[2:8])
                self.servos_max = list(parts[8:14])
                self.collision_threshold = parts[14]
                self.collision_deviation_threshold = parts[15]
            self.get_logger().info(f"Received config. Abs:{self.collision_threshold:.2f}A, Dev:{self.collision_deviation_threshold:.2f}A")
            self.emit_status()
            if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': 'Loaded config from MCU.'})
        except struct.error as e:
            self.get_logger().error(f"Failed to unpack config packet: {e}")
        except Exception as e:
             self.get_logger().error(f"Unexpected error in process_config_packet: {e}")

    # --- Refactored Status Processing ---
    def process_status_packet(self, data):
        """Unpacks real status packets from serial."""
        try:
            _, _, main, grip, a0, a1, a2, a3, a4, a5, collision_flag, _ = struct.unpack(STATUS_PACKET_FORMAT, data)
            # Call the shared update logic
            self._update_state_from_status(main, grip, a0, a1, a2, a3, a4, a5, collision_flag)
        except struct.error as e:
            self.get_logger().error(f"Failed to unpack status packet: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in process_status_packet: {e}", exc_info=True)

    def _update_state_from_status(self, main, grip, a0, a1, a2, a3, a4, a5, collision_flag):
        """
        Shared logic to update state from status data (real or simulated).
        This function is the new core of status processing.
        """
        self.last_status_time = self.get_clock().now() # Reset timeout timer
        should_emit_immediately = False
        try:
            with self.state_lock:
                self.main_current, self.gripper_current = main, grip
                self.current_joint_angles_deg = [a0, a1, a2, a3, a4, a5]

            mcu_estop_active = (collision_flag == 1)
            
            with self.state_lock:
                # Log state changes for E-Stop
                if mcu_estop_active and not self.emergency_stop_active:
                    self.get_logger().error("E-STOP triggered!")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'E-STOP triggered!'})
                elif not mcu_estop_active and self.emergency_stop_active:
                    self.get_logger().info("E-Stop cleared.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': 'E-Stop released.'})

                # Sync Python state with MCU/Sim state
                if mcu_estop_active != self.emergency_stop_active:
                    self.emergency_stop_active = mcu_estop_active
                    self.emergency_stop_publisher.publish(Bool(data=mcu_estop_active))
                    should_emit_immediately = True

            if self.debug and not self.simulation_mode: # Don't spam logs in sim mode
                angles_str = ", ".join([f"{angle:.1f}" for angle in self.current_joint_angles_deg])
                self.get_logger().info(f"RECV ◄ 'S'|M:{main:.2f}A G:{grip:.1f}mA A:[{angles_str}] E:{mcu_estop_active}", throttle_duration_sec=0.2)

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']
            joint_state_msg.position = [self.deg_to_rad(angle) for angle in self.current_joint_angles_deg]

            # Publish ROS topics
            self.joint_state_publisher.publish(joint_state_msg)
            self.main_current_publisher.publish(Float32(data=main))
            self.gripper_current_publisher.publish(Float32(data=grip))

            # Update GUI (throttled)
            now = self.get_clock().now()
            if should_emit_immediately or (now - self.last_emit_time > self.emit_interval):
                self.emit_status()
                self.last_emit_time = now
        except Exception as e:
            self.get_logger().error(f"Unexpected error in _update_state_from_status: {e}", exc_info=True)


    def send_packet(self, command_id: int, angles_deg: list, speed_factor: float, gripper_current: float):
        """Packs and sends a command packet. Intercepts for simulation."""
        
        if self.simulation_mode:
            return self._process_sim_command(command_id, angles_deg, speed_factor, gripper_current)

        if not self.is_connected or not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial not connected or not open. Cannot send command.", throttle_duration_sec=2)
            if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'Command failed: Not connected!'})
            return False 
        try:
            payload_angles = list(angles_deg)
            while len(payload_angles) < 6: payload_angles.append(0.0)
            payload = payload_angles[:6] + [speed_factor, gripper_current]

            packet_data = struct.pack(COMMAND_PACKET_PAYLOAD_FORMAT, HEADER_BYTE, command_id, *payload)
            checksum = self.calculate_checksum(packet_data)
            self.ser.write(packet_data + struct.pack('<B', checksum))
            self.ser.flush()

            if self.debug:
                cmd_char = chr(command_id)
                log_msg = f"SENT ► ID:'{cmd_char}'"
                if cmd_char in ['N', 'X']: log_msg += f", Limits:[{', '.join(f'{a:.1f}' for a in payload_angles)}]"
                elif cmd_char == 'T': log_msg += f", AbsThr:{speed_factor:.2f}A"
                elif cmd_char == 'D': log_msg += f", DevThr:{speed_factor:.2f}A"
                elif cmd_char == 'F': log_msg += f", FanDuty:{payload_angles[0]:.0f}"
                elif cmd_char in ['M', 'G', 'H']:
                    log_msg += f", Angles:[{', '.join(f'{a:.1f}' for a in payload_angles)}], Spd:{speed_factor:.0f}"
                    if cmd_char == 'G': log_msg += f", GripCur:{gripper_current:.0f}mA"
                self.get_logger().info(log_msg)
            return True
        except (serial.SerialException, OSError, BrokenPipeError) as e:
            self.get_logger().error(f"Serial write error: {e}. Closing connection.")
            if self.ser:
                try: self.ser.close()
                except Exception: pass
                self.ser = None
            with self.state_lock: self.is_connected = False
            self.emit_status()
            return False
        except Exception as e:
            self.get_logger().error(f"Error sending packet: {e}")
            return False

    # --- ROS Callbacks (Unchanged) ---
    def joint_command_callback(self, msg: JointState):
        if len(msg.position) != 6: return
        angles_deg = [self.rad_to_deg(p) for p in msg.position]
        self.send_packet(ord('M'), angles_deg, self.speed_factor, 0.0)
    def gripper_command_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2: return
        target_angle_deg, current_ma = msg.data[0], msg.data[1]
        with self.state_lock: current_angles = self.current_joint_angles_deg[:]
        current_angles[5] = target_angle_deg
        self.send_packet(ord('G'), current_angles, self.speed_factor, current_ma)
    def set_speed_factor_callback(self, msg: Float32):
        if msg.data > 0:
            with self.state_lock: self.speed_factor = msg.data
            if not self.simulation_mode:
                self.get_logger().info(f"Speed factor set to: {self.speed_factor}")
    def home_robot_callback(self, msg: Bool):
        if msg.data: self.send_home_command_from_gui()
    def set_min_limits_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 6: self.send_min_limits_from_gui(list(msg.data))
    def set_max_limits_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 6: self.send_max_limits_from_gui(list(msg.data))
    def set_threshold_callback(self, msg: Float32):
        self.send_threshold_from_gui(msg.data)
    def set_dev_threshold_callback(self, msg: Float32):
        self.send_dev_threshold_from_gui(msg.data)
    def release_estop_callback(self, msg: Bool):
        if msg.data: self.release_estop_from_gui()
    def set_fan_speed_callback(self, msg: Float32):
        self.send_fan_command_from_gui(int(msg.data))

    # --- GUI-facing Methods (Unchanged) ---
    def send_joint_command_from_gui(self, angles_deg):
        if not angles_deg or len(angles_deg) != 6:
            self.get_logger().error("Invalid joint command from GUI.")
            return
        self.send_packet(ord('M'), angles_deg, self.speed_factor, 0.0)
    def send_home_command_from_gui(self):
        self.get_logger().info("Homing command received.")
        self.send_packet(ord('H'), [90.0] * 6, 0.0, 0.0)
    def set_speed_factor_from_gui(self, speed):
        try:
            speed = float(speed)
            if speed > 0:
                with self.state_lock: self.speed_factor = speed
                if not self.simulation_mode:
                    self.get_logger().info(f"Speed factor set via GUI: {self.speed_factor}")
                self.emit_status() 
            else:
                self.get_logger().warn(f"Ignoring invalid speed factor: {speed}")
        except ValueError:
            self.get_logger().error(f"Invalid speed value received from GUI: {speed}")
    def send_gripper_command_from_gui(self, angle, current):
        try:
            angle = float(angle)
            current = float(current)
            with self.state_lock: current_angles = self.current_joint_angles_deg[:]
            current_angles[5] = angle
            self.send_packet(ord('G'), current_angles, self.speed_factor, current)
        except ValueError:
            self.get_logger().error(f"Invalid gripper command values from GUI: angle={angle}, current={current}")
    def send_min_limits_from_gui(self, limits):
        try:
            limits = [float(l) for l in limits]
            if len(limits) != 6: raise ValueError("Incorrect number of limits")
            with self.state_lock: self.servos_min = limits[:]
            self.get_logger().info(f"Setting MIN limits via GUI: {limits}")
            self.send_packet(ord('N'), limits, 0.0, 0.0)
            self.emit_status()
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"Invalid min limits from GUI: {limits} - {e}")
    def send_max_limits_from_gui(self, limits):
        try:
            limits = [float(l) for l in limits]
            if len(limits) != 6: raise ValueError("Incorrect number of limits")
            with self.state_lock: self.servos_max = limits[:]
            self.get_logger().info(f"Setting MAX limits via GUI: {limits}")
            self.send_packet(ord('X'), limits, 0.0, 0.0)
            self.emit_status()
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"Invalid max limits from GUI: {limits} - {e}")
    def send_threshold_from_gui(self, threshold):
        try:
            threshold = float(threshold)
            if 0.1 <= threshold <= 20.0:
                with self.state_lock: self.collision_threshold = threshold
                self.get_logger().info(f"Setting absolute threshold via GUI: {threshold:.2f}A")
                self.send_packet(ord('T'), [], threshold, 0.0)
                self.emit_status()
            else:
                self.get_logger().warn(f"Ignoring out-of-range absolute threshold: {threshold}")
        except ValueError:
            self.get_logger().error(f"Invalid absolute threshold value from GUI: {threshold}")
    def send_dev_threshold_from_gui(self, threshold):
        try:
            threshold = float(threshold)
            if 0.05 <= threshold <= 10.0:
                with self.state_lock: self.collision_deviation_threshold = threshold
                self.get_logger().info(f"Setting deviation threshold via GUI: {threshold:.2f}A")
                self.send_packet(ord('D'), [], threshold, 0.0)
                self.emit_status()
            else:
                self.get_logger().warn(f"Ignoring out-of-range deviation threshold: {threshold}")
        except ValueError:
            self.get_logger().error(f"Invalid deviation threshold value from GUI: {threshold}")
    def send_fan_command_from_gui(self, speed_value):
        try:
            speed_value = max(0, min(255, int(speed_value)))
            with self.state_lock: self.fan_speed = speed_value
            self.send_packet(ord('F'), [float(speed_value)], 0.0, 0.0)
            self.emit_status()
        except ValueError:
            self.get_logger().error(f"Invalid fan speed value from GUI: {speed_value}")
    def request_config_from_mcu(self):
        self.get_logger().info("Requesting config from MCU...")
        self.send_packet(ord('R'), [], 0.0, 0.0)
    def save_config_on_mcu(self):
        self.get_logger().info("Requesting MCU save config to EEPROM...")
        if self.send_packet(ord('C'), [], 0.0, 0.0):
            if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': 'Save command sent.'})
        else:
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Failed to send save command!'})
    def release_estop_from_gui(self):
        self.get_logger().info("Attempting E-Stop release...")
        if self.send_packet(ord('E'), [], 0.0, 0.0):
            if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'E-Stop release command sent...'})
        else:
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Failed to send E-Stop release command!'})

    # --- Sequence Methods (Unchanged) ---
    def add_sequence_point(self, delay_ms=500):
        try:
            delay_ms = int(delay_ms)
        except ValueError:
            delay_ms = 500
            self.get_logger().warn("Invalid delay value, using default 500ms.")
        with self.state_lock, self.sequence_lock:
            if not self.current_joint_angles_deg or len(self.current_joint_angles_deg) != 6:
                self.get_logger().error("Cannot add sequence point: Invalid current angles.")
                if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Cannot save pose: Invalid current angles.'})
                return
            point = {"angles": self.current_joint_angles_deg[:], "speed": self.speed_factor, "delay_ms": delay_ms}
            self.sequence.append(point)
            self.get_logger().info(f"Added sequence point {len(self.sequence)}.")
            if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': f'Pose added to sequence (point {len(self.sequence)}).'})
        self.emit_status() 

    def clear_sequence(self):
        with self.sequence_lock:
            self.sequence.clear()
            self.get_logger().info("Sequence cleared.")
            if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'Sequence cleared.'})
        self.emit_status()

    def delete_sequence_point(self, index):
        try:
            index = int(index)
            with self.sequence_lock:
                if 0 <= index < len(self.sequence):
                    del self.sequence[index]
                    self.get_logger().info(f"Deleted sequence point {index+1}.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': f'Sequence point {index+1} deleted.'})
                else:
                    raise IndexError("Index out of bounds")
        except (ValueError, IndexError, TypeError) as e: 
            self.get_logger().error(f"Invalid index for deleting sequence point: {index} ({e})")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Invalid sequence index: {index+1 if isinstance(index, int) else index}'})
        self.emit_status()

    def save_sequence_to_file(self, filename):
        filename = filename if filename else DEFAULT_SEQUENCE_FILE
        filename = "".join(c if c.isalnum() or c in ('_', '.', '-') else '_' for c in filename)
        if not filename.endswith(".json"):
            filename += ".json"
        filepath = os.path.join(os.getcwd(), filename)
        try:
            with self.sequence_lock, open(filepath, 'w') as f:
                json.dump(self.sequence, f, indent=4)
            self.get_logger().info(f"Sequence saved to {filepath}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'success', 'message': f'Sequence saved to {filename}.'})
        except IOError as e:
            self.get_logger().error(f"Error saving sequence to {filepath}: {e}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Error saving sequence: {e}'})
        except Exception as e:
            self.get_logger().error(f"Unexpected error saving sequence to {filepath}: {e}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Unexpected error saving sequence.'})

    def load_sequence_from_file(self, filename):
        filename = filename if filename else DEFAULT_SEQUENCE_FILE
        filename = "".join(c if c.isalnum() or c in ('_', '.', '-') else '_' for c in filename)
        if not filename.endswith(".json"):
            filename += ".json"
        filepath = os.path.join(os.getcwd(), filename)
        try:
            with open(filepath, 'r') as f:
                loaded_sequence = json.load(f)
            if isinstance(loaded_sequence, list) and all(
                isinstance(p, dict) and
                'angles' in p and isinstance(p['angles'], list) and len(p['angles']) == 6 and
                'speed' in p and isinstance(p['speed'], (int, float)) and
                'delay_ms' in p and isinstance(p['delay_ms'], int)
                for p in loaded_sequence
            ):
                with self.sequence_lock:
                    self.sequence = loaded_sequence
                self.get_logger().info(f"Sequence loaded from {filepath} ({len(self.sequence)} points)")
                if self.socketio: self.socketio.emit('log_message', {'level': 'success', 'message': f'Sequence loaded from {filename}.'})
                self.emit_status()
            else:
                raise ValueError("Invalid sequence file format or content")
        except FileNotFoundError:
            self.get_logger().warn(f"Sequence file {filepath} not found. Starting with empty sequence.")
            if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': f'Sequence file {filename} not found.'})
            with self.sequence_lock: self.sequence = [] 
            self.emit_status()
        except (IOError, json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f"Error loading sequence from {filepath}: {e}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Error loading sequence file {filename}: {e}'})
            with self.sequence_lock: self.sequence = [] 
            self.emit_status()
        except Exception as e: 
            self.get_logger().error(f"Unexpected error loading sequence from {filepath}: {e}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Unexpected error loading sequence file {filename}.'})
            with self.sequence_lock: self.sequence = []
            self.emit_status()

    def play_sequence(self):
        with self.state_lock:
            if self.playback_thread and self.playback_thread.is_alive():
                self.get_logger().warn("Sequence playback already in progress.")
                return
            if self.emergency_stop_active:
                self.get_logger().error("Cannot play sequence while E-Stop is active.")
                if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Cannot play: E-Stop Active!'})
                return
        with self.sequence_lock:
            if not self.sequence:
                self.get_logger().warn("Sequence is empty, nothing to play.")
                if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'Sequence is empty.'})
                return
        self.stop_playback_flag.clear()
        self.playback_thread = threading.Thread(target=self._sequence_playback_worker, daemon=True)
        self.playback_thread.start()
        self.get_logger().info("Starting sequence playback...")
        if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': 'Sequence playback started.'})
        self.emit_status()

    def stop_sequence(self):
        if self.playback_thread and self.playback_thread.is_alive():
            self.get_logger().info("Stopping sequence playback...")
            self.stop_playback_flag.set()
            if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'Sequence playback stopping...'})
        else:
            self.get_logger().info("No active sequence playback to stop.")
        if self.socketio:
            self.socketio.emit('playback_stopped') 

    def _sequence_playback_worker(self):
        last_angles = None
        current_step = 0
        sequence_len = 0
        try:
            with self.state_lock:
                if self.current_joint_angles_deg and len(self.current_joint_angles_deg) == 6:
                    last_angles = self.current_joint_angles_deg[:]
                else:
                    self.get_logger().error("Cannot start playback: Invalid initial angles.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Playback failed: Invalid start pose.'})
                    return 
            with self.sequence_lock:
                sequence_copy = self.sequence[:]
                sequence_len = len(sequence_copy)
            if not sequence_copy: return 
            for i, point in enumerate(sequence_copy):
                current_step = i + 1
                if self.stop_playback_flag.is_set():
                    self.get_logger().info(f"Playback stopped by user at step {current_step}.")
                    break
                with self.state_lock:
                    if self.emergency_stop_active:
                        self.get_logger().error(f"E-Stop activated during playback at step {current_step}. Stopping sequence.")
                        if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'E-Stop during playback step {current_step}!'})
                        break
                angles = point.get("angles")
                speed = point.get("speed", self.speed_factor)
                delay_ms = point.get("delay_ms", 0)
                if not angles or len(angles) != 6:
                    self.get_logger().error(f"Invalid angles found at sequence point {current_step}. Skipping.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Invalid angles at step {current_step}. Skipping.'})
                    continue
                self.get_logger().info(f"Moving to sequence point {current_step}/{sequence_len}...")
                if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': f'Moving to step {current_step}/{sequence_len}...' })
                max_delta = 0.0
                if last_angles:
                    for j in range(6):
                        delta = abs(angles[j] - last_angles[j])
                        if delta > max_delta: max_delta = delta
                else:
                    max_delta = 90.0 
                
                estimated_duration_ms = max(50.0, max_delta * speed)
                
                if not self.send_packet(ord('M'), angles, speed, 0.0):
                    self.get_logger().error(f"Failed to send command during sequence playback (step {current_step}). Stopping.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Send failed at step {current_step}!'})
                    break
                
                # --- Wait Logic ---
                wait_time_sec = (estimated_duration_ms + delay_ms) / 1000.0
                
                if self.simulation_mode:
                    wait_start_time = time.time()
                    max_wait_sec = 10.0 # Timeout for sim move
                    while time.time() - wait_start_time < max_wait_sec:
                        if self.stop_playback_flag.is_set(): break
                        with self.state_lock:
                            if self.emergency_stop_active: break
                            at_target = True
                            for j in range(6):
                                if abs(self.current_joint_angles_deg[j] - angles[j]) > 0.5: # 0.5 deg tolerance
                                    at_target = False
                                    break
                            if at_target:
                                break
                        time.sleep(0.05) # Poll quickly
                    
                    if delay_ms > 0 and not self.stop_playback_flag.is_set():
                        self.stop_playback_flag.wait(timeout=delay_ms / 1000.0)
                
                else:
                    # Original wait logic for real robot
                    check_interval = 0.05
                    end_time = time.time() + wait_time_sec
                    interrupted = False
                    while time.time() < end_time:
                        if self.stop_playback_flag.wait(timeout=check_interval):
                            self.get_logger().info("Playback interrupted during wait.")
                            interrupted = True
                            break
                        with self.state_lock:
                            if self.emergency_stop_active:
                                self.get_logger().warn("E-Stop detected during wait.")
                                interrupted = True
                                break
                    if interrupted: break 

                last_angles = angles[:] 
        except Exception as e:
            self.get_logger().error(f"Unexpected error during sequence playback worker: {e}", exc_info=True)
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Critical error during playback!'})
        finally:
            self.stop_playback_flag.clear()
            final_message = "Sequence playback finished." if current_step == sequence_len and not self.emergency_stop_active else "Sequence playback stopped."
            self.get_logger().info(final_message)
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'info', 'message': final_message})
                self.socketio.emit('playback_stopped')

    # --- Saved Position Methods (Unchanged) ---
    def save_position(self, name):
        if not name or not isinstance(name, str) or not name.strip():
            self.get_logger().error("Invalid name for saving position.")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Invalid position name.'})
            return
        name = name.strip()
        with self.state_lock, self.position_lock:
            if not self.current_joint_angles_deg or len(self.current_joint_angles_deg) != 6:
                self.get_logger().error("Cannot save position: Invalid current angles.")
                if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Cannot save pose: Invalid current angles.'})
                return
            self.saved_positions[name] = {"angles": self.current_joint_angles_deg[:]}
            self.get_logger().info(f"Saved position '{name}'.")
            if self.socketio: self.socketio.emit('log_message', {'level': 'success', 'message': f"Position '{name}' saved."})
        self.emit_status()
        self.save_positions_to_file(DEFAULT_POSITIONS_FILE) 

    def go_to_position(self, name):
        with self.position_lock:
            position_data = self.saved_positions.get(name)
        if position_data and "angles" in position_data:
            with self.state_lock: 
                if self.emergency_stop_active:
                    self.get_logger().error(f"Cannot go to position '{name}': E-Stop Active.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': 'Cannot move: E-Stop Active!'})
                    return
                if self.playback_thread and self.playback_thread.is_alive():
                    self.get_logger().warn(f"Cannot go to position '{name}' during sequence playback.")
                    if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': 'Stop sequence before moving to position.'})
                    return
            self.get_logger().info(f"Moving to saved position '{name}'...")
            if self.socketio: self.socketio.emit('log_message', {'level': 'info', 'message': f"Moving to '{name}'..."})
            self.send_packet(ord('M'), position_data["angles"], self.speed_factor, 0.0)
        else:
            self.get_logger().error(f"Saved position '{name}' not found.")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f"Position '{name}' not found."})

    def delete_position(self, name):
        with self.position_lock:
            if name in self.saved_positions:
                del self.saved_positions[name]
                self.get_logger().info(f"Deleted saved position '{name}'.")
                if self.socketio: self.socketio.emit('log_message', {'level': 'warn', 'message': f"Position '{name}' deleted."})
                self.emit_status()
                self.save_positions_to_file(DEFAULT_POSITIONS_FILE)
            else:
                self.get_logger().warn(f"Saved position '{name}' not found for deletion.")
                if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f"Position '{name}' not found."})

    def save_positions_to_file(self, filename):
        filename = filename if filename else DEFAULT_POSITIONS_FILE
        filename = "".join(c if c.isalnum() or c in ('_', '.', '-') else '_' for c in filename)
        if not filename.endswith(".json"): filename += ".json"
        filepath = os.path.join(os.getcwd(), filename)
        try:
            with self.position_lock, open(filepath, 'w') as f:
                json.dump(self.saved_positions, f, indent=4)
            self.get_logger().debug(f"Saved positions to {filepath}")
        except IOError as e:
            self.get_logger().error(f"Error saving positions to {filepath}: {e}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Error saving positions: {e}'})
        except Exception as e:
            self.get_logger().error(f"Unexpected error saving positions: {e}")

    def load_positions_from_file(self, filename):
        filename = filename if filename else DEFAULT_POSITIONS_FILE
        filename = "".join(c if c.isalnum() or c in ('_', '.', '-') else '_' for c in filename)
        if not filename.endswith(".json"): filename += ".json"
        filepath = os.path.join(os.getcwd(), filename)
        try:
            if not os.path.exists(filepath):
                self.get_logger().warn(f"Positions file {filepath} not found. Starting with empty positions.")
                with self.position_lock: self.saved_positions = {}
                self.emit_status() 
                return
            with open(filepath, 'r') as f:
                loaded_positions = json.load(f)
            if isinstance(loaded_positions, dict) and all(
                isinstance(p, dict) and 'angles' in p and isinstance(p['angles'], list) and len(p['angles']) == 6
                for p in loaded_positions.values()
                ):
                with self.position_lock:
                    self.saved_positions = loaded_positions
                self.get_logger().info(f"Loaded {len(self.saved_positions)} positions from {filepath}")
                self.emit_status()
            else:
                raise ValueError("Invalid positions file format or content")
        except (IOError, json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f"Error loading positions from {filepath}: {e}")
            if self.socketio: self.socketio.emit('log_message', {'level': 'error', 'message': f'Error loading positions file {filename}: {e}'})
            with self.position_lock: self.saved_positions = {}
            self.emit_status()
        except Exception as e:
            self.get_logger().error(f"Unexpected error loading positions: {e}")
            with self.position_lock: self.saved_positions = {}
            self.emit_status()


    # --- Other Methods (Modified) ---
    def check_status_timeout(self):
        """Timer callback ONLY for complete communication loss."""
        # --- Do not run timeout check in simulation mode ---
        if self.simulation_mode:
            return 
            
        if not self.is_connected: return 
        now = self.get_clock().now()
        duration = now - self.last_status_time
        with self.state_lock:
            if not self.emergency_stop_active and self.is_connected:
                if duration.nanoseconds / 1e9 > STATUS_TIMEOUT_SEC:
                    self.get_logger().error("Status timeout! MCU communication lost. Activating E-Stop.")
                    self.emergency_stop_active = True 
                    self.is_connected = False 
                    self.emergency_stop_publisher.publish(Bool(data=True))
                    if self.ser and self.ser.is_open:
                        self.get_logger().warn("Closing serial port due to timeout.")
                        try: self.ser.close()
                        except Exception as e: self.get_logger().error(f"Error closing serial port on timeout: {e}")
                        self.ser = None
                    minimal_update = {'emergency_stop_active': True, 'is_connected': False}
                    if self.socketio:
                        self.socketio.emit('status_update', minimal_update)
                        self.socketio.emit('log_message', {'level': 'error', 'message': 'MCU communication timeout! E-Stop activated.'})

    def emit_status(self):
        """Gathers all state data and emits it to all connected GUI clients."""
        if not self.socketio: return
        status_data = {}
        try:
            with self.state_lock:
                status_data = {
                    "is_connected": self.is_connected,
                    "emergency_stop_active": self.emergency_stop_active,
                    "main_current": self.main_current,
                    "gripper_current": self.gripper_current,
                    "joint_angles": self.current_joint_angles_deg[:] if self.current_joint_angles_deg else [0.0]*6,
                    "servos_min": self.servos_min[:],
                    "servos_max": self.servos_max[:],
                    "collision_threshold": self.collision_threshold,
                    "collision_deviation_threshold": self.collision_deviation_threshold,
                    "fan_speed": self.fan_speed,
                    "current_speed_factor": self.speed_factor,
                    "is_playing": self.playback_thread and self.playback_thread.is_alive(),
                    "simulation_mode": self.simulation_mode
                }
            with self.sequence_lock:
                status_data["sequence"] = self.sequence[:]
            with self.position_lock:
                status_data["saved_positions"] = list(self.saved_positions.keys()) 

            self.socketio.emit('status_update', status_data)
        except Exception as e:
            self.get_logger().error(f"Error during emit_status: {e}", exc_info=True)

    def calculate_checksum(self, data: bytes) -> int:
        checksum = 0
        for byte in data: checksum ^= byte
        return checksum
    def deg_to_rad(self, deg): return deg * math.pi / 180.0
    def rad_to_deg(self, rad): return rad * 180.0 / math.pi

    def cleanup(self):
        self.get_logger().info("Cleaning up node...")
        self.stop_sequence() 
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=0.2)
        
        if self.simulation_mode:
            self.simulation_mode = False 
        
        self.save_positions_to_file(DEFAULT_POSITIONS_FILE)
        self.save_sequence_to_file(DEFAULT_SEQUENCE_FILE)
        if self.ser and self.ser.is_open:
            self.get_logger().info("Closing serial port.")
            try: self.ser.close()
            except Exception as e: self.get_logger().error(f"Error closing serial port during cleanup: {e}")
            self.ser = None
        self.reconnect_event.set()