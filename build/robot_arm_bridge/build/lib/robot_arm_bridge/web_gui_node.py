import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32, Bool
import serial
import struct
import time
import threading
import math
import json
from flask import Flask, render_template_string, jsonify, request
from flask_socketio import SocketIO
import logging

# ==============================================================================
# 1. CONSTANTS AND PACKET DEFINITIONS
# ==============================================================================

HEADER_BYTE = 0xA5
# =========================================================
# === PACKET FORMAT FIX APPLIED HERE ======================
# =========================================================
# PC to ESP32: header(B), cmd_id(B), 6 angles + speed + current (8 floats)
COMMAND_PACKET_PAYLOAD_FORMAT = '<BB8f' # Use unambiguous format for 8 floats
# =========================================================
# ESP32 to PC: header(B), main_current(f), grip_current(f), 6 angles(f), checksum(B)
STATUS_PACKET_FORMAT = '<Bff6fB'
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FORMAT)
STATUS_TIMEOUT_SEC = 2.0  # Seconds without status packet before warning/E-Stop

# Disable verbose logging from Flask and SocketIO to keep the console clean
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)

# ==============================================================================
# 2. ROS 2 AND FLASK BRIDGE NODE
# ==============================================================================

class RobotArmBridge(Node):
    """
    Bridges robot arm serial communication to ROS and a Flask-SocketIO web GUI.
    """
    def __init__(self):
        super().__init__('robot_arm_bridge_with_gui')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('debug', True) # For verbose logging
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.ser = None

        # --- Threading and State Management ---
        self.state_lock = threading.Lock()
        self.sequence_lock = threading.Lock()
        self.connection_thread = threading.Thread(target=self.manage_connection, daemon=True)
        self.playback_thread = None
        self.stop_playback_flag = threading.Event()

        # --- State Variables ---
        self.speed_factor = 150.0
        self.current_joint_angles_deg = [90.0] * 6
        self.main_current = 0.0
        self.gripper_current = 0.0
        self.is_connected = False
        self.emergency_stop_active = False
        self.sequence = []

        # --- ROS Publishers & Subscribers ---
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.main_current_publisher = self.create_publisher(Float32, 'main_current', 10)
        self.gripper_current_publisher = self.create_publisher(Float32, 'gripper_current', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.joint_command_subscriber = self.create_subscription(JointState, 'joint_targets', self.joint_command_callback, 10)
        self.gripper_command_subscriber = self.create_subscription(Float32MultiArray, 'gripper_command', self.gripper_command_callback, 10)
        self.speed_factor_subscriber = self.create_subscription(Float32, 'set_speed_factor', self.set_speed_factor_callback, 10)
        self.homing_subscriber = self.create_subscription(Bool, 'home_robot', self.home_robot_callback, 10)
        self.last_status_time = self.get_clock().now()
        self.estop_timer = self.create_timer(1.0, self.check_status_timeout)
        
        self.get_logger().info("Robot Arm Bridge with Web GUI initialized.")
        if self.debug: self.get_logger().info("Debug mode is ON.")
        self.connection_thread.start()

    def manage_connection(self):
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                with self.state_lock: self.is_connected = False
                try:
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                    with self.state_lock: self.is_connected = True
                    self.get_logger().info(f"Successfully connected to {self.serial_port}")
                    self.emergency_stop_active = False
                    self.emergency_stop_publisher.publish(Bool(data=False))
                    time.sleep(0.1)
                    self.ser.reset_input_buffer()
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to connect: {e}. Retrying...", throttle_duration_sec=5)
                    time.sleep(5)
                    continue
            self.read_from_serial()

    def read_from_serial(self):
        read_buffer = bytearray()
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    read_buffer.extend(self.ser.read(self.ser.in_waiting))
                
                while len(read_buffer) >= STATUS_PACKET_SIZE:
                    header_index = read_buffer.find(HEADER_BYTE)
                    if header_index == -1: read_buffer.clear(); continue
                    if header_index > 0: read_buffer = read_buffer[header_index:]
                    if len(read_buffer) < STATUS_PACKET_SIZE: break

                    packet_data = read_buffer[:STATUS_PACKET_SIZE]
                    received_checksum = packet_data[-1]
                    calculated_checksum = self.calculate_checksum(packet_data[:-1])

                    if received_checksum == calculated_checksum:
                        self.process_status_packet(packet_data)
                        read_buffer = read_buffer[STATUS_PACKET_SIZE:]
                    else:
                        self.get_logger().warn("Checksum mismatch! Discarding packet.", throttle_duration_sec=1)
                        socketio.emit('log_message', {'level': 'warn', 'message': 'MCU checksum mismatch!'})
                        read_buffer = read_buffer[1:] # Move forward one byte to find next potential header

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Serial error: {e}. Closing connection.")
                if self.ser: self.ser.close(); self.ser = None
                with self.state_lock: self.is_connected = False
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read loop: {e}")
            time.sleep(0.001)

    def process_status_packet(self, data):
        self.last_status_time = self.get_clock().now()
        
        with self.state_lock:
            if self.emergency_stop_active:
                self.get_logger().info("Communication re-established, clearing E-Stop.")
                self.emergency_stop_publisher.publish(Bool(data=False))
                self.emergency_stop_active = False

        try:
            _, main, grip, a0, a1, a2, a3, a4, a5, _ = struct.unpack(STATUS_PACKET_FORMAT, data)
            
            with self.state_lock:
                self.main_current, self.gripper_current = main, grip
                self.current_joint_angles_deg = [a0, a1, a2, a3, a4, a5]

            if self.debug:
                angles_str = ", ".join([f"{angle:.1f}" for angle in self.current_joint_angles_deg])
                self.get_logger().info(f"RECV ◄ Main:{main:.2f}A, Grip:{grip:.1f}mA, Angles:[{angles_str}]", throttle_duration_sec=0.2)
            
            self.main_current_publisher.publish(Float32(data=main))
            self.gripper_current_publisher.publish(Float32(data=grip))
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']
            joint_state_msg.position = [self.deg_to_rad(angle) for angle in self.current_joint_angles_deg]
            self.joint_state_publisher.publish(joint_state_msg)
            
            self.emit_status()

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack status packet: {e}")

    def send_packet(self, command_id, angles_deg, speed_factor, gripper_current):
        if not self.is_connected or not self.ser:
            self.get_logger().warn("Serial not connected. Cannot send command.", throttle_duration_sec=2)
            socketio.emit('log_message', {'level': 'warn', 'message': 'Command failed: Not connected!'})
            return
        
        try:
            packet_data = struct.pack(COMMAND_PACKET_PAYLOAD_FORMAT, HEADER_BYTE, command_id, *angles_deg, speed_factor, gripper_current)
            checksum = self.calculate_checksum(packet_data)
            self.ser.write(packet_data + struct.pack('<B', checksum))
            
            if self.debug:
                angles_str = ", ".join([f"{angle:.1f}" for angle in angles_deg])
                self.get_logger().info(f"SENT ► ID:{chr(command_id)}, Angles:[{angles_str}], Speed:{speed_factor:.0f}, Grip Cur:{gripper_current:.0f}mA")

        except Exception as e:
            self.get_logger().error(f"Error sending packet: {e}")

    def save_sequence(self):
        with self.sequence_lock:
            try:
                with open("sequence.json", "w") as f:
                    json.dump(self.sequence, f, indent=2)
                self.get_logger().info("Sequence saved to sequence.json")
                socketio.emit('log_message', {'level': 'info', 'message': 'Sequence saved.'})
            except Exception as e:
                self.get_logger().error(f"Failed to save sequence: {e}")
                socketio.emit('log_message', {'level': 'error', 'message': 'Failed to save sequence.'})

    def load_sequence(self):
        try:
            with open("sequence.json", "r") as f:
                loaded_sequence = json.load(f)
            with self.sequence_lock:
                self.sequence = loaded_sequence
            self.get_logger().info("Sequence loaded from sequence.json")
            socketio.emit('log_message', {'level': 'info', 'message': 'Sequence loaded.'})
            self.emit_status()
        except FileNotFoundError:
            self.get_logger().warn("sequence.json not found.")
            socketio.emit('log_message', {'level': 'warn', 'message': 'sequence.json not found.'})
        except Exception as e:
            self.get_logger().error(f"Failed to load sequence: {e}")
            socketio.emit('log_message', {'level': 'error', 'message': 'Error loading sequence file.'})
            
    def check_status_timeout(self):
        if not self.is_connected: return
        with self.state_lock:
            if not self.emergency_stop_active:
                duration = self.get_clock().now() - self.last_status_time
                if duration.nanoseconds / 1e9 > STATUS_TIMEOUT_SEC:
                    self.get_logger().error("Status timeout! MCU may have stopped. Activating E-Stop.")
                    self.emergency_stop_publisher.publish(Bool(data=True))
                    self.emergency_stop_active = True
                    self.emit_status()

    def joint_command_callback(self, msg: JointState):
        angles_rad = list(msg.position)
        if len(angles_rad) == 5:
            base_angle = angles_rad[0]
            angles_rad = [base_angle, base_angle] + angles_rad[1:5]
        elif len(angles_rad) != 6:
            self.get_logger().warn(f"Command requires 5 or 6 joint angles, received {len(angles_rad)}. Ignoring.")
            return
        angles_deg = [self.rad_to_deg(p) for p in angles_rad]
        self.send_packet(ord('M'), angles_deg, self.speed_factor, 200.0)

    def gripper_command_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2: return
        target_angle_deg, current_ma = msg.data[0], msg.data[1]
        with self.state_lock: current_angles = self.current_joint_angles_deg[:]
        current_angles[5] = target_angle_deg
        self.send_packet(ord('G'), current_angles, self.speed_factor, current_ma)
        
    def set_speed_factor_callback(self, msg: Float32):
        if msg.data > 0:
            with self.state_lock: self.speed_factor = msg.data
            self.get_logger().info(f"Robot speed factor set to: {self.speed_factor}")

    def home_robot_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Homing command received from ROS.")
            self.send_packet(ord('H'), [0.0] * 6, self.speed_factor, 0.0)

    def send_joint_command_from_gui(self, angles_deg):
        self.send_packet(ord('M'), angles_deg, self.speed_factor, 200.0)

    def send_home_command_from_gui(self):
        self.get_logger().info("Homing command received from GUI.")
        self.send_packet(ord('H'), [0.0] * 6, self.speed_factor, 0.0)

    def send_gripper_command_from_gui(self, angle, current):
        with self.state_lock: current_angles = self.current_joint_angles_deg[:]
        current_angles[5] = angle
        self.send_packet(ord('G'), current_angles, self.speed_factor, current)

    def set_speed_factor_from_gui(self, speed):
        with self.state_lock: self.speed_factor = float(speed)
            
    def record_waypoint(self):
        with self.state_lock, self.sequence_lock:
            self.sequence.append(self.current_joint_angles_deg[:])
            self.get_logger().info(f"Waypoint recorded: {self.current_joint_angles_deg}")
        self.emit_status()

    def clear_sequence(self):
        with self.sequence_lock: self.sequence.clear()
        self.get_logger().info("Sequence cleared.")
        self.emit_status()

    def delete_waypoint(self, index):
        with self.sequence_lock:
            if 0 <= index < len(self.sequence):
                self.sequence.pop(index)
                self.get_logger().info(f"Waypoint {index} deleted.")
        self.emit_status()

    def play_sequence(self, delay):
        if self.playback_thread and self.playback_thread.is_alive():
            self.stop_playback_flag.set()
            return
        self.stop_playback_flag.clear()
        self.playback_thread = threading.Thread(target=self._play_sequence_thread, args=(delay,), daemon=True)
        self.playback_thread.start()

    def _play_sequence_thread(self, delay):
        self.get_logger().info(f"Starting sequence playback with {delay}s delay.")
        socketio.emit('log_message', {'level': 'info', 'message': f'Playback started ({delay}s delay)...'})
        self.emit_status()
        with self.sequence_lock: waypoints = self.sequence[:] 
        for i, angles in enumerate(waypoints):
            if self.stop_playback_flag.is_set():
                self.get_logger().info("Sequence playback stopped by user.")
                socketio.emit('log_message', {'level': 'info', 'message': 'Playback stopped.'})
                break
            self.send_joint_command_from_gui(angles)
            time.sleep(delay)
        self.stop_playback_flag.clear()
        self.get_logger().info("Sequence playback finished.")
        if not self.stop_playback_flag.is_set():
            socketio.emit('log_message', {'level': 'info', 'message': 'Playback finished.'})
        self.emit_status()

    def emit_status(self):
        with self.state_lock, self.sequence_lock:
            is_playing = self.playback_thread and self.playback_thread.is_alive()
            status = {
                "is_connected": self.is_connected, "emergency_stop_active": self.emergency_stop_active,
                "main_current": self.main_current, "gripper_current": self.gripper_current,
                "joint_angles": self.current_joint_angles_deg, "sequence": self.sequence,
                "is_playing": is_playing,
            }
        socketio.emit('status_update', status)
    
    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """
        Calculates the checksum using the bitwise XOR method to match the firmware.
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    @staticmethod
    def deg_to_rad(deg): return deg * math.pi / 180.0
    @staticmethod
    def rad_to_deg(rad): return rad * 180.0 / math.pi

# ==============================================================================
# 3. FLASK-SOCKETIO WEB APPLICATION
# ==============================================================================

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')
ros_node: RobotArmBridge = None

# The HTML_TEMPLATE, Flask routes, and main execution block are identical to the
# previous version and can be copied from there. They do not need any changes.
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Robot Arm Control</title>
    <style> 
        :root { --bg-color: #1a1a1d; --panel-color: #2c2f33; --text-color: #c3c3c3; --heading-color: #ffffff; --border-color: #4f545c; --input-bg-color: #23272a; --accent-color: #7289da; --accent-hover: #677bc4; --danger-color: #f04747; --danger-hover: #d84040; --success-color: #43b581; --info-color: #3498db; --warn-color: #f1c40f;}
        * { box-sizing: border-box; }
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; display: flex; justify-content: center; align-items: flex-start; background-color: var(--bg-color); color: var(--text-color); margin: 20px; flex-wrap: wrap; }
        .container { display: flex; gap: 20px; flex-wrap: wrap; justify-content: center; width: 100%;}
        .panel { background-color: var(--panel-color); padding: 25px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); min-width: 350px; margin-bottom: 20px; }
        h2 { margin-top: 0; color: var(--heading-color); border-bottom: 1px solid var(--border-color); padding-bottom: 10px; }
        hr { border: none; border-top: 1px solid var(--border-color); margin: 20px 0; }
        .control-group, .status-item { margin-bottom: 20px; }
        label { display: block; margin-bottom: 8px; font-weight: 500; }
        input[type="number"], input[type=file]::file-selector-button { width: 80px; text-align: right; border: 1px solid var(--border-color); background-color: var(--input-bg-color); color: var(--text-color); border-radius: 4px; padding: 8px; }
        .value-display { font-weight: bold; color: var(--accent-color); margin-left: 10px; }
        button, input[type=file]::file-selector-button { background-color: var(--accent-color); color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; font-size: 1em; font-weight: 500; transition: background-color 0.2s; }
        button:hover, input[type=file]::file-selector-button:hover { background-color: var(--accent-hover); }
        .btn-danger { background-color: var(--danger-color) !important; }
        .btn-danger:hover { background-color: var(--danger-hover) !important; }
        .status-value { font-family: 'Courier New', Courier, monospace; background-color: var(--input-bg-color); padding: 5px 8px; border-radius: 4px; border: 1px solid var(--border-color); }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px 20px;}
        .status-box { padding: 15px; border-radius: 5px; text-align: center; font-weight: bold; color: white; }
        .status-ok { background-color: var(--success-color); }
        .status-warn { background-color: var(--danger-color); }
        #sequence-list { list-style-type: decimal; padding-left: 20px; max-height: 250px; overflow-y: auto; background: var(--input-bg-color); border: 1px solid var(--border-color); border-radius: 4px; padding: 10px; padding-left: 30px; }
        #sequence-list li { margin-bottom: 8px; display: flex; justify-content: space-between; align-items: center; }
        .waypoint-angles { font-family: monospace; font-size: 0.9em; color: #a0a0a0; }
        .delete-waypoint { background: var(--danger-color); color: white; border: none; border-radius: 50%; width: 24px; height: 24px; cursor: pointer; font-weight: bold; line-height: 24px; text-align: center; transition: background-color 0.2s; }
        .delete-waypoint:hover { background: var(--danger-hover); }
        .btn-group { display: flex; gap: 10px; flex-wrap: wrap; align-items: center;}
        input[type="range"] { -webkit-appearance: none; width: 100%; height: 8px; background: var(--input-bg-color); border-radius: 5px; outline: none; cursor: pointer; border: 1px solid var(--border-color); }
        input[type="range"]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; background: var(--accent-color); border-radius: 50%; cursor: pointer; transition: background-color 0.2s; }
        input[type="range"]::-webkit-slider-thumb:hover { background: var(--accent-hover); }
        #log-panel { width: 100%; max-width: 1150px; }
        #log-box { background-color: var(--input-bg-color); border: 1px solid var(--border-color); border-radius: 5px; height: 150px; overflow-y: scroll; padding: 10px; font-family: monospace; font-size: 0.9em; color: var(--text-color); }
        #log-box p { margin: 0 0 5px 0; }
        .log-info { color: var(--info-color); }
        .log-warn { color: var(--warn-color); }
        .log-error { color: var(--danger-color); }
    </style>
</head>
<body>
<div class="container">
    <div class="panel">
        <h2>Controls</h2>
        <div id="joint-sliders"></div><hr>
        <div class="control-group">
            <label for="speed-slider">Speed Factor: <span id="speed-value" class="value-display">150</span></label>
            <input type="range" id="speed-slider" min="10" max="500" value="150">
        </div><hr>
        <div class="control-group">
            <label>Gripper Command</label>
            <div class="btn-group">
                <input type="number" id="gripper-angle-input" placeholder="Angle °" value="90" min="0" max="180">
                <input type="number" id="gripper-current-input" placeholder="Current mA" value="200" min="0">
                <button onclick="sendGripperCommand()">Set Gripper</button>
            </div>
        </div><hr>
        <div class="control-group"><button onclick="sendHomeCommand()">Home Robot</button></div>
    </div>
    <div class="panel">
        <h2>Status</h2>
        <div class="status-item">
            <label>Connection Status:</label>
            <div id="connection-status" class="status-box status-warn">DISCONNECTED</div>
        </div>
        <div class="status-item">
            <label>Emergency Stop:</label>
            <div id="estop-status" class="status-box status-warn">INACTIVE</div>
        </div>
        <div class="status-item">
            <label>Currents:</label>
            <span class="status-value" id="main-current">0.00 A</span> | <span class="status-value" id="gripper-current">0.00 mA</span>
        </div>
        <div class="status-item">
            <label>Current Joint Angles (°):</label>
            <div id="joint-status-display" class="status-grid"></div>
        </div>
    </div>
    <div class="panel" id="sequence-panel">
        <h2>Movement Sequence</h2>
        <div class="btn-group control-group">
            <button onclick="recordWaypoint()">Record Waypoint</button>
            <button id="play-sequence-btn" onclick="playSequence()">Play Sequence</button>
            <button onclick="clearSequence()" class="btn-danger">Clear All</button>
        </div>
        <div class="btn-group control-group">
            <button onclick="saveSequence()">Save to File</button>
            <button onclick="loadSequence()">Load from File</button>
        </div>
        <div class="control-group">
            <label for="delay-input">Delay Between Moves (s):</label>
            <input type="number" id="delay-input" value="1.0" step="0.1" min="0.1">
        </div>
        <ol id="sequence-list"></ol>
    </div>
    <div class="panel" id="log-panel">
        <h2>Event Log</h2>
        <div id="log-box"></div>
    </div>
</div>

<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<script>
    const JOINT_NAMES = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Gripper'];
    let jointSliderTimeout = null, speedSliderTimeout = null, activeSlider = null;

    document.addEventListener('DOMContentLoaded', () => {
        const slidersContainer = document.getElementById('joint-sliders');
        const statusContainer = document.getElementById('joint-status-display');
        JOINT_NAMES.forEach((name, i) => {
            slidersContainer.innerHTML += `<div class="control-group"><label for="j${i}">${name}: <span id="j${i}-value" class="value-display">90</span>°</label><input type="range" id="j${i}" min="0" max="180" value="90"></div>`;
            statusContainer.innerHTML += `<div>${name}: <span id="s${i}" class="status-value">0.00</span></div>`;
        });
        JOINT_NAMES.forEach((_, i) => {
            const slider = document.getElementById(`j${i}`);
            slider.addEventListener('input', (e) => {
                document.getElementById(`j${i}-value`).textContent = e.target.value;
                sendJointCommandDebounced();
            });
            slider.addEventListener('mousedown', () => activeSlider = slider.id);
        });
        document.addEventListener('mouseup', () => activeSlider = null);
        document.getElementById('speed-slider').addEventListener('input', e => {
            document.getElementById('speed-value').textContent = e.target.value;
            setSpeedDebounced();
        });
        
        const socket = io();

        socket.on('connect', () => {
            logMessage('info', 'Connected to server!');
            document.getElementById('connection-status').textContent = 'CONNECTED (SERVER)';
            document.getElementById('connection-status').className = 'status-box status-ok';
        });

        socket.on('disconnect', () => {
            logMessage('error', 'Disconnected from server!');
            document.getElementById('connection-status').textContent = 'DISCONNECTED (SERVER)';
            document.getElementById('connection-status').className = 'status-box status-warn';
        });

        socket.on('status_update', (data) => {
            updateStatusUI(data);
        });

        socket.on('log_message', (data) => {
            logMessage(data.level, data.message);
        });
    });

    const logMessage = (level, message) => {
        const logBox = document.getElementById('log-box');
        const timestamp = new Date().toLocaleTimeString();
        logBox.innerHTML += `<p class="log-${level}">[${timestamp}] ${message}</p>`;
        logBox.scrollTop = logBox.scrollHeight;
    };

    const apiPost = (endpoint, body = {}) => fetch(endpoint, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const sendJointCommandDebounced = () => { clearTimeout(jointSliderTimeout); jointSliderTimeout = setTimeout(sendJointCommand, 50); };
    const sendJointCommand = () => { const angles = JOINT_NAMES.map((_, i) => parseFloat(document.getElementById(`j${i}`).value)); apiPost('/api/send_joints', { angles }); };
    const sendHomeCommand = () => apiPost('/api/home');
    const sendGripperCommand = () => { const angle = parseFloat(document.getElementById('gripper-angle-input').value); const current = parseFloat(document.getElementById('gripper-current-input').value); apiPost('/api/send_gripper', { angle, current }); };
    const setSpeedDebounced = () => { clearTimeout(speedSliderTimeout); speedSliderTimeout = setTimeout(setSpeed, 100); };
    const setSpeed = () => { const speed = parseFloat(document.getElementById('speed-slider').value); apiPost('/api/set_speed', { speed }); };
    const recordWaypoint = () => apiPost('/api/record_waypoint');
    const clearSequence = () => apiPost('/api/clear_sequence');
    const deleteWaypoint = index => apiPost('/api/delete_waypoint', { index });
    const playSequence = () => { const delay = parseFloat(document.getElementById('delay-input').value); apiPost('/api/play_sequence', { delay }); };
    const saveSequence = () => apiPost('/api/save_sequence');
    const loadSequence = () => apiPost('/api/load_sequence');

    function updateStatusUI(data) {
        const connStatus = document.getElementById('connection-status');
        connStatus.textContent = data.is_connected ? 'MCU CONNECTED' : 'MCU DISCONNECTED';
        connStatus.className = 'status-box ' + (data.is_connected ? 'status-ok' : 'status-warn');
        
        const estopStatus = document.getElementById('estop-status');
        estopStatus.textContent = data.emergency_stop_active ? 'ACTIVE (TIMEOUT)' : 'INACTIVE';
        estopStatus.className = 'status-box ' + (data.emergency_stop_active ? 'status-warn' : 'status-ok');
        
        document.getElementById('main-current').textContent = `${data.main_current.toFixed(2)} A`;
        document.getElementById('gripper-current').textContent = `${data.gripper_current.toFixed(2)} mA`;
        
        data.joint_angles.forEach((angle, i) => {
            const sliderId = `j${i}`;
            document.getElementById(`s${i}`).textContent = angle.toFixed(2);
            if (activeSlider !== sliderId) {
                document.getElementById(sliderId).value = angle;
                document.getElementById(`${sliderId}-value`).textContent = Math.round(angle);
            }
        });

        const sequenceList = document.getElementById('sequence-list');
        sequenceList.innerHTML = '';
        data.sequence.forEach((waypoint, index) => {
            const li = document.createElement('li');
            const anglesText = waypoint.map(a => a.toFixed(1)).join(', ');
            li.innerHTML = `<span class="waypoint-angles">[${anglesText}]</span><button class="delete-waypoint" onclick="deleteWaypoint(${index})">X</button>`;
            sequenceList.appendChild(li);
        });
        
        const playBtn = document.getElementById('play-sequence-btn');
        const isPlaying = data.is_playing;
        playBtn.textContent = isPlaying ? 'Stop Sequence' : 'Play Sequence';
        playBtn.classList.toggle('btn-danger', isPlaying);
    }
</script>
</body>
</html>
"""

@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)

@app.route('/api/send_joints', methods=['POST'])
def send_joints():
    if ros_node: ros_node.send_joint_command_from_gui(request.json.get('angles'))
    return jsonify({"status": "ok"})

@app.route('/api/home', methods=['POST'])
def home_robot():
    if ros_node: ros_node.send_home_command_from_gui()
    return jsonify({"status": "ok"})

@app.route('/api/send_gripper', methods=['POST'])
def send_gripper():
    data = request.json
    if ros_node: ros_node.send_gripper_command_from_gui(data.get('angle'), data.get('current'))
    return jsonify({"status": "ok"})

@app.route('/api/set_speed', methods=['POST'])
def set_speed():
    if ros_node: ros_node.set_speed_factor_from_gui(request.json.get('speed'))
    return jsonify({"status": "ok"})

@app.route('/api/record_waypoint', methods=['POST'])
def record_waypoint():
    if ros_node: ros_node.record_waypoint()
    return jsonify({"status": "ok"})

@app.route('/api/clear_sequence', methods=['POST'])
def clear_sequence():
    if ros_node: ros_node.clear_sequence()
    return jsonify({"status": "ok"})

@app.route('/api/delete_waypoint', methods=['POST'])
def delete_waypoint():
    index = request.json.get('index')
    if ros_node and index is not None: ros_node.delete_waypoint(index)
    return jsonify({"status": "ok"})

@app.route('/api/play_sequence', methods=['POST'])
def play_sequence():
    if ros_node: ros_node.play_sequence(request.json.get('delay', 1.0))
    return jsonify({"status": "ok"})
    
@app.route('/api/save_sequence', methods=['POST'])
def save_sequence():
    if ros_node: ros_node.save_sequence()
    return jsonify({"status": "ok"})

@app.route('/api/load_sequence', methods=['POST'])
def load_sequence():
    if ros_node: ros_node.load_sequence()
    return jsonify({"status": "ok"})

# ==============================================================================
# 4. MAIN EXECUTION BLOCK
# ==============================================================================
def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = RobotArmBridge()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    print("\n\033[92m============================================================\033[0m")
    print("🤖 \033[1mRobot Arm Bridge and Web GUI are running!\033[0m")
    print("   Open your browser to \033[4mhttp://0.0.0.0:5000\033[0m")
    print("\033[92m============================================================\033[0m\n")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if ros_node:
            if ros_node.ser and ros_node.ser.is_open: ros_node.ser.close()
            ros_node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()