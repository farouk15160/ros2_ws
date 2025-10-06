import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Float32MultiArray, Float32, Bool
import serial
import struct
import time
import threading
import math

from flask import Flask, render_template_string, jsonify, request

# ==============================================================================
# 1. ROS 2 ROBOT ARM BRIDGE
# ==============================================================================

HEADER_BYTE = 0xA5
STATUS_PACKET_FORMAT = '<Bff6fB'
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FORMAT)
STATUS_TIMEOUT_SEC = 2.0 

class RobotArmBridge(Node):
    def __init__(self):
        super().__init__('robot_arm_bridge_web')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.speed_factor = 150.0 
        self.ser = None

        # --- Threading and State Management ---
        self.state_lock = threading.Lock()
        self.sequence_lock = threading.Lock()
        self.connection_thread = threading.Thread(target=self.manage_connection, daemon=True)
        self.playback_thread = None
        self.stop_playback_flag = threading.Event()

        # --- State Variables ---
        self.current_joint_angles_deg = [90.0] * 6 # Initialize to center
        self.main_current = 0.0
        self.gripper_current = 0.0
        self.is_connected = False
        self.emergency_stop_active = False
        self.sequence = []
        
        self.last_status_time = self.get_clock().now()
        self.estop_timer = self.create_timer(1.0, self.check_status_timeout)
        
        self.get_logger().info("Robot Arm Bridge for Web GUI initialized.")
        self.connection_thread.start()

    def manage_connection(self):
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                with self.state_lock:
                    self.is_connected = False
                try:
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                    with self.state_lock:
                        self.is_connected = True
                    self.get_logger().info(f"Successfully connected to {self.serial_port}")
                    self.emergency_stop_active = False
                    time.sleep(0.1)
                    self.ser.reset_input_buffer()
                except serial.SerialException as e:
                    time.sleep(5)
                    continue
            self.read_from_serial()

    def check_status_timeout(self):
        if not self.is_connected: return
        with self.state_lock:
            if not self.emergency_stop_active:
                duration = self.get_clock().now() - self.last_status_time
                if duration.nanoseconds / 1e9 > STATUS_TIMEOUT_SEC:
                    self.get_logger().error("Status timeout! MCU may have stopped or disconnected.")
                    self.emergency_stop_active = True

    def read_from_serial(self):
        read_buffer = bytearray()
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    read_buffer.extend(self.ser.read(self.ser.in_waiting))
                while len(read_buffer) >= STATUS_PACKET_SIZE:
                    header_index = read_buffer.find(HEADER_BYTE)
                    if header_index == -1:
                        read_buffer.clear(); continue
                    if header_index > 0: read_buffer = read_buffer[header_index:]
                    if len(read_buffer) < STATUS_PACKET_SIZE: break

                    packet_data = read_buffer[:STATUS_PACKET_SIZE]
                    received_checksum = packet_data[-1]
                    calculated_checksum = self.calculate_checksum(packet_data[:-1])

                    if received_checksum == calculated_checksum:
                        self.process_status_packet(packet_data)
                        read_buffer = read_buffer[STATUS_PACKET_SIZE:]
                    else:
                        self.get_logger().warn("Checksum mismatch!", throttle_duration_sec=1)
                        read_buffer = read_buffer[1:]
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Serial error: {e}. Closing connection.")
                if self.ser: self.ser.close(); self.ser = None
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read loop: {e}")
            time.sleep(0.001)

    def process_status_packet(self, data):
        with self.state_lock:
            self.last_status_time = self.get_clock().now()
            if self.emergency_stop_active:
                self.get_logger().info("Communication re-established, clearing E-Stop.")
            try:
                _, main, grip, a0, a1, a2, a3, a4, a5, _ = struct.unpack(STATUS_PACKET_FORMAT, data)
                self.main_current, self.gripper_current = main, grip
                self.current_joint_angles_deg = [a0, a1, a2, a3, a4, a5]
                self.emergency_stop_active = False
            except struct.error as e:
                self.get_logger().error(f"Failed to unpack status packet: {e}")

    # --- Command Methods ---
    def send_packet(self, command_id, angles_deg, speed_factor, gripper_current):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial not connected.", throttle_duration_sec=2)
            return
        if len(angles_deg) != 6: return
        try:
            packet_data = struct.pack('<BB6fff', HEADER_BYTE, command_id, *angles_deg, speed_factor, gripper_current)
            checksum = self.calculate_checksum(packet_data)
            self.ser.write(packet_data + struct.pack('<B', checksum))
        except Exception as e:
            self.get_logger().error(f"Error sending packet: {e}")
    
    def send_joint_command_from_gui(self, angles_deg):
        self.send_packet(ord('M'), angles_deg, self.speed_factor, 200.0)

    def send_home_command_from_gui(self):
        self.send_packet(ord('H'), [0.0] * 6, self.speed_factor, 0.0)

    def send_gripper_command_from_gui(self, angle, current):
        current_angles = self.current_joint_angles_deg[:]
        current_angles[5] = angle
        self.send_packet(ord('G'), current_angles, self.speed_factor, current)

    def set_speed_factor_from_gui(self, speed):
        with self.state_lock: self.speed_factor = speed

    # --- Sequence Methods ---
    def record_waypoint(self):
        with self.state_lock, self.sequence_lock:
            self.sequence.append(self.current_joint_angles_deg[:])

    def clear_sequence(self):
        with self.sequence_lock: self.sequence.clear()
        
    def delete_waypoint(self, index):
        with self.sequence_lock:
            if 0 <= index < len(self.sequence):
                self.sequence.pop(index)

    def play_sequence(self, delay):
        if self.playback_thread and self.playback_thread.is_alive():
            self.stop_playback_flag.set()
            return
        self.stop_playback_flag.clear()
        self.playback_thread = threading.Thread(target=self._play_sequence_thread, args=(delay,), daemon=True)
        self.playback_thread.start()

    def _play_sequence_thread(self, delay):
        with self.sequence_lock:
            waypoints = self.sequence[:]
        for angles in waypoints:
            if self.stop_playback_flag.is_set():
                self.get_logger().info("Sequence playback stopped by user.")
                break
            self.send_joint_command_from_gui(angles)
            time.sleep(delay)
        self.stop_playback_flag.clear()

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        return sum(byte for byte in data) & 0xFF

# ==============================================================================
# 2. FLASK WEB APPLICATION
# ==============================================================================

app = Flask(__name__)
ros_node: RobotArmBridge = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Arm Control</title>
    <style>
        :root { --accent-color: #007bff; --accent-hover: #0056b3; --danger-color: #dc3545; --success-color: #28a745; }
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; display: flex; justify-content: center; align-items: flex-start; background-color: #f0f2f5; margin-top: 20px; flex-wrap: wrap; }
        .container { display: flex; gap: 20px; flex-wrap: wrap; justify-content: center; }
        .panel { background-color: white; padding: 25px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); min-width: 350px; margin-bottom: 20px;}
        h2 { margin-top: 0; color: #333; border-bottom: 1px solid #ddd; padding-bottom: 10px; }
        .control-group, .status-item { margin-bottom: 20px; }
        label { display: block; margin-bottom: 5px; font-weight: 500; color: #555; }
        input[type="range"] { width: 100%; cursor: pointer; }
        input[type="number"] { width: 80px; text-align: right; border: 1px solid #ccc; border-radius: 4px; padding: 5px;}
        .value-display { font-weight: bold; color: var(--accent-color); margin-left: 10px; }
        button { background-color: var(--accent-color); color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; font-size: 1em; transition: background-color 0.2s; }
        button:hover { background-color: var(--accent-hover); }
        .status-value { font-family: 'Courier New', Courier, monospace; background-color: #e9ecef; padding: 5px 8px; border-radius: 4px; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px 20px;}
        .status-box { padding: 15px; border-radius: 5px; text-align: center; font-weight: bold; color: white; }
        .status-ok { background-color: var(--success-color); }
        .status-warn { background-color: var(--danger-color); }
        #sequence-list { list-style-type: decimal; padding-left: 20px; max-height: 250px; overflow-y: auto; background: #f8f9fa; border-radius: 4px; padding-top: 10px; padding-bottom: 10px; }
        #sequence-list li { margin-bottom: 8px; display: flex; justify-content: space-between; align-items: center; }
        .waypoint-angles { font-family: monospace; font-size: 0.9em; }
        .delete-waypoint { background: var(--danger-color); color: white; border: none; border-radius: 50%; width: 24px; height: 24px; cursor: pointer; font-weight: bold; line-height: 24px; text-align: center; }
        .btn-group { display: flex; gap: 10px; }
    </style>
</head>
<body>
<div class="container">
    <div class="panel"> <!-- CONTROLS PANEL -->
        <h2>Controls</h2>
        <div id="joint-sliders"></div>
        <hr>
        <div class="control-group">
            <label for="speed-slider">Speed Factor: <span id="speed-value" class="value-display">150</span></label>
            <input type="range" id="speed-slider" min="10" max="500" value="150">
        </div>
        <hr>
        <div class="control-group">
            <label>Gripper Command</label>
            <div class="btn-group">
                <input type="number" id="gripper-angle-input" placeholder="Angle °" value="90">
                <input type="number" id="gripper-current-input" placeholder="Current mA" value="200">
                <button onclick="sendGripperCommand()">Set Gripper</button>
            </div>
        </div>
        <hr>
        <div class="control-group"><button onclick="sendHomeCommand()">Home Robot</button></div>
    </div>
    <div class="panel"> <!-- STATUS PANEL -->
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
            <span class="status-value" id="main-current">0.00 A</span> | <span class="status-value" id="gripper-current">0.00 A</span>
        </div>
        <div class="status-item">
            <label>Current Joint Angles (°):</label>
            <div id="joint-status-display" class="status-grid"></div>
        </div>
    </div>
    <div class="panel" id="sequence-panel"> <!-- SEQUENCE PANEL -->
        <h2>Movement Sequence</h2>
        <div class="btn-group control-group">
            <button onclick="recordWaypoint()">Record Waypoint</button>
            <button id="play-sequence-btn" onclick="playSequence()">Play Sequence</button>
            <button onclick="clearSequence()" style="background:var(--danger-color)">Clear Sequence</button>
        </div>
        <div class="control-group">
            <label for="delay-input">Delay Between Moves (seconds):</label>
            <input type="number" id="delay-input" value="1.0" step="0.1" min="0.1">
        </div>
        <ol id="sequence-list"></ol>
    </div>
</div>
<script>
    const JOINT_NAMES = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Gripper'];
    let jointSliderTimeout = null, speedSliderTimeout = null, activeSlider = null, isPlaying = false;

    document.addEventListener('DOMContentLoaded', () => {
        const slidersContainer = document.getElementById('joint-sliders');
        const statusContainer = document.getElementById('joint-status-display');
        JOINT_NAMES.forEach((name, i) => {
            slidersContainer.innerHTML += `<div class="control-group"><label for="j${i}">${name}: <span id="j${i}-value" class="value-display">90</span>°</label><input type="range" id="j${i}" min="0" max="180" value="90" oninput="updateSliderValue(this.id)"></div>`;
            statusContainer.innerHTML += `<div>${name}: <span id="s${i}" class="status-value">0.00</span></div>`;
        });
        JOINT_NAMES.forEach((_, i) => {
            const slider = document.getElementById(`j${i}`);
            slider.addEventListener('input', sendJointCommandDebounced);
            slider.addEventListener('mousedown', () => activeSlider = slider.id);
        });
        document.addEventListener('mouseup', () => activeSlider = null);
        document.getElementById('speed-slider').addEventListener('input', e => {
            document.getElementById('speed-value').textContent = e.target.value;
            setSpeedDebounced();
        });
        setInterval(updateStatus, 200); // Faster polling
        updateStatus();
    });

    const apiPost = (endpoint, body = {}) => fetch(endpoint, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const updateSliderValue = id => document.getElementById(`${id}-value`).textContent = document.getElementById(id).value;
    const sendJointCommandDebounced = () => { clearTimeout(jointSliderTimeout); jointSliderTimeout = setTimeout(sendJointCommand, 50); };
    const sendJointCommand = () => { const angles = JOINT_NAMES.map((_, i) => parseFloat(document.getElementById(`j${i}`).value)); apiPost('/api/send_joints', { angles }); };
    const sendHomeCommand = () => apiPost('/api/home');
    const sendGripperCommand = () => { const angle = parseFloat(document.getElementById('gripper-angle-input').value); const current = parseFloat(document.getElementById('gripper-current-input').value); apiPost('/api/send_gripper', { angle, current }); };
    const setSpeedDebounced = () => { clearTimeout(speedSliderTimeout); speedSliderTimeout = setTimeout(setSpeed, 100); };
    const setSpeed = () => { const speed = parseFloat(document.getElementById('speed-slider').value); apiPost('/api/set_speed', { speed }); };
    const recordWaypoint = () => apiPost('/api/record_waypoint');
    const clearSequence = () => apiPost('/api/clear_sequence');
    const deleteWaypoint = index => apiPost('/api/delete_waypoint', { index });
    const playSequence = () => { const delay = parseFloat(document.getElementById('delay-input').value); apiPost('/api/play_sequence', { delay }); isPlaying = !isPlaying; updatePlayButton(); };
    const updatePlayButton = () => { const btn = document.getElementById('play-sequence-btn'); btn.textContent = isPlaying ? 'Stop' : 'Play Sequence'; btn.style.background = isPlaying ? 'var(--danger-color)' : 'var(--accent-color)'; };

    async function updateStatus() {
        try {
            const response = await fetch('/api/status');
            const data = await response.json();
            
            document.getElementById('connection-status').textContent = data.is_connected ? 'CONNECTED' : 'DISCONNECTED';
            document.getElementById('connection-status').className = 'status-box ' + (data.is_connected ? 'status-ok' : 'status-warn');
            document.getElementById('estop-status').textContent = data.emergency_stop_active ? 'ACTIVE (TIMEOUT)' : 'INACTIVE';
            document.getElementById('estop-status').className = 'status-box ' + (data.emergency_stop_active ? 'status-warn' : 'status-ok');
            document.getElementById('main-current').textContent = `${data.main_current.toFixed(2)} A`;
            document.getElementById('gripper-current').textContent = `${data.gripper_current.toFixed(2)} A`;
            
            if (data.is_playing === false && isPlaying) { isPlaying = false; updatePlayButton(); }

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
        } catch (error) { console.error('Failed to fetch status:', error); }
    }
</script>
</body>
</html>
"""

# --- Flask API Routes ---
@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)

@app.route('/api/status')
def get_status():
    if not ros_node: return jsonify({"error": "ROS node not initialized"}), 500
    with ros_node.state_lock, ros_node.sequence_lock:
        is_playing = ros_node.playback_thread and ros_node.playback_thread.is_alive()
        status = {
            "is_connected": ros_node.is_connected,
            "emergency_stop_active": ros_node.emergency_stop_active,
            "main_current": ros_node.main_current,
            "gripper_current": ros_node.gripper_current,
            "joint_angles": ros_node.current_joint_angles_deg,
            "sequence": ros_node.sequence,
            "is_playing": is_playing,
        }
    return jsonify(status)

@app.route('/api/send_joints', methods=['POST'])
def send_joints():
    angles_deg = request.json.get('angles')
    if ros_node and angles_deg: ros_node.send_joint_command_from_gui(angles_deg)
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
    speed = request.json.get('speed')
    if ros_node: ros_node.set_speed_factor_from_gui(speed)
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
    delay = request.json.get('delay', 1.0)
    if ros_node: ros_node.play_sequence(delay)
    return jsonify({"status": "ok"})

# ==============================================================================
# 3. MAIN EXECUTION BLOCK
# ==============================================================================
def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = RobotArmBridge()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    print("\nRobot Arm Web GUI is running! Open your browser to http://0.0.0.0:5000\n")
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if ros_node: ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
