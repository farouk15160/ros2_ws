import os
import logging
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
from ament_index_python.packages import get_package_share_directory

# Import constants (for logging setup)
from .bridge_constants import *

# This socketio object will be imported by the main launcher
socketio = SocketIO(async_mode='threading', ping_timeout=10, ping_interval=5)

def create_app(ros_node):
    """
    Creates and configures the Flask app, registers routes,
    and initializes SocketIO.
    """
    
    package_share_dir = get_package_share_directory('visiona_bridge')
    static_dir = os.path.join(package_share_dir, 'static')
    template_dir = os.path.join(package_share_dir, 'templates') 
    
    if not os.path.isdir(static_dir):
        print(f"Warning: Static directory not found at {static_dir}. 3D models may not load.")
    if not os.path.isdir(template_dir):
        print(f"Warning: Templates directory not found at {template_dir}. Creating it.")
        os.makedirs(template_dir, exist_ok=True)
    
    app = Flask(__name__, static_folder=static_dir, template_folder=template_dir)
    
    if ros_node:
        ros_node.socketio = socketio

    # ==============================================================================
    # 4. FLASK API ROUTING (VERSION 4.1.0)
    # ==============================================================================
    @app.route('/')
    def index():
        return render_template('index.html')

    # --- New Simulation Route ---
    @app.route('/api/set_simulation_mode', methods=['POST'])
    def set_simulation_mode_api():
        enable = request.json.get('enable')
        if ros_node and enable is not None:
            ros_node.set_simulation_mode(enable)
        return jsonify({"status": "ok"})

    # --- Existing API Routes ---
    @app.route('/api/send_joints', methods=['POST'])
    def send_joints():
        angles = request.json.get('angles')
        if ros_node and angles is not None: ros_node.send_joint_command_from_gui(angles)
        return jsonify({"status": "ok"})
    @app.route('/api/home', methods=['POST'])
    def home_robot():
        if ros_node: ros_node.send_home_command_from_gui()
        return jsonify({"status": "ok"})
    @app.route('/api/send_gripper', methods=['POST'])
    def send_gripper():
        data = request.json
        angle = data.get('angle')
        current = data.get('current')
        if ros_node and angle is not None and current is not None:
            ros_node.send_gripper_command_from_gui(angle, current)
        return jsonify({"status": "ok"})
    @app.route('/api/set_speed', methods=['POST'])
    def set_speed():
        speed = request.json.get('speed')
        if ros_node and speed is not None: ros_node.set_speed_factor_from_gui(speed)
        return jsonify({"status": "ok"})
    @app.route('/api/set_fan_speed', methods=['POST'])
    def set_fan_speed():
        speed = request.json.get('speed')
        if ros_node and speed is not None: ros_node.send_fan_command_from_gui(speed)
        return jsonify({"status": "ok"})
    @app.route('/api/set_min_limits', methods=['POST'])
    def set_min_limits():
        limits = request.json.get('limits')
        if ros_node and limits: ros_node.send_min_limits_from_gui(limits)
        return jsonify({"status": "ok"})
    @app.route('/api/set_max_limits', methods=['POST'])
    def set_max_limits():
        limits = request.json.get('limits')
        if ros_node and limits: ros_node.send_max_limits_from_gui(limits)
        return jsonify({"status": "ok"})
    @app.route('/api/set_threshold', methods=['POST'])
    def set_threshold():
        threshold = request.json.get('threshold')
        if ros_node and threshold is not None: ros_node.send_threshold_from_gui(threshold)
        return jsonify({"status": "ok"})
    @app.route('/api/set_dev_threshold', methods=['POST'])
    def set_dev_threshold():
        threshold = request.json.get('threshold')
        if ros_node and threshold is not None: ros_node.send_dev_threshold_from_gui(threshold)
        return jsonify({"status": "ok"})
    @app.route('/api/save_mcu_config', methods=['POST'])
    def save_mcu_config():
        if ros_node: ros_node.save_config_on_mcu()
        return jsonify({"status": "ok"})
    @app.route('/api/release_estop', methods=['POST'])
    def release_estop():
        if ros_node: ros_node.release_estop_from_gui()
        return jsonify({"status": "ok"})
    @app.route('/api/reconnect', methods=['POST'])
    def reconnect_mcu_endpoint():
        if ros_node: ros_node.trigger_reconnect()
        return jsonify({"status": "ok"})

    # --- Sequence API Routes (Unchanged) ---
    @app.route('/api/add_sequence_point', methods=['POST'])
    def add_sequence_point_api():
        delay = request.json.get('delay_ms', 500)
        if ros_node and delay is not None: ros_node.add_sequence_point(delay)
        return jsonify({"status": "ok"})
    @app.route('/api/clear_sequence', methods=['POST'])
    def clear_sequence_api():
        if ros_node: ros_node.clear_sequence()
        return jsonify({"status": "ok"})
    @app.route('/api/delete_sequence_point', methods=['POST'])
    def delete_sequence_point_api():
        index = request.json.get('index')
        if ros_node and index is not None: ros_node.delete_sequence_point(index)
        return jsonify({"status": "ok"})
    @app.route('/api/save_sequence', methods=['POST'])
    def save_sequence_api():
        filename = request.json.get('filename')
        if ros_node: ros_node.save_sequence_to_file(filename)
        return jsonify({"status": "ok"})
    @app.route('/api/load_sequence', methods=['POST'])
    def load_sequence_api():
        filename = request.json.get('filename')
        if ros_node: ros_node.load_sequence_from_file(filename)
        return jsonify({"status": "ok"})
    @app.route('/api/play_sequence', methods=['POST'])
    def play_sequence_api():
        if ros_node: ros_node.play_sequence()
        return jsonify({"status": "ok"})
    @app.route('/api/stop_sequence', methods=['POST'])
    def stop_sequence_api():
        if ros_node: ros_node.stop_sequence()
        return jsonify({"status": "ok"})

    # --- Saved Position API Routes (Unchanged) ---
    @app.route('/api/save_position', methods=['POST'])
    def save_position_api():
        name = request.json.get('name')
        if ros_node and name: ros_node.save_position(name)
        return jsonify({"status": "ok"})
    @app.route('/api/go_to_position', methods=['POST'])
    def go_to_position_api():
        name = request.json.get('name')
        if ros_node and name: ros_node.go_to_position(name)
        return jsonify({"status": "ok"})
    @app.route('/api/delete_position', methods=['POST'])
    def delete_position_api():
        name = request.json.get('name')
        if ros_node and name: ros_node.delete_position(name)
        return jsonify({"status": "ok"})
    
    socketio.init_app(app)
    
    return app, socketio