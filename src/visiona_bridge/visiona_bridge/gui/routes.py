"""
Flask API routes for robot control.

Provides REST API endpoints for controlling the robot from the web GUI.
"""

from flask import jsonify, request


def register_routes(app, ros_node):
    """
    Register all API routes with the Flask app.
    
    Args:
        app: Flask application instance
        ros_node: ROS bridge node instance
    """
    
    @app.route('/')
    def index():
        from flask import render_template
        return render_template('index.html')
    
    @app.route('/api/set_simulation_mode', methods=['POST'])
    def set_simulation_mode_api():
        enable = request.json.get('enable')
        if ros_node and enable is not None:
            ros_node.set_simulation_mode(enable)
        return jsonify({"status": "ok"})
    
    @app.route('/api/send_joints', methods=['POST'])
    def send_joints():
        angles = request.json.get('angles')
        if ros_node and angles is not None:
            ros_node.send_joint_command(angles)
        return jsonify({"status": "ok"})
    
    @app.route('/api/home', methods=['POST'])
    def home_robot():
        if ros_node:
            ros_node.send_home_command()
        return jsonify({"status": "ok"})
    
    @app.route('/api/send_gripper', methods=['POST'])
    def send_gripper():
        data = request.json
        angle = data.get('angle')
        current = data.get('current')
        if ros_node and angle is not None and current is not None:
            ros_node.send_gripper_command(angle, current)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_speed', methods=['POST'])
    def set_speed():
        speed = request.json.get('speed')
        if ros_node and speed is not None:
            ros_node.set_speed_factor(speed)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_fan_speed', methods=['POST'])
    def set_fan_speed():
        speed = request.json.get('speed')
        if ros_node and speed is not None:
            ros_node.set_fan_speed(speed)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_min_limits', methods=['POST'])
    def set_min_limits():
        limits = request.json.get('limits')
        if ros_node and limits:
            ros_node.set_min_limits(limits)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_max_limits', methods=['POST'])
    def set_max_limits():
        limits = request.json.get('limits')
        if ros_node and limits:
            ros_node.set_max_limits(limits)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_threshold', methods=['POST'])
    def set_threshold():
        threshold = request.json.get('threshold')
        if ros_node and threshold is not None:
            ros_node.set_collision_threshold(threshold)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_dev_threshold', methods=['POST'])
    def set_dev_threshold():
        threshold = request.json.get('threshold')
        if ros_node and threshold is not None:
            ros_node.set_deviation_threshold(threshold)
        return jsonify({"status": "ok"})
    
    @app.route('/api/save_mcu_config', methods=['POST'])
    def save_mcu_config():
        if ros_node:
            ros_node.save_config()
        return jsonify({"status": "ok"})
    
    @app.route('/api/release_estop', methods=['POST'])
    def release_estop():
        if ros_node:
            ros_node.release_estop()
        return jsonify({"status": "ok"})
    
    @app.route('/api/reconnect', methods=['POST'])
    def reconnect_mcu_endpoint():
        if ros_node:
            ros_node.trigger_reconnect()
        return jsonify({"status": "ok"})
    
    @app.route('/api/kill_motors', methods=['POST'])
    def kill_motors():
        if ros_node:
            ros_node.kill_motors()
        return jsonify({"status": "ok"})
    
    @app.route('/api/jog', methods=['POST'])
    def jog_cartesian_api():
        data = request.json
        dx = data.get('x', 0.0)
        dy = data.get('y', 0.0)
        dz = data.get('z', 0.0)
        if ros_node:
            ros_node.jog_cartesian(dx, dy, dz)
        return jsonify({"status": "ok"})
    
    @app.route('/api/set_collision_safety', methods=['POST'])
    def set_collision_safety():
        enable = request.json.get('enable')
        if ros_node and enable is not None:
            ros_node.set_collision_safety(enable)
        return jsonify({"status": "ok"})
    
    # Sequence API Routes
    @app.route('/api/add_sequence_point', methods=['POST'])
    def add_sequence_point_api():
        delay = request.json.get('delay_ms', 500)
        if ros_node and delay is not None:
            ros_node.add_sequence_point(delay)
        return jsonify({"status": "ok"})
    
    @app.route('/api/clear_sequence', methods=['POST'])
    def clear_sequence_api():
        if ros_node:
            ros_node.clear_sequence()
        return jsonify({"status": "ok"})
    
    @app.route('/api/delete_sequence_point', methods=['POST'])
    def delete_sequence_point_api():
        index = request.json.get('index')
        if ros_node and index is not None:
            ros_node.delete_sequence_point(index)
        return jsonify({"status": "ok"})
    
    @app.route('/api/save_sequence', methods=['POST'])
    def save_sequence_api():
        filename = request.json.get('filename')
        if ros_node:
            ros_node.save_sequence(filename)
        return jsonify({"status": "ok"})
    
    @app.route('/api/load_sequence', methods=['POST'])
    def load_sequence_api():
        filename = request.json.get('filename')
        if ros_node:
            ros_node.load_sequence(filename)
        return jsonify({"status": "ok"})
    
    @app.route('/api/play_sequence', methods=['POST'])
    def play_sequence_api():
        if ros_node:
            ros_node.play_sequence()
        return jsonify({"status": "ok"})
    
    @app.route('/api/stop_sequence', methods=['POST'])
    def stop_sequence_api():
        if ros_node:
            ros_node.stop_sequence()
        return jsonify({"status": "ok"})
    
    # Saved Position API Routes
    @app.route('/api/save_position', methods=['POST'])
    def save_position_api():
        name = request.json.get('name')
        if ros_node and name:
            ros_node.save_position(name)
        return jsonify({"status": "ok"})
    
    @app.route('/api/go_to_position', methods=['POST'])
    def go_to_position_api():
        name = request.json.get('name')
        if ros_node and name:
            ros_node.go_to_position(name)
        return jsonify({"status": "ok"})
    
    @app.route('/api/delete_position', methods=['POST'])
    def delete_position_api():
        name = request.json.get('name')
        if ros_node and name:
            ros_node.delete_position(name)
        return jsonify({"status": "ok"})
