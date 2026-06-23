"""
Flask API routes for robot control.

Provides REST API endpoints for controlling the robot from the web GUI.
"""

from flask import jsonify, request


def register_routes(app, ros_node, cartesian_interface=None, socketio=None, node=None, logger=None):
    """
    Register all API routes with the Flask app.
    
    Args:
        app: Flask application instance
        ros_node: ROS bridge node instance
        cartesian_interface: Cartesian control interface (optional)
        socketio: SocketIO instance (optional)
        node: ROS2 node for timestamps (optional)
        logger: Logger instance (optional)
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
    def jog():
        """Handle jogging requests"""
        logger.info("🔍 /api/jog route called!")
        data = request.json
        logger.info(f"🔍 Received jog data: {data}")
        
        dx = float(data.get('x', 0))
        dy = float(data.get('y', 0))
        dz = float(data.get('z', 0))
        
        logger.info(f'🕹️ Jog request: dx={dx}, dy={dy}, dz={dz}')
        
        if cartesian_interface:
            logger.info("✅ Calling cartesian_interface.jog()")
            success = cartesian_interface.jog(dx, dy, dz, socketio)
            logger.info(f"✅ Jog returned: {success}")
            return jsonify({'success': success})
        
        logger.error("❌ Cartesian interface not available for jog!")
        return jsonify({'success': False, 'error': 'Cartesian interface not available'})
    
    @app.route('/api/move_xyz', methods=['POST'])
    def move_xyz():
        """Move to absolute XYZ position"""
        logger.info("🔍 /api/move_xyz route called!")
        data = request.json
        logger.info(f"🔍 Received data: {data}")
        
        x = float(data['x'])
        y = float(data['y'])
        z = float(data['z'])
        
        logger.info(f'📍 Move to XYZ request: x={x}, y={y}, z={z}')
        
        # Create target pose message
        from geometry_msgs.msg import PoseStamped
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.header.stamp = node.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0  # Identity quaternion
        
        logger.info(f"🔍 cartesian_interface available: {cartesian_interface is not None}")
        
        if cartesian_interface:
            logger.info("✅ Calling cartesian_interface.handle_target_pose()")
            success = cartesian_interface.handle_target_pose(target_pose)
            logger.info(f"✅ handle_target_pose returned: {success}")
            return jsonify({'success': success})
        
        logger.error("❌ Cartesian interface not available!")
        return jsonify({'success': False, 'error': 'Cartesian interface not available'})
    
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


    #  JARVIS AI Routes 
    @app.route('/api/jarvis_command', methods=['POST'])
    def jarvis_command_api():
        """Send a natural language command to the JARVIS LLM pipeline."""
        try:
            data = request.get_json(silent=True) or {}
            command = data.get('command', '').strip()
            if not command:
                return jsonify({'status': 'error', 'message': 'No command provided'}), 400

            if ros_node is None:
                return jsonify({'status': 'error', 'message': 'ROS node not ready'}), 503

            from std_msgs.msg import String as RosString
            msg = RosString()
            msg.data = command

            if not hasattr(ros_node, '_jarvis_cmd_pub'):
                import rclpy
                ros_node._jarvis_cmd_pub = ros_node.create_publisher(
                    RosString, '/jarvis/command', 10)

            ros_node._jarvis_cmd_pub.publish(msg)

            if logger:
                logger.info(f'JARVIS command published: {command!r}')

            return jsonify({'status': 'ok', 'command': command})
        except Exception as e:
            if logger:
                logger.error(f'jarvis_command error: {e}')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    @app.route('/api/jarvis_status', methods=['GET'])
    def jarvis_status_api():
        """Return the current JARVIS LLM status."""
        status = getattr(ros_node, '_jarvis_llm_status', 'unknown') if ros_node else 'offline'
        world  = getattr(ros_node, '_jarvis_world_state', {}) if ros_node else {}
        return jsonify({'status': status, 'world_model': world})

    # ─── URAF / Kinematics API ───────────────────────────────────────────
    from .kinematics_api import _load_kinematics

    @app.route('/api/kinematics')
    def kinematics_api():
        return jsonify(_load_kinematics())

    @app.route('/api/uraf/config')
    def uraf_config_get():
        from ..uraf.config_store import ConfigStore
        from ament_index_python.packages import get_package_share_directory
        import os
        store = ConfigStore()
        fallback = os.path.join(get_package_share_directory('visiona_bridge'), 'config', 'uraf_config.yaml')
        return jsonify(store.load(fallback_path=fallback))

    @app.route('/api/uraf/discovery', methods=['POST'])
    def uraf_discovery_trigger():
        if ros_node is None:
            return jsonify({'status': 'error', 'message': 'ROS not ready'}), 503
        from std_msgs.msg import String as RosString
        if not hasattr(ros_node, '_discovery_trigger_pub'):
            ros_node._discovery_trigger_pub = ros_node.create_publisher(
                RosString, '/uraf/discovery/trigger', 10)
        msg = RosString()
        msg.data = 'run'
        ros_node._discovery_trigger_pub.publish(msg)
        return jsonify({'status': 'ok'})

    @app.route('/api/uraf/health')
    def uraf_health_get():
        health = getattr(ros_node, '_uraf_health', {'status': 'unknown'}) if ros_node else {}
        return jsonify(health)

    @app.route('/api/uraf/generate_urdf', methods=['POST'])
    def uraf_generate_urdf():
        if ros_node is None:
            return jsonify({'status': 'error', 'message': 'ROS not ready'}), 503
        from std_msgs.msg import String as RosString
        if not hasattr(ros_node, '_urdf_generate_pub'):
            ros_node._urdf_generate_pub = ros_node.create_publisher(
                RosString, '/uraf/generate_urdf', 10)
        msg = RosString()
        msg.data = 'generate'
        ros_node._urdf_generate_pub.publish(msg)
        return jsonify({'status': 'ok', 'message': 'URDF generation triggered'})

    @app.route('/api/uraf/learning')
    def uraf_learning_get():
        stats = getattr(ros_node, '_uraf_learning', {}) if ros_node else {}
        return jsonify(stats)

    @app.route('/api/uraf/recovery')
    def uraf_recovery_get():
        recovery = getattr(ros_node, '_uraf_recovery', {}) if ros_node else {}
        return jsonify(recovery)

    @app.route('/api/uraf/twin')
    def uraf_twin_get():
        twin = getattr(ros_node, '_uraf_twin', {}) if ros_node else {}
        return jsonify(twin)

    @app.route('/api/uraf/twin/validate', methods=['POST'])
    def uraf_twin_validate():
        if ros_node is None:
            return jsonify({'status': 'error', 'message': 'ROS not ready'}), 503
        data = request.get_json(force=True, silent=True) or {}
        from std_msgs.msg import String as RosString
        if not hasattr(ros_node, '_twin_validate_pub'):
            ros_node._twin_validate_pub = ros_node.create_publisher(
                RosString, '/visiona/twin/validate_motion', 10)
        import uuid
        payload = {
            'request_id': str(uuid.uuid4()),
            'x': float(data.get('x', 0)),
            'y': float(data.get('y', 0.25)),
            'z': float(data.get('z', 0.35)),
        }
        msg = RosString()
        msg.data = __import__('json').dumps(payload)
        ros_node._twin_validate_pub.publish(msg)
        return jsonify({'status': 'ok', 'request_id': payload['request_id']})

    @app.route('/api/uraf/wizard')
    def uraf_wizard_get():
        from ..uraf.wizard_store import WizardStore
        store = WizardStore()
        return jsonify({'state': store.load()})

    @app.route('/api/uraf/wizard', methods=['POST'])
    def uraf_wizard_post():
        from ..uraf.wizard_store import WizardStore
        data = request.get_json(force=True, silent=True) or {}
        store = WizardStore()
        if 'step' in data and len(data) == 1:
            state = store.advance(int(data['step']))
        else:
            state = store.update(data)
        return jsonify({'status': 'ok', 'state': state})

    @app.route('/api/uraf/wizard/reset', methods=['POST'])
    def uraf_wizard_reset():
        from ..uraf.wizard_store import WizardStore
        store = WizardStore()
        state = store.save(store.default_state())
        return jsonify({'status': 'ok', 'state': state})

    @app.route('/api/uraf/plugins')
    def uraf_plugins_get():
        plugins = getattr(ros_node, '_uraf_plugins', {}) if ros_node else {}
        return jsonify(plugins)

    @app.route('/api/uraf/plugins/install', methods=['POST'])
    def uraf_plugins_install():
        from ..uraf.plugin_registry import PluginRegistry
        from ament_index_python.packages import get_package_share_directory
        import os
        data = request.get_json(force=True, silent=True) or {}
        name = data.get('name', 'visiona_fal')
        share = get_package_share_directory('visiona_bridge')
        manifest = os.path.join(share, 'plugins', name, 'plugin.yaml')
        reg = PluginRegistry()
        try:
            entry = reg.install_from_manifest(__import__('pathlib').Path(manifest))
            return jsonify({'status': 'ok', 'plugin': entry})
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 400

    @app.route('/api/uraf/community')
    def uraf_community_get():
        catalog = getattr(ros_node, '_uraf_community', {}) if ros_node else {}
        return jsonify(catalog)

    @app.route('/api/uraf/community/lookup', methods=['POST'])
    def uraf_community_lookup():
        if ros_node is None:
            return jsonify({'status': 'error', 'message': 'ROS not ready'}), 503
        data = request.get_json(force=True, silent=True) or {}
        from std_msgs.msg import String as RosString
        if not hasattr(ros_node, '_community_lookup_pub'):
            ros_node._community_lookup_pub = ros_node.create_publisher(
                RosString, '/uraf/community/lookup', 10)
        msg = RosString()
        msg.data = __import__('json').dumps(data)
        ros_node._community_lookup_pub.publish(msg)
        return jsonify({'status': 'ok'})

    @app.route('/api/uraf/safety')
    def uraf_safety_get():
        safety = getattr(ros_node, '_uraf_safety', {}) if ros_node else {}
        return jsonify(safety)

    @app.route('/api/uraf/safety/command', methods=['POST'])
    def uraf_safety_command():
        if ros_node is None:
            return jsonify({'status': 'error', 'message': 'ROS not ready'}), 503
        data = request.get_json(force=True, silent=True) or {}
        cmd = data.get('command', 'arm')
        from std_msgs.msg import String as RosString
        if not hasattr(ros_node, '_safety_cmd_pub'):
            ros_node._safety_cmd_pub = ros_node.create_publisher(
                RosString, '/uraf/safety/command', 10)
        msg = RosString()
        msg.data = str(cmd)
        ros_node._safety_cmd_pub.publish(msg)
        return jsonify({'status': 'ok', 'command': cmd})

    #  JARVIS SocketIO handlers 
    if socketio:
        @socketio.on('jarvis_command')
        def handle_jarvis_command(data):
            command = data.get('command', '').strip()
            if not command or ros_node is None:
                return
            from std_msgs.msg import String as RosString
            if not hasattr(ros_node, '_jarvis_cmd_pub'):
                ros_node._jarvis_cmd_pub = ros_node.create_publisher(
                    RosString, '/jarvis/command', 10)
            msg = RosString(); msg.data = command
            ros_node._jarvis_cmd_pub.publish(msg)
            if logger:
                logger.info(f'JARVIS socket command: {command!r}')
