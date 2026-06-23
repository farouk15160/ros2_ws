"""
SocketIO event handlers for real-time GUI communication.

Handles emitting robot state updates and log messages to connected clients.
"""


def _socketio_ready(socketio) -> bool:
    """True only after socketio.run() has started the Engine.IO server."""
    return bool(socketio and getattr(socketio, "server", None) is not None)


def safe_emit(socketio, event, data):
    """Emit to GUI clients; no-op before the Flask-SocketIO server is running."""
    if not _socketio_ready(socketio):
        return
    try:
        socketio.emit(event, data)
    except (AttributeError, RuntimeError):
        pass


def emit_status(socketio, state_dict, sequence_data=None, position_names=None):
    """
    Emit robot status to all connected GUI clients.
    
    Args:
        socketio: SocketIO instance
        state_dict: Dictionary containing robot state
        sequence_data: List of sequence points (optional)
        position_names: List of saved position names (optional)
    """
    if not _socketio_ready(socketio):
        return

    try:
        status_data = {
            "is_connected": state_dict.get('is_connected', False),
            "emergency_stop_active": state_dict.get('emergency_stop', False),
            "main_current": state_dict.get('main_current', 0.0),
            "gripper_current": state_dict.get('gripper_current', 0.0),
            "joint_angles": state_dict.get('joint_angles', [0.0]*6),
            "servos_min": state_dict.get('servos_min', [-180.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]),
            "servos_max": state_dict.get('servos_max', [180.0 , 180.0 , 180.0 , 180.0 , 180.0 , 180.0]),
            "collision_threshold": state_dict.get('collision_threshold', 5.0),
            "collision_deviation_threshold": state_dict.get('collision_deviation_threshold', 1.0),
            "fan_speed": state_dict.get('fan_speed', 18),
            "current_speed_factor": state_dict.get('speed_factor', 150.0),
            "is_playing": state_dict.get('is_playing', False),
            "simulation_mode": state_dict.get('simulation_mode', False),
            "collision_detection_enabled": state_dict.get('collision_detection_enabled', False)
        }
        
        if sequence_data is not None:
            status_data["sequence"] = sequence_data
        
        if position_names is not None:
            status_data["saved_positions"] = position_names
        
        safe_emit(socketio, 'status_update', status_data)
    except Exception:
        pass


def emit_log_message(socketio, level: str, message: str):
    """
    Emit a log message to GUI clients.
    
    Args:
        socketio: SocketIO instance
        level: Log level ('info', 'warn', 'error', 'success')
        message: Log message text
    """
    safe_emit(socketio, 'log_message', {'level': level, 'message': message})


def emit_cartesian_pose(socketio, x: float, y: float, z: float):
    """
    Emit current cartesian pose to GUI.
    
    Args:
        socketio: SocketIO instance
        x, y, z: Cartesian coordinates
    """
    safe_emit(socketio, 'cartesian_pose', {
        'x': round(x, 3),
        'y': round(y, 3),
        'z': round(z, 3),
    })


def emit_jarvis_feedback(socketio, message: str):
    safe_emit(socketio, 'jarvis_feedback', {'message': message})


def emit_jarvis_world_state(socketio, state_json: str):
    safe_emit(socketio, 'jarvis_world_state', state_json)


def emit_jarvis_action_plan(socketio, plan_json: str):
    safe_emit(socketio, 'jarvis_action_plan', plan_json)


def emit_llm_status(socketio, status: str):
    safe_emit(socketio, 'llm_status', {'status': status})
