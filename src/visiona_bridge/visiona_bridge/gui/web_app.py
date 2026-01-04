"""
Flask web application factory for robot GUI.

Creates and configures the Flask app with SocketIO support.
"""

import os
import logging
from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS
from ament_index_python.packages import get_package_share_directory

from .routes import register_routes

# SocketIO instance (shared across modules)
socketio = SocketIO(
    async_mode='threading',
    ping_timeout=10,
    ping_interval=5,
    cors_allowed_origins="*"
)

# Suppress noisy Flask/SocketIO logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)


def create_app(ros_node):
    """
    Create and configure the Flask application.
    
    Args:
        ros_node: ROS bridge node instance
        
    Returns:
        Tuple of (app, socketio)
    """
    # Get package directories
    package_share_dir = get_package_share_directory('visiona_bridge')
    static_dir = os.path.join(package_share_dir, 'static')
    template_dir = os.path.join(package_share_dir, 'templates')
    
    # Check directories exist
    if not os.path.isdir(static_dir):
        print(f"Warning: Static directory not found at {static_dir}. 3D models may not load.")
    if not os.path.isdir(template_dir):
        print(f"Warning: Templates directory not found at {template_dir}. Creating it.")
        os.makedirs(template_dir, exist_ok=True)
    
    # Create Flask app
    app = Flask(__name__, static_folder=static_dir, template_folder=template_dir)
    CORS(app, resources={r"/*": {"origins": "*"}})
    
    # Attach socketio to ROS node if provided
    if ros_node:
        ros_node.socketio = socketio
    
    # Register Flask routes
    from .routes import register_routes
    register_routes(
        app, 
        ros_node,
        cartesian_interface=ros_node.cartesian if hasattr(ros_node, 'cartesian') else None,
        socketio=socketio,
        node=ros_node,
        logger=ros_node.get_logger()
    )
    
    # Initialize SocketIO with app
    socketio.init_app(app)
    
    return app, socketio