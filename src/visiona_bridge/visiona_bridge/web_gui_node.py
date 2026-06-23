#!/usr/bin/env python3
"""
Main execution entry point for Visiona Bridge with Web GUI.

Launches the ROS2 node and Flask web server.
"""

import rclpy
import threading
import signal
import os
import time
from ament_index_python.packages import get_package_share_directory

# Import from new structure
from .bridge_node import RobotArmBridge
from .gui import create_app, socketio


def main(args=None):
    """Main entry point."""
    import sys
    use_gui = '--no-gui' not in sys.argv
    
    rclpy.init(args=args)
    
    # Initialize ROS Node
    ros_node = RobotArmBridge()
    
    # Start ROS spin thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # Create Flask app if GUI enabled
    app = None
    if use_gui:
        app, _ = create_app(ros_node)
    
    # Graceful shutdown handler
    def signal_handler(sig, frame):
        print("\nCtrl+C received, shutting down...")
        if ros_node:
            ros_node.cleanup()
        
        if use_gui:
            try:
                socketio.stop()
            except Exception:
                pass
        
        rclpy.try_shutdown()
        
        if ros_thread.is_alive():
            ros_thread.join(timeout=1.0)
        
        print("Shutdown complete.")
        os._exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Print startup message
    try:
        package_share_dir = get_package_share_directory('visiona_bridge')
        static_dir = os.path.join(package_share_dir, 'static')
    except Exception:
        static_dir = "Unknown"
    
    print("\n\033[92m============================================================\033[0m")
    if use_gui:
        print("🤖 \033[1mVisiona Robotics Studio v6.0\033[0m")
        print(f"       Static files: \033[33m{static_dir}\033[0m")
        print("       Open browser: \033[4mhttp://0.0.0.0:5000\033[0m")
    else:
        print("🤖 \033[1mRobot Arm Bridge V5.0 (HEADLESS MODE)\033[0m")
        print("       Web GUI is DISABLED.")
    
    print("       Press \033[1mCtrl+C\033[0m to shut down.")
    print("\033[92m============================================================\033[0m\n")
    
    # Run Flask server or wait loop
    try:
        if use_gui:
            socketio.run(app, host='0.0.0.0', port=5000, 
                        allow_unsafe_werkzeug=True, use_reloader=False, log_output=False)
        else:
            while True:
                time.sleep(1.0)
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("Cleanup initiated...")
        if rclpy.ok():
            if ros_node:
                ros_node.cleanup()
            rclpy.try_shutdown()
        if ros_thread.is_alive():
            ros_thread.join(timeout=0.5)
        print("Cleanup finished.")


if __name__ == '__main__':
    main()