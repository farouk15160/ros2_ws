#!/usr/bin/env python3
import rclpy
import threading
import signal
import os
from ament_index_python.packages import get_package_share_directory

# Import our local package modules
from .bridge_node import RobotArmBridge
from .web_app import create_app, socketio # Import the app factory and socketio instance

# ==============================================================================
# 5. MAIN EXECUTION BLOCK (VERSION 4.0.1)
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    
    # --- Initialize ROS Node ---
    ros_node = RobotArmBridge()
    
    # --- Start ROS Spin Thread ---
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # --- Create Flask App ---
    # Pass the initialized ros_node instance to the app factory
    # This will also attach the socketio instance to the ros_node
    app, _ = create_app(ros_node) 

    # --- Graceful Shutdown Handler ---
    def signal_handler(sig, frame):
        print("\nCtrl+C received, shutting down...")
        if ros_node:
            ros_node.cleanup() # Save data, close serial
        
        # Try to stop socketio server gracefully
        try:
            socketio.stop()
        except Exception:
            pass # Ignore if already stopped
        
        # Request ROS node shutdown
        rclpy.try_shutdown() 

        # Wait briefly for threads
        if ros_thread.is_alive():
            ros_thread.join(timeout=1.0)
        if ros_node and ros_node.connection_thread.is_alive():
            ros_node.connection_thread.join(timeout=1.0)
        
        print("Shutdown complete.")
        os._exit(0) # Force exit if threads hang

    signal.signal(signal.SIGINT, signal_handler)

    # --- Print Startup Message ---
    try:
        package_share_dir = get_package_share_directory('robot_arm_bridge')
        static_dir = os.path.join(package_share_dir, 'static')
    except Exception:
        static_dir = "Unknown (package not found?)"

    print("\n\033[92m============================================================\033[0m")
    print("🤖 \033[1mRobot Arm Bridge V4.0.1 with Web GUI is running!\033[0m")
    print(f"       Serving static files from: \033[33m{static_dir}\033[0m")
    print("       Open your browser to \033[4mhttp://0.0.0.0:5000\033[0m")
    print("       Press \033[1mCtrl+C\033[0m to shut down gracefully.")
    print("\033[92m============================================================\033[0m\n")

    # --- Run Flask-SocketIO Server (blocks main thread) ---
    try:
        # Use the socketio object that was configured by create_app
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True, use_reloader=False, log_output=False)
    except Exception as e:
        print(f"\nError running Flask-SocketIO: {e}")
    finally:
        # This cleanup runs if the server stops for reasons other than SIGINT
        print("Flask server attempting cleanup...")
        if rclpy.ok():
            if ros_node: ros_node.cleanup()
            rclpy.try_shutdown()
        if ros_thread.is_alive():
            ros_thread.join(timeout=0.5)
        print("Flask cleanup finished.")


if __name__ == '__main__':
    main()