#!/usr/bin/env python3
"""
Test script for LLM Control System

Quick script to send test commands to the LLM planner.

Usage:
    # Send a single command
    python3 test_llm.py "Pick up the red cube"
    
    # Interactive mode
    python3 test_llm.py
"""
import rclpy
from std_msgs.msg import String
import sys
import time


def main():
    rclpy.init()
    node = rclpy.create_node('llm_test_client')
    
    # Publisher
    command_pub = node.create_publisher(String, '/llm/command', 10)
    
    # Subscriber for status
    status_messages = []
    def status_callback(msg):
        status_messages.append(msg.data)
        node.get_logger().info(f'Status: {msg.data}')
    
    status_sub = node.create_subscription(String, '/llm/status', status_callback, 10)
    
    # Wait for connections
    time.sleep(0.5)
    
    if len(sys.argv) > 1:
        # Command from arguments
        command = ' '.join(sys.argv[1:])
        node.get_logger().info(f'Sending command: {command}')
        
        msg = String()
        msg.data = command
        command_pub.publish(msg)
        
        # Wait for response
        rclpy.spin_once(node, timeout_sec=10.0)
        
    else:
        # Interactive mode
        print("\n=== LLM Control Test Client ===")
        print("Commands:")
        print("  - Pick up the red cube")
        print("  - Move to home position")
        print("  - Nimm den blauen Block (German)")
        print("\nType your command (or 'quit' to exit):\n")
        
        while True:
            try:
                command = input("> ")
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                    
                if command.strip():
                    msg = String()
                    msg.data = command
                    command_pub.publish(msg)
                    node.get_logger().info(f'Sent: {command}')
                    
                    # Spin to receive status
                    rclpy.spin_once(node, timeout_sec=2.0)
                    
            except KeyboardInterrupt:
                break
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
