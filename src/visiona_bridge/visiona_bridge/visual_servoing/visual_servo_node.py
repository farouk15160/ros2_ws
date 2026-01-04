#!/usr/bin/env python3
"""
Visual Servoing Controller Node (Stub Implementation)

Provides closed-loop visual control.
This is a placeholder that will be fully implemented in Week 4.

Author: Antigravity AI
Date: 2026-01-04
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class VisualServoController(Node):
    """Stub visual servoing controller"""
    
    def __init__(self):
        super().__init__('visual_servo_controller')
        
        self.get_logger().info('Visual Servo Controller (STUB) - Starting...')
        self.get_logger().warn(
            'This is a stub implementation. '
            'Full visual servoing will be implemented in Week 4.'
        )
        
        # Subscribe to goal poses
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/visual_servo/goal',
            self.goal_callback,
            10
        )
        
        # Publish status
        self.status_pub = self.create_publisher(
            String,
            '/visual_servo/status',
            10
        )
        
        self.get_logger().info('Visual Servo Controller ready (stub mode)')
        
    def goal_callback(self, msg):
        """Handle incoming goal poses"""
        self.get_logger().info(
            f'[STUB] Received goal: '
            f'({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, '
            f'{msg.pose.position.z:.3f})'
        )
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Goal received (stub mode)'
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
