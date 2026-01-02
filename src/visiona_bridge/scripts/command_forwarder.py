#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

class CommandForwarder(Node):
    def __init__(self):
        super().__init__('command_forwarder')
        
        # Subscribe to controller state (which contains the desired/reference position)
        self.sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.state_callback,
            10
        )
        
        # Publish to bridge command topic
        self.pub = self.create_publisher(
            JointState,
            'joint_targets',
            10
        )
        
        self.get_logger().info("Command Forwarder Started: /joint_trajectory_controller/state -> joint_targets")

    def state_callback(self, msg):
        # Create JointState message
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Map joint names and positions from controller reference
        # msg.joint_names and msg.reference.positions
        cmd.name = msg.joint_names
        cmd.position = msg.reference.positions
        
        # Assuming velocity/effort are not critical for the bridge's 'M' packet (which is position based)
        
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CommandForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
