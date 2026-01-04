#!/usr/bin/env python3
"""
Test script to try multiple robot positions
Tests both joint control and Cartesian control
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import time

class PositionTester(Node):
    def __init__(self):
        super().__init__('position_tester')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_targets', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visiona/target_pose', 10)
        
        # Wait for publishers to connect
        time.sleep(1.0)
        
        self.get_logger().info('🎯 Position Tester Ready!')
        self.get_logger().info('='*60)
        
    def test_joint_position(self, name, positions):
        """Test a joint position"""
        self.get_logger().info(f'\n📍 Testing Joint Position: {name}')
        self.get_logger().info(f'   Angles: {positions}')
        
        msg = JointState()
        msg.name = [
            'base_link_joint',
            'link_1_shoulder_joint', 
            'link_2_elbow_joint',
            'link_3_wrist_joint',
            'link_3_wrist_to_gripper_base_joint',
            'gripper_joint'
        ]
        msg.position = positions
        
        self.joint_pub.publish(msg)
        self.get_logger().info(f'   ✅ Command sent! Waiting 5 seconds for movement...')
        time.sleep(5.0)
        
    def test_cartesian_position(self, name, x, y, z, qx=0.0, qy=0.707, qz=0.0, qw=0.707):
        """Test a Cartesian position"""
        self.get_logger().info(f'\n🎯 Testing Cartesian Position: {name}')
        self.get_logger().info(f'   Target: x={x}, y={y}, z={z}')
        
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.pose_pub.publish(msg)
        self.get_logger().info(f'   ✅ Command sent! Waiting 7 seconds for planning and execution...')
        time.sleep(7.0)
        
    def run_test_sequence(self):
        """Run comprehensive test sequence"""
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🚀 STARTING COMPREHENSIVE POSITION TEST')
        self.get_logger().info('='*60)
        
        # ========================================
        # TEST 1: Joint Positions (Direct Control)
        # ========================================
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 1: Joint Control (Direct - Always Works)')
        self.get_logger().info('='*60)
        
        # Home position
        self.test_joint_position(
            'Home Position',
            [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]
        )
        
        # Up position
        self.test_joint_position(
            'Up Position',
            [0.0, 1.2, 1.5, 1.57, 1.57, 0.26]
        )
        
        # Side position
        self.test_joint_position(
            'Side Position',
            [1.57, 1.2, 1.5, 1.57, 1.57, 0.26]
        )
        
        # Forward position
        self.test_joint_position(
            'Forward Reach',
            [0.0, 0.8, 1.2, 1.5, 1.57, 0.26]
        )
        
        # Back to home
        self.test_joint_position(
            'Back to Home',
            [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]
        )
        
        # ========================================
        # TEST 2: Cartesian Positions (MoveIt Planning)
        # ========================================
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 2: Cartesian Control (MoveIt - May Fail)')
        self.get_logger().info('='*60)
        
        # First, move to a known good starting pose
        self.get_logger().info('\n📌 Moving to safe starting pose for Cartesian tests...')
        self.test_joint_position(
            'Safe Start Pose',
            [0.0, 1.2, 1.5, 1.57, 1.57, 0.26]
        )
        
        # Test 1: Straight up
        self.test_cartesian_position(
            'Straight Up',
            x=0.0, y=0.0, z=0.5,
            qx=0.0, qy=1.0, qz=0.0, qw=0.0
        )
        
        # Test 2: Forward and up
        self.test_cartesian_position(
            'Forward and Up',
            x=0.3, y=0.0, z=0.4,
            qx=0.0, qy=0.707, qz=0.0, qw=0.707
        )
        
        # Test 3: To the right
        self.test_cartesian_position(
            'To the Right',
            x=0.2, y=0.2, z=0.35,
            qx=0.0, qy=0.707, qz=0.0, qw=0.707
        )
        
        # Test 4: To the left
        self.test_cartesian_position(
            'To the Left',
            x=0.2, y=-0.2, z=0.35,
            qx=0.0, qy=0.707, qz=0.0, qw=0.707
        )
        
        # Test 5: Close and high
        self.test_cartesian_position(
            'Close and High',
            x=0.15, y=0.0, z=0.45,
            qx=0.0, qy=1.0, qz=0.0, qw=0.0
        )
        
        # ========================================
        # Final: Return home
        # ========================================
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🏁 TEST COMPLETE - Returning to Home')
        self.get_logger().info('='*60)
        
        self.test_joint_position(
            'Final Home Position',
            [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]
        )
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('✅ ALL TESTS COMPLETED!')
        self.get_logger().info('='*60)
        self.get_logger().info('\nNOTE: Joint control should work 100%.')
        self.get_logger().info('      Cartesian control depends on MoveIt planning.')
        self.get_logger().info('      Check the cartesian_controller terminal for results!')
        

def main():
    rclpy.init()
    
    tester = PositionTester()
    
    try:
        # Run the test sequence
        tester.run_test_sequence()
        
    except KeyboardInterrupt:
        tester.get_logger().info('\n⚠️  Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
