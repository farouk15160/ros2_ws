#!/usr/bin/env python3
"""
Simple Inverse Kinematics Solver for Visiona Robot

Provides fast, direct XYZ to joint angle conversion without MoveIt overhead.
Uses numerical IK with Jacobian pseudoinverse for smooth Cartesian control.

Author: Robot Control System
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np
import time


class SimpleIKSolver(Node):
    """
    Simple IK solver using numerical methods (Jacobian pseudoinverse).
    
    Subscribes to Cartesian commands and publishes joint targets.
    Includes trajectory interpolation for smooth motion.
    """
    
    def __init__(self):
        super().__init__('simple_ik_solver')
        
        # Parameters
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('tolerance', 0.001)  # 1mm
        self.declare_parameter('step_size', 0.01)  # 1cm per waypoint
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('max_speed', 0.5)  # m/s
        
        self.max_iter = self.get_parameter('max_iterations').value
        self.tolerance = self.get_parameter('tolerance').value
        self.step_size = self.get_parameter('step_size').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        
        
        # Robot DH parameters (extracted from URDF visiona.urdf.xacro)
        # Format: [a, alpha, d, theta_offset]
        # Based on URDF joint origins and orientations
        # URDF shows: base(z=-0.001), shoulder(z=0.14), elbow(z=0.185), wrist(y=-0.119)
        self.dh_params = [
            [0.0,     np.pi/2,   0.14,   0.0],       # Joint 1
            [0.185,   0.0,       0.0,    0.0],       # Joint 2: Option 3 worked!
            [0.119,   0.0,       0.0,    0.0],       # Joint 3
            [0.12,    0.0,       0.0,    0.0],       # Joint 4: Increased to reach gripper tip
        ]
        
        # No separate gripper offset needed (included in Joint 4)
        self.gripper_offset = 0.0
        
        # Current joint state
        self.current_joints = np.array([0.0, 1.57, 1.57, 1.57, 0.0, 0.26])
        self.joints_updated = False
        
        # Publishers and Subscribers
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_targets',
            10
        )
        
        # Publisher for target marker (RViz visualization)
        self.marker_pub = self.create_publisher(
            Marker,
            '/visiona/target_marker',
            10
        )
        
        # Publisher for current EE position marker (blue sphere)
        self.current_ee_marker_pub = self.create_publisher(
            Marker,
            '/visiona/current_ee_marker',
            10
        )
        
        # Publisher for current EE pose (GUI display)
        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            '/visiona/current_pose',
            10
        )
        
        self.cartesian_sub = self.create_subscription(
            PoseStamped,
            '/visiona/cartesian_command',
            self.cartesian_callback,
            10
        )
        
        # QoS profile for joint_states (match default sensor QoS)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.get_logger().info('✅ Simple IK Solver Started')
        self.get_logger().info(f'   Max iterations: {self.max_iter}')
        self.get_logger().info(f'   Tolerance: {self.tolerance}m')
        self.get_logger().info(f'   Step size: {self.step_size}m')
        self.get_logger().info(f'   Control rate: {self.control_rate}Hz')
        
    def joint_state_callback(self, msg):
        """Update current joint positions from joint_states topic"""
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6])
            self.joints_updated = True
            
            # Publish current EE pose for GUI display
            current_xyz = self.forward_kinematics(self.current_joints)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = float(current_xyz[0])
            pose_msg.pose.position.y = float(current_xyz[1])
            pose_msg.pose.position.z = float(current_xyz[2])
            pose_msg.pose.orientation.w = 1.0
            self.current_pose_pub.publish(pose_msg)
            
            # Publish current EE position marker (blue sphere) for RViz
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'current_ee'
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(current_xyz[0])
            marker.pose.position.y = float(current_xyz[1])
            marker.pose.position.z = float(current_xyz[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.015  # 1.5cm (smaller than target)
            marker.scale.y = 0.015
            marker.scale.z = 0.015
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0  # Blue
            marker.color.a = 0.8  # Semi-transparent
            self.current_ee_marker_pub.publish(marker)
    
    def dh_transform(self, a, alpha, d, theta):
        """Compute DH transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct,    -st*ca,  st*sa,   a*ct],
            [st,     ct*ca, -ct*sa,   a*st],
            [0,      sa,     ca,      d   ],
            [0,      0,      0,       1   ]
        ])
    
    def forward_kinematics(self, joints):
        """
        Compute end-effector XYZ position from joint angles.
        
        Args:
            joints: Array of joint angles [6 elements]
        
        Returns:
            xyz: End-effector position [x, y, z]
        """
        T = np.eye(4)
        
        # Apply transformations for the 4-DOF arm (first 4 joints)
        for i in range(4):
            a, alpha, d, offset = self.dh_params[i]
            theta = joints[i] + offset
            T = T @ self.dh_transform(a, alpha, d, theta)
        
        # Extract position (gripper offset already in DH params)
        return T[:3, 3]
    
    def compute_jacobian(self, joints, epsilon=1e-6):
        """
        Compute numerical Jacobian matrix.
        
        Args:
            joints: Current joint angles
            epsilon: Small perturbation for numerical derivative
        
        Returns:
            J: 3x4 Jacobian matrix (xyz vs first 4 joints)
        """
        J = np.zeros((3, 4))
        pos0 = self.forward_kinematics(joints)
        
        for i in range(4):
            joints_pert = joints.copy()
            joints_pert[i] += epsilon
            pos_pert = self.forward_kinematics(joints_pert)
            J[:, i] = (pos_pert - pos0) / epsilon
        
        return J
    
    def compute_ik(self, target_xyz, initial_joints):
        """
        Compute inverse kinematics using Jacobian pseudoinverse.
        
        Args:
            target_xyz: Target end-effector position [x, y, z]
            initial_joints: Starting joint configuration
        
        Returns:
            joints: Solution joint angles (or None if failed)
        """
        joints = initial_joints.copy()
        alpha = 0.5  # Step size factor
        
        for iteration in range(self.max_iter):
            # Current position
            current_xyz = self.forward_kinematics(joints)
            
            # Error
            error = target_xyz - current_xyz
            error_norm = np.linalg.norm(error)
            
            # Check convergence
            if error_norm < self.tolerance:
                self.get_logger().debug(f'IK converged in {iteration} iterations')
                return joints
            
            # Compute Jacobian
            J = self.compute_jacobian(joints)
            
            # Pseudoinverse
            J_pinv = np.linalg.pinv(J)
            
            # Update joints (only first 4 DOF)
            delta_joints = alpha * (J_pinv @ error)
            joints[:4] += delta_joints
            
            # Joint limits (safety)
            joints = np.clip(joints, -np.pi, np.pi)
        
        self.get_logger().warn(f'IK did not converge after {self.max_iter} iterations')
        return None
    
    def interpolate_trajectory(self, start_xyz, end_xyz):
        """
        Generate smooth trajectory with intermediate waypoints.
        
        Args:
            start_xyz: Starting position
            end_xyz: Target position
        
        Returns:
            waypoints: List of intermediate XYZ positions
        """
        distance = np.linalg.norm(end_xyz - start_xyz)
        num_steps = max(int(distance / self.step_size), 1)
        
        waypoints = []
        for i in range(num_steps + 1):
            alpha = i / num_steps
            waypoint = start_xyz + alpha * (end_xyz - start_xyz)
            waypoints.append(waypoint)
        
        return waypoints
    
    def publish_joint_target(self, joints):
        """Publish joint target to /joint_targets topic"""
        msg = JointState()
        msg.name = [
            'base_link_joint',
            'link_1_shoulder_joint',
            'link_2_elbow_joint',
            'link_3_wrist_joint',
            'link_3_wrist_to_gripper_base_joint',
            'gripper_joint'
        ]
        msg.position = joints.tolist()
        self.joint_pub.publish(msg)
    
    def publish_target_marker(self, target_xyz):
        """Publish visualization marker for target position in RViz"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_position'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(target_xyz[0])
        marker.pose.position.y = float(target_xyz[1])
        marker.pose.position.z = float(target_xyz[2])
        marker.pose.orientation.w = 1.0
        
        # Scale (2cm sphere)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        # Color (bright green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 5  # Display for 5 seconds
        
        self.marker_pub.publish(marker)
    
    def cartesian_callback(self, msg: PoseStamped):
        """
        Handle incoming Cartesian position command.
        
        Generates smooth trajectory and executes motion.
        """
        # Wait for joint state
        if not self.joints_updated:
            self.get_logger().warn('Waiting for joint states...')
            return
        
        # Extract target position
        target_xyz = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        self.get_logger().info(f'🎯 Cartesian command: x={target_xyz[0]:.3f}, '
                              f'y={target_xyz[1]:.3f}, z={target_xyz[2]:.3f}')
        
        # Publish marker for RViz visualization
        self.publish_target_marker(target_xyz)
        
        # Get current EE position
        current_xyz = self.forward_kinematics(self.current_joints)
        
        # Generate waypoints
        waypoints = self.interpolate_trajectory(current_xyz, target_xyz)
        self.get_logger().info(f'   Generated {len(waypoints)} waypoints')
        
        # Execute trajectory
        sleep_time = 1.0 / self.control_rate
        current_joints = self.current_joints.copy()
        
        for i, waypoint in enumerate(waypoints):
            # Compute IK for this waypoint
            solution = self.compute_ik(waypoint, current_joints)
            
            if solution is None:
                self.get_logger().error(f'❌ IK failed at waypoint {i}/{len(waypoints)}')
                return
            
            # Publish joint target
            self.publish_joint_target(solution)
            current_joints = solution
            
            # Rate limiting
            time.sleep(sleep_time)
        
        self.get_logger().info('✅ Cartesian motion complete')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleIKSolver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Simple IK Solver...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
