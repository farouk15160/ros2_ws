#!/usr/bin/env python3
"""
Simple Inverse Kinematics Solver for Visiona Robot

Provides fast, direct XYZ to joint conversion using Damped Least Squares (DLS).
Interpolates paths in Joint Space using a Minimum Jerk trajectory for smooth motion.
Collision checking verifies all arm links against the Octomap point cloud.

Author: Robot Control System
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap
import numpy as np
import struct

class SimpleIKSolver(Node):
    def __init__(self):
        super().__init__('simple_ik_solver')
        
        # --- IK Solver Parameters (overridable from kinematic_params.yaml) ---
        self.declare_parameter('ik_solver.max_iterations', 200)
        self.declare_parameter('ik_solver.tolerance', 0.001)  # 1mm
        self.declare_parameter('ik_solver.damping', 0.05)
        self.declare_parameter('ik_solver.step_alpha', 0.5)
        self.declare_parameter('trajectory.control_rate', 50.0)
        self.declare_parameter('trajectory.max_joint_speed', 1.0)
        
        # Legacy flat-name params (for backward compat with old configs)
        self.declare_parameter('max_iterations', 200)
        self.declare_parameter('tolerance', 0.001)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_joint_speed', 1.0)
        
        self.max_iter = (self.get_parameter('ik_solver.max_iterations').value
                         or self.get_parameter('max_iterations').value)
        self.tolerance = (self.get_parameter('ik_solver.tolerance').value
                          or self.get_parameter('tolerance').value)
        self.ik_damping = (self.get_parameter('ik_solver.damping').value or 0.05)
        self.ik_alpha = (self.get_parameter('ik_solver.step_alpha').value or 0.5)
        self.control_rate = (self.get_parameter('trajectory.control_rate').value
                             or self.get_parameter('control_rate').value)
        self.max_joint_speed = (self.get_parameter('trajectory.max_joint_speed').value
                                or self.get_parameter('max_joint_speed').value)
        
        # --- DH Parameters (loaded from kinematic_params.yaml or fallback) ---
        # Declare per-joint DH params so they can be overridden from YAML
        dh_defaults = [
            [0.0,   np.pi/2, 0.14,   0.0],   # Joint 0: base → shoulder
            [0.185, 0.0,     0.0,    0.0],   # Joint 1: shoulder → elbow
            [0.119, 0.0,     0.0,    0.0],   # Joint 2: elbow → wrist
            [0.25,  0.0,     -0.005, 0.0],   # Joint 3: wrist → gripper (incl. gripper)
        ]
        self.dh_params = []
        for i, (a, alpha, d, off) in enumerate(dh_defaults):
            self.declare_parameter(f'dh_params.joint_{i}.a', a)
            self.declare_parameter(f'dh_params.joint_{i}.alpha', alpha)
            self.declare_parameter(f'dh_params.joint_{i}.d', d)
            self.declare_parameter(f'dh_params.joint_{i}.theta_offset', off)
            self.dh_params.append([
                self.get_parameter(f'dh_params.joint_{i}.a').value,
                self.get_parameter(f'dh_params.joint_{i}.alpha').value,
                self.get_parameter(f'dh_params.joint_{i}.d').value,
                self.get_parameter(f'dh_params.joint_{i}.theta_offset').value,
            ])
        
        self.get_logger().info(f'DH params loaded: {self.dh_params}')
        
        self.current_joints = np.array([0.0, 1.57, 1.57, 1.57, 0.0, 0.26])
        self.joints_updated = False
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_targets', 10)
        self.marker_pub = self.create_publisher(Marker, '/visiona/target_marker', 10)
        self.current_ee_marker_pub = self.create_publisher(Marker, '/visiona/current_ee_marker', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/visiona/current_pose', 10)
        
        # Subscribers
        self.cartesian_sub = self.create_subscription(PoseStamped, '/visiona/cartesian_command', self.cartesian_callback, 10)
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos_profile)
        
        # Obstacle Avoidance (overridable from kinematic_params.yaml)
        self.declare_parameter('collision.enable_obstacle_avoidance', True)
        self.declare_parameter('collision.obstacle_safety_margin', 0.10)
        self.declare_parameter('collision.self_collision_min_dist', 0.05)
        self.declare_parameter('collision.ground_clearance', -0.02)
        self.declare_parameter('collision.max_elbow_sum', 5.5)
        # Legacy flat names
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('obstacle_safety_margin', 0.10)
        
        self.enable_obstacle_avoidance = (
            self.get_parameter('collision.enable_obstacle_avoidance').value
            and self.get_parameter('enable_obstacle_avoidance').value)
        self.obstacle_safety_margin = (
            self.get_parameter('collision.obstacle_safety_margin').value or
            self.get_parameter('obstacle_safety_margin').value)
        self.self_collision_min_dist = self.get_parameter('collision.self_collision_min_dist').value
        self.ground_clearance = self.get_parameter('collision.ground_clearance').value
        self.max_elbow_sum = self.get_parameter('collision.max_elbow_sum').value
        
        self.occupied_voxels = set()
        self.octomap_resolution = 0.02
        self.octomap_received = False
        self.octomap_sub = self.create_subscription(Octomap, '/octomap_binary', self.octomap_callback, 10)
        
        from sensor_msgs.msg import PointCloud2
        self.obstacle_pc_sub = self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.obstacle_pointcloud_callback, 10)
        
        # Non-blocking Execution Timer
        self.trajectory_points = []
        self.trajectory_idx = 0
        self.is_moving = False
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_timer_callback)
        
        self.get_logger().info('✅ Simple IK Solver Started (Smooth Joint-Space DLS Version)')
        
    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6])
            self.joints_updated = True
            
            current_xyz = self.forward_kinematics(self.current_joints)
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = float(current_xyz[0])
            pose_msg.pose.position.y = float(current_xyz[1])
            pose_msg.pose.position.z = float(current_xyz[2])
            pose_msg.pose.orientation.w = 1.0
            self.current_pose_pub.publish(pose_msg)
            
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
            marker.scale.x = marker.scale.y = marker.scale.z = 0.025
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.5, 1.0, 1.0
            self.current_ee_marker_pub.publish(marker)

    def octomap_callback(self, msg: Octomap):
        self.octomap_resolution = msg.resolution
        self.octomap_received = True
        
    def obstacle_pointcloud_callback(self, msg):
        try:
            fields = {f.name: f for f in msg.fields}
            if 'x' not in fields or 'y' not in fields or 'z' not in fields: return
            x_offset, y_offset, z_offset = fields['x'].offset, fields['y'].offset, fields['z'].offset
            point_step, data = msg.point_step, msg.data
            
            new_voxels = set()
            sample_step = max(1, len(data) // (point_step * 2000))
            
            for i in range(0, len(data) - point_step, point_step * sample_step):
                x = struct.unpack_from('f', data, i + x_offset)[0]
                y = struct.unpack_from('f', data, i + y_offset)[0]
                z = struct.unpack_from('f', data, i + z_offset)[0]
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    key = (round(x/self.octomap_resolution)*self.octomap_resolution,
                           round(y/self.octomap_resolution)*self.octomap_resolution,
                           round(z/self.octomap_resolution)*self.octomap_resolution)
                    new_voxels.add(key)
            self.occupied_voxels = new_voxels
            self.octomap_received = True
        except Exception as e:
            self.get_logger().debug(f'Obstacle pointcloud error: {e}')

    def dh_transform(self, a, alpha, d, theta):
        ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct],
                         [st,  ct*ca,-ct*sa, a*st],
                         [0,   sa,    ca,    d   ],
                         [0,   0,     0,     1   ]])
                         
    def forward_kinematics(self, joints):
        T = np.eye(4)
        for i in range(4):
            a, alpha, d, offset = self.dh_params[i]
            T = T @ self.dh_transform(a, alpha, d, joints[i] + offset)
        return T[:3, 3]

    def compute_link_positions(self, joints):
        positions = [np.array([0.0, 0.0, 0.0])]
        T = np.eye(4)
        for i in range(4):
            a, alpha, d, offset = self.dh_params[i]
            T = T @ self.dh_transform(a, alpha, d, joints[i] + offset)
            positions.append(T[:3, 3].copy())
        return positions

    def compute_jacobian(self, joints, epsilon=1e-5):
        """ Numeric Jacobian """
        J = np.zeros((3, 4))
        pos0 = self.forward_kinematics(joints)
        for i in range(4):
            j_pert = joints.copy()
            j_pert[i] += epsilon
            pos_pert = self.forward_kinematics(j_pert)
            J[:, i] = (pos_pert - pos0) / epsilon
        return J

    def check_workspace_limits(self, xyz):
        """Verify target is within reachable workspace."""
        x, y, z = xyz
        max_reach = 0.69
        dist = np.sqrt(x**2 + y**2 + z**2)
        if dist < 0.12:
            self.get_logger().warn(f'Target too close to base: {dist:.3f}m < 0.12m')
            return False
        if not (-max_reach <= x <= max_reach):
            self.get_logger().warn(f'X={x:.3f} outside workspace [{-max_reach}, {max_reach}]')
            return False
        if not (-max_reach <= y <= max_reach):
            self.get_logger().warn(f'Y={y:.3f} outside workspace [{-max_reach}, {max_reach}]')
            return False
        if not (0.0 <= z <= max_reach):
            self.get_logger().warn(f'Z={z:.3f} outside workspace [0.0, {max_reach}]')
            return False
        return True

    def check_joint_limits(self, joints):
        """Verify all joints are within mechanical limits."""
        limits = [(-np.pi, np.pi), (0.0, np.pi), (0.0, np.pi), (0.0, np.pi)]
        for i in range(4):
            if not (limits[i][0] <= joints[i] <= limits[i][1]):
                self.get_logger().debug(f'Joint {i} = {np.rad2deg(joints[i]):.1f}° out of limits')
                return False
        return True

    def check_simple_collision(self, joints):
        """Check for self-collision using link capsule distances and arm geometry."""
        pos = self.compute_link_positions(joints)
        ee_pos = pos[-1]
        
        # Check 1: End-effector too close to base (self-collision)
        if np.linalg.norm(ee_pos - pos[0]) < self.self_collision_min_dist:
            self.get_logger().debug('Self-collision: EE too close to base')
            return False
        
        # Check 2: Non-adjacent link proximity (wrist close to shoulder)
        if len(pos) >= 4 and np.linalg.norm(pos[3] - pos[1]) < self.self_collision_min_dist:
            self.get_logger().debug('Self-collision: wrist too close to shoulder')
            return False
        
        # Check 3: All non-adjacent link pairs (capsule check)
        for i in range(len(pos)):
            for j in range(i + 2, len(pos)):  # skip adjacent links
                dist = np.linalg.norm(pos[j] - pos[i])
                if dist < self.self_collision_min_dist:
                    self.get_logger().debug(f'Self-collision: link {i} ↔ link {j} dist={dist:.3f}m')
                    return False
        
        # Check 4: Arm folding back on itself
        if (joints[1] + joints[2]) > self.max_elbow_sum:
            self.get_logger().debug(f'Arm fold-back: elbow sum {np.rad2deg(joints[1]+joints[2]):.0f}°')
            return False
        
        # Check 5: Ground collision
        if ee_pos[2] < self.ground_clearance:
            self.get_logger().debug(f'Ground collision: EE z={ee_pos[2]:.3f}m')
            return False
        
        return True

    def check_octomap_collision_all_links(self, joints):
        """ Check all segments of the arm against the octomap point cloud """
        if not self.enable_obstacle_avoidance or not self.octomap_received:
            return True, "Path clear"
            
        link_pos = self.compute_link_positions(joints)
        margin = self.obstacle_safety_margin
        
        points_to_check = []
        for i in range(len(link_pos) - 1):
            p1, p2 = link_pos[i], link_pos[i+1]
            dist = np.linalg.norm(p2 - p1)
            num_samples = max(2, int(dist / 0.03)) # sample every 3cm
            for j in range(num_samples + 1):
                points_to_check.append(p1 + (j/max(1,num_samples))*(p2 - p1))
                
        # Fast distance check
        for pt in points_to_check:
            for vx, vy, vz in self.occupied_voxels:
                if (abs(pt[0]-vx) < margin and abs(pt[1]-vy) < margin and abs(pt[2]-vz) < margin):
                    return False, f"Collision near ({vx:.2f}, {vy:.2f}, {vz:.2f})"
        return True, "Path clear"

    def compute_ik(self, target_xyz, initial_joints):
        """
        IK solver using Damped Least Squares (Levenberg-Marquardt).
        
        Algorithm:
          1. Compute FK for current joint guess
          2. Compute Cartesian error e = target - current
          3. Compute Jacobian J (3×4 numeric)
          4. DLS update: dq = J^T (J J^T + λ²I)^{-1} e
          5. Clip joints to limits, repeat
        
        Returns: joint array or None if no solution found.
        """
        if not self.check_workspace_limits(target_xyz):
            return None
        
        joints = initial_joints.copy()
        alpha = self.ik_alpha
        damping = self.ik_damping
        
        for iteration in range(self.max_iter):
            current_xyz = self.forward_kinematics(joints)
            error = target_xyz - current_xyz
            err_norm = np.linalg.norm(error)
            
            if err_norm < self.tolerance:
                if not self.check_joint_limits(joints):
                    self.get_logger().debug(f'IK converged but joints out of limits')
                    return None
                if not self.check_simple_collision(joints):
                    self.get_logger().debug(f'IK converged but collision detected')
                    return None
                self.get_logger().debug(f'IK solved in {iteration} iterations, error={err_norm:.4f}m')
                return joints
                
            J = self.compute_jacobian(joints)
            # Damped Least Squares: dq = J^T (J J^T + λ²I)^{-1} e
            J_dls = J.T @ np.linalg.inv(J @ J.T + (damping**2) * np.eye(3))
            
            joints[:4] += alpha * (J_dls @ error)
            joints[0] = np.clip(joints[0], -2*np.pi, 2*np.pi)
            joints[1:4] = np.clip(joints[1:4], 0.0, np.pi)
        
        self.get_logger().warn(f'IK failed to converge after {self.max_iter} iterations')
        return None

    def generate_joint_trajectory(self, start_joints, end_joints):
        """ Generate Minimum Jerk trajectory in joint space with angle wrap handling """
        
        # Calculate difference and normalize base joint (J0) to shortest path between [-pi, pi]
        diff = end_joints - start_joints
        diff[0] = (diff[0] + np.pi) % (2 * np.pi) - np.pi
        
        optimal_end_joints = start_joints + diff
        
        max_diff = np.max(np.abs(diff[:4]))
        duration = max(1.0, max_diff / self.max_joint_speed)
        
        num_steps = int(duration * self.control_rate)
        trajectory = []
        for i in range(num_steps + 1):
            t = i / max(1, num_steps)
            # Minimum jerk polynomial: s(t) = 10t^3 - 15t^4 + 6t^5
            s = 10*(t**3) - 15*(t**4) + 6*(t**5)
            traj_point = start_joints + s * diff
            trajectory.append(traj_point)
            
        self.get_logger().info(f"Generated {num_steps} trajectory points over {duration:.1f}s")
        return trajectory

    def publish_target_marker(self, target_xyz):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_position'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_xyz[0])
        marker.pose.position.y = float(target_xyz[1])
        marker.pose.position.z = float(target_xyz[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
        marker.lifetime.sec = 5
        self.marker_pub.publish(marker)

    def cartesian_callback(self, msg: PoseStamped):
        if not self.joints_updated: return
        if self.is_moving:
            self.get_logger().warn("Already moving, ignoring command")
            return
            
        target_xyz = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        if not self.check_workspace_limits(target_xyz): return
        target_joints = self.compute_ik(target_xyz, self.current_joints)
        if target_joints is None: 
            self.get_logger().error("Cannot find IK solution")
            return
            
        is_clear, obs_msg = self.check_octomap_collision_all_links(target_joints)
        if not is_clear:
            self.get_logger().error(f"Target causes collision: {obs_msg}")
            return
            
        self.publish_target_marker(target_xyz)
        self.trajectory_points = self.generate_joint_trajectory(self.current_joints, target_joints)
        
        for i in range(0, len(self.trajectory_points), max(1, len(self.trajectory_points)//10)):
            is_clear, obs_msg = self.check_octomap_collision_all_links(self.trajectory_points[i])
            if not is_clear:
                self.get_logger().error(f"Path blocked mid-way: {obs_msg}")
                return
                
        self.trajectory_idx = 0
        self.is_moving = True

    def control_timer_callback(self):
        """ Executed dynamically based on control rate (e.g. 50Hz) """
        if self.is_moving and self.trajectory_idx < len(self.trajectory_points):
            joints = self.trajectory_points[self.trajectory_idx]
            
            # Dynamic collision check during movement (check interval)
            if self.trajectory_idx % 5 == 0:
                is_clear, msg = self.check_octomap_collision_all_links(joints)
                if not is_clear:
                    self.get_logger().error(f"EMERGENCY STOP (Dynamic Obstacle): {msg}")
                    self.is_moving = False
                    return
                    
            msg = JointState()
            msg.name = ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint']
            msg.position = joints.tolist()
            self.joint_pub.publish(msg)
            
            self.trajectory_idx += 1
            if self.trajectory_idx >= len(self.trajectory_points):
                self.is_moving = False
                self.get_logger().info('✅ Cartesian motion complete')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleIKSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
