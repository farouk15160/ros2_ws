"""
ROS2 Publishers for Visiona Bridge.

Manages all ROS2 publishers for joint states, currents, and poses.
"""

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


class RobotPublishers:
    """Manages all ROS2 publishers for the robot."""
    
    def __init__(self, node, deg_to_rad_fn):
        """
        Initialize publishers.
        
        Args:
            node: ROS2 node instance
            deg_to_rad_fn: Function to convert degrees to radians
        """
        self.node = node
        self.deg_to_rad = deg_to_rad_fn
        
        # Create publishers
        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        self.joint_state_pub = node.create_publisher(JointState, 'joint_states', qos_profile=sensor_qos)
        self.gazebo_traj_pub = node.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.main_current_pub = node.create_publisher(Float32, 'main_current', 10)
        self.gripper_current_pub = node.create_publisher(Float32, 'gripper_current', 10)
        self.cartesian_pose_pub = node.create_publisher(PoseStamped, '/visiona/current_pose', 10)
    
    def publish_joint_state(self, joint_angles_deg, sync_gazebo=False):
        """
        Publish joint state message.
        
        Args:
            joint_angles_deg: List of 6 joint angles in degrees
            sync_gazebo: Whether to also publish to Gazebo controller
        """
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [
            'base_link_joint',
            'link_1_shoulder_joint',
            'link_2_elbow_joint',
            'link_3_wrist_joint',
            'link_3_wrist_to_gripper_base_joint',
            'gripper',
            'right_finger_joint'
        ]
        
        # Convert to radians
        positions_rad = [self.deg_to_rad(a) for a in joint_angles_deg]
        positions_rad.append(-positions_rad[5])  # Mimic joint
        msg.position = positions_rad
        
        self.joint_state_pub.publish(msg)
        
        # Sync to Gazebo if requested
        if sync_gazebo:
            self.publish_gazebo_sync(msg)
    
    def publish_gazebo_sync(self, joint_state_msg):
        """
        Publish trajectory to sync Gazebo with current state.
        
        Args:
            joint_state_msg: JointState message to sync
        """
        traj = JointTrajectory()
        traj.header.frame_id = 'base_link'
        traj.joint_names = joint_state_msg.name[:6]
        
        point = JointTrajectoryPoint()
        point.positions = joint_state_msg.position[:6]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50ms
        
        traj.points.append(point)
        self.gazebo_traj_pub.publish(traj)
    
    def publish_currents(self, main_current, gripper_current):
        """Publish motor current values."""
        self.main_current_pub.publish(Float32(data=main_current))
        self.gripper_current_pub.publish(Float32(data=gripper_current))
    
    def publish_cartesian_pose(self, tf_buffer, planning_frame, ee_link):
        """
        Publish current cartesian pose of end effector.
        
        Args:
            tf_buffer: TF2 buffer
            planning_frame: Planning frame name
            ee_link: End effector link name
            
        Returns:
            Tuple of (x, y, z) or None if TF not available
        """
        try:
            import rclpy.time
            t = tf_buffer.lookup_transform(planning_frame, ee_link, rclpy.time.Time())
            
            msg = PoseStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = planning_frame
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            
            self.cartesian_pose_pub.publish(msg)
            
            return (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        except:
            return None
