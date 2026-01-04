#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import sys

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        
        # --- Parameters ---
        self.declare_parameter('group_name', 'arm')  # Fixed: was 'visiona_arm', should be 'arm'
        self.declare_parameter('end_effector_link', 'gripper_base') # Check SRDF, seems to be gripper_base
        self.declare_parameter('planning_frame', 'world')

        self.group_name = self.get_parameter('group_name').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.planning_frame = self.get_parameter('planning_frame').value

        # --- Components ---
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Publisher for current pose
        self.pose_pub = self.create_publisher(PoseStamped, '/visiona/current_pose', 10)
        
        # Subscriber for target pose
        self.target_sub = self.create_subscription(PoseStamped, '/visiona/target_pose', self.target_callback, 10)
        
        # TF Buffer for reading current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for publishing
        self.timer = self.create_timer(0.5, self.publish_current_pose)
        
        self.get_logger().info(f'Cartesian Controller Started. Listening on /visiona/target_pose. Publishing to /visiona/current_pose.')

    def publish_current_pose(self):
        try:
            # Lookup transform from world to EE
            t = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_link,
                rclpy.time.Time())
                
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.planning_frame
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            
            self.pose_pub.publish(msg)
            
        except Exception as e:
            # self.get_logger().warn(f'Could not lookup transform: {e}')
            pass

    def target_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received Target Pose: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}')
        
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveGroup action server not available.')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Create Constraints
        constraints = Constraints()
        constraints.name = "cartesian_goal"
        
        # Position Constraint
        pc = PositionConstraint()
        pc.header.frame_id = self.planning_frame
        pc.link_name = self.ee_link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # 1cm tolerance
        
        # The region pose is the target pose
        region_pose = msg.pose
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # Orientation Constraint
        oc = OrientationConstraint()
        oc.header.frame_id = self.planning_frame
        oc.link_name = self.ee_link
        oc.orientation = msg.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info('Sending MoveGroup Goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted. Executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result Error Code: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
