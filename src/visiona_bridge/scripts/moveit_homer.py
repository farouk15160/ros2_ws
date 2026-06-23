#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
import sys

# Joint names and their "Home" values (all 1.57 as per SRDF)
HOME_JOINTS = {
    'base_link_joint': 1.57,
    'link_1_shoulder_joint': 1.57,
    'link_2_elbow_joint': 1.57,
    'link_3_wrist_joint': 1.57,
    'link_3_wrist_to_gripper_base_joint': 1.57
}

class MoveItHomer(Node):
    def __init__(self):
        super().__init__('moveit_homer')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_home_goal(self):
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available. Cannot home.')
            return

        self.get_logger().info('Constructing Home Goal...')
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Construct Joint Constraints for "Home"
        for joint_name, target_val in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_msg.request.goal_constraints.append(Constraints(joint_constraints=[jc]))
            # Note: usually we append one Constraints object with multiple joint_constraints,
            # or multiple Constraints objects? 
            # MoveIt generally expects ONE Constraints object in the list that defines the goal state.
            
        # Correctly grouping them:
        goal_msg.request.goal_constraints = []
        target_constraints = Constraints()
        for joint_name, target_val in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            target_constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(target_constraints)

        self.get_logger().info('Sending Goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Moving to Home...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code.val}')
        # Clean shutdown after completion
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    homer = MoveItHomer()
    homer.send_home_goal()
    rclpy.spin(homer)

if __name__ == '__main__':
    main()
