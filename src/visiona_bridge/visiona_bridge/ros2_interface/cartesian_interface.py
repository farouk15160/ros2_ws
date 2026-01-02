"""
Cartesian Interface for MoveIt Integration.

Handles cartesian space control and MoveIt action integration.
"""

import rclpy.time
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class CartesianInterface:
    """Manages cartesian space control and MoveIt integration."""
    
    def __init__(self, node, logger):
        """
        Initialize cartesian interface.
        
        Args:
            node: ROS2 node instance
            logger: Logger instance
        """
        self.node = node
        self.logger = logger
        
        # Configuration
        self.planning_frame = 'world'
        self.ee_link = 'gripper_base'
        self.move_group_name = 'visiona_arm'
        
        # Workspace bounds (meters)
        self.workspace_x_range = (-0.35, 0.35)
        self.workspace_y_range = (-0.35, 0.35)
        self.workspace_z_range = (0.05, 0.50)
        
        # TF2 for pose lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        # MoveIt action client
        self._move_action_client = ActionClient(node, MoveGroup, 'move_action')
        
        logger.info("Cartesian Interface Initialized.")
    
    def get_current_pose(self):
        """
        Get current end-effector pose.
        
        Returns:
            Transform or None if not available
        """
        try:
            return self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_link,
                rclpy.time.Time()
            )
        except:
            return None
    
    def jog(self, dx, dy, dz, socketio=None):
        """
        Jog in cartesian space.
        
        Args:
            dx, dy, dz: Relative movement in meters
            socketio: SocketIO instance for feedback
            
        Returns:
            True if jog initiated, False otherwise
        """
        try:
            # Get current pose
            t = self.get_current_pose()
            if not t:
                self.logger.warn("TF not available for jogging")
                return False
            
            # Calculate target
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.planning_frame
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
            target_pose.pose.position.x = t.transform.translation.x + dx
            target_pose.pose.position.y = t.transform.translation.y + dy
            target_pose.pose.position.z = t.transform.translation.z + dz
            target_pose.pose.orientation = t.transform.rotation
            
            # Validate workspace bounds
            x, y, z = target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z
            
            if not self._in_workspace(x, y, z):
                self.logger.warn(f"Jog target ({x:.2f}, {y:.2f}, {z:.2f}) outside workspace")
                if socketio:
                    from ..gui.socketio_handlers import emit_log_message
                    emit_log_message(socketio, 'warn', f'Jog outside workspace limits')
                return False
            
            self.logger.info(f"Jogging to: {x:.2f}, {y:.2f}, {z:.2f}")
            
            # Send to MoveIt
            return self.handle_target_pose(target_pose)
            
        except Exception as e:
            self.logger.error(f"Jog failed: {e}")
            return False
    
    def _in_workspace(self, x, y, z):
        """Check if position is within workspace bounds."""
        return (self.workspace_x_range[0] <= x <= self.workspace_x_range[1] and
                self.workspace_y_range[0] <= y <= self.workspace_y_range[1] and
                self.workspace_z_range[0] <= z <= self.workspace_z_range[1])
    
    def handle_target_pose(self, msg: PoseStamped):
        """
        Handle cartesian target pose request.
        
        Args:
            msg: Target pose message
            
        Returns:
            True if goal sent, False otherwise
        """
        self.logger.info(
            f'Cartesian target: {msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}'
        )
        
        # Check if action server is available
        if not self._move_action_client.wait_for_server(timeout_sec=2.0):
            self.logger.error('MoveGroup action server not available.')
            return False
        
        # Build goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 1.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Create constraints
        constraints = Constraints()
        constraints.name = "cartesian_goal"
        
        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = self.planning_frame
        pc.link_name = self.ee_link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.05])
        )
        pc.constraint_region.primitive_poses.append(msg.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = self.planning_frame
        oc.link_name = self.ee_link
        oc.orientation = msg.pose.orientation
        oc.absolute_x_axis_tolerance = 0.5
        oc.absolute_y_axis_tolerance = 0.5
        oc.absolute_z_axis_tolerance = 0.5
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # Send goal
        self.logger.info('Sending MoveGroup Goal...')
        self._send_goal_future = self._move_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def _goal_response_callback(self, future):
        """Handle MoveGroup goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Cartesian Goal rejected')
            return
        
        self.logger.info('Cartesian Goal accepted. Executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """Handle MoveGroup result."""
        result = future.result().result
        self.logger.info(f'Cartesian Result Error Code: {result.error_code.val}')
