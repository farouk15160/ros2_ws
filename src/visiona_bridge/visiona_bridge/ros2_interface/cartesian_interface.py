"""
Cartesian Interface for Simple IK Solver Integration.

Handles cartesian space control via direct topic publishing to simple_ik_solver.
No MoveIt dependency - fast and lightweight!
"""

import rclpy.time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped


class CartesianInterface:
    """Manages cartesian space control via simple IK solver."""
    
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
        
        # Workspace bounds (meters)
        self.workspace_x_range = (-0.5, 0.5)
        self.workspace_y_range = (-0.5, 0.5)
        self.workspace_z_range = (0.0, 0.8)
        
        # TF2 for pose lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        # Publisher for cartesian commands (simple IK solver)
        self.cartesian_pub = node.create_publisher(
            PoseStamped,
            '/visiona/cartesian_command',
            10
        )
        
        logger.info("✅ Cartesian Interface Initialized (Simple IK Mode)")
    
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
                self.logger.error("❌ TF not available for jogging - cannot get current end-effector pose")
                if socketio:
                    from ..gui.socketio_handlers import emit_log_message
                    emit_log_message(socketio, 'error', 'TF not available - cannot jog')
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
            
            self.logger.info(f"✅ Jogging to: {x:.2f}, {y:.2f}, {z:.2f}")
            
            # Send to simple IK solver
            return self.handle_target_pose(target_pose)
            
        except Exception as e:
            self.logger.error(f"❌ Jog failed: {e}")
            if socketio:
                from ..gui.socketio_handlers import emit_log_message
                emit_log_message(socketio, 'error', f'Jog failed: {str(e)}')
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
            True (always succeeds - IK solver handles execution)
        """
        self.logger.info(
            f'🎯 Cartesian target: {msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}'
        )
        
        # Publish to simple IK solver (no action client needed!)
        self.cartesian_pub.publish(msg)
        
        return True
