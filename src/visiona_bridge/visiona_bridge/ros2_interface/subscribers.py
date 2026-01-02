"""
ROS2 Subscribers for Visiona Bridge.

Manages all ROS2 subscribers and their callbacks.
"""

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState


class RobotSubscribers:
    """Manages all ROS2 subscribers for the robot."""
    
    def __init__(self, node, bridge):
        """
        Initialize subscribers.
        
        Args:
            node: ROS2 node instance
            bridge: Bridge instance with command methods
        """
        self.node = node
        self.bridge = bridge
        
        # Create subscribers
        node.create_subscription(JointState, 'joint_targets', self._joint_command_cb, 10)
        node.create_subscription(Float32MultiArray, 'gripper_command', self._gripper_command_cb, 10)
        node.create_subscription(Float32, 'set_speed_factor', self._speed_factor_cb, 10)
        node.create_subscription(Float32MultiArray, 'set_min_limits', self._min_limits_cb, 10)
        node.create_subscription(Float32MultiArray, 'set_max_limits', self._max_limits_cb, 10)
        node.create_subscription(Float32, 'set_collision_threshold', self._collision_threshold_cb, 10)
        node.create_subscription(Float32, 'set_collision_dev_threshold', self._dev_threshold_cb, 10)
        node.create_subscription(Float32, 'set_fan_speed', self._fan_speed_cb, 10)
        node.create_subscription(PoseStamped, '/visiona/target_pose', self._cartesian_target_cb, 10)
        node.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/controller_state',
            self._moveit_forward_cb,
            10
        )
    
    def _joint_command_cb(self, msg: JointState):
        """Handle joint position command."""
        if len(msg.position) != 6:
            return
        angles_deg = [self.bridge.rad_to_deg(p) for p in msg.position]
        self.bridge.send_joint_command(angles_deg)
    
    def _gripper_command_cb(self, msg: Float32MultiArray):
        """Handle gripper command."""
        if len(msg.data) < 2:
            return
        angle_deg, current_ma = msg.data[0], msg.data[1]
        self.bridge.send_gripper_command(angle_deg, current_ma)
    
    def _speed_factor_cb(self, msg: Float32):
        """Handle speed factor update."""
        if msg.data > 0:
            self.bridge.set_speed_factor(msg.data)
    
    def _min_limits_cb(self, msg: Float32MultiArray):
        """Handle minimum limits update."""
        if len(msg.data) == 6:
            self.bridge.set_min_limits(list(msg.data))
    
    def _max_limits_cb(self, msg: Float32MultiArray):
        """Handle maximum limits update."""
        if len(msg.data) == 6:
            self.bridge.set_max_limits(list(msg.data))
    
    def _collision_threshold_cb(self, msg: Float32):
        """Handle collision threshold update."""
        self.bridge.set_collision_threshold(msg.data)
    
    def _dev_threshold_cb(self, msg: Float32):
        """Handle deviation threshold update."""
        self.bridge.set_deviation_threshold(msg.data)
    
    def _fan_speed_cb(self, msg: Float32):
        """Handle fan speed update."""
        self.bridge.set_fan_speed(int(msg.data))
    
    def _cartesian_target_cb(self, msg: PoseStamped):
        """Handle cartesian target pose (for MoveIt)."""
        # Delegate to cartesian interface
        if hasattr(self.bridge, 'cartesian'):
            self.bridge.cartesian.handle_target_pose(msg)
    
    def _moveit_forward_cb(self, msg: JointTrajectoryControllerState):
        """Forward MoveIt controller commands to hardware."""
        if not hasattr(msg, 'desired') or not hasattr(msg.desired, 'positions'):
            return
        if len(msg.desired.positions) < 5:
            return
        
        # Convert radians to degrees
        angles_deg = [self.bridge.rad_to_deg(p) for p in msg.desired.positions[:6]]
        if len(angles_deg) == 5:
            angles_deg.append(90.0)  # Default gripper angle
        
        self.bridge.send_joint_command(angles_deg)
