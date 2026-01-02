"""
ROS2 Services for Visiona Bridge.

Manages all ROS2 service handlers.
"""

from std_srvs.srv import Trigger, SetBool


class RobotServices:
    """Manages all ROS2 services for the robot."""
    
    def __init__(self, node, bridge):
        """
        Initialize services.
        
        Args:
            node: ROS2 node instance
            bridge: Bridge instance with command methods
        """
        self.node = node
        self.bridge = bridge
        
        # Create services
        node.create_service(Trigger, 'home_robot', self._home_robot_srv)
        node.create_service(Trigger, 'emergency_stop', self._trigger_estop_srv)
        node.create_service(Trigger, 'release_emergency_stop', self._release_estop_srv)
        node.create_service(Trigger, 'kill_motors', self._kill_motors_srv)
        node.create_service(SetBool, 'set_collision_safety', self._set_collision_safety_srv)
    
    def _home_robot_srv(self, request, response):
        """Handle home robot service request."""
        self.bridge.send_home_command()
        response.success = True
        response.message = "Homing initiated"
        return response
    
    def _trigger_estop_srv(self, request, response):
        """Handle emergency stop trigger."""
        self.bridge.hardware.emergency_stop_active = True
        self.node.get_logger().warn("E-STOP triggered via ROS Service!")
        self.bridge.emit_full_status()
        response.success = True
        response.message = "E-Stop triggered"
        return response
    
    def _release_estop_srv(self, request, response):
        """Handle emergency stop release."""
        self.bridge.release_estop()
        response.success = True
        response.message = "E-Stop release initiated"
        return response
    
    def _kill_motors_srv(self, request, response):
        """Handle kill motors request."""
        self.bridge.kill_motors()
        response.success = True
        response.message = "Kill motors command sent"
        return response
    
    def _set_collision_safety_srv(self, request, response):
        """Handle collision safety toggle."""
        self.bridge.set_collision_safety(request.data)
        response.success = True
        response.message = f"Collision Safety {'ENABLED' if request.data else 'DISABLED'}"
        return response
