import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GazeboMirror(Node):
    def __init__(self):
        super().__init__('gazebo_mirror_node')
        
        # This node listens to the REAL robot's state
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10)
            
        # This node commands the SIMULATED robot's controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
            
        self.get_logger().info('Gazebo Mirror node started. Listening to /joint_states and publishing to /joint_trajectory_controller/joint_trajectory')

    def state_callback(self, msg: JointState):
        if not msg.position:
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        # The joint names must match the order in your controller config
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Create a single point in the trajectory
        point = JointTrajectoryPoint()
        point.positions = list(msg.position)
        point.time_from_start = Duration(sec=0, nanosec=500000000) # Reach in 0.5s

        trajectory_msg.points.append(point)
        self.publisher.publish(trajectory_msg)

# ===============================================================
# === THE MISSING MAIN FUNCTION BOILERPLATE IS NOW ADDED HERE ===
# ===============================================================
def main(args=None):
    rclpy.init(args=args)
    gazebo_mirror_node = GazeboMirror()
    try:
        rclpy.spin(gazebo_mirror_node)
    except KeyboardInterrupt:
        pass
    finally:
        gazebo_mirror_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

