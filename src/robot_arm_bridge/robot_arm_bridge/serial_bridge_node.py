import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Float32MultiArray
import serial
import struct
import time
import threading

# Match the definitions in the ESP32 firmware
HEADER_BYTE = 0xA5
# PC to ESP32: header, cmd_id, 6 angles (float), speed (float), grip_current (float), checksum
COMMAND_PACKET_FORMAT = '<BB6ffB' # Note: format string was corrected from 'ffffff' to '6f' for clarity
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FORMAT)

# ESP32 to PC: header, main_current (float), grip_current (float), 6 angles (float), checksum
STATUS_PACKET_FORMAT = '<Bff6fB'
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FORMAT)

class RobotArmBridge(Node):
    def __init__(self):
        super().__init__('robot_arm_bridge')
        
        # Declare parameters for serial port, baud rate, etc.
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.ser = None
        self.connection_thread = threading.Thread(target=self.manage_connection)
        self.connection_thread.daemon = True
        self.connection_thread.start()

        # Publisher for the robot's current joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber for target joint commands
        self.joint_command_subscriber = self.create_subscription(
            JointState,
            'joint_targets',
            self.joint_command_callback,
            10)
            
        # --- NEW: Subscriber for gripper commands ---
        self.gripper_command_subscriber = self.create_subscription(
            Float32MultiArray,
            'gripper_command',
            self.gripper_command_callback,
            10)
            
        self.get_logger().info(f"Robot Arm Bridge initialized. Waiting for serial connection...")

    def manage_connection(self):
        """Runs in a separate thread to handle serial connection and reading."""
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                    self.get_logger().info(f"Successfully connected to {self.serial_port}")
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to connect to {self.serial_port}: {e}. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue
            
            # If connected, start reading loop
            self.read_from_serial()

    def read_from_serial(self):
        """Continuously read and process status packets from the ESP32."""
        read_buffer = bytearray()
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    read_buffer.extend(self.ser.read(self.ser.in_waiting))

                    while len(read_buffer) >= STATUS_PACKET_SIZE:
                        # Search for header byte
                        header_index = read_buffer.find(HEADER_BYTE)
                        if header_index == -1:
                            read_buffer.clear() # No header found, clear buffer
                            continue
                        
                        # Discard bytes before header
                        if header_index > 0:
                            read_buffer = read_buffer[header_index:]

                        # Check if we have a full packet
                        if len(read_buffer) < STATUS_PACKET_SIZE:
                            break # Not enough data yet, wait for more

                        # Potential packet found, unpack and verify
                        packet_data = read_buffer[:STATUS_PACKET_SIZE]
                        
                        # Verify checksum
                        received_checksum = packet_data[-1]
                        calculated_checksum = self.calculate_checksum(packet_data[:-1])

                        if received_checksum == calculated_checksum:
                            self.process_status_packet(packet_data)
                            read_buffer = read_buffer[STATUS_PACKET_SIZE:] # Move buffer past this packet
                        else:
                            self.get_logger().warn(f"Checksum mismatch! Discarding packet. Got {received_checksum}, expected {calculated_checksum}")
                            read_buffer = read_buffer[1:] # Discard header and try again
            
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}. Closing connection.")
                self.ser.close()
                self.ser = None
                break
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred in read loop: {e}")
            
            time.sleep(0.001) # Small sleep to prevent busy-looping

    def process_status_packet(self, data):
        """Unpack status data and publish it as a JointState message."""
        try:
            unpacked_data = struct.unpack(STATUS_PACKET_FORMAT, data)
            # _, main_current, gripper_current, a0, a1, a2, a3, a4, a5, _ = unpacked_data
            
            # Create and publish JointState message
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']
            # Angles from firmware need conversion from degrees to radians for ROS standard
            joint_state_msg.position = [self.deg_to_rad(angle) for angle in unpacked_data[3:9]]
            # You can also publish velocity and effort (currents) if needed
            # joint_state_msg.effort = [main_current, gripper_current] 
            self.joint_state_publisher.publish(joint_state_msg)

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack status packet: {e}")

    def joint_command_callback(self, msg: JointState):
        """Receive a JointState command, pack it, and send to ESP32."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial not connected. Cannot send command.")
            return

        if len(msg.position) != 6:
            self.get_logger().warn(f"Command requires 6 joint angles, but received {len(msg.position)}. Ignoring.")
            return

        # Convert angles from radians back to degrees for the firmware
        angles_deg = [self.rad_to_deg(p) for p in msg.position]
        
        # Default values for speed and gripper current
        speed_factor = 150.0  
        gripper_current = 200.0 # Not used for 'M' command, but needed for packet structure

        # Pack data into the binary format
        command_id = ord('M') # 'M' for Move
        self.send_packet(command_id, angles_deg, speed_factor, gripper_current)

    # --- NEW: Callback for gripper commands ---
    def gripper_command_callback(self, msg: Float32MultiArray):
        """Receives gripper command [target_angle_deg, current_threshold_mA]"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial not connected. Cannot send gripper command.")
            return
        
        if len(msg.data) < 2:
            self.get_logger().warn(f"Gripper command requires 2 values, but got {len(msg.data)}. Ignoring.")
            return

        target_angle_deg = msg.data[0]
        current_threshold_ma = msg.data[1]

        # For a 'G' command, the firmware only needs the gripper angle and current.
        # We send placeholder values for the other angles and speed.
        angles_deg = [0.0] * 6
        angles_deg[5] = target_angle_deg # Index 5 is the gripper
        speed_factor = 150.0 # Placeholder, not used by firmware for grip command

        command_id = ord('G') # 'G' for Grip
        self.send_packet(command_id, angles_deg, speed_factor, current_threshold_ma)
        self.get_logger().info(f"Sent gripper command: angle={target_angle_deg}, current={current_threshold_ma}")

    def send_packet(self, command_id, angles_deg, speed_factor, gripper_current):
        """Helper function to pack and send a command packet."""
        try:
            # Create the packet payload
            payload = struct.pack('<B6fff', command_id, *angles_deg, speed_factor, gripper_current)
            
            # Prepend header and calculate checksum
            header_and_payload = struct.pack('<B', HEADER_BYTE) + payload
            checksum = self.calculate_checksum(header_and_payload)
            
            # Construct the final packet with checksum
            full_packet = header_and_payload + struct.pack('<B', checksum)

            self.ser.write(full_packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")
            self.ser.close()
            self.ser = None
        except Exception as e:
            self.get_logger().error(f"Error sending packet: {e}")

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """Calculate a simple XOR checksum."""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
        
    @staticmethod
    def deg_to_rad(deg):
        return deg * 3.1415926535 / 180.0
        
    @staticmethod
    def rad_to_deg(rad):
        return rad * 180.0 / 3.1415926535

def main(args=None):
    rclpy.init(args=args)
    bridge_node = RobotArmBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        if bridge_node.ser and bridge_node.ser.is_open:
            bridge_node.ser.close()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

