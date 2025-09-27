import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Float32MultiArray, Float32, Bool
import serial
import struct
import time
import threading

# Match the definitions in the ESP32 firmware
HEADER_BYTE = 0xA5
# PC to ESP32: header, cmd_id, 6 angles (float), speed (float), grip_current (float), checksum
COMMAND_PACKET_FORMAT = '<BB6ffB'
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FORMAT)

# ESP32 to PC: header, main_current (float), grip_current (float), 6 angles (float), checksum
STATUS_PACKET_FORMAT = '<Bff6fB'
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FORMAT)

# --- New Constants ---
STATUS_TIMEOUT_SEC = 2.0 # Seconds without status packet before warning/E-Stop

class RobotArmBridge(Node):
    def __init__(self):
        super().__init__('robot_arm_bridge')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.speed_factor = 20.0 
        self.ser = None

        self.connection_thread = threading.Thread(target=self.manage_connection)
        self.connection_thread.daemon = True
        self.connection_thread.start()

        # --- Publishers ---
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.main_current_publisher = self.create_publisher(Float32, 'main_current', 10)
        self.gripper_current_publisher = self.create_publisher(Float32, 'gripper_current', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # --- Subscribers ---
        self.joint_command_subscriber = self.create_subscription(
            JointState, 'joint_targets', self.joint_command_callback, 10)
        self.gripper_command_subscriber = self.create_subscription(
            Float32MultiArray, 'gripper_command', self.gripper_command_callback, 10)
        self.speed_factor_subscriber = self.create_subscription(
            Float32, 'set_speed_factor', self.set_speed_factor_callback, 10)
        
        # === NEW HOMING SUBSCRIBER ===
        self.homing_subscriber = self.create_subscription(
            Bool, 'home_robot', self.home_robot_callback, 10)

        # --- E-Stop Detection Timer ---
        self.last_status_time = self.get_clock().now()
        self.estop_timer = self.create_timer(1.0, self.check_status_timeout)
        self.emergency_stop_active = False

        self.get_logger().info(f"Robot Arm Bridge initialized. Default speed factor: {self.speed_factor} ms/degree.")

    def manage_connection(self):
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                    self.get_logger().info(f"Successfully connected to {self.serial_port}")
                    self.emergency_stop_active = False
                    time.sleep(0.1) # Wait a moment for serial port to initialize
                    self.ser.reset_input_buffer() # Clear any old data from the buffer
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to connect: {e}. Retrying...", throttle_duration_sec=5)
                    time.sleep(5)
                    continue
            self.read_from_serial()

    def check_status_timeout(self):
        if not self.emergency_stop_active:
            duration_since_last_status = self.get_clock().now() - self.last_status_time
            if duration_since_last_status.nanoseconds / 1e9 > STATUS_TIMEOUT_SEC:
                self.get_logger().error("Status timeout! MCU may have emergency stopped or disconnected.")
                self.emergency_stop_publisher.publish(Bool(data=True))
                self.emergency_stop_active = True

    def read_from_serial(self):
        read_buffer = bytearray()
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    read_buffer.extend(self.ser.read(self.ser.in_waiting))

                while len(read_buffer) >= STATUS_PACKET_SIZE:
                    header_index = read_buffer.find(HEADER_BYTE)
                    if header_index == -1:
                        read_buffer.clear(); continue
                    if header_index > 0:
                        read_buffer = read_buffer[header_index:]
                    if len(read_buffer) < STATUS_PACKET_SIZE:
                        break

                    packet_data = read_buffer[:STATUS_PACKET_SIZE]
                    received_checksum = packet_data[-1]
                    calculated_checksum = self.calculate_checksum(packet_data[:-1])

                    if received_checksum == calculated_checksum:
                        self.process_status_packet(packet_data)
                        read_buffer = read_buffer[STATUS_PACKET_SIZE:]
                    else:
                        self.get_logger().warn("Checksum mismatch! Discarding packet.")
                        read_buffer = read_buffer[1:]
            
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}. Closing connection.")
                self.ser.close(); self.ser = None; break
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read loop: {e}")
            
            time.sleep(0.001)

    def process_status_packet(self, data):
        self.last_status_time = self.get_clock().now()
        if self.emergency_stop_active:
             self.get_logger().info("Communication re-established, clearing E-Stop.")
             self.emergency_stop_publisher.publish(Bool(data=False))
             self.emergency_stop_active = False
             
        try:
            _, main_current, gripper_current, a0, a1, a2, a3, a4, a5, _ = struct.unpack(STATUS_PACKET_FORMAT, data)
            
            self.main_current_publisher.publish(Float32(data=main_current))
            self.gripper_current_publisher.publish(Float32(data=gripper_current))

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']
            joint_state_msg.position = [self.deg_to_rad(angle) for angle in [a0, a1, a2, a3, a4, a5]]
            self.joint_state_publisher.publish(joint_state_msg)

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack status packet: {e}")

    def joint_command_callback(self, msg: JointState):
        # FIX: Restored the correct logic for this function.
        angles_rad = []
        position_list = list(msg.position) # Define position_list from the message
        
        if len(position_list) == 5:
            # Handle 5-element command (base, j3, j4, j5, gripper)
            base_angle = position_list[0]
            # Duplicate base angle for joint 1 and 2
            angles_rad = [base_angle, base_angle] + position_list[1:5]
        elif len(position_list) == 6:
            # Handle 6-element command
            angles_rad = position_list
        else:
            self.get_logger().warn(f"Command requires 5 or 6 joint angles, received {len(position_list)}. Ignoring.")
            return

        angles_deg = [self.rad_to_deg(p) for p in angles_rad]
        self.send_packet(ord('M'), angles_deg, self.speed_factor, 200.0)

    # === NEW HOMING CALLBACK ===
    def home_robot_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Homing command received. Sending to robot.")
            # The firmware's 'H' command doesn't need angle data, so we send zeros.
            dummy_angles = [0.0] * 6
            self.send_packet(ord('H'), dummy_angles, self.speed_factor, 0.0)

    def gripper_command_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(f"Gripper command requires 2 values, got {len(msg.data)}. Ignoring.")
            return
        target_angle_deg, current_ma = msg.data[0], msg.data[1]
        angles_deg = [0.0] * 6
        angles_deg[5] = target_angle_deg
        self.send_packet(ord('G'), angles_deg, self.speed_factor, current_ma)
        self.get_logger().info(f"Sent gripper command: angle={target_angle_deg}, current={current_ma}")

    def set_speed_factor_callback(self, msg: Float32):
        new_speed = msg.data
        if new_speed > 0:
            self.speed_factor = new_speed
            self.get_logger().info(f"Robot arm speed factor set to: {self.speed_factor} ms/degree")
        else:
            self.get_logger().warn(f"Speed factor must be positive. Received {new_speed}.")
            
    def send_packet(self, command_id, angles_deg, speed_factor, gripper_current):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial not connected. Cannot send command.")
            return

        # Add a guard clause for internal safety
        if len(angles_deg) != 6:
            self.get_logger().error(f"send_packet internal error: Expected 6 angles, got {len(angles_deg)}. Dropping command.")
            return
            
        try:
            # FIX: Rewritten packet creation to be explicit and robust, preventing pack errors.
            # Using 'ffffff' is more explicit than '6f'.
            main_packet_bytes = struct.pack('<BBffffff',
                HEADER_BYTE,
                command_id,
                angles_deg[0],
                angles_deg[1],
                angles_deg[2],
                angles_deg[3],
                angles_deg[4],
                angles_deg[5]
            )
            
            # Pack the remaining floats separately and append
            main_packet_bytes += struct.pack('<ff', speed_factor, gripper_current)
            
            checksum = self.calculate_checksum(main_packet_bytes)
            full_packet = main_packet_bytes + struct.pack('<B', checksum)

            self.ser.write(full_packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")
            self.ser.close(); self.ser = None
        except Exception as e:
            self.get_logger().error(f"Error sending packet: {e}")

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
        
    @staticmethod
    def deg_to_rad(deg): return deg * 3.1415926535 / 180.0
        
    @staticmethod
    def rad_to_deg(rad): return rad * 180.0 / 3.1415926535

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

