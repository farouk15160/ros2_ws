"""
Robot hardware abstraction layer.

Provides a high-level interface to the robot hardware, handling state management,
simulation mode, and translation between ROS concepts and hardware commands.
"""

import threading
import time
from typing import List, Callable, Optional
from .packet_protocol import PacketProtocol


# Simulation constants
SIM_LOOP_HZ = 100.0
SIM_MAX_SPEED_DPS = 180.0
SIM_MIN_SPEED_DPS = 20.0


class RobotHardware:
    """
    Hardware abstraction layer for the Visiona robot.
    
    Manages robot state, simulation mode, and provides methods to
    send commands and process hardware feedback.
    """
    
    def __init__(self, logger, on_state_update: Callable, get_clock: Callable, socketio=None):
        """
        Initialize robot hardware interface.
        
        Args:
            logger: ROS logger instance
            on_state_update: Callback when state changes (receives state dict)
            get_clock: Function to get current ROS time
            socketio: Optional SocketIO instance for GUI logging
        """
        self.logger = logger
        self.on_state_update = on_state_update
        self.get_clock = get_clock
        self.socketio = socketio
        
        # State variables
        self.state_lock = threading.Lock()
        self.current_joint_angles_deg = [90.0] * 6
        self.main_current = 0.0
        self.gripper_current = 0.0
        self.emergency_stop_active = False
        self.speed_factor = 150.0
        
        # Logging throttling
        self.last_terminal_print_time = time.time()
        
        # Configuration
        self.servos_min = [15.0] * 6
        self.servos_max = [345.0] * 6
        self.collision_threshold = 5.0
        self.collision_deviation_threshold = 1.0
        self.fan_speed = 18
        self.collision_detection_enabled = False
        
        # Simulation mode
        self.simulation_mode = False
        self.simulation_thread: Optional[threading.Thread] = None
        self.sim_current_joint_angles_deg = [90.0] * 6
        self.sim_target_joint_angles_deg = [90.0] * 6
        self.sim_last_update_time = self.get_clock().now()
    
    def process_status_packet(self, data: bytes):
        """Process a status packet from the hardware."""
        try:
            main, grip, joint_angles, collision_flag = PacketProtocol.parse_status_packet(data)
            
            # Log to GUI
            if self.socketio:
                from ..gui.socketio_handlers import emit_log_message
                msg = (f"MCU ◄ Status: J:[{', '.join(f'{a:.1f}' for a in joint_angles)}°] "
                       f"I:{main:.2f}A G:{grip:.1f}mA E:{collision_flag}")
                emit_log_message(self.socketio, 'info', msg)  # Changed from 'debug' to 'info'
            
            # Log to terminal (throttled)
            from ..bridge_constants import MCU_STATUS_PRINT_INTERVAL_SEC
            current_time = time.time()
            if current_time - self.last_terminal_print_time >= MCU_STATUS_PRINT_INTERVAL_SEC:
                self.logger.info(
                    f"MCU ◄ Status: Joints=[{', '.join(f'{a:.1f}' for a in joint_angles)}°] "
                    f"Main:{main:.2f}A Grip:{grip:.1f}mA Estop:{collision_flag}"
                )
                self.last_terminal_print_time = current_time
            
            self._update_state(main, grip, joint_angles, collision_flag)
        except Exception as e:
            self.logger.error(f"Failed to process status packet: {e}", exc_info=True)
    
    def process_config_packet(self, data: bytes):
        """Process a configuration packet from the hardware."""
        try:
            servos_min, servos_max, threshold, dev_threshold = PacketProtocol.parse_config_packet(data)
            
            with self.state_lock:
                self.servos_min = servos_min
                self.servos_max = servos_max
                self.collision_threshold = threshold
                self.collision_deviation_threshold = dev_threshold
            
            self.logger.info(
                f"Received config. Abs:{threshold:.2f}A, Dev:{dev_threshold:.2f}A"
            )
            self.on_state_update(self.get_state_dict())
            
        except Exception as e:
            self.logger.error(f"Failed to process config packet: {e}")
    
    def _update_state(self, main: float, grip: float, angles: List[float], collision_flag: int):
        """Update robot state from status data."""
        should_emit_immediately = False
        mcu_estop_active = (collision_flag == 1)
        
        with self.state_lock:
            self.main_current = main
            self.gripper_current = grip
            self.current_joint_angles_deg = angles
            
            # Check for E-stop state changes
            if mcu_estop_active and not self.emergency_stop_active:
                self.logger.error("E-STOP triggered!")
                should_emit_immediately = True
            elif not mcu_estop_active and self.emergency_stop_active:
                self.logger.info("E-Stop cleared.")
                should_emit_immediately = True
            
            self.emergency_stop_active = mcu_estop_active
        
        # Notify state update
        self.on_state_update(self.get_state_dict(), should_emit_immediately)
    
    def get_state_dict(self) -> dict:
        """Get current robot state as a dictionary."""
        with self.state_lock:
            return {
                'joint_angles': self.current_joint_angles_deg[:],
                'main_current': self.main_current,
                'gripper_current': self.gripper_current,
                'emergency_stop': self.emergency_stop_active,
                'speed_factor': self.speed_factor,
                'servos_min': self.servos_min[:],
                'servos_max': self.servos_max[:],
                'collision_threshold': self.collision_threshold,
                'collision_deviation_threshold': self.collision_deviation_threshold,
                'fan_speed': self.fan_speed,
                'collision_detection_enabled': self.collision_detection_enabled,
                'simulation_mode': self.simulation_mode
            }
    
    def set_simulation_mode(self, enable: bool):
        """Enable or disable simulation mode."""
        with self.state_lock:
            if self.simulation_mode == enable:
                return
            
            self.simulation_mode = enable
            
            if enable:
                self.logger.warn(">>> SIMULATION MODE ENABLED <<<")
                self.emergency_stop_active = False
                
                # Initialize sim state
                self.sim_current_joint_angles_deg = self.current_joint_angles_deg[:]
                self.sim_target_joint_angles_deg = self.current_joint_angles_deg[:]
                self.sim_last_update_time = self.get_clock().now()
                
                # Start simulation worker
                if self.simulation_thread is None or not self.simulation_thread.is_alive():
                    self.simulation_thread = threading.Thread(
                        target=self._simulation_worker, daemon=True
                    )
                    self.simulation_thread.start()
            else:
                self.logger.warn(">>> SIMULATION MODE DISABLED <<<")
        
        self.on_state_update(self.get_state_dict(), immediate=True)
    
    def _simulation_worker(self):
        """Worker thread for running the simulation loop."""
        self.logger.info("Simulation worker started.")
        sleep_duration = 1.0 / SIM_LOOP_HZ
        
        while self.simulation_mode:
            now = self.get_clock().now()
            dt = (now - self.sim_last_update_time).nanoseconds / 1e9
            self.sim_last_update_time = now
            
            if dt <= 0:
                time.sleep(sleep_duration / 2.0)
                continue
            
            # Simulate joint movement
            with self.state_lock:
                speed_range = 500.0 - 10.0
                speed_normalized = max(0.0, min(1.0, (self.speed_factor - 10.0) / speed_range))
                deg_per_sec = SIM_MAX_SPEED_DPS - (speed_normalized * (SIM_MAX_SPEED_DPS - SIM_MIN_SPEED_DPS))
                max_change_this_frame = deg_per_sec * dt
                
                is_moving = False
                new_angles = self.sim_current_joint_angles_deg[:]
                
                for i in range(6):
                    target = self.sim_target_joint_angles_deg[i]
                    current = self.sim_current_joint_angles_deg[i]
                    diff = target - current
                    
                    if abs(diff) < 0.01:
                        continue
                    
                    is_moving = True
                    change = max(-max_change_this_frame, min(max_change_this_frame, diff))
                    new_angles[i] = current + change
                
                self.sim_current_joint_angles_deg = new_angles
                sim_main_current = 0.15 if is_moving else 0.05
                sim_gripper_current = 5.0 if abs(new_angles[5] - self.sim_target_joint_angles_deg[5]) > 0.01 else 0.0
                sim_estop = 1 if self.emergency_stop_active else 0
            
            # Update state
            try:
                self._update_state(sim_main_current, sim_gripper_current, new_angles, sim_estop)
            except Exception as e:
                self.logger.error(f"Error in sim worker update: {e}", exc_info=True)
            
            time.sleep(sleep_duration)
        
        self.logger.info("Simulation worker stopped.")
    
    def process_sim_command(self, command_id: int, angles_deg: List[float], 
                          speed_factor: float, gripper_current: float) -> bool:
        """Process a command in simulation mode."""
        cmd_char = chr(command_id)
        
        with self.state_lock:
            if self.emergency_stop_active and cmd_char != 'E':
                self.logger.warn("Sim E-Stop active, ignoring command.")
                return True
            
            if cmd_char == 'M':
                self.sim_target_joint_angles_deg = angles_deg[:]
            elif cmd_char == 'G':
                self.sim_target_joint_angles_deg = angles_deg[:]
            elif cmd_char == 'H':
                self.sim_target_joint_angles_deg = [90.0] * 6
            elif cmd_char == 'K':
                self.logger.warn("Sim: Motors Killed (Relaxed).")
            elif cmd_char == 'E':
                self.logger.warn("Sim E-Stop Released!")
                self.emergency_stop_active = False
            elif cmd_char == 'N':
                self.servos_min = angles_deg[:]
            elif cmd_char == 'X':
                self.servos_max = angles_deg[:]
            elif cmd_char == 'T':
                self.collision_threshold = speed_factor
            elif cmd_char == 'D':
                self.collision_deviation_threshold = speed_factor
            elif cmd_char == 'F':
                self.fan_speed = int(angles_deg[0])
            elif cmd_char == 'O':
                enabled = (speed_factor == 1.0)
                self.collision_detection_enabled = enabled
                state_str = "ENABLED" if enabled else "DISABLED"
                self.logger.warn(f"Sim: Collision Safety {state_str}")
            
            if cmd_char not in ['M', 'G', 'H']:
                self.on_state_update(self.get_state_dict(), immediate=True)
        
        return True
