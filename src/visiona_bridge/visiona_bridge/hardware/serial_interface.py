"""
Serial interface for robot communication.

Manages serial port connection, reading, and writing with robust error handling.
"""

import serial
import threading
import time
from typing import Optional, Callable
from .packet_protocol import PacketProtocol, HEADER_BYTE, STATUS_PACKET_ID, STATUS_PACKET_SIZE
from .packet_protocol import CONFIG_PACKET_ID, CONFIG_PACKET_SIZE


class SerialInterface:
    """
    Manages serial port communication with the robot MCU.
    
    Handles connection management, reconnection logic, and robust packet reading.
    """
    
    def __init__(self, port: str, baudrate: int, logger, 
                 on_status_packet: Callable, on_config_packet: Callable,
                 on_connection_changed: Callable):
        """
        Initialize serial interface.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            baudrate: Baud rate for serial communication
            logger: ROS logger instance
            on_status_packet: Callback for status packets
            on_config_packet: Callback for config packets
            on_connection_changed: Callback when connection state changes
        """
        self.port = port
        self.baudrate = baudrate
        self.logger = logger
        self.on_status_packet = on_status_packet
        self.on_config_packet = on_config_packet
        self.on_connection_changed = on_connection_changed
        
        self.ser: Optional[serial.Serial] = None
        self.is_connected = False
        self.should_stop = False
        self.reconnect_event = threading.Event()
        
        # Start connection manager thread
        self.connection_thread = threading.Thread(
            target=self._connection_worker, daemon=True
        )
        self.connection_thread.start()
    
    def _connection_worker(self):
        """Worker thread that manages connection and reading."""
        while not self.should_stop:
            if self.ser is None or not self.ser.is_open:
                self._set_connected(False)
                self._attempt_connection()
            
            if self.ser and self.ser.is_open:
                self._read_packets()
            else:
                time.sleep(1.0)
    
    def _attempt_connection(self):
        """Attempt to open serial connection."""
        try:
            self.logger.info(f"Attempting connection to {self.port}...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self._set_connected(True)
            self.logger.info(f"Successfully connected to {self.port}")
            self.reconnect_event.clear()
            
            # Clear input buffer and request config
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            
        except serial.SerialException as e:
            self.logger.warn(
                f"Connection failed: {e}. Waiting to retry...",
                throttle_duration_sec=5
            )
            self.reconnect_event.wait(timeout=5.0)
            if self.reconnect_event.is_set():
                self.logger.info("Reconnect triggered.")
                self.reconnect_event.clear()
    
    def _read_packets(self):
        """Read and process packets from serial port."""
        read_buffer = bytearray()
        
        while not self.should_stop and self.ser and self.ser.is_open:
            try:
                # Read available bytes
                bytes_to_read = self.ser.in_waiting
                if bytes_to_read > 0:
                    read_buffer.extend(self.ser.read(bytes_to_read))
                elif not read_buffer:
                    time.sleep(0.005)
                    continue
                
                # Process packets in buffer
                processed_bytes = self._process_buffer(read_buffer)
                if processed_bytes > 0:
                    read_buffer = read_buffer[processed_bytes:]
                    
            except (serial.SerialException, OSError) as e:
                self.logger.error(f"Serial error: {e}. Closing connection.")
                self._close_port()
                break
            except Exception as e:
                self.logger.error(
                    f"Unexpected error in read loop: {type(e).__name__} - {e}",
                    exc_info=True
                )
                read_buffer.clear()
                time.sleep(0.1)
    
    def _process_buffer(self, buffer: bytearray) -> int:
        """
        Process packets in the read buffer.
        
        Returns:
            Number of bytes processed
        """
        processed_bytes = 0
        
        while len(buffer) - processed_bytes >= 2:
            # Find header
            header_index = buffer.find(HEADER_BYTE, processed_bytes)
            if header_index == -1:
                return len(buffer)
            
            if header_index > processed_bytes:
                self.logger.warn(
                    f"Discarding {header_index - processed_bytes} bytes before header.",
                    throttle_duration_sec=2
                )
                processed_bytes = header_index
            
            # Check if we have enough bytes for packet type
            if len(buffer) - processed_bytes < 2:
                break
            
            packet_id = buffer[processed_bytes + 1]
            packet_size, callback = self._get_packet_info(packet_id)
            
            if packet_size is None:
                self.logger.warn(
                    f"Unknown packet ID: {hex(packet_id)}. Discarding header byte.",
                    throttle_duration_sec=1
                )
                processed_bytes += 1
                continue
            
            # Check if we have complete packet
            if len(buffer) - processed_bytes < packet_size:
                break
            
            # Extract and verify packet
            packet_data = buffer[processed_bytes:processed_bytes + packet_size]
            if self._verify_and_process_packet(packet_data, callback):
                processed_bytes += packet_size
            else:
                processed_bytes += 1  # Skip bad packet header
        
        return processed_bytes
    
    def _get_packet_info(self, packet_id: int):
        """Get packet size and callback for packet ID."""
        if packet_id == STATUS_PACKET_ID:
            return STATUS_PACKET_SIZE, self.on_status_packet
        elif packet_id == CONFIG_PACKET_ID:
            return CONFIG_PACKET_SIZE, self.on_config_packet
        return None, None
    
    def _verify_and_process_packet(self, packet_data: bytes, callback: Callable) -> bool:
        """Verify checksum and process packet."""
        received_checksum = packet_data[-1]
        calculated_checksum = PacketProtocol.calculate_checksum(packet_data[:-1])
        
        if received_checksum == calculated_checksum:
            try:
                callback(packet_data)
                return True
            except Exception as e:
                self.logger.error(f"Error processing packet: {e}")
                return False
        else:
            self.logger.warn(
                f"Checksum mismatch! Discarding packet.",
                throttle_duration_sec=1
            )
            return False
    
    def write_packet(self, packet_data: bytes) -> bool:
        """
        Write a packet to the serial port.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected or not self.ser or not self.ser.is_open:
            self.logger.warn(
                "Serial not connected. Cannot send command.",
                throttle_duration_sec=2
            )
            return False
        
        try:
            self.ser.write(packet_data)
            self.ser.flush()
            return True
        except (serial.SerialException, OSError, BrokenPipeError) as e:
            self.logger.error(f"Serial write error: {e}. Closing connection.")
            self._close_port()
            return False
        except Exception as e:
            self.logger.error(f"Error sending packet: {e}")
            return False
    
    def trigger_reconnect(self):
        """Manually trigger a reconnection attempt."""
        self.logger.info("Manual reconnect requested.")
        self._close_port()
        self.reconnect_event.set()
    
    def _close_port(self):
        """Close the serial port."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as e:
                self.logger.error(f"Error closing serial port: {e}")
        self.ser = None
        self._set_connected(False)
    
    def _set_connected(self, connected: bool):
        """Update connection state and notify."""
        if self.is_connected != connected:
            self.is_connected = connected
            self.on_connection_changed(connected)
    
    def shutdown(self):
        """Shutdown the serial interface."""
        self.logger.info("Shutting down serial interface...")
        self.should_stop = True
        self._close_port()
        if self.connection_thread.is_alive():
            self.connection_thread.join(timeout=1.0)
