"""
Sequence management for robot arm.

Handles recording, playback, and persistence of movement sequences.
"""

import json
import os
import threading
import time
from typing import List, Dict, Optional, Callable


class SequenceManager:
    """
    Manages sequence recording and playback for the robot arm.
    
    Sequences are lists of waypoints with joint angles, speeds, and delays.
    """
    
    def __init__(self, logger, get_clock: Callable, send_command: Callable,
                 get_current_state: Callable, socketio=None):
        """
        Initialize sequence manager.
        
        Args:
            logger: ROS logger instance
            get_clock: Function to get current ROS time
            send_command: Function to send commands to robot
            get_current_state: Function to get current robot state
            socketio: SocketIO instance for GUI updates
        """
        self.logger = logger
        self.get_clock = get_clock
        self.send_command = send_command
        self.get_current_state = get_current_state
        self.socketio = socketio
        
        self.sequence_lock = threading.Lock()
        self.sequence: List[Dict] = []
        
        self.playback_thread: Optional[threading.Thread] = None
        self.stop_playback_flag = threading.Event()
    
    def add_point(self, delay_ms: int = 500):
        """Add current robot position to sequence."""
        try:
            delay_ms = int(delay_ms)
        except ValueError:
            delay_ms = 500
            self.logger.warn("Invalid delay value, using default 500ms.")
        
        state = self.get_current_state()
        angles = state.get('joint_angles')
        speed = state.get('speed_factor', 150.0)
        
        if not angles or len(angles) != 6:
            self.logger.error("Cannot add sequence point: Invalid current angles.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error', 
                    'message': 'Cannot save pose: Invalid current angles.'})
            return
        
        with self.sequence_lock:
            point = {"angles": angles[:], "speed": speed, "delay_ms": delay_ms}
            self.sequence.append(point)
            self.logger.info(f"Added sequence point {len(self.sequence)}.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'info',
                    'message': f'Pose added to sequence (point {len(self.sequence)}).'})
    
    def clear(self):
        """Clear all sequence points."""
        with self.sequence_lock:
            self.sequence.clear()
            self.logger.info("Sequence cleared.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'warn', 'message': 'Sequence cleared.'})
    
    def delete_point(self, index: int):
        """Delete a sequence point by index."""
        try:
            index = int(index)
            with self.sequence_lock:
                if 0 <= index < len(self.sequence):
                    del self.sequence[index]
                    self.logger.info(f"Deleted sequence point {index+1}.")
                    if self.socketio:
                        self.socketio.emit('log_message', {'level': 'info',
                            'message': f'Sequence point {index+1} deleted.'})
                else:
                    raise IndexError("Index out of bounds")
        except (ValueError, IndexError, TypeError) as e:
            self.logger.error(f"Invalid index for deleting sequence point: {index} ({e})")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': f'Invalid sequence index: {index+1 if isinstance(index, int) else index}'})
    
    def get_sequence(self) -> List[Dict]:
        """Get a copy of the current sequence."""
        with self.sequence_lock:
            return self.sequence[:]
    
    def save_to_file(self, filename: str):
        """Save sequence to JSON file."""
        filename = filename or "current_sequence.json"
        
        # Check if absolute path (e.g. from bridge_constants)
        if os.path.isabs(filename):
            filepath = filename
        else:
            # Sanitize relative filename
            filename = "".join(c if c.isalnum() or c in ('_', '.', '-') else '_' for c in filename)
            if not filename.endswith(".json"):
                filename += ".json"
            filepath = os.path.join(os.getcwd(), filename)
        
        try:
            with self.sequence_lock, open(filepath, 'w') as f:
                json.dump(self.sequence, f, indent=4)
            self.logger.info(f"Sequence saved to {filepath}")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'success',
                    'message': f'Sequence saved to {filename}.'})
        except IOError as e:
            self.logger.error(f"Error saving sequence to {filepath}: {e}")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': f'Error saving sequence: {e}'})
    
    def load_from_file(self, filename: str):
        """Load sequence from JSON file."""
        filename = filename or "current_sequence.json"
        
        # Check if absolute path (e.g. from bridge_constants)
        if os.path.isabs(filename):
            filepath = filename
        else:
            # Sanitize relative filename
            filename = "".join(c if c.isalnum() or c in ('_', '.', '-') else '_' for c in filename)
            if not filename.endswith(".json"):
                filename += ".json"
            filepath = os.path.join(os.getcwd(), filename)
        
        try:
            with open(filepath, 'r') as f:
                loaded_sequence = json.load(f)
            
            # Validate sequence format
            if self._validate_sequence(loaded_sequence):
                with self.sequence_lock:
                    self.sequence = loaded_sequence
                self.logger.info(f"Sequence loaded from {filepath} ({len(self.sequence)} points)")
                if self.socketio:
                    self.socketio.emit('log_message', {'level': 'success',
                        'message': f'Sequence loaded from {filename}.'})
            else:
                raise ValueError("Invalid sequence file format or content")
                
        except FileNotFoundError:
            self.logger.warn(f"Sequence file {filepath} not found. Starting with empty sequence.")
            with self.sequence_lock:
                self.sequence = []
        except (IOError, json.JSONDecodeError, ValueError) as e:
            self.logger.error(f"Error loading sequence from {filepath}: {e}")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': f'Error loading sequence file {filename}: {e}'})
            with self.sequence_lock:
                self.sequence = []
    
    def _validate_sequence(self, seq: List) -> bool:
        """Validate sequence data structure."""
        return isinstance(seq, list) and all(
            isinstance(p, dict) and
            'angles' in p and isinstance(p['angles'], list) and len(p['angles']) == 6 and
            'speed' in p and isinstance(p['speed'], (int, float)) and
            'delay_ms' in p and isinstance(p['delay_ms'], int)
            for p in seq
        )
    
    def play(self):
        """Start sequence playback."""
        if self.playback_thread and self.playback_thread.is_alive():
            self.logger.warn("Sequence playback already in progress.")
            return
        
        state = self.get_current_state()
        if state.get('emergency_stop'):
            self.logger.error("Cannot play sequence while E-Stop is active.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': 'Cannot play: E-Stop Active!'})
            return
        
        with self.sequence_lock:
            if not self.sequence:
                self.logger.warn("Sequence is empty, nothing to play.")
                if self.socketio:
                    self.socketio.emit('log_message', {'level': 'warn',
                        'message': 'Sequence is empty.'})
                return
        
        self.stop_playback_flag.clear()
        self.playback_thread = threading.Thread(target=self._playback_worker, daemon=True)
        self.playback_thread.start()
        self.logger.info("Starting sequence playback...")
        if self.socketio:
            self.socketio.emit('log_message', {'level': 'info',
                'message': 'Sequence playback started.'})
    
    def stop(self):
        """Stop sequence playback."""
        if self.playback_thread and self.playback_thread.is_alive():
            self.logger.info("Stopping sequence playback...")
            self.stop_playback_flag.set()
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'warn',
                    'message': 'Sequence playback stopping...'})
        else:
            self.logger.info("No active sequence playback to stop.")
        
        if self.socketio:
            self.socketio.emit('playback_stopped')
    
    def _playback_worker(self):
        """Worker thread for sequence playback."""
        with self.sequence_lock:
            sequence_copy = self.sequence[:]
        
        sequence_len = len(sequence_copy)
        
        for i, point in enumerate(sequence_copy):
            current_step = i + 1
            
            if self.stop_playback_flag.is_set():
                self.logger.info(f"Playback stopped by user at step {current_step}.")
                break
            
            state = self.get_current_state()
            if state.get('emergency_stop'):
                self.logger.error(f"E-Stop activated during playback at step {current_step}. Stopping sequence.")
                if self.socketio:
                    self.socketio.emit('log_message', {'level': 'error',
                        'message': f'E-Stop during playback step {current_step}!'})
                break
            
            angles = point.get("angles")
            speed = point.get("speed", 150.0)
            delay_ms = point.get("delay_ms", 0)
            
            if not angles or len(angles) != 6:
                self.logger.error(f"Invalid angles found at sequence point {current_step}. Skipping.")
                continue
            
            self.logger.info(f"Moving to sequence point {current_step}/{sequence_len}...")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'info',
                    'message': f'Moving to step {current_step}/{sequence_len}...'})
            
            # Send command
            if not self.send_command(ord('M'), angles, speed, 0.0):
                self.logger.error(f"Failed to send command during sequence playback (step {current_step}). Stopping.")
                break
            
            # Wait for movement + delay
            # Simplified wait - in real implementation would check if target reached
            time.sleep((delay_ms + 500) / 1000.0)
        
        self.stop_playback_flag.clear()
        self.logger.info("Sequence playback finished.")
        if self.socketio:
            self.socketio.emit('log_message', {'level': 'info', 'message': 'Sequence playback finished.'})
            self.socketio.emit('playback_stopped')
    
    def is_playing(self) -> bool:
        """Check if sequence is currently playing."""
        return self.playback_thread and self.playback_thread.is_alive()
