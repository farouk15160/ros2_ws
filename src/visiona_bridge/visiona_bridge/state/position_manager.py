"""
Position management for robot arm.

Handles saving, loading, and managing named robot positions.
"""

import json
import os
import threading
from typing import Dict, List, Callable, Optional


class PositionManager:
    """
    Manages named positions for the robot arm.
    
    Positions are saved joint configurations that can be recalled by name.
    """
    
    def __init__(self, logger, send_command: Callable, get_current_state: Callable,
                 socketio=None):
        """
        Initialize position manager.
        
        Args:
            logger: ROS logger instance
            send_command: Function to send commands to robot
            get_current_state: Function to get current robot state
            socketio: SocketIO instance for GUI updates
        """
        self.logger = logger
        self.send_command = send_command
        self.get_current_state = get_current_state
        self.socketio = socketio
        
        self.position_lock = threading.Lock()
        self.saved_positions: Dict[str, Dict] = {}
    
    def save(self, name: str):
        """Save current robot position with a name."""
        if not name or not isinstance(name, str) or not name.strip():
            self.logger.error("Invalid name for saving position.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': 'Invalid position name.'})
            return
        
        name = name.strip()
        state = self.get_current_state()
        angles = state.get('joint_angles')
        
        if not angles or len(angles) != 6:
            self.logger.error("Cannot save position: Invalid current angles.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': 'Cannot save pose: Invalid current angles.'})
            return
        
        with self.position_lock:
            self.saved_positions[name] = {"angles": angles[:]}
            self.logger.info(f"Saved position '{name}'.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'success',
                    'message': f"Position '{name}' saved."})
    
    def go_to(self, name: str):
        """Move robot to a saved position."""
        with self.position_lock:
            position_data = self.saved_positions.get(name)
        
        if not position_data or "angles" not in position_data:
            self.logger.error(f"Saved position '{name}' not found.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': f"Position '{name}' not found."})
            return
        
        state = self.get_current_state()
        if state.get('emergency_stop'):
            self.logger.error(f"Cannot go to position '{name}': E-Stop Active.")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': 'Cannot move: E-Stop Active!'})
            return
        
        self.logger.info(f"Moving to saved position '{name}'...")
        if self.socketio:
            self.socketio.emit('log_message', {'level': 'info',
                'message': f"Moving to '{name}'..."})
        
        speed = state.get('speed_factor', 150.0)
        self.send_command(ord('M'), position_data["angles"], speed, 0.0)
    
    def delete(self, name: str):
        """Delete a saved position."""
        with self.position_lock:
            if name in self.saved_positions:
                del self.saved_positions[name]
                self.logger.info(f"Deleted saved position '{name}'.")
                if self.socketio:
                    self.socketio.emit('log_message', {'level': 'warn',
                        'message': f"Position '{name}' deleted."})
            else:
                self.logger.warn(f"Saved position '{name}' not found for deletion.")
                if self.socketio:
                    self.socketio.emit('log_message', {'level': 'error',
                        'message': f"Position '{name}' not found."})
    
    def get_position_names(self) -> List[str]:
        """Get list of saved position names."""
        with self.position_lock:
            return list(self.saved_positions.keys())
    
    def save_to_file(self, filename: str):
        """Save all positions to JSON file."""
        filename = filename or "saved_positions.json"
        
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
            with self.position_lock, open(filepath, 'w') as f:
                json.dump(self.saved_positions, f, indent=4)
            self.logger.debug(f"Saved positions to {filepath}")
        except IOError as e:
            self.logger.error(f"Error saving positions to {filepath}: {e}")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': f'Error saving positions: {e}'})
    
    def load_from_file(self, filename: str):
        """Load positions from JSON file."""
        filename = filename or "saved_positions.json"
        
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
            if not os.path.exists(filepath):
                self.logger.warn(f"Positions file {filepath} not found. Starting with empty positions.")
                with self.position_lock:
                    self.saved_positions = {}
                return
            
            with open(filepath, 'r') as f:
                loaded_positions = json.load(f)
            
            # Validate positions format
            if self._validate_positions(loaded_positions):
                with self.position_lock:
                    self.saved_positions = loaded_positions
                self.logger.info(f"Loaded {len(self.saved_positions)} positions from {filepath}")
            else:
                raise ValueError("Invalid positions file format or content")
                
        except (IOError, json.JSONDecodeError, ValueError) as e:
            self.logger.error(f"Error loading positions from {filepath}: {e}")
            if self.socketio:
                self.socketio.emit('log_message', {'level': 'error',
                    'message': f'Error loading positions file {filename}: {e}'})
            with self.position_lock:
                self.saved_positions = {}
    
    def _validate_positions(self, positions: Dict) -> bool:
        """Validate positions data structure."""
        return isinstance(positions, dict) and all(
            isinstance(p, dict) and
            'angles' in p and isinstance(p['angles'], list) and len(p['angles']) == 6
            for p in positions.values()
        )
