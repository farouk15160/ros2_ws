import struct
import logging

# ==============================================================================
# 1. CONSTANTS AND PACKET DEFINITIONS (VERSION 3.9.1 COMPATIBLE)
# ==============================================================================
HEADER_BYTE = 0xA5
COMMAND_PACKET_PAYLOAD_FORMAT = '<BB8f'
STATUS_PACKET_ID = ord('S')
STATUS_PACKET_FORMAT = '<BBff6fBB'
STATUS_PACKET_SIZE = struct.calcsize(STATUS_PACKET_FORMAT)
CONFIG_PACKET_ID = ord('R')
CONFIG_PACKET_FORMAT = '<BB6f6fffB'
CONFIG_PACKET_SIZE = struct.calcsize(CONFIG_PACKET_FORMAT)
STATUS_TIMEOUT_SEC = 0.5

# --- NEW V4.0: Default filenames for persistence ---
# Save data files in user's home directory - paths resolved at runtime only
def get_data_dir():
    """Get data directory at runtime (never at build time)."""
    import os
    data_dir = os.path.expanduser("~/.visiona_bridge")
    os.makedirs(data_dir, exist_ok=True)
    return data_dir

def get_config_dir():
    """URAF config store directory (~/.visiona_bridge)."""
    return get_data_dir()

def get_positions_file():
    """Get positions file path at runtime."""
    import os
    return os.path.join(get_data_dir(), "saved_positions.json")

def get_sequence_file():
    """Get sequence file path at runtime."""
    import os
    return os.path.join(get_data_dir(), "current_sequence.json")

# --- V5.0: MCU Communication Logging ---
MCU_STATUS_PRINT_INTERVAL_SEC = 2.0  # Print MCU status to terminal every N seconds

# Suppress noisy Flask/SocketIO logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)