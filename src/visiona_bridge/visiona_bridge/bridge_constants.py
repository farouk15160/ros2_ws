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
DEFAULT_POSITIONS_FILE = "saved_positions.json"
DEFAULT_SEQUENCE_FILE = "current_sequence.json"

# Suppress noisy Flask/SocketIO logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)