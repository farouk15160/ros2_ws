# Visiona Bridge - File Organization

## 📁 Directory Structure

```
visiona_bridge/
├── 📂 hardware/              # Layer 1: Hardware Communication
│   ├── __init__.py
│   ├── packet_protocol.py   # Packet encoding/decoding
│   ├── serial_interface.py  # Serial port management
│   └── robot_hardware.py    # Hardware state & simulation
│
├── 📂 ros2_interface/        # Layer 2: ROS2 Integration
│   ├── __init__.py
│   ├── publishers.py        # ROS2 publishers
│   ├── subscribers.py       # ROS2 subscribers
│   ├── services.py          # ROS2 services
│   └── cartesian_interface.py # MoveIt integration
│
├── 📂 gui/                   # Layer 3: Web GUI
│   ├── __init__.py
│   ├── web_app.py           # Flask app factory
│   ├── routes.py            # API routes
│   └── socketio_handlers.py # Real-time communication
│
├── 📂 state/                 # State Management
│   ├── __init__.py
│   ├── sequence_manager.py  # Sequence recording/playback
│   └── position_manager.py  # Named position storage
│
├── 📂 static/                # Web GUI Assets (CSS, JS, 3D models)
├── 📂 templates/             # HTML templates
│
├── 📄 bridge_node.py         # Main ROS2 orchestrator node
├── 📄 web_gui_node.py        # Entry point (main)
├── 📄 bridge_constants.py    # Shared constants
├── 📄 __init__.py            # Package initialization
│
├── 💾 current_sequence.json  # Runtime: Current sequence data
├── 💾 saved_positions.json   # Runtime: Saved positions data
└── 📝 instructions.txt       # Documentation (legacy)
```

## 📝 File Categories

### Core Files (Root Level)
- `bridge_node.py` - Main orchestrator coordinating all layers
- `web_gui_node.py` - Entry point that launches ROS2 + Flask
- `bridge_constants.py` - Shared constants across modules
- `__init__.py` - Package initialization

### Module Folders
- `hardware/` - All hardware communication code
- `ros2_interface/` - All ROS2 publishers/subscribers/services
- `gui/` - All Flask/SocketIO web interface code
- `state/` - Sequence and position management

### Assets
- `static/` - Web GUI resources (JavaScript, CSS, 3D models)
- `templates/` - HTML templates for web interface

### Runtime Data
- `current_sequence.json` - Saved sequence data (auto-generated)
- `saved_positions.json` - Saved positions (auto-generated)

### Legacy/Optional
- `instructions.txt` - Old documentation file (can be deleted if not needed)
- `__pycache__/` - Python bytecode cache (auto-generated)

## ✨ Clean Organization Benefits

1. **Clear Layering**: Each folder represents a distinct architectural layer
2. **Easy Navigation**: Find code by its responsibility
3. **Scalability**: Easy to add new modules within appropriate folders
4. **Testability**: Test each layer independently
5. **Maintainability**: Small, focused files in logical groupings

## 🔄 Import Examples

```python
# Import from hardware layer
from visiona_bridge.hardware import PacketProtocol, SerialInterface

# Import from ROS2 layer
from visiona_bridge.ros2_interface.publishers import RobotPublishers

# Import from GUI layer
from visiona_bridge.gui import create_app, socketio

# Import from state management
from visiona_bridge.state import SequenceManager, PositionManager
```

## 📊 File Count by Category

| Category | Files | Lines |
|----------|-------|-------|
| Hardware | 4 | 636 |
| ROS2 Interface | 5 | 500 |
| GUI | 4 | 357 |
| State Management | 3 | 451 |
| Core | 4 | 464 |
| **Total** | **20** | **~2,408** |

**Average: 120 lines per file** ✅
