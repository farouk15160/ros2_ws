# Visiona Robotics Studio

> **Universal Robot Arm Foundation (URAF) on Visiona** — A 6-DOF desktop manipulator with ESP32 firmware, ROS 2 Humble control stack, custom analytical IK, OctoMap obstacle avoidance, RGB-D perception, and the **JARVIS** natural-language AI pipeline. Controlled through a premium web GUI inspired by industrial HMIs, Isaac Sim, and Apple Vision Pro design language.

**Version:** 6.0.0  
**ROS distro:** Humble (Ubuntu 22.04)  
**Primary package:** `visiona_bridge`

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [System Architecture](#2-system-architecture)
3. [Hardware](#3-hardware)
4. [Firmware (ESP32)](#4-firmware-esp32)
5. [Software Stack](#5-software-stack)
6. [URAF Foundation (Phase 0–1)](#6-uraf-foundation-phase-01)
7. [Motion & Kinematics](#7-motion--kinematics)
8. [Perception & Mapping](#8-perception--mapping)
9. [JARVIS AI Pipeline](#9-jarvis-ai-pipeline)
10. [Web GUI — Robotics Studio](#10-web-gui--robotics-studio)
11. [Launch System](#11-launch-system)
12. [ROS 2 Topics & Services](#12-ros-2-topics--services)
13. [Configuration Reference](#13-configuration-reference)
14. [Build & Run](#14-build--run)
15. [Troubleshooting](#15-troubleshooting)
16. [Project Roadmap (PLAN.md)](#16-project-roadmap-planmd)

---

## 1. Executive Summary

Visiona is a **serial 6-DOF desktop robot arm** controlled from a host PC over USB serial. The software stack delivers:

| Capability | Technology |
|---|---|
| Real-time MCU control | ESP32 + FreeRTOS, 921600 baud packet protocol |
| ROS 2 control | `ros2_control`, joint trajectory controller |
| Cartesian motion | Custom DLS IK solver + minimum-jerk trajectories |
| Obstacle avoidance | OctoMap voxel collision checks in IK path |
| Perception | Orbbec Astra RGB-D + MobileSAM + Ollama VLM |
| Natural language tasks | JARVIS: Ollama planner → action executor |
| Robot setup | URAF hardware discovery + health monitor |
| Operator UI | Flask + Socket.IO **Robotics Studio** (dark glassmorphism) |

**One-command launch:**

```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real camera:=true mapping:=high jarvis:=true
```

Open **http://localhost:5000** for the full control studio.

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     VISIONA ROBOTICS STUDIO (Web GUI)                    │
│  Top bar · Sidebar · 3D Viewport · Control · Sequencer · JARVIS · Setup │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │ Flask REST + Socket.IO
┌───────────────────────────────▼─────────────────────────────────────────┐
│                         visiona_bridge (V6)                              │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐  ┌───────────────┐  │
│  │  Hardware   │  │  ROS2 I/F    │  │  State Mgr  │  │  URAF Agents  │  │
│  │  Serial/FK  │  │  Pub/Sub/Svc │  │  Seq/Pos    │  │  Disc/Health  │  │
│  └──────┬──────┘  └──────┬───────┘  └─────────────┘  └───────────────┘  │
└─────────┼────────────────┼──────────────────────────────────────────────┘
          │ UART 921600    │ ROS 2 topics
┌─────────▼────────┐  ┌───▼──────────────────────────────────────────────┐
│  ESP32 Firmware  │  │  simple_ik_solver · colored_map · JARVIS nodes   │
│  PCA9685 servos  │  │  octomap_server · rtabmap · ascamera (optional)  │
│  A4988 stepper   │  └──────────────────────────────────────────────────┘
└──────────────────┘
```

### JARVIS data flow

```
User (GUI or topic)
       │
       ▼ /jarvis/command
┌──────────────────┐     ┌─────────────────────┐
│ jarvis_llm_planner│◄────│ jarvis_world_model  │
│ (Ollama mistral)  │     │ (object registry)   │
└────────┬─────────┘     └──────────▲──────────┘
         │ /jarvis/action_plan       │ /jarvis/pose_status
         ▼                           │
┌──────────────────┐     ┌───────────┴───────────┐
│ jarvis_action_   │     │ detect → segment →   │
│ executor         │     │ pose (VLM + SAM)     │
└────────┬─────────┘     └──────────────────────┘
         │ HybridMotionPlanner (auto / moveit / simple_ik)
         ▼
   simple_ik_solver OR move_group → joint_trajectory_controller → bridge → MCU
```

---

## 3. Hardware

### 3.1 Mechanical & kinematic chain

| Segment | Length | Joint |
|---|---|---|
| Base column | 140 mm | J0 (stepper, base rotation) |
| Upper arm | 185 mm | J1 (shoulder servo) |
| Forearm | 119 mm | J2 (elbow servo) |
| Wrist + gripper | 250 mm | J3–J5 (servos) |

**Max reach:** ~690 mm

### 3.2 Actuators

| Joint | Type | Notes |
|---|---|---|
| J0 | NEMA17 stepper + A4988 | 5:1 gear, 16× microstepping |
| J1–J5 | PWM hobby servos | 500–2500 µs via PCA9685 |

### 3.3 Sensing & electronics

| Component | Purpose |
|---|---|
| PCA9685 (I2C 0x40) | 16-ch PWM driver |
| ACS712 (GPIO 34) | Main arm current (±5 A) |
| INA219 (I2C 0x41) | Gripper current |
| Orbbec Astra / HP60C | RGB-D camera (optional) |

### 3.4 Serial connection

- **Port:** `/dev/ttyUSB0` (default)
- **Baud:** `921600`
- **Protocol:** 0xA5 header + XOR checksum (36-byte command, 37-byte status)

---

## 4. Firmware (ESP32)

**Location:** `src/visiona_firmware/firmware/`  
**Build:** PlatformIO

```bash
cd src/visiona_firmware/firmware
pio run
pio run --target upload
```

### FreeRTOS tasks

| Task | Core | Rate | Role |
|---|---|---|---|
| `task_command_parser` | 0 | event | UART decode, command dispatch |
| `task_motion_interpolator` | 1 | 100 Hz | IIR servo smoothing, stepper steps |
| `task_monitoring` | 0 | 20 Hz | Current sensing, collision detect |

### Command reference

| ID | Command | Description |
|---|---|---|
| `M` | Move | Joint angles + speed |
| `H` | Home | Default home pose |
| `G` | Gripper | Angle + current limit |
| `E` | E-stop release | Re-enable motors |
| `K` | Kill | Safe power-off |
| `O` | Collision | Enable/disable safety |
| `R` | Read config | Returns limits + thresholds |

---

## 5. Software Stack

### 5.1 Workspace packages

| Package | Role |
|---|---|
| `visiona_bridge` | Main stack: bridge, IK, GUI, JARVIS, URAF |
| `visiona_firmware` | ESP32 PlatformIO project |
| `visiona_moveit_config` | MoveIt 2 SRDF, kinematics, controllers |
| `visiona_vision` | Camera driver (`ascamera`), SLAM, tracking |

### 5.2 `visiona_bridge` module layout

```
visiona_bridge/
├── bridge_node.py           # Main orchestrator (V6)
├── web_gui_node.py          # Flask entry point
├── simple_ik_solver.py      # DLS IK + min-jerk + OctoMap
├── colored_map_node.py      # RGB voxel map accumulator
├── hardware/                # Serial, packets, sim mode
├── ros2_interface/          # Publishers, subscribers, Cartesian jog
├── state/                   # Sequences, saved positions
├── gui/                     # Flask routes, SocketIO, JARVIS bridge
├── perception/              # VLM detector, SAM, pose estimator
├── world_model/             # Object registry with TTL
├── llm/                     # Ollama planner + action executor
├── visual_servoing/         # Drift correction during execution
└── uraf/                    # Discovery, health, config store, agent bus
```

---

## 6. URAF Foundation (Phases 0–6 + §17–21)

Aligned with **PLAN.md** Universal Robot Arm Foundation. Integrated into `visiona_bridge` (not a separate workspace yet).

### 6.1 Components

| Agent | Node | Topic | Purpose |
|---|---|---|---|
| Hardware Discovery | `uraf_hardware_discovery` | `/uraf/hardware_profile` | Scan serial ports + ROS ecosystem |
| Self-Healing | `uraf_self_healing` | `/uraf/health`, `/uraf/recovery/status` | Health monitoring + auto-recovery |
| Learning | `uraf_learning_agent` | `/uraf/learning/stats` | Task/grasp/planner learning |
| Multi-Robot | `uraf_multi_robot_coordinator` | `/uraf/multi_robot/status` | Namespace + priority coordination |
| URDF Generator | `uraf_urdf_generator` | `/uraf/urdf_generated` | Auto URDF/SRDF from profile + DH params |
| Digital Twin | `uraf_digital_twin` | `/uraf/twin/state` | Pre-execution validation + state mirror |
| Plugin Manager | `uraf_plugin_manager` | `/uraf/plugins/status` | Plugin registry and dynamic loading |
| Community Library | `uraf_community_library` | `/uraf/community/catalog` | Robot profile fingerprinting + lookup |
| Safety Monitor | `uraf_safety_monitor` | `/uraf/safety/status` | SF-01–11 enforcement + safe startup |
| Agent Bus | (in-process) | `/uraf/agent_bus` | JSON agent messages (PLAN §6.2.2) |
| Config Store | (file-based) | — | `~/.visiona_bridge/uraf_config.yaml` |

### 6.2 Trigger discovery

**GUI:** Setup Wizard tab → **Scan Hardware**

**API:**

```bash
curl -X POST http://localhost:5000/api/uraf/discovery
```

**Topic:**

```bash
ros2 topic pub --once /uraf/discovery/trigger std_msgs/String "data: 'run'"
ros2 topic echo /uraf/hardware_profile
```

### 6.2.1 Generate URDF (Phase 2)

**GUI:** Setup Wizard → generate URDF (or API below)

**API:**

```bash
curl -X POST http://localhost:5000/api/uraf/generate_urdf
```

**Output:** `~/.visiona_bridge/generated/visiona_v1.urdf.xacro` and matching `.srdf`

**Topic:**

```bash
ros2 topic pub --once /uraf/generate_urdf std_msgs/String "data: 'generate'"
ros2 topic echo /uraf/urdf_generated
```

### 6.2.2 Self-Healing (Phase 6)

Monitors joint states, serial connection, e-stop, and JARVIS motion failures. Auto-recovery actions:

| Failure | Recovery |
|---|---|
| Stale joint states | Trigger discovery → republish home |
| Serial disconnect | Trigger discovery + user alert |
| Motion timeout | Republish home + alert |
| Perception degraded | Trigger scene rescan |

**Topics:** `/uraf/health`, `/uraf/recovery/status`, `/visiona/system_status`

**API:** `GET /api/uraf/recovery`

### 6.2.3 Learning Agent (Phase 6)

Records JARVIS executions to `~/.visiona_bridge/learning/`:

- `executions.jsonl` — step-by-step events
- `grasp_success_model.json` — per-object success rates
- `planner_performance.json` — Simple IK vs MoveIt stats

**API:** `GET /api/uraf/learning`  
**Topic:** `/jarvis/execution_event` → `/uraf/learning/stats`

### 6.2.4 Digital Twin (§18)

Mirrors physical robot state and validates motions **before** physical execution (PLAN pre-execution protocol).

| Backend | Launch | Use case |
|---|---|---|
| `bridge_sim` | `twin:=true` (default) | Software twin + workspace validation |
| `gazebo` | `mode:=gazebo twin:=true` | Full physics simulation |
| `rviz_only` | `viz:=rviz` | Visualization only |

**Validation checks:** workspace bounds, min radius, max reach (from `kinematic_params.yaml`)

**Topics:**
- `/uraf/twin/state` — live twin state
- `/visiona/twin/validate_motion` — validation request (JSON)
- `/uraf/twin/validation` — approval/rejection result

**API:** `GET /api/uraf/twin`, `POST /api/uraf/twin/validate`

```bash
# Software twin (no Gazebo)
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim twin:=true jarvis:=true

# Full Gazebo digital twin
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=gazebo twin:=true viz:=rviz
```

### 6.2.5 Plugin System (§20)

Extensible plugin registry at `~/.visiona_bridge/plugins/`.

| Type | Interface | Example |
|---|---|---|
| FAL Driver | `FALDriverPlugin` | `visiona_fal` (bundled) |
| Perception | `PerceptionPlugin` | Custom detectors |
| Task Primitive | `TaskPrimitivePlugin` | Custom JARVIS actions |

**CLI:**
```bash
ros2 run visiona_bridge uraf_plugin.py list
ros2 run visiona_bridge uraf_plugin.py install $(ros2 pkg prefix visiona_bridge)/share/visiona_bridge/plugins/visiona_fal/plugin.yaml
```

**API:** `GET /api/uraf/plugins`, `POST /api/uraf/plugins/install`

### 6.2.6 Community Robot Library (§21)

Bundled profiles in `community/` — fingerprint-matched against discovery results.

| Profile | Status |
|---|---|
| `visiona_v1` | verified |
| `generic_6dof_serial` | experimental |

**API:** `GET /api/uraf/community`, `POST /api/uraf/community/lookup`

### 6.2.7 Setup Wizard (§17)

7-step guided wizard in the **Setup** tab: Welcome → Hardware → Discovery → Confirm → Visualize → Calibrate → Complete.

**API:** `GET/POST /api/uraf/wizard`, `POST /api/uraf/wizard/reset`

### 6.2.8 Safety Architecture (§19)

Software safety layer with monitored safety functions SF-01 through SF-11, safe startup sequence, and JARVIS motion gating.

**Config:** `config/safety_params.yaml`

**Topics:** `/uraf/safety/status`, `/uraf/safety/motion_scale`, `/uraf/safety/startup`

**API:** `GET /api/uraf/safety`, `POST /api/uraf/safety/command` (`arm` | `disarm` | `startup`)

### 6.2.9 Testing (§23)

```bash
colcon test --packages-select visiona_bridge
colcon test-result --verbose
```

Unit tests: `test/test_safety_engine.py`, `test/test_uraf_components.py`

### 6.3 Master config

Default template: `config/uraf_config.yaml`  
User overrides: `~/.visiona_bridge/uraf_config.yaml`  
History snapshots: `~/.visiona_bridge/config_history/`

---

## 7. Motion & Kinematics

### 7.1 DH parameters (single source of truth)

File: `config/kinematic_params.yaml`

| Joint | a (m) | α (rad) | d (m) | θ offset |
|---|---|---|---|---|
| J0 | 0.000 | 1.5708 | 0.140 | 0.0 |
| J1 | 0.185 | 0.0 | 0.000 | 0.0 |
| J2 | 0.119 | 0.0 | 0.000 | 0.0 |
| J3 | 0.250 | 0.0 | −0.005 | 0.0 |

### 7.2 IK solver (DLS)

- **Algorithm:** Damped Least Squares, numerical Jacobian
- **Target:** 3D position (X, Y, Z) using J0–J3
- **Trajectory:** Minimum jerk polynomial at 50 Hz
- **Safety:** Workspace limits, self-collision, OctoMap clearance (10 cm)

### 7.3 Cartesian command

```bash
ros2 topic pub --once /visiona/cartesian_command geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.1, z: 0.25}, orientation: {w: 1.0}}}"
```

### 7.4 Direct joint command

```bash
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint','link_1_shoulder_joint','link_2_elbow_joint',
           'link_3_wrist_joint','link_3_wrist_to_gripper_base_joint','gripper_joint'],
    position: [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]}"
```

---

## 8. Perception & Mapping

Requires `camera:=true` at launch.

| Component | Output | Use |
|---|---|---|
| `ascamera` HP60C | RGB + depth point cloud | JARVIS vision, OctoMap input |
| `octomap_server` | `/octomap_point_cloud_centers` | IK obstacle avoidance |
| `rtabmap` | `/cloud_map` (colored SLAM) | Dense RGB map |
| `colored_map` node | `/colored_map` | Persistent voxel RGB map |

### Mapping quality

| Launch arg | Resolution | Use case |
|---|---|---|
| `mapping:=low` | ~2 cm | Fast obstacle avoidance |
| `mapping:=high` | ~1.5 cm | Precision manipulation |

---

## 9. JARVIS AI Pipeline

Requires **Ollama** and `jarvis:=true`.

### 9.1 Prerequisites

```bash
# Install Ollama: https://ollama.com
ollama pull mistral
ollama pull llava

# Optional: MobileSAM weights
# Default path: /home/farouk/mobile_sam.pt
```

### 9.2 Pipeline nodes

| Node | Model / Tech | Role |
|---|---|---|
| `jarvis_object_detector` | Ollama `llava` | VLM object detection (scan / vision commands only) |
| `jarvis_segmentation` | MobileSAM | Instance masks (bbox fallback if SAM missing) |
| `jarvis_pose_estimator` | Depth + TF | 3D poses in `world` frame |
| `jarvis_world_model` | — | Object registry, 15 s forget timeout |
| `jarvis_llm_planner` | Ollama `mistral` | NL → JSON action plan |
| `jarvis_action_executor` | — | pick / place / scan / home / gripper |
| `visual_servo_controller` | — | Drift correction during execution |

### 9.3 Action plan schema

```json
{
  "reasoning": "User wants to pick the red cup",
  "actions": [
    { "type": "scan" },
    { "type": "pick", "object": "red cup", "approach_z_offset": 0.05 },
    { "type": "place", "x": 0.2, "y": 0.0, "z": 0.08 },
    { "type": "home" }
  ]
}
```

### 9.4 Send commands

**GUI:** JARVIS tab → type command → Send

**Topic:**

```bash
ros2 topic pub --once /jarvis/command std_msgs/String \
  "data: 'Pick up the red cup'"
```

**Quick commands:** Scan · Home · Open/Close gripper · What do you see?

### 9.5 Key JARVIS topics

| Topic | Type | Direction |
|---|---|---|
| `/jarvis/command` | `String` | in |
| `/jarvis/world_state` | `String` (JSON) | out |
| `/jarvis/action_plan` | `String` (JSON) | out |
| `/jarvis/feedback` | `String` | out |
| `/jarvis/llm_status` | `String` | out |
| `/jarvis/detected_objects` | `String` (JSON) | internal |
| `/jarvis/object_poses` | `PoseArray` | internal |

---

## 10. Web GUI — Robotics Studio

**URL:** http://localhost:5000  
**Design:** Dark glassmorphism per `DESIGN.md` (Vision Pro / Isaac Sim inspired)

### 10.1 Layout

| Region | Content |
|---|---|
| **Top bar** | Connection, E-stop, live XYZ, clock |
| **Sidebar** | Control · Sequencer · JARVIS · Setup · Settings |
| **Main** | Active tab panels |
| **Bottom console** | Real-time event log |

### 10.2 Tabs

| Tab | Features |
|---|---|
| **Control** | 3D DH viewport, status, joint sliders, Cartesian jog, gripper, fan, saved positions |
| **Sequencer** | Record/play waypoint sequences, save/load JSON |
| **JARVIS** | Chat feed, world model panel, action plan viewer, quick commands |
| **Setup** | URAF wizard: hardware scan, health, config load |
| **Settings** | Collision thresholds, joint limits, MCU config save |

### 10.3 REST API

| Endpoint | Method | Description |
|---|---|---|
| `/api/send_joints` | POST | Joint angles (degrees) |
| `/api/home` | POST | Home command |
| `/api/jog` | POST | Cartesian jog `{x,y,z}` delta |
| `/api/move_xyz` | POST | Absolute XYZ target |
| `/api/jarvis_command` | POST | Natural language command |
| `/api/kinematics` | GET | DH params for 3D viewport |
| `/api/uraf/config` | GET | URAF master config |
| `/api/uraf/discovery` | POST | Trigger hardware scan |
| `/api/uraf/generate_urdf` | POST | Generate URDF/SRDF from robot profile |
| `/api/uraf/learning` | GET | Learning stats (grasp/planner models) |
| `/api/uraf/recovery` | GET | Last self-healing recovery action |
| `/api/uraf/twin` | GET | Digital twin state |
| `/api/uraf/twin/validate` | POST | Validate motion target |
| `/api/uraf/wizard` | GET/POST | Wizard progress |
| `/api/uraf/wizard/reset` | POST | Reset wizard |
| `/api/uraf/plugins` | GET | Installed plugins |
| `/api/uraf/plugins/install` | POST | Install bundled plugin |
| `/api/uraf/community` | GET | Community profile catalog |
| `/api/uraf/community/lookup` | POST | Profile lookup |
| `/api/uraf/safety` | GET | Safety monitor status |
| `/api/uraf/safety/command` | POST | `arm`, `disarm`, or `startup` |
| `/api/uraf/health` | GET | System health snapshot |

### 10.4 Socket.IO events (server → client)

| Event | Payload |
|---|---|
| `status_update` | Full robot state |
| `cartesian_pose` | Live X, Y, Z |
| `log_message` | `{level, message}` |
| `jarvis_feedback` | Step-by-step JARVIS messages |
| `jarvis_world_state` | Object registry JSON |
| `jarvis_action_plan` | Plan JSON |
| `llm_status` | Planner state |
| `uraf_discovery` | Hardware profile |
| `uraf_health` | Health report |
| `uraf_recovery` | Self-healing recovery event |
| `uraf_learning` | Learning stats update |
| `uraf_twin` | Digital twin state |
| `uraf_plugins` | Plugin registry |
| `uraf_community` | Community catalog |
| `uraf_community_match` | Profile match result |
| `uraf_safety` | Safety status (operational, violations) |

---

## 11. Launch System

**Main file:** `launch/spawn_visiona.launch.py`

### 11.1 Launch arguments

| Parameter | Values | Default | Description |
|---|---|---|---|
| `mode` | `real`, `sim`, `gazebo` | `real` | Hardware mode |
| `gui` | `true`, `false` | `true` | Web GUI on port 5000 |
| `viz` | `none`, `rviz`, `moveit` | `none` | RViz or MoveIt RViz |
| `planning` | `simple_ik`, `moveit`, `auto` | `auto` | Motion backend for JARVIS |
| `twin` | `true`, `false` | `true` | Digital twin pre-validation |
| `camera` | `true`, `false` | `false` | RGB-D camera + mapping stack |
| `mapping` | `low`, `high` | `low` | OctoMap resolution |
| `jarvis` | `true`, `false` | `false` | JARVIS AI pipeline |

### 11.2 Launch topology

```
spawn_visiona.launch.py
├── ALWAYS
│   ├── robot_state_publisher
│   ├── ros2_control_node + controllers
│   ├── simple_ik_solver
│   ├── web_gui (bridge + Flask)
│   ├── uraf_hardware_discovery
│   ├── uraf_self_healing
│   ├── uraf_learning_agent
│   ├── uraf_multi_robot_coordinator
│   ├── uraf_urdf_generator
│   ├── uraf_plugin_manager
│   └── uraf_community_library
│   └── uraf_safety_monitor
├── [twin:=true]
│   └── digital_twin.launch.py
├── [mode:=gazebo]
│   └── gazebo_sim.launch.py
├── [camera:=true]
│   ├── ascamera HP60C
│   ├── octomap_server
│   ├── rtabmap
│   └── colored_map_accumulator
├── [planning:=moveit|auto]
│   └── moveit_control.launch.py (move_group)
├── [viz:=rviz]
│   └── rviz2
├── [viz:=moveit]
│   └── MoveIt RViz (via moveit_control)
└── [jarvis:=true]
    └── llm_control.launch.py (HybridMotionPlanner executor)
```

### 11.3 Examples

```bash
# Simulation — no hardware
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim viz:=rviz

# Real robot + GUI
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real

# Full stack: camera + high-res map + JARVIS + MoveIt
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real camera:=true mapping:=high jarvis:=true planning:=auto viz:=moveit

# Software twin + JARVIS (no hardware)
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim twin:=true jarvis:=true

# Gazebo digital twin
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=gazebo twin:=true viz:=rviz

# Headless bridge (no web GUI)
ros2 launch visiona_bridge spawn_visiona.launch.py gui:=false
```

---

## 12. ROS 2 Topics & Services

### Published (key outputs)

| Topic | Type | Source |
|---|---|---|
| `/joint_states` | `JointState` | bridge_node |
| `/visiona/cartesian_command` | `PoseStamped` | IK, executor, GUI |
| `/visiona/current_pose` | `PoseStamped` | simple_ik_solver |
| `/colored_map` | `PointCloud2` | colored_map_node |
| `/jarvis/world_state` | `String` | jarvis_world_model |
| `/uraf/hardware_profile` | `String` | uraf_hardware_discovery |
| `/uraf/health` | `String` | uraf_self_healing |
| `/uraf/recovery/status` | `String` | uraf_self_healing |
| `/uraf/learning/stats` | `String` | uraf_learning_agent |
| `/uraf/multi_robot/status` | `String` | uraf_multi_robot_coordinator |
| `/visiona/system_status` | `String` | bridge_node |
| `/uraf/twin/state` | `String` | uraf_digital_twin |
| `/uraf/twin/validation` | `String` | uraf_digital_twin |
| `/uraf/plugins/status` | `String` | uraf_plugin_manager |
| `/uraf/community/catalog` | `String` | uraf_community_library |
| `/uraf/community/match` | `String` | uraf_community_library |
| `/uraf/safety/status` | `String` | uraf_safety_monitor |
| `/uraf/safety/motion_scale` | `Float32` | uraf_safety_monitor |
| `/jarvis/execution_event` | `String` | jarvis_action_executor |

### Subscribed (key inputs)

| Topic | Type | Consumer |
|---|---|---|
| `/joint_targets` | `JointState` | bridge_node → MCU |
| `/octomap_point_cloud_centers` | `PointCloud2` | simple_ik_solver |
| `/jarvis/command` | `String` | planner, detector (vision keywords) |

---

## 13. Configuration Reference

| File | Purpose |
|---|---|
| `config/kinematic_params.yaml` | DH params, IK tuning, workspace, collision |
| `config/uraf_config.yaml` | URAF master config template |
| `config/robots/visiona_v1.yaml` | Robot profile (community library seed) |
| `config/simple_ik_config.yaml` | IK node parameter overrides |
| `config/visiona_controllers.yaml` | ros2_control controllers |
| `config/visual_servo_params.yaml` | Visual servo tuning |
| `urdf/visiona.urdf.xacro` | Robot description |
| `~/.visiona_bridge/` | Runtime: positions, sequences, URAF config |

---

## 14. Build & Run

### 14.1 Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-moveit ros-humble-octomap-server \
                 ros-humble-cv-bridge python3-opencv

# Python
pip install flask flask-socketio flask-cors pyserial numpy pyyaml requests

# Optional: JARVIS
# Install Ollama + pull mistral, llava
```

### 14.2 Build

```bash
cd ~/ros2_ws
colcon build --packages-select visiona_bridge --symlink-install
source install/setup.bash
```

### 14.3 Serial permissions

```bash
sudo usermod -aG dialout $USER
# Log out and back in
ls /dev/ttyUSB*
```

### 14.4 First run checklist

1. Build workspace and source install
2. Connect ESP32 USB → verify `/dev/ttyUSB0`
3. Launch: `ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real`
4. Open http://localhost:5000
5. Setup tab → Scan Hardware
6. Control tab → verify 3D viewport + joint updates
7. (Optional) Start Ollama, relaunch with `jarvis:=true camera:=true`

---

## 15. Troubleshooting

### Serial not connecting

- Check `dialout` group membership
- Verify baud `921600` matches firmware
- Try GUI **Reconnect to MCU** or simulation mode toggle

### IK solver rejects targets

- Workspace: X,Y ∈ [−0.63, 0.63] m, Z ∈ [0, 0.63] m, radius ≥ 0.12 m
- Clear OctoMap: `ros2 service call /octomap_server/reset std_srvs/srv/Empty`

### JARVIS not responding

- Confirm Ollama running: `curl http://localhost:11434/api/tags`
- Check models: `ollama list` (need `mistral`, `llava`)
- Monitor: `ros2 topic echo /jarvis/llm_status`

### Camera / perception empty

- Launch with `camera:=true`
- Verify topics: `ros2 topic list | grep ascamera`
- Check TF: `ros2 run tf2_ros tf2_echo world ascamera_hp60c_ascamera_0`

### E-stop triggered

- GUI → **Enable (Restart)** or release via bridge
- Reduce collision threshold in Settings tab

### 3D viewport blank

- Check browser console for Three.js errors
- Verify `/api/kinematics` returns DH params
- Hard refresh (Ctrl+Shift+R)

---

## 16. Project Roadmap (PLAN.md)

This repository implements **Visiona Robotics Studio** as the first deployment of the **URAF** vision described in `PLAN.md` and `DESIGN.md`.

| PLAN Phase | Status in this repo |
|---|---|
| **Phase 0** — Foundation (agent bus, config store) | ✅ Integrated in `visiona_bridge/uraf/` |
| **Phase 1** — Hardware discovery | ✅ Visiona-focused discovery agent |
| **Phase 2** — URDF / model generation | ✅ `uraf/urdf_generator.py` + `/api/uraf/generate_urdf` |
| **Phase 3** — MoveIt / planning plugins | ✅ `planning:=auto\|moveit\|simple_ik`, JARVIS HybridMotionPlanner |
| **Phase 4** — Perception & mapping | ✅ Camera, OctoMap, RTAB-Map, colored map |
| **Phase 5** — JARVIS intelligence | ✅ VLM + SAM + planner + executor |
| **Phase 6** — Self-healing / production | ✅ Self-healing agent, learning agent, multi-robot coordinator |
| **GUI Wizard (§17)** | ✅ 7-step guided wizard + auto setup mode |
| **Digital twin (§18)** | ✅ Twin agent, pre-validation, Gazebo launch (`mode:=gazebo`) |
| **Plugin system (§20)** | ✅ Registry, loader, bundled `visiona_fal`, CLI + API |
| **Community Library (§21)** | ✅ Fingerprinting, bundled profiles, discovery matching |
| **Safety Architecture (§19)** | ✅ Safety monitor, SF-01–11, safe startup, JARVIS gate |
| **Testing (§23)** | ✅ Unit tests for safety, wizard, community library |

**Long-term goal (PLAN.md):** Connect any robot → guided setup → fully operational MoveIt2 system with zero manual URDF authoring. Visiona V1 is the reference platform proving the architecture.

---

## Related documentation

| Document | Content |
|---|---|
| [PLAN.md](PLAN.md) | Full URAF agentic implementation plan |
| [DESIGN.md](DESIGN.md) | Robotics Studio UI/UX specification |
| [SKILLS/SKILL.md](SKILLS/SKILL.md) | ROS 2 engineering reference |

---

*Visiona Robotics Studio — built for makers, researchers, and engineers who want industrial-grade robot control with natural-language autonomy.*
