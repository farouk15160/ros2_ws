# UNIVERSAL ROBOT ARM FOUNDATION (URAF)

## AI-Powered Plug-and-Play Framework for Any Robotic Manipulator

### Full AI Agentic Implementation Plan — v1.0

---

> **Document Purpose:** This is the master agentic implementation plan for the URAF project. It defines every agent, every phase, every goal, every technology choice, and every success metric required to build a production-grade, robot-agnostic, AI-driven robotic framework. This document is intended to guide development teams, robotics engineers, and AI architects from zero to a fully operational system.

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Mission, Vision & Core Goals](#2-mission-vision--core-goals)
3. [Core Design Principles](#3-core-design-principles)
4. [System Architecture](#4-system-architecture)
5. [Agent Orchestration Layer](#5-agent-orchestration-layer)
6. [Phase 0 — Foundation Infrastructure](#6-phase-0--foundation-infrastructure)
7. [Phase 1 — Robot Discovery Layer](#7-phase-1--robot-discovery-layer)
8. [Phase 2 — Model Generation Layer](#8-phase-2--model-generation-layer)
9. [Phase 3 — Control & Planning Layer](#9-phase-3--control--planning-layer)
10. [Phase 4 — Perception & Environment Mapping](#10-phase-4--perception--environment-mapping)
11. [Phase 5 — Intelligence & Autonomous Capabilities](#11-phase-5--intelligence--autonomous-capabilities)
12. [Phase 6 — Self-Healing, Learning & Production](#12-phase-6--self-healing-learning--production)
13. [Detailed Agent Specifications](#13-detailed-agent-specifications)
14. [Firmware Abstraction Layer (FAL)](#14-firmware-abstraction-layer-fal)
15. [Technology Stack & Dependency Map](#15-technology-stack--dependency-map)
16. [Data Flow & Inter-Agent Communication](#16-data-flow--inter-agent-communication)
17. [GUI Wizard — Full Specification](#17-gui-wizard--full-specification)
18. [Digital Twin Subsystem](#18-digital-twin-subsystem)
19. [Safety Architecture](#19-safety-architecture)
20. [Plugin System Architecture](#20-plugin-system-architecture)
21. [Community Robot Library (CRL)](#21-community-robot-library-crl)
22. [Configuration Schema Reference](#22-configuration-schema-reference)
23. [Testing Strategy](#23-testing-strategy)
24. [Success Metrics & KPIs](#24-success-metrics--kpis)
25. [Risk Analysis & Mitigation](#25-risk-analysis--mitigation)
26. [Stretch Goals Roadmap](#26-stretch-goals-roadmap)
27. [Project Timeline Summary](#27-project-timeline-summary)

---

## 1. Executive Summary

The Universal Robot Arm Foundation (URAF) is an ambitious, first-of-its-kind AI-agentic framework designed to eliminate the integration barrier between robotic hardware and software. Today, integrating a new robotic manipulator into a ROS2/MoveIt2 environment requires deep expertise in URDF authoring, ros2_control configuration, kinematics solvers, sensor calibration, and dozens of interdependent software packages. This process takes experienced engineers weeks to months per robot platform.

URAF replaces this manual process with an autonomous multi-agent AI system. When a robot is connected, a pipeline of specialized AI agents automatically discovers the hardware, infers the robot's physical structure, generates all required configuration files, calibrates sensors and joints, configures motion planning, and enables perception — all without requiring the user to write a single line of code.

**The core promise:** Connect a robot → answer a few questions (or none at all) → receive a fully operational, MoveIt2-enabled robotic system ready for task execution.

**Key Innovation Areas:**

- Hardware-agnostic discovery and abstraction
- AI-driven URDF and configuration generation
- Multi-solver IK benchmarking and automatic selection
- Autonomous calibration using vision and motion primitives
- Confidence-gated human-in-the-loop validation
- Self-healing configuration management
- Community-sourced robot profile library

---

## 2. Mission, Vision & Core Goals

### 2.1 Mission Statement

> Design and implement an AI-powered, robot-agnostic framework capable of automatically adapting to virtually any robotic manipulator without requiring manual software rewriting, enabling any engineer — regardless of ROS2 expertise level — to deploy a fully operational robotic system in under one hour.

### 2.2 Vision

URAF should become the **"Universal Plug and Play standard for robotic manipulators"** — the equivalent of how USB standardized peripheral connections. Just as USB eliminated the need to write device drivers for every peripheral, URAF eliminates the need to write integration code for every robot arm.

### 2.3 Primary Goals

| ID   | Goal                                                                       | Priority | Phase     |
| ---- | -------------------------------------------------------------------------- | -------- | --------- |
| G-01 | Auto-detect any connected robot hardware (serial, CAN, EtherCAT, ROS2)     | Critical | Phase 1   |
| G-02 | Infer robot DOF, joint types, limits, and structure without CAD files      | Critical | Phase 1–2 |
| G-03 | Automatically generate valid URDF, SRDF, ros2_control, and MoveIt2 configs | Critical | Phase 2   |
| G-04 | Configure and benchmark multiple IK solvers, auto-select the best          | High     | Phase 3   |
| G-05 | Detect and configure cameras and depth sensors automatically               | High     | Phase 4   |
| G-06 | Perform autonomous joint and camera calibration using vision               | High     | Phase 1–4 |
| G-07 | Support 3-DOF to 20+ DOF arms across all major hardware interfaces         | Critical | Phase 1–3 |
| G-08 | Provide a no-code GUI wizard for complete robot setup                      | High     | Phase 2   |
| G-09 | Generate and maintain a live digital twin in simulation                    | Medium   | Phase 3   |
| G-10 | Enable object detection, grasp planning, and pick-and-place autonomously   | High     | Phase 5   |
| G-11 | Implement a plugin system for extending capabilities without code changes  | High     | Phase 3   |
| G-12 | Self-heal broken configurations using AI agents                            | Medium   | Phase 6   |
| G-13 | Support multi-robot and mobile manipulator configurations                  | Medium   | Phase 6   |
| G-14 | Expose a natural language interface for task programming                   | Low      | Phase 6   |
| G-15 | Maintain a community library of robot profiles shareable across projects   | Medium   | Phase 5   |

### 2.4 Non-Goals (Explicit Exclusions for v1.0)

- URAF does not aim to replace MoveIt2 — it configures and uses it
- URAF does not provide cloud robotics infrastructure (v1.0 is local-first)
- URAF does not generate novel firmware from scratch (it generates adapters for known firmware)
- URAF does not support legged robots in v1.0 (manipulators only)

---

## 3. Core Design Principles

### P-01: Zero-Code by Default

The user should never need to touch source code during normal operation. All modifications happen through YAML configuration, the GUI wizard, or AI-assisted Q&A. Code generation happens invisibly within agents.

### P-02: Progressive Disclosure

The system should be operable at multiple levels:

- **Beginner mode:** GUI wizard, everything automatic
- **Intermediate mode:** Single YAML file with well-documented options
- **Expert mode:** Full access to generated configs, manual overrides, plugin hooks

### P-03: Confidence-Gated Automation

Every agent outputs a **confidence score (0.0–1.0)** for its outputs. Actions with confidence ≥ 0.85 are executed automatically. Actions with confidence 0.60–0.85 are proposed to the user for confirmation. Actions below 0.60 trigger a human-in-the-loop dialogue before proceeding.

```
Confidence ≥ 0.85  →  Auto-execute
Confidence 0.60–0.85  →  Propose + Confirm
Confidence < 0.60  →  Human-in-the-loop Q&A
```

### P-04: Simulation-First Validation

All generated configurations are validated in a Gazebo or MuJoCo simulation environment before being applied to physical hardware. If simulation validation fails, the AI Configuration Agent is triggered to repair the config before any physical motion is attempted.

### P-05: Versioned Configuration Management

All generated configurations are stored in a git-managed config store. Every change is committed with a descriptive message generated by the AI agent. Users can roll back to any previous working state with one command.

### P-06: Hardware Abstraction Without Performance Penalty

The Firmware Abstraction Layer (FAL) must add no more than 2ms of additional latency over direct hardware access. All hardware-specific code lives strictly below the FAL boundary; upper layers are hardware-agnostic.

### P-07: Modular Agent Independence

Each agent must be independently deployable, testable, and replaceable. Agents communicate exclusively through defined message schemas on the Agent Orchestration Bus. No agent has direct dependencies on another agent's internal implementation.

### P-08: Graceful Degradation

If any agent fails, the system must fall back to the highest level of capability that can be achieved with the remaining agents. For example, if the Vision Agent fails, the system should still enable full motion planning in a static environment.

### P-09: Community-Driven Knowledge

The system maintains a local-first Community Robot Library where successfully configured robots are fingerprinted and their profiles optionally shared. When a known robot is connected, the system can load a proven community profile instead of rediscovering everything from scratch.

### P-10: Self-Healing Architecture

Long-running deployments must be able to detect configuration drift, hardware changes, and degraded performance, and autonomously trigger re-calibration or re-configuration without human intervention.

---

## 4. System Architecture

### 4.1 High-Level Architecture

```
╔══════════════════════════════════════════════════════════════════════════╗
║                    UNIVERSAL ROBOT ARM FOUNDATION (URAF)                 ║
╚══════════════════════════════════════════════════════════════════════════╝

┌─────────────────────────────────────────────────────────────────────────┐
│                          USER INTERFACE LAYER                            │
│   ┌──────────────────┐   ┌──────────────────┐   ┌─────────────────────┐│
│   │   GUI Wizard     │   │  Natural Language │   │  YAML Config File   ││
│   │  (Web + Desktop) │   │    Interface      │   │   (Single Source)   ││
│   └──────────────────┘   └──────────────────┘   └─────────────────────┘│
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
┌────────────────────────────────▼────────────────────────────────────────┐
│                      AGENT ORCHESTRATION LAYER                           │
│   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │
│   │Orchestrator  │  │ Config Store │  │ State Manager│  │ Agent Bus  │ │
│   │  (LLM Core)  │  │  (git-based) │  │  (Redis/ROS2)│  │ (pub/sub)  │ │
│   └──────────────┘  └──────────────┘  └──────────────┘  └────────────┘ │
└──────┬──────────────┬─────────────────┬──────────────────┬──────────────┘
       │              │                 │                  │
┌──────▼──────┐ ┌─────▼──────┐  ┌──────▼──────┐   ┌──────▼──────────────┐
│  DISCOVERY  │ │   MODEL    │  │  PLANNING & │   │    PERCEPTION &     │
│    LAYER    │ │ GENERATION │  │   CONTROL   │   │   INTELLIGENCE      │
│             │ │   LAYER    │  │    LAYER    │   │      LAYER          │
│ • HW Disc.  │ │ • URDF Gen │  │ • MoveIt2   │   │ • Vision Agent      │
│ • FW Agent  │ │ • IK Agent │  │ • Planners  │   │ • Mapping Agent     │
│ • ROS2 Disc.│ │ • Geometry │  │ • Trajectory│   │ • Grasp Agent       │
│ • EE Agent  │ │ • Collision│  │ • Safety    │   │ • Object Detection  │
└──────┬──────┘ └─────┬──────┘  └──────┬──────┘   └──────┬──────────────┘
       │              │                 │                  │
┌──────▼──────────────▼─────────────────▼──────────────────▼──────────────┐
│                       FIRMWARE ABSTRACTION LAYER (FAL)                   │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐ │
│  │ CANOpen  │ │EtherCAT  │ │ Serial/  │ │ ROS2 HW  │ │  MicroROS     │ │
│  │ Driver   │ │ Driver   │ │ UART     │ │ Interface│ │  (ESP32/STM)  │ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └───────────────┘ │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐ │
│  │ Dynamixel│ │  ODrive  │ │  VESC    │ │  LinuxCNC│ │  Unitree Act. │ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └───────────────┘ │
└──────────────────────────────────────────────────────────────────────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                   ▼
         PHYSICAL            DIGITAL TWIN        SIMULATION
         HARDWARE            (RViz + TF)         (Gazebo/MuJoCo)
```

### 4.2 Agent Communication Architecture

All agents communicate via the **URAF Agent Bus**, which is a dual-channel system:

- **Channel A (ROS2 Topics/Services/Actions):** Real-time robot data (joint states, sensor data, transforms)
- **Channel B (URAF Event Bus — Redis Pub/Sub or gRPC):** High-level agent coordination, configuration events, status updates, confidence reports

```
Agent Message Schema (Channel B):
{
  "agent_id": "kinematics_agent_v1",
  "message_type": "RESULT | REQUEST | STATUS | ERROR",
  "timestamp": 1700000000.000,
  "confidence": 0.92,
  "payload": { ... },
  "requires_confirmation": false,
  "rollback_id": "cfg_2024_001_abc123"
}
```

### 4.3 Configuration State Machine

The system moves through discrete states during setup:

```
UNINITIALIZED
    │
    ▼
HARDWARE_DISCOVERY ──(fail)──► DISCOVERY_FAILED → Human Intervention
    │
    ▼
GEOMETRY_INFERENCE ──(low confidence)──► HUMAN_IN_LOOP_GEOMETRY
    │
    ▼
MODEL_GENERATION ──(validation fail)──► SIM_REPAIR → retry
    │
    ▼
SIM_VALIDATION ──(pass)──► CONTROLLER_SETUP
    │
    ▼
CALIBRATION ──(fail)──► CALIBRATION_RETRY (max 3)
    │
    ▼
PERCEPTION_SETUP
    │
    ▼
OPERATIONAL ──(drift detected)──► SELF_HEAL
    │
    ▼
TASK_EXECUTION
```

---

## 5. Agent Orchestration Layer

### 5.1 Master Orchestrator

The Master Orchestrator is the top-level AI agent that coordinates all sub-agents. It is powered by a local LLM (default: Llama 3.1 70B via Ollama, or GPT-4o via API if configured) and uses a structured reasoning framework.

**Goals:**

- Maintain global awareness of the robot integration state
- Coordinate agent execution order based on dependencies
- Resolve conflicts between agent outputs
- Generate human-readable explanations of all decisions
- Manage the human-in-the-loop confirmation flow
- Handle agent failures and trigger recovery procedures

**Orchestrator Execution Model:**

```python
class MasterOrchestrator:
    """
    Uses a Plan → Execute → Verify → Adapt loop.
    """
    def run_integration_pipeline(self, robot_context: RobotContext):
        plan = self.generate_plan(robot_context)          # LLM plans agent sequence
        for step in plan.steps:
            result = self.execute_agent(step.agent, step.inputs)
            if result.confidence < 0.60:
                result = self.human_in_loop(step, result)
            self.verify_result(result)                    # Sim validation
            self.commit_config(result)                    # Git commit
            if not result.success:
                self.trigger_recovery(step, result)       # Self-heal
```

### 5.2 Configuration Store

All generated configurations are managed by a git-based store located at `~/.uraf/configs/<robot_fingerprint>/`.

```
~/.uraf/
├── configs/
│   └── <robot_fingerprint>/        # SHA-256 of (joints + interface + geometry)
│       ├── robot.yaml              # Master config
│       ├── urdf/
│       │   └── robot.urdf.xacro
│       ├── srdf/
│       │   └── robot.srdf
│       ├── moveit2/
│       │   ├── kinematics.yaml
│       │   ├── joint_limits.yaml
│       │   ├── ompl_planning.yaml
│       │   └── controllers.yaml
│       ├── ros2_control/
│       │   └── controllers.yaml
│       ├── calibration/
│       │   ├── joint_offsets.yaml
│       │   ├── camera_extrinsics.yaml
│       │   └── tcp_calibration.yaml
│       └── .git/                   # Full git history
├── community_profiles/             # CRL downloaded profiles
├── plugins/                        # User plugins
└── logs/                          # Agent execution logs
```

### 5.3 State Manager

The State Manager provides a shared, consistent view of robot state across all agents using a Redis-backed store (or ROS2 parameter server for lightweight deployments).

**Keys managed:**

```yaml
uraf.robot.fingerprint: "sha256:abc123..."
uraf.robot.dof: 6
uraf.robot.interface: "can"
uraf.state.current: "CALIBRATION"
uraf.agents.discovery.status: "COMPLETE"
uraf.agents.kinematics.confidence: 0.94
uraf.calibration.joint_offsets: [0.0, 0.02, -0.01, 0.0, 0.0, 0.0]
uraf.perception.cameras_detected: ["realsense_d435", "usb_cam_01"]
uraf.safety.estop_armed: true
```

---

## 6. Phase 0 — Foundation Infrastructure

**Duration:** 4 weeks  
**Goal:** Establish the core infrastructure that all other phases depend on. No robot-specific functionality yet — this phase creates the skeleton.

### 6.1 Objectives

- Set up the ROS2 workspace structure
- Implement the Agent Orchestration Bus
- Build the Configuration Store
- Build the State Manager
- Define all inter-agent message schemas
- Set up the simulation environment
- Establish CI/CD pipelines for config validation
- Create the plugin registry

### 6.2 Deliverables

#### 6.2.1 URAF ROS2 Workspace Structure

```
uraf_ws/
├── src/
│   ├── uraf_core/                   # Core orchestration
│   │   ├── uraf_orchestrator/
│   │   ├── uraf_state_manager/
│   │   ├── uraf_config_store/
│   │   └── uraf_agent_bus/
│   ├── uraf_discovery/              # Phase 1 agents
│   │   ├── uraf_hw_discovery/
│   │   ├── uraf_firmware_agent/
│   │   └── uraf_ros2_discovery/
│   ├── uraf_model/                  # Phase 2 agents
│   │   ├── uraf_urdf_generator/
│   │   ├── uraf_kinematics_agent/
│   │   └── uraf_geometry_agent/
│   ├── uraf_control/                # Phase 3 agents
│   │   ├── uraf_moveit2_configurator/
│   │   ├── uraf_trajectory_agent/
│   │   └── uraf_safety_agent/
│   ├── uraf_perception/             # Phase 4 agents
│   │   ├── uraf_vision_agent/
│   │   ├── uraf_mapping_agent/
│   │   └── uraf_calibration_agent/
│   ├── uraf_intelligence/           # Phase 5 agents
│   │   ├── uraf_grasp_agent/
│   │   ├── uraf_task_agent/
│   │   └── uraf_nl_interface/
│   ├── uraf_gui/                    # GUI wizard
│   ├── uraf_digital_twin/           # Digital twin
│   ├── uraf_plugins/                # Plugin system
│   └── uraf_community/              # Community Robot Library
├── docker/
│   ├── Dockerfile.base
│   ├── Dockerfile.gpu
│   └── docker-compose.yml
├── tests/
│   ├── unit/
│   ├── integration/
│   └── simulation/
└── scripts/
    ├── install.sh
    ├── uraf_setup.py
    └── uraf_cli.py
```

#### 6.2.2 Agent Message Schema (Protobuf / JSON Schema)

All inter-agent messages follow this schema:

```json
{
  "$schema": "uraf/agent-message/v1",
  "agent_id": "string",
  "message_type": "enum[RESULT, REQUEST, STATUS, ERROR, CONFIRMATION_REQUIRED]",
  "timestamp_ns": "uint64",
  "pipeline_id": "string (UUID)",
  "confidence": "float [0.0-1.0]",
  "payload_type": "string",
  "payload": "object",
  "requires_human_confirmation": "bool",
  "confirmation_question": "string?",
  "rollback_id": "string?",
  "parent_message_id": "string?"
}
```

#### 6.2.3 Master YAML Config Schema (v1)

```yaml
# uraf_config.yaml — Master configuration file
# URAF v1.0

uraf:
  version: "1.0"
  mode: "auto" # auto | guided | expert

robot:
  name: "my_robot" # Human-readable name
  description: "" # Optional description

  # Hardware interface — set to "auto" for autodiscovery
  interface:
    type: "auto" # auto | can | ethercat | serial | ros2 | microros
    can:
      interface: "can0"
      bitrate: 1000000
    serial:
      port: "auto" # auto | /dev/ttyUSB0 | etc.
      baudrate: 115200
    ethercat:
      interface: "eth0"
    ros2:
      namespace: "/"

  # Kinematics — set to "auto" for inference
  kinematics:
    dof: "auto" # auto | 3-20
    type: "auto" # auto | serial | parallel | delta | scara
    joint_limits: "auto" # auto | manual
    home_position: "auto"

  # Geometry — set to "auto" or provide manually
  geometry:
    source: "auto" # auto | yaml | urdf | cad
    link_lengths: [] # populated by geometry agent if source=auto
    base_frame: "auto"
    tool_frame: "auto"

  # End-effector
  end_effector:
    type: "auto" # auto | gripper | vacuum | tool_changer | none

# Planning
planning:
  framework: "moveit2"
  planners:
    - "ompl"
    - "pilz"
    - "chomp"
  ik_solver: "auto" # auto | kdl | trac_ik | bio_ik | ikfast | pinocchio
  collision_checking: true

# Perception
perception:
  enabled: "auto"
  cameras: "auto" # auto | manual list
  mapping:
    enabled: "auto"
    type: "auto" # octomap | voxblox | tsdf

# Calibration
calibration:
  auto_calibrate: true
  method: "auto" # apriltag | aruco | motion_primitive

# Safety
safety:
  estop_enabled: true
  joint_limits_enforced: true
  self_collision_checking: true
  workspace_limits: "auto"

# GUI
gui:
  enabled: true
  port: 8080

# Digital twin
digital_twin:
  enabled: true
  simulator: "auto" # auto | gazebo | mujoco | rviz_only

# Community
community:
  auto_lookup: true # Look up profile in Community Robot Library
  share_profile: false # Upload profile after successful setup
```

#### 6.2.4 Docker Infrastructure

URAF runs in a containerized environment with GPU support:

```yaml
# docker-compose.yml
services:
  uraf_core:
    build: ./docker/Dockerfile.base
    environment:
      - ROS_DOMAIN_ID=0
      - URAF_MODE=auto
    volumes:
      - ~/.uraf:/root/.uraf
      - /dev:/dev
    devices:
      - /dev/bus/usb
    network_mode: host
    privileged: true # Required for CAN, EtherCAT access

  uraf_gpu:
    build: ./docker/Dockerfile.gpu
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    environment:
      - NVIDIA_VISIBLE_DEVICES=all

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"

  uraf_gui:
    build: ./docker/Dockerfile.gui
    ports:
      - "8080:8080"
    environment:
      - URAF_CORE_HOST=uraf_core
```

### 6.3 Success Criteria (Phase 0)

- [ ] ROS2 workspace builds without errors
- [ ] Agent bus sends and receives messages with < 1ms latency
- [ ] Config store performs git commit/rollback correctly
- [ ] State manager maintains consistent state across 3+ simultaneous agent connections
- [ ] Docker containers start and all services are healthy
- [ ] CI/CD pipeline validates config schema changes automatically
- [ ] Plugin registry loads and unloads test plugins correctly

---

## 7. Phase 1 — Robot Discovery Layer

**Duration:** 6 weeks  
**Goal:** Automatically detect any connected robotic hardware and output a structured description of the robot's joints, interface, and capabilities — with no prior knowledge required.

**Prerequisite:** Phase 0 complete

### 7.1 Robot Discovery Agent

**Agent ID:** `hardware_discovery_agent`  
**Priority:** Critical  
**Confidence Target:** ≥ 0.90 for known interfaces, ≥ 0.70 for unknown

#### 7.1.1 Goals

1. Enumerate all physical communication interfaces on the host system
2. Identify which interfaces have robotic hardware connected
3. Determine the communication protocol in use
4. Perform initial handshake and capability query
5. Identify the number of joints/axes
6. Detect encoder presence and type
7. Detect control mode support (position, velocity, torque)
8. Estimate the robot family (if known)
9. Output a structured `HardwareProfile` object

#### 7.1.2 Discovery Sequence

```
START
 │
 ├─── Scan USB devices (/dev/ttyUSB*, /dev/ttyACM*)
 │     ├── Query baud rates: 9600, 57600, 115200, 921600, 1000000
 │     ├── Send probe packets for known protocols
 │     │    ├── Dynamixel protocol 1.0 / 2.0 (0xFF 0xFF probe)
 │     │    ├── Modbus RTU
 │     │    ├── Custom UART (heuristic matching)
 │     │    └── MicroROS framing
 │     └── Record responding devices
 │
 ├─── Scan CAN interfaces (ip link show type can)
 │     ├── Bring up interface: ip link set canX up type can bitrate 1000000
 │     ├── Listen for heartbeat messages (CANOpen node guarding)
 │     ├── Enumerate nodes via SDO requests
 │     ├── Query node type, manufacturer, firmware version
 │     └── Map node IDs → joint indices
 │
 ├─── Scan EtherCAT (ethercat slaves)
 │     ├── Enumerate EtherCAT slaves
 │     ├── Read ESI (EtherCAT Slave Information) files
 │     ├── Map PDO (Process Data Objects) → joint channels
 │     └── Identify servo drive type
 │
 ├─── Query ROS2 ecosystem
 │     ├── ros2 topic list → find /joint_states, /tf, /robot_description
 │     ├── ros2 node list → find controller_manager, robot_state_publisher
 │     ├── ros2 param get /robot_description → extract existing URDF
 │     └── ros2 control list_hardware_interfaces
 │
 └─── Compile HardwareProfile → emit to Agent Bus
```

#### 7.1.3 Hardware Profile Output Schema

```yaml
hardware_profile:
  discovery_timestamp: "2024-01-01T00:00:00Z"
  confidence: 0.94

  robot_id: "sha256:abc123..." # Fingerprint

  interfaces:
    - type: "can"
      device: "can0"
      bitrate: 1000000
      protocol: "canopen"
      nodes:
        - node_id: 1
          device_type: "servo_drive"
          manufacturer: "Maxon"
          firmware_version: "3.2.1"
          channels:
            - type: "position"
            - type: "velocity"
            - type: "torque"

  joints:
    count: 6
    type: "revolute" # revolute | prismatic | continuous | mixed
    encoders: true
    encoder_type: "absolute" # absolute | incremental | resolver
    control_modes:
      - position
      - velocity
      - torque

  known_robot_family: null # null or "universal_robots_ur5" etc.

  community_profile_match:
    found: false
    profile_id: null
    match_confidence: null
```

### 7.2 Firmware Detection Agent

**Agent ID:** `firmware_detection_agent`

#### 7.2.1 Goals

1. Identify the exact firmware type running on each detected hardware node
2. Load the appropriate FAL (Firmware Abstraction Layer) driver
3. Query firmware capabilities and report limitations
4. Check firmware version compatibility and flag updates if needed

#### 7.2.2 Supported Firmware Matrix

| Firmware                | Detection Method                          | Protocol   | Driver Module       |
| ----------------------- | ----------------------------------------- | ---------- | ------------------- |
| Dynamixel Protocol 1.0  | Ping packet 0xFF 0xFF 0xFE 0x02 0x01 0xFA | Serial     | `fal_dynamixel_v1`  |
| Dynamixel Protocol 2.0  | Ping packet (header: 0xFF 0xFF 0xFD 0x00) | Serial     | `fal_dynamixel_v2`  |
| ODrive v3.x             | USB CDC, JSON API                         | Serial/USB | `fal_odrive_v3`     |
| ODrive Pro / S1         | USB CDC, CANSimple                        | CAN/USB    | `fal_odrive_pro`    |
| VESC                    | UART COMM_GET_VALUES                      | Serial     | `fal_vesc`          |
| CANOpen CiA402          | SDO object 0x6040                         | CAN        | `fal_canopen_402`   |
| EtherCAT CiA402         | ESI file + PDO mapping                    | EtherCAT   | `fal_ethercat_402`  |
| MicroROS (ESP32)        | DDS discovery                             | UDP/Serial | `fal_microros`      |
| Unitree Actuators       | RS485 custom protocol                     | RS485      | `fal_unitree`       |
| LinuxCNC HAL            | HAL pins interface                        | IPC        | `fal_linuxcnc`      |
| ROS2 Hardware Interface | ros2_control API                          | ROS2       | `fal_ros2_control`  |
| Custom Serial ASCII     | Heuristic pattern match                   | Serial     | `fal_custom_serial` |
| Custom CAN              | Candump + ML classifier                   | CAN        | `fal_custom_can`    |

#### 7.2.3 Unknown Firmware Handling

When firmware cannot be identified, the agent enters **protocol learning mode**:

1. **Passive listening:** Record all traffic for 30 seconds
2. **Pattern analysis:** Use a lightweight ML model to identify framing patterns (start bytes, length fields, checksums)
3. **Probe testing:** Send systematic probe sequences and observe responses
4. **Protocol inference:** Construct a protocol description file (`.uraf_proto`) describing the discovered protocol
5. **Human confirmation:** Present findings to user: "I detected a custom protocol with 7-byte frames. Does your firmware use `[0xAA][LEN][CMD][DATA...][CRC]` format? (Y/N)"
6. **Driver generation:** Generate a custom FAL driver based on the confirmed protocol

### 7.3 ROS2 Discovery Agent

**Agent ID:** `ros2_discovery_agent`

#### 7.3.1 Goals

1. Detect any existing ROS2 robot infrastructure on the network
2. Extract robot_description (URDF) if already published
3. Identify running controllers and their capabilities
4. Map existing topics/services to URAF data model
5. Avoid duplicating already-running functionality

#### 7.3.2 Discovery Logic

```python
class ROS2DiscoveryAgent(Node):
    def discover(self):
        results = ROS2Profile()

        # 1. Find robot_description
        if self.has_parameter('/robot_state_publisher', 'robot_description'):
            results.urdf = self.get_urdf()
            results.urdf_source = "existing"

        # 2. Find joint_states
        if '/joint_states' in self.get_topic_names():
            results.joint_states_available = True
            results.joint_names = self.sample_joint_states()

        # 3. Find controller_manager
        if self.service_exists('/controller_manager/list_controllers'):
            results.controllers = self.list_controllers()

        # 4. Find MoveIt2
        if self.service_exists('/move_action'):
            results.moveit2_running = True

        # 5. Find cameras
        for topic in self.get_topic_names():
            if 'image' in topic or 'depth' in topic or 'pointcloud' in topic:
                results.camera_topics.append(topic)

        return results
```

### 7.4 End-Effector Discovery Agent

**Agent ID:** `end_effector_agent`

#### 7.4.1 Goals

1. Detect the type of end-effector (EE) attached to the robot
2. Identify EE communication interface (if any)
3. Configure appropriate MoveIt2 end-effector group
4. Generate EE-specific grasp configurations

#### 7.4.2 Detection Methods

**Electrical detection:**

- GPIO pin sensing on tool changer connectors
- CAN node enumeration on wrist bus
- I2C device scan on tool interface (0x00-0x7F sweep)
- Analog current signature when EE is powered

**Vision detection (if camera available):**

- Run object detection on wrist camera
- Classify EE type from silhouette
- Measure gripper finger width from stereo if available

**User confirmation:**

- Show detected candidate to user with image
- "I detected what appears to be a parallel jaw gripper (confidence: 78%). Is this correct?"

**Supported End-Effectors:**

| EE Type              | Detection                 | Generated Group    | Additional Config           |
| -------------------- | ------------------------- | ------------------ | --------------------------- |
| Parallel jaw gripper | Width sensing + vision    | `gripper` group    | Finger width, stroke, force |
| Vacuum gripper       | Pneumatic pressure sensor | `vacuum` group     | Suction cup diameter        |
| Robotiq 2F-85        | USB/CAN ID 0x09           | `robotiq_2f`       | Full Robotiq driver         |
| Robotiq 2F-140       | USB/CAN ID 0x0A           | `robotiq_2f140`    | Full Robotiq driver         |
| Tool changer         | Solenoid + sensor         | `tool_changer`     | Tool library                |
| Force/torque sensor  | FT sensor protocol        | `ft_sensor` group  | Wrench monitoring           |
| Camera (wrist)       | UVC + depth probe         | Visual servo group | Camera calibration          |
| Custom               | Vision classification     | `custom_ee`        | User-defined                |
| None                 | Default assumption        | `flange`           | TCP at flange               |

### 7.5 Phase 1 Success Criteria

- [ ] Detects Dynamixel, ODrive, VESC, CANOpen, EtherCAT, and MicroROS hardware automatically in ≤ 30 seconds
- [ ] Correctly identifies DOF for 3–20 joint arms with ≥ 95% accuracy on known hardware
- [ ] Detects unknown protocols and generates valid protocol description files
- [ ] Outputs a HardwareProfile with confidence ≥ 0.85 for known hardware families
- [ ] Community Robot Library lookup completes in ≤ 5 seconds
- [ ] All discovery results are committed to the config store with rollback capability

---

## 8. Phase 2 — Model Generation Layer

**Duration:** 8 weeks  
**Goal:** Transform the HardwareProfile from Phase 1 into a complete, valid, simulation-verified robot model including URDF, SRDF, kinematic tree, and all MoveIt2 configuration files.

**Prerequisite:** Phase 1 complete and HardwareProfile available

### 8.1 Geometry Extraction Agent

**Agent ID:** `geometry_extraction_agent`

#### 8.1.1 Goals

1. Determine the physical dimensions of each robot link
2. Estimate joint axis orientations (DH-compatible)
3. Determine joint limits (soft and hard)
4. Identify the base mounting frame
5. Identify the tool center point (TCP)
6. Output complete geometry parameters with confidence scores per parameter

#### 8.1.2 Geometry Sources (in priority order)

**Source 1: Existing URDF/SDF (Confidence: 0.99)**

```python
if existing_urdf:
    extract_geometry_from_urdf(existing_urdf)
    confidence = 0.99
```

**Source 2: User-provided YAML (Confidence: 0.95)**

```yaml
robot:
  geometry:
    source: yaml
    links:
      - name: link1
        length: 0.150 # meters
        mass: 0.500 # kg
      - name: link2
        length: 0.250
        mass: 0.800
    joint_limits:
      - joint: joint1
        lower: -3.14159
        upper: 3.14159
        velocity: 1.0
        effort: 10.0
```

**Source 3: CAD-derived (Confidence: 0.97)**

- Accept STEP, STL, OBJ, or COLLADA files
- Run bounding-box analysis to estimate link dimensions
- Extract principal axes for joint orientation estimation
- Generate collision meshes (simplified convex hull or primitive approximation)

**Source 4: Motion-Based Geometric Calibration (Confidence: 0.70–0.85)**
This is the most complex case: the robot's geometry is entirely unknown.

```
MOTION CALIBRATION PROTOCOL:
1. Command each joint to move from -45° to +45° while others are held fixed
2. Track a fiducial marker (AprilTag 16h5) mounted at the end of each link
3. Using camera tracking, reconstruct the arc of motion
4. Fit a cylinder to the arc → extract link length and joint axis
5. Repeat for all joints
6. Build DH parameter table from accumulated observations
7. Report confidence per joint based on arc fitting residuals
```

**Source 5: Encoder-Derived Estimation (Confidence: 0.55–0.70)**
When no camera is available:

- Move each joint through full range while recording encoder counts
- Use physical arm constraint inference (arm returns to same point = same geometry)
- Bootstrap with reasonable anthropomorphic defaults scaled by arm weight class
- Clearly flag low-confidence parameters for user verification

#### 8.1.3 DH Parameter Extraction

For each joint, output Modified DH parameters:

```yaml
dh_parameters:
  - joint: "joint1"
    alpha: 0.0 # rad — twist angle
    a: 0.0 # m — link length
    d: 0.089159 # m — link offset
    theta: 0.0 # rad — joint angle (variable)
    confidence: 0.94
  - joint: "joint2"
    alpha: 1.5708
    a: 0.0
    d: 0.0
    theta: 0.0
    confidence: 0.91
  # ... etc.
```

### 8.2 Automatic URDF Generator

**Agent ID:** `urdf_generator_agent`

#### 8.2.1 Goals

1. Generate a complete, valid URDF from geometry parameters
2. Generate appropriate collision geometry (simple primitives or mesh-based)
3. Generate visual geometry with reasonable default materials
4. Generate inertial parameters (estimated from link dimensions and materials if not known)
5. Output as xacro for modularity and parametrization
6. Validate the URDF using `check_urdf` and `urdf_parser`

#### 8.2.2 URDF Generation Strategy

```python
class URDFGeneratorAgent:
    """
    Generates URDF using a template + geometry injection approach.
    All geometry parameters are injected into a Jinja2 xacro template.
    """

    def generate(self, geometry: GeometryProfile, hardware: HardwareProfile) -> URDFOutput:
        # 1. Select base template based on robot morphology
        template = self.select_template(geometry)  # serial | scara | delta | parallel

        # 2. Inject link geometry
        for link in geometry.links:
            self.inject_link(template, link, self.estimate_inertia(link))

        # 3. Inject joint parameters
        for joint in geometry.joints:
            self.inject_joint(template, joint)

        # 4. Generate collision geometry
        for link in geometry.links:
            collision = self.generate_collision_primitive(link)  # box or cylinder
            self.inject_collision(template, link, collision)

        # 5. Inject ros2_control tags
        self.inject_ros2_control_tags(template, hardware)

        # 6. Validate
        validation = self.validate_urdf(template.render())

        return URDFOutput(
            urdf=template.render(),
            validation=validation,
            confidence=self.compute_confidence(geometry, validation)
        )

    def estimate_inertia(self, link: LinkGeometry) -> InertialParameters:
        """
        Estimate inertial parameters using cylinder/cuboid approximation.
        I_xx = 1/12 * m * (3r² + h²)  for cylinder
        """
        density = self.estimate_density(link)   # Based on material guess (aluminum/steel)
        mass = density * link.volume
        inertia = InertialTensor.cylinder(mass, link.radius, link.length)
        return InertialParameters(mass=mass, inertia=inertia, confidence=0.65)
```

#### 8.2.3 Generated xacro Structure

```xml
<?xml version="1.0"?>
<!-- AUTO-GENERATED BY URAF v1.0 — DO NOT EDIT MANUALLY -->
<!-- Generated: 2024-01-01 | Robot: my_robot | Confidence: 0.92 -->
<robot name="${robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Parameters — safe to override via xacro:arg -->
  <xacro:arg name="prefix" default="" />
  <xacro:arg name="use_fake_hardware" default="false" />
  <xacro:arg name="fake_sensor_commands" default="false" />

  <!-- Robot geometry parameters -->
  <xacro:property name="d1" value="0.089159"/>
  <xacro:property name="a2" value="-0.42500"/>
  <!-- ... (all DH params) -->

  <!-- Materials -->
  <material name="uraf_default_link_color">
    <color rgba="0.7 0.7 0.75 1.0"/>
  </material>

  <!-- Base link -->
  <link name="${prefix}base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="..." ixy="0" ixz="0" iyy="..." iyz="0" izz="..."/>
    </inertial>
  </link>

  <!-- Joint 1 -->
  <joint name="${prefix}joint_1" type="revolute">
    <parent link="${prefix}base_link"/>
    <child link="${prefix}link_1"/>
    <origin xyz="0 0 ${d1}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" velocity="2.094" effort="150.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- ... (remaining links and joints) -->

  <!-- ros2_control tags -->
  <ros2_control name="${prefix}robot_hardware" type="system">
    <hardware>
      <plugin>uraf_hardware/${hardware_plugin}</plugin>
      <param name="interface_type">${interface_type}</param>
      <param name="device">${device}</param>
    </hardware>
    <!-- joint interfaces generated per DOF -->
    <xacro:macro name="joint_interface" params="name">
      <joint name="${prefix}${name}">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </xacro:macro>
  </ros2_control>

</robot>
```

### 8.3 SRDF Generator Agent

**Agent ID:** `srdf_generator_agent`

#### 8.3.1 Goals

1. Define planning groups (arm, end-effector)
2. Generate self-collision exclusion matrix
3. Define named robot states (home, ready, folded)
4. Define virtual joints and passive joints
5. Output a valid SRDF that MoveIt2 can consume directly

#### 8.3.2 Collision Matrix Generation

The collision matrix is generated by running a Monte Carlo sweep in simulation:

```python
def generate_collision_matrix(urdf, n_samples=10000):
    """
    Sample random configurations and check self-collision.
    Pairs that NEVER collide → mark as "never collides" (disabled).
    Pairs that ALWAYS collide in default pose → mark as "default".
    """
    robot = load_urdf_in_sim(urdf)
    collision_counts = defaultdict(int)

    for _ in range(n_samples):
        q = sample_random_config(robot.joint_limits)
        collisions = robot.check_self_collision(q)
        for pair in collisions:
            collision_counts[pair] += 1

    matrix = {}
    for pair, count in collision_counts.items():
        if count == 0:
            matrix[pair] = "never"          # Disable this pair check
        elif count == n_samples:
            matrix[pair] = "default"        # Always adjacent
        else:
            matrix[pair] = "active"         # Keep checking

    return matrix
```

### 8.4 Kinematics Agent

**Agent ID:** `kinematics_agent`

#### 8.4.1 Goals

1. Set up and configure multiple IK solvers for the generated robot model
2. Benchmark each solver on a standardized test suite
3. Automatically select the best solver based on performance profile
4. Generate solver-specific configuration files for MoveIt2
5. Provide fallback solver chain: primary → secondary → analytical backup

#### 8.4.2 IK Solver Benchmark Protocol

```
BENCHMARK SUITE (1000 test poses per solver):
├── Near-singular configurations (10%)
├── Full workspace random poses (60%)
├── Constrained poses (orientation-only, 15%)
├── Near joint-limit poses (10%)
└── Cartesian path segments (5%)

METRICS:
├── Success rate (%)
├── Solve time — mean, p50, p95, p99 (ms)
├── Solution quality — distance from seed state (rad)
├── Joint limit violations (count)
└── Path smoothness (for sequential IK)
```

#### 8.4.3 Solver Selection Matrix

| Robot Type             | Recommended Solver     | Fallback  | Notes                            |
| ---------------------- | ---------------------- | --------- | -------------------------------- |
| 6-DOF serial (UR-like) | IKFast (if generated)  | TRAC-IK   | IKFast gives analytical solution |
| 7-DOF redundant        | BioIK                  | TRAC-IK   | BioIK handles redundancy best    |
| 3–5 DOF                | KDL                    | TRAC-IK   | Simple robots, KDL is sufficient |
| Scara                  | Analytical (generated) | KDL       | Closed-form exists for SCARA     |
| Delta                  | Custom analytical      | Pinocchio | Delta needs special handling     |
| 8+ DOF                 | Pinocchio              | BioIK     | Pinocchio handles high-DOF well  |
| Unknown morphology     | TRAC-IK                | BioIK     | Safe default                     |

#### 8.4.4 Auto-Generated kinematics.yaml

```yaml
# AUTO-GENERATED by URAF Kinematics Agent
# Solver selected: trac_ik (benchmark score: 0.94)
# Benchmark date: 2024-01-01 | Robot: my_robot

robot_arm:
  kinematics_solver: trac_ik/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3

  # TRAC-IK specific
  solve_type: Speed # Speed | Distance | Manip1 | Manip2

  # Fallback solver
  kinematics_solver_fallback: kdl_kinematics_plugin/KDLKinematicsPlugin

  # Benchmark results (embedded for reference)
  # Primary solver: trac_ik
  #   success_rate: 99.7%
  #   mean_solve_time: 1.2ms
  #   p99_solve_time: 8.4ms
```

### 8.5 Phase 2 Success Criteria

- [ ] Generates valid URDF for 3–9 DOF serial arms from geometry YAML alone
- [ ] Generated URDF passes `check_urdf` validation 100% of the time
- [ ] Collision matrix generation completes in ≤ 60 seconds
- [ ] IK solver benchmark runs in ≤ 5 minutes and correctly identifies fastest solver
- [ ] Simulation-first validation detects and rejects invalid configs before physical deployment
- [ ] All generated files are stored in config store with full git history

---

## 9. Phase 3 — Control & Planning Layer

**Duration:** 8 weeks  
**Goal:** Configure a complete motion planning stack using MoveIt2, set up hardware controllers, implement trajectory generation, and establish the full safety layer.

**Prerequisite:** Phase 2 complete and validated URDF/SRDF available

### 9.1 MoveIt2 Configurator Agent

**Agent ID:** `moveit2_configurator_agent`

#### 9.1.1 Goals

1. Generate all MoveIt2 configuration files from the robot model
2. Configure all supported planners (OMPL, Pilz, CHOMP, STOMP, TrajOpt)
3. Set up the MoveIt2 move_group node with correct parameters
4. Configure the planning scene with workspace bounds
5. Set up the MoveIt2 servo (real-time Cartesian control) interface
6. Validate the complete MoveIt2 stack in simulation before deployment

#### 9.1.2 Generated MoveIt2 Configuration Suite

```
moveit2_config/
├── config/
│   ├── kinematics.yaml            ← from Kinematics Agent
│   ├── joint_limits.yaml          ← from Geometry Agent
│   ├── ompl_planning.yaml         ← planner configurations
│   ├── pilz_industrial_motion_planner_planning.yaml
│   ├── chomp_planning.yaml
│   ├── stomp_config.yaml
│   ├── trajopt_config.yaml
│   ├── controllers.yaml           ← from ros2_control configurator
│   ├── moveit_cpp.yaml            ← MoveIt2 C++ API config
│   ├── sensors_3d.yaml            ← from Perception Agent (Phase 4)
│   └── servo_config.yaml          ← MoveIt servo
└── launch/
    ├── move_group.launch.py
    ├── moveit_rviz.launch.py
    ├── servo.launch.py
    └── uraf_complete.launch.py    ← Master launch file
```

#### 9.1.3 Multi-Planner Configuration Strategy

```yaml
# ompl_planning.yaml (auto-generated, tuned per robot)
planner_configs:
  # Fast planners (< 500ms) — for simple motions
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0
  RRTstar:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05

  # Optimizing planners — for quality-critical paths
  BITstar:
    type: geometric::BITstar

  # Cartesian planners — for straight-line paths
  PRMstar:
    type: geometric::PRMstar

# Group-specific planner selection
robot_arm:
  default_planner_config: RRTConnect # Fast default
  planner_configs:
    - RRTConnect
    - RRTstar
    - BITstar
    - LBKPIECE
    - EST

# Auto-tuned timeouts based on robot workspace volume
planning_plugin: ompl_interface/OMPLPlanner
request_adapters:
  - default_planner_request_adapters/AddTimeOptimalParameterization
  - default_planner_request_adapters/ResolveConstraintFrames
  - default_planner_request_adapters/FixWorkspaceBounds
  - default_planner_request_adapters/FixStartStateBounds
  - default_planner_request_adapters/FixStartStateCollision
  - default_planner_request_adapters/FixStartStatePathConstraints
```

### 9.2 ros2_control Configurator Agent

**Agent ID:** `ros2_control_configurator_agent`

#### 9.2.1 Goals

1. Generate ros2_control hardware interface configuration
2. Configure the JointTrajectoryController for arm motion
3. Configure GripperActionController for end-effector
4. Configure JointStateBroadcaster for state publishing
5. Set up controller spawning and lifecycle management
6. Tune PID parameters per joint (or load from FAL defaults)

#### 9.2.2 Auto-Generated Controller Configuration

```yaml
# controllers.yaml (auto-generated)
controller_manager:
  ros__parameters:
    update_rate: 500 # Hz — auto-set based on hardware capability

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    robot_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: gripper_action_controller/GripperActionController

robot_arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    # PID gains — auto-tuned or loaded from FAL defaults
    gains:
      joint_1: { p: 100.0, i: 0.1, d: 10.0, i_clamp: 1.0 }
      joint_2: { p: 150.0, i: 0.1, d: 12.0, i_clamp: 1.0 }
      # ...
    open_loop_control: false
    allow_partial_joints_goal: false
    allow_integration_in_goal_trajectories: false
```

### 9.3 Trajectory Generation Agent

**Agent ID:** `trajectory_agent`

#### 9.3.1 Goals

1. Implement multiple trajectory interpolation profiles
2. Integrate Ruckig for online time-optimal trajectory generation
3. Post-process MoveIt2 trajectories with time parameterization
4. Implement Cartesian trajectory generation with linear blending
5. Support dynamic replanning with trajectory splicing

#### 9.3.2 Trajectory Profile Library

**Profile 1: Minimum Jerk (Polynomial)**

```
s(t) = 10t³ − 15t⁴ + 6t⁵
```

Guarantees zero acceleration at start/end. Used for slow, precise motions.

**Profile 2: S-Curve (Jerk-Limited)**
Trapezoidal velocity with smooth transitions. Used for most general motions.

**Profile 3: Time-Optimal (Ruckig)**

```python
# Integration via Ruckig Python bindings
from ruckig import Ruckig, Trajectory, InputParameter, OutputParameter

ruckig = Ruckig(dof=6, delta_time=0.001)
inp = InputParameter(6)
inp.current_position = q_start
inp.current_velocity = [0.0] * 6
inp.current_acceleration = [0.0] * 6
inp.target_position = q_goal
inp.target_velocity = [0.0] * 6
inp.target_acceleration = [0.0] * 6
inp.max_velocity = joint_velocity_limits
inp.max_acceleration = joint_acceleration_limits
inp.max_jerk = joint_jerk_limits

out = OutputParameter(6)
result = ruckig.calculate(inp, out)
```

**Profile 4: Quintic Spline (Multi-Waypoint)**
For smooth multi-waypoint paths. Each segment is a quintic polynomial with continuous position, velocity, and acceleration.

### 9.4 Safety Agent

**Agent ID:** `safety_agent`

#### 9.4.1 Goals

1. Enforce hard and soft joint limits at the hardware interface level
2. Implement workspace boundary checking
3. Monitor joint torques and trigger soft stops on overload
4. Monitor motor temperatures via FAL and issue thermal warnings
5. Implement hardware and software emergency stop
6. Enforce velocity limits in Cartesian space near obstacles
7. Implement watchdog timeout for controller heartbeat
8. Generate safety audit logs for all motion commands

#### 9.4.2 Safety Architecture (Layered)

```
LAYER 1 — Hardware (< 1ms):
  Firmware-level current/torque cutoffs (configured via FAL)

LAYER 2 — HAL (1–5ms):
  ros2_control command interface limits
  JointLimits enforcement plugin

LAYER 3 — Planning (5–100ms):
  MoveIt2 CollisionChecking
  Workspace bounding box
  Self-collision SRDF rules

LAYER 4 — Supervisory (100ms–1s):
  Safety Agent monitoring loop
  Watchdog for controller heartbeat
  Thermal monitoring
  Force/torque anomaly detection

LAYER 5 — Application:
  Emergency stop API (ROS2 action)
  Safety status topic
  Operator acknowledgement required for reset
```

#### 9.4.3 Emergency Stop Implementation

```yaml
# E-stop configuration (auto-generated)
safety:
  estop:
    software_estop_topic: "/uraf/emergency_stop"
    hardware_estop_gpio: null # Set if GPIO is available
    estop_type: "controlled_stop" # controlled_stop | power_cut
    controlled_stop_deceleration: 5.0 # rad/s²

  joint_monitoring:
    torque_limit_multiplier: 1.5 # Soft stop at 150% rated torque
    temperature_warning_c: 75.0
    temperature_critical_c: 85.0

  watchdog:
    controller_timeout_ms: 100
    action_on_timeout: "estop"

  workspace:
    auto_infer: true # Infer from geometry + safety margin
    safety_margin: 0.05 # 5cm margin inside physical limits
```

### 9.5 Phase 3 Success Criteria

- [ ] MoveIt2 successfully plans and executes motions for all test robots
- [ ] At least 3 planners (OMPL, Pilz, CHOMP) working and switchable at runtime
- [ ] Ruckig trajectory generation achieves < 5ms compute time for 6-DOF
- [ ] Safety layer triggers controlled stop within 50ms of estop signal
- [ ] Controller update rate achieves ≥ 250Hz on standard hardware
- [ ] All configurations load without errors from a cold start via master launch file

---

## 10. Phase 4 — Perception & Environment Mapping

**Duration:** 6 weeks  
**Goal:** Detect all connected sensors, configure perception pipelines, build real-time environment maps, and enable vision-guided motion.

**Prerequisite:** Phase 3 complete

### 10.1 Vision Agent

**Agent ID:** `vision_agent`

#### 10.1.1 Goals

1. Enumerate all connected cameras (RGB, RGBD, stereo, event)
2. Identify camera hardware type and load appropriate driver
3. Configure image processing pipelines
4. Register camera frames in the TF tree
5. Configure camera intrinsics (auto-calibrate if needed)
6. Set up RealSense, Orbbec, ZED SDK integrations
7. Enable visual servoing streams for MoveIt2

#### 10.1.2 Camera Detection Matrix

| Camera Type               | Detection Method                   | Driver           | Topics Generated                                              |
| ------------------------- | ---------------------------------- | ---------------- | ------------------------------------------------------------- |
| Intel RealSense D435/D455 | USB VID:PID 8086:0B3A              | realsense2_ros   | `/color/image_raw`, `/depth/image_raw`, `/depth/color/points` |
| Intel RealSense L515      | USB VID:PID 8086:0B64              | realsense2_ros   | LiDAR-based depth + RGB                                       |
| Orbbec Astra              | USB VID:PID 2BC5                   | orbbec_ros2      | RGB + structured light depth                                  |
| ZED 2i / ZED X            | USB 3.0 stereo pair                | zed_ros2_wrapper | Stereo + depth + IMU                                          |
| Azure Kinect              | USB VID:PID 045E:097D              | k4a_ros2         | RGB + ToF depth                                               |
| OAK-D (Luxonis)           | USB VID:PID 03E7                   | depthai_ros      | Stereo + RGB + Neural AI                                      |
| Generic UVC               | Linux V4L2                         | usb_cam          | `/image_raw`                                                  |
| IP Camera                 | RTSP stream                        | rtsp_ros         | `/image_raw`                                                  |
| Wrist Camera              | Same as above + TF parent = flange | Same             | + `/wrist_camera/...`                                         |

#### 10.1.3 Automatic Camera Calibration

When a new camera is detected:

**Intrinsic Calibration:**

1. Display checkerboard pattern on screen (if monitor available) OR
2. Print PDF checkerboard and prompt user to move it
3. Collect 20+ images at varied angles and distances
4. Run `camera_calibration` (OpenCV-based) automatically
5. Save calibration file to config store

**Extrinsic Calibration (Camera ↔ Robot):**
See Self-Calibration Agent (Section 10.3)

#### 10.1.4 Auto-Generated sensors_3d.yaml for MoveIt2

```yaml
# sensors_3d.yaml — AUTO-GENERATED by URAF Vision Agent
sensors:
  - sensor_plugin: occupancy_map_updater/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 5.0
    point_subsample: 1
    padding_offset: 0.1
    padding_scale: 1.0
    max_update_rate: 1.0
    filtered_cloud_topic: /camera/depth/color/filtered_points
```

### 10.2 Mapping Agent

**Agent ID:** `mapping_agent`

#### 10.2.1 Goals

1. Build and maintain a 3D occupancy map of the robot's workspace
2. Update the MoveIt2 planning scene with dynamic obstacles
3. Support multiple mapping backends (Octomap, Voxblox, TSDF)
4. Generate semantic maps if a semantic segmentation pipeline is available
5. Detect and track dynamic objects

#### 10.2.2 Mapping Backend Selection

```python
def select_mapping_backend(context: SystemContext) -> MappingBackend:
    if context.gpu_available and context.ram_gb >= 16:
        if context.camera_type in ["realsense", "zed", "oak"]:
            return VoxbloxBackend(voxel_size=0.05)   # Dense, GPU-accelerated
    elif context.camera_type in ["realsense", "orbbec"]:
        return OctomapBackend(resolution=0.05)        # CPU-friendly
    else:
        return OctomapBackend(resolution=0.10)        # Minimal, CPU
```

#### 10.2.3 Dynamic Obstacle Tracking

```
Point Cloud Input
      │
      ▼
Background Subtraction (using static map)
      │
      ▼
Cluster Detection (Euclidean clustering, min_cluster_size=50)
      │
      ▼
Object Tracker (Kalman filter per cluster)
      │
      ▼
Predicted Positions → MoveIt2 Planning Scene Update
      │
      ▼
Velocity-Aware Collision Margins (expand bounding box by velocity × lookahead_time)
```

### 10.3 Self-Calibration Agent

**Agent ID:** `calibration_agent`

#### 10.3.1 Goals

1. Calibrate joint zero positions (home/offset calibration)
2. Calibrate the Tool Center Point (TCP)
3. Perform hand-eye calibration (camera-to-robot base transform)
4. Calibrate wrist-mounted camera (eye-in-hand)
5. Verify calibration quality and flag degraded calibrations
6. Re-trigger calibration on demand or on schedule

#### 10.3.2 Joint Zero Calibration

**Method A: Mechanical hard-stop detection**

```
For each joint:
1. Move slowly toward mechanical limit at 5% velocity
2. Detect stall via torque increase OR current spike
3. Record encoder position at stall = mechanical limit
4. Apply known mechanical offset → compute zero position
5. Update joint_offsets.yaml
```

**Method B: Vision-based fiducial alignment**

```
1. Mount AprilTag 36h11 family marker on link 1
2. Command known joint positions
3. Camera observes marker
4. Reprojection error minimization → solve for joint offsets
5. Iterate until reprojection error < 1 pixel
```

#### 10.3.3 TCP Calibration (4-Point Method)

```
1. Mount a calibration pin (sharp point) somewhere in workspace
2. Command robot to touch the point from 4 different orientations
3. All 4 positions must be IDENTICAL in robot base frame
4. Minimize the position error:
   minimize: Σ ||T_base_flange_i × T_flange_tcp − p_fixed||²
5. Solved via nonlinear least squares → output T_flange_tcp
6. Save to tcp_calibration.yaml
```

#### 10.3.4 Hand-Eye Calibration (Eye-to-Hand)

```
Method: Tsai-Lenz / Park-Martin (ROS hand_eye_calibration package)

For each of N poses (N ≥ 15):
1. Command robot to random pose
2. Detect calibration target (AprilTag / ChArUco board)
3. Record: robot_pose_i (from forward kinematics) + target_pose_in_camera_i

Solve:  AX = XB   (Tsai-Lenz)
Where:
  A = robot_pose_i^{-1} × robot_pose_j
  B = target_pose_i × target_pose_j^{-1}
  X = unknown camera-to-base transform

Output: T_base_camera → saved to camera_extrinsics.yaml
```

### 10.4 Phase 4 Success Criteria

- [ ] Auto-detects RealSense D435/D455, Orbbec Astra, and ZED cameras within 10 seconds
- [ ] Intrinsic camera calibration achieves reprojection error < 0.5 pixels
- [ ] Hand-eye calibration achieves position accuracy < 5mm at 500mm range
- [ ] Octomap updates at ≥ 2Hz on standard CPU hardware
- [ ] MoveIt2 correctly avoids dynamically added obstacles in planning scene
- [ ] TCP calibration achieves < 3mm repeatability

---

## 11. Phase 5 — Intelligence & Autonomous Capabilities

**Duration:** 10 weeks  
**Goal:** Add AI-powered perception, grasp planning, task execution, and high-level autonomy. Enable the robot to perform complex tasks from natural language instructions.

**Prerequisite:** Phase 4 complete

### 11.1 Perception Intelligence Agent

**Agent ID:** `perception_intelligence_agent`

#### 11.1.1 Goals

1. Detect and classify objects in the workspace using YOLO-based detection
2. Perform instance segmentation using SAM2
3. Estimate 6-DOF object poses using FoundationPose / MegaPose
4. Track objects through occlusions
5. Publish perceived objects to MoveIt2 planning scene
6. Maintain a semantic workspace model

#### 11.1.2 Perception Pipeline

```
RGB Image Input
      │
      ├──► Object Detection (YOLO v11 / RT-DETR)
      │         │
      │         ▼
      │    Bounding Boxes + Class Labels + Confidence
      │
      ├──► Instance Segmentation (SAM2)
      │         │
      │         ▼
      │    Per-Object Masks
      │
      ├──► Depth Fusion (RGBD backprojection)
      │         │
      │         ▼
      │    Per-Object Point Clouds
      │
      └──► Pose Estimation (FoundationPose / MegaPose)
                │
                ▼
           6-DOF Pose per Object (T_base_object)
                │
                ▼
           Tracked Objects → Planning Scene → Grasp Agent
```

#### 11.1.3 YOLO Integration (Model Auto-Selection)

```python
class PerceptionModel:
    """Auto-selects YOLO variant based on hardware"""

    def select_model(self, hardware: HardwareProfile) -> YOLOVariant:
        if hardware.gpu_vram_gb >= 8:
            return YOLOv11x()              # Highest accuracy
        elif hardware.gpu_vram_gb >= 4:
            return YOLOv11l()              # Good balance
        elif hardware.gpu_available:
            return YOLOv11m()              # Medium
        else:
            return YOLOv11n()              # Nano, CPU-only
```

#### 11.1.4 Custom Object Registration

Users can register custom objects without retraining:

```yaml
# objects.yaml — register custom objects
custom_objects:
  - name: "red_cube"
    dimensions: [0.05, 0.05, 0.05] # meters
    reference_image: "red_cube_ref.jpg"
    pose_estimation: "foundationpose" # or "template_matching"
    grasp_approach: "top_down"

  - name: "gear_assembly"
    cad_model: "gear_assembly.obj"
    pose_estimation: "megapose"
    grasp_approach: "side_grasp"
```

### 11.2 Grasp Planning Agent

**Agent ID:** `grasp_planning_agent`

#### 11.2.1 Goals

1. Generate grasp candidates for detected objects using AnyGrasp / GraspNet
2. Filter grasps by kinematic reachability
3. Filter grasps by collision-free pre-grasp approach
4. Rank grasps by success probability and execution quality
5. Execute the best available grasp via MoveIt2
6. Learn from grasp outcomes (success/failure feedback)

#### 11.2.2 Grasp Generation Pipeline

```
Object Pose + Point Cloud
        │
        ▼
AnyGrasp / GraspNet
(Generate N grasp candidates)
        │
        ▼
Reachability Filter
(IK exists for each candidate?)
        │
        ▼
Collision Filter
(Pre-grasp and approach are collision-free?)
        │
        ▼
Quality Ranking
(Score = grasp_quality × reachability_score × approach_clearance)
        │
        ▼
Execute Top Grasp via MoveIt2 Action Server
        │
     ┌──▼──┐
     │     │
   SUCCESS FAILURE
     │     │
     ▼     ▼
  Store  Try next
  result candidate
  (learning)
```

#### 11.2.3 Supported Grasp Types

| Type                 | Use Case         | Required EE       | Planning Method              |
| -------------------- | ---------------- | ----------------- | ---------------------------- |
| Top-down power grasp | Box pick         | Parallel gripper  | AnyGrasp + MoveIt2 Cartesian |
| Side grasp           | Cylinder objects | Parallel gripper  | AnyGrasp                     |
| Pinch grasp          | Small parts      | Precision gripper | GraspNet                     |
| Vacuum pick          | Flat surfaces    | Vacuum gripper    | Surface normal estimation    |
| Tool grasp           | Handles, tools   | Adaptive gripper  | Template matching            |
| Bin picking          | Cluttered tray   | Any gripper       | GraspNet with point cloud    |

### 11.3 Task Execution Agent

**Agent ID:** `task_execution_agent`

#### 11.3.1 Goals

1. Parse high-level task descriptions into executable motion primitives
2. Generate multi-step task plans using LLM reasoning
3. Execute plans with error recovery
4. Monitor task progress and adapt to failures
5. Learn successful task sequences for future reuse

#### 11.3.2 Task Primitive Library

```python
PRIMITIVE_LIBRARY = {
    "MOVE_TO_POSE":     MoveToPoseAction,       # MoveIt2 goal pose
    "MOVE_JOINT":       MoveJointAction,         # Joint-space motion
    "MOVE_CARTESIAN":   CartesianPathAction,     # Linear Cartesian path
    "PICK":             PickAction,              # Integrated grasp + lift
    "PLACE":            PlaceAction,             # Move + release + retreat
    "OPEN_GRIPPER":     GripperAction(open=True),
    "CLOSE_GRIPPER":    GripperAction(open=False),
    "WAIT":             WaitAction,
    "HOME":             HomeAction,
    "SCAN":             PerceptionScanAction,    # Turn head, update scene
    "APPROACH":         ApproachAction,          # Collision-free approach
    "RETREAT":          RetreatAction,           # Safe post-action retreat
}
```

#### 11.3.3 LLM-Based Task Planning

```python
class TaskPlanningAgent:
    """
    Uses LLM to decompose natural language tasks into primitive sequences.
    """
    SYSTEM_PROMPT = """
    You are a robot task planner. You have access to these primitives:
    {primitive_descriptions}

    Current workspace objects: {scene_objects}
    Robot capabilities: {robot_capabilities}

    Decompose the user's task into an ordered list of primitives.
    Return JSON: {"plan": [{"primitive": "...", "params": {...}}, ...]}
    Include error recovery alternatives for each step.
    """

    def plan(self, task: str, scene: WorkspaceScene) -> TaskPlan:
        response = llm.complete(
            system=self.SYSTEM_PROMPT.format(...),
            user=task
        )
        plan = json.loads(response)
        return TaskPlan(plan["plan"])
```

### 11.4 Natural Language Interface Agent

**Agent ID:** `nl_interface_agent`

#### 11.4.1 Goals

1. Accept natural language commands from the user
2. Parse intent (motion command / config change / status query / system command)
3. Route to appropriate subsystem
4. Provide natural language feedback on outcomes
5. Support voice input via speech recognition (Whisper)

#### 11.4.2 Command Categories

```
"Move to home position"               → Task Execution Agent (HOME primitive)
"Pick up the red cube"                → Perception + Grasp + Task Agent
"How many joints does this robot have?" → Status Query → State Manager
"Is joint 3 inverted?"                → Config Query → AI Config Agent
"Calibrate the TCP"                   → Calibration Agent
"Enable collision checking"           → MoveIt2 Configurator
"Show me what the camera sees"        → Vision Agent → GUI stream
"Set maximum velocity to 50%"         → Safety Agent → Controller update
"What went wrong with that motion?"   → Log analysis → LLM explanation
```

### 11.5 AI Configuration Agent

**Agent ID:** `ai_config_agent`

#### 11.5.1 Goals

1. Modify URDF, SRDF, and MoveIt2 configs in response to user feedback
2. Diagnose broken configurations and propose fixes
3. Answer robot-specific questions from the user
4. Generate missing configuration files
5. Perform config validation after every modification

#### 11.5.2 Self-Healing Configuration Loop

```
Config Failure Detected
        │
        ▼
Error Log Analysis (LLM parses stderr/rosout)
        │
        ▼
Root Cause Identification
(e.g., "URDF joint_2 axis is inverted — FK gives wrong sign")
        │
        ▼
Proposed Fix Generation
("I will flip the axis of joint_2 from xyz='0 0 1' to xyz='0 0 -1'")
        │
     ┌──▼──┐
     │ Confidence ≥ 0.85?
     │
    Yes      No
     │        │
     ▼        ▼
  Auto-fix  User confirmation
     │        │
     └────────┘
           │
           ▼
    Apply fix to config file
           │
           ▼
    Git commit with explanation
           │
           ▼
    Re-run simulation validation
           │
        Pass?
       /     \
      Yes     No
       │       │
    Deploy   Repeat (max 5 attempts → escalate to user)
```

### 11.6 Phase 5 Success Criteria

- [ ] YOLO detection achieves ≥ 85% mAP on a standard household object benchmark
- [ ] AnyGrasp achieves ≥ 80% grasp success rate on known objects
- [ ] LLM task planner correctly decomposes 20/20 standard manipulation tasks
- [ ] Natural language commands parsed with ≥ 90% intent accuracy
- [ ] AI Config Agent correctly diagnoses and fixes 15/20 seeded config errors
- [ ] Pick-and-place cycle time < 10 seconds end-to-end for standard objects

---

## 12. Phase 6 — Self-Healing, Learning & Production

**Duration:** 8 weeks  
**Goal:** Harden the system for long-running production deployments, implement self-healing, add learning from experience, and enable multi-robot support.

**Prerequisite:** Phase 5 complete

### 12.1 Self-Healing Agent

**Agent ID:** `self_healing_agent`

#### 12.1.1 Goals

1. Continuously monitor all system components for health
2. Detect configuration drift (robot behavior deviating from model)
3. Detect hardware degradation (backlash, encoder drift, motor wear)
4. Automatically trigger re-calibration when drift exceeds threshold
5. Restart failed ROS2 nodes automatically
6. Alert the user when automatic recovery is not possible

#### 12.1.2 Health Monitoring Metrics

```yaml
monitored_metrics:
  # Joint-level
  joint_position_error_rms:
    threshold_warn: 0.005 # rad
    threshold_crit: 0.020 # rad → trigger recalibration

  joint_velocity_error_rms:
    threshold_warn: 0.010 # rad/s
    threshold_crit: 0.050 # rad/s

  motor_temperature:
    threshold_warn: 75 # °C
    threshold_crit: 85 # °C → controlled stop

  # Controller health
  controller_loop_jitter:
    threshold_warn: 1.0 # ms
    threshold_crit: 5.0 # ms → restart controller

  # Calibration health
  tcp_drift:
    threshold_warn: 0.003 # m (3mm)
    threshold_crit: 0.010 # m (10mm) → force recalibration

  hand_eye_reprojection_error:
    threshold_warn: 2.0 # pixels
    threshold_crit: 5.0 # pixels → force camera recalibration

  # Perception health
  detection_confidence_rolling_avg:
    threshold_warn: 0.65
    threshold_crit: 0.50 # Camera may be dirty or moved
```

### 12.2 Learning Agent

**Agent ID:** `learning_agent`

#### 12.2.1 Goals

1. Record all task executions and their outcomes
2. Learn task-specific grasp success models (which grasps work for which objects)
3. Learn joint-specific controller tuning (auto-tune PIDs from execution data)
4. Learn workspace-specific planning heuristics (which planners work best for this robot)
5. Export learned knowledge to the Community Robot Library (with user consent)

#### 12.2.2 Learning Architecture

```
Execution Event Stream
      │
      ├── Grasp Outcome Recorder
      │     → Updates: grasp_success_model_<object_class>.pkl
      │
      ├── Trajectory Quality Scorer
      │     → Updates: planner_performance_model.json
      │
      ├── Controller Performance Logger
      │     → Feeds: auto_pid_tuner (Bayesian optimization)
      │
      └── Task Success Recorder
            → Updates: task_plan_library.json
```

### 12.3 Multi-Robot Support

**Agent ID:** `multi_robot_coordinator`

#### 12.3.1 Goals

1. Support 2+ robot arms sharing a planning scene
2. Manage robot namespaces to prevent topic collisions
3. Coordinate motions to avoid inter-robot collisions
4. Support mobile manipulator configurations (base + arm)
5. Support gantry/SCARA multi-axis systems

#### 12.3.2 Multi-Robot Configuration

```yaml
# uraf_config.yaml for dual-arm setup
robots:
  - name: "left_arm"
    namespace: "/left"
    config: "ur5e_config"
    base_frame_pose:
      x: -0.3
      y: 0.0
      z: 0.0
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

  - name: "right_arm"
    namespace: "/right"
    config: "ur5e_config"
    base_frame_pose:
      x: 0.3
      y: 0.0
      z: 0.0
      roll: 0.0
      pitch: 0.0
      yaw: 3.14159 # Facing opposite direction

multi_robot:
  shared_planning_scene: true
  coordination_strategy: "prioritized" # prioritized | coupled | decoupled
  priority: ["right_arm", "left_arm"] # Higher priority plans first
```

### 12.4 Phase 6 Success Criteria

- [ ] Self-healing agent detects and recovers from ≥ 85% of common failure modes automatically
- [ ] TCP drift detection triggers recalibration within 5 minutes of exceeding threshold
- [ ] Grasp success model improves by ≥ 15% after 100 training attempts vs baseline
- [ ] Dual-arm setup correctly avoids inter-robot collisions in 100% of test cases
- [ ] System runs continuously for ≥ 72 hours without human intervention in endurance test

---

## 13. Detailed Agent Specifications

### Summary Table — All Agents

| Agent ID                          | Phase | Priority | Confidence Target | Self-Healing | LLM-Powered |
| --------------------------------- | ----- | -------- | ----------------- | ------------ | ----------- |
| `master_orchestrator`             | 0     | Critical | N/A               | Yes          | Yes         |
| `hardware_discovery_agent`        | 1     | Critical | ≥ 0.90            | Yes          | No          |
| `firmware_detection_agent`        | 1     | Critical | ≥ 0.85            | No           | Partial     |
| `ros2_discovery_agent`            | 1     | High     | ≥ 0.95            | No           | No          |
| `end_effector_agent`              | 1     | High     | ≥ 0.75            | No           | Partial     |
| `geometry_extraction_agent`       | 2     | Critical | ≥ 0.80            | Yes          | Partial     |
| `urdf_generator_agent`            | 2     | Critical | ≥ 0.85            | Yes          | Yes         |
| `srdf_generator_agent`            | 2     | High     | ≥ 0.90            | Yes          | No          |
| `kinematics_agent`                | 2     | Critical | ≥ 0.90            | Yes          | No          |
| `moveit2_configurator_agent`      | 3     | Critical | ≥ 0.90            | Yes          | Partial     |
| `ros2_control_configurator_agent` | 3     | Critical | ≥ 0.90            | Yes          | No          |
| `trajectory_agent`                | 3     | High     | ≥ 0.95            | No           | No          |
| `safety_agent`                    | 3     | Critical | ≥ 0.99            | Yes          | No          |
| `vision_agent`                    | 4     | High     | ≥ 0.80            | Yes          | No          |
| `mapping_agent`                   | 4     | Medium   | ≥ 0.75            | Yes          | No          |
| `calibration_agent`               | 4     | High     | ≥ 0.85            | Yes          | Partial     |
| `perception_intelligence_agent`   | 5     | High     | ≥ 0.75            | No           | No          |
| `grasp_planning_agent`            | 5     | High     | ≥ 0.70            | Yes          | No          |
| `task_execution_agent`            | 5     | Medium   | ≥ 0.80            | Yes          | Yes         |
| `nl_interface_agent`              | 5     | Low      | ≥ 0.85            | No           | Yes         |
| `ai_config_agent`                 | 5     | Medium   | ≥ 0.75            | N/A          | Yes         |
| `self_healing_agent`              | 6     | High     | ≥ 0.90            | Self         | Partial     |
| `learning_agent`                  | 6     | Medium   | N/A               | No           | No          |
| `multi_robot_coordinator`         | 6     | Medium   | ≥ 0.85            | Yes          | No          |

---

## 14. Firmware Abstraction Layer (FAL)

### 14.1 Architecture

The FAL is the most critical infrastructure component. It ensures all hardware variety is hidden behind a uniform interface.

```
┌─────────────────────────────────────────────────────────────────┐
│                    FAL UNIFIED INTERFACE                         │
│                                                                  │
│  write_command(joint_id, mode, value) → bool                    │
│  read_state(joint_id) → JointState                              │
│  set_control_mode(joint_id, mode) → bool                        │
│  enable_torque(joint_id, enable) → bool                         │
│  get_diagnostics(joint_id) → JointDiagnostics                   │
│  emergency_stop() → void                                         │
│  firmware_update(joint_id, firmware_path) → bool                │
└────────────────────────────────┬────────────────────────────────┘
                                 │
        ┌────────────────────────┼───────────────────────────┐
        │                        │                           │
┌───────▼──────┐      ┌──────────▼──────┐         ┌────────▼──────┐
│ fal_dynamixel│      │ fal_canopen_402 │         │fal_ros2_control│
│              │      │                 │         │               │
│ Protocol:    │      │ Protocol:       │         │ Protocol:     │
│ Serial       │      │ CAN             │         │ ROS2 topics   │
│ 0xFF 0xFF    │      │ CiA 402         │         │ + services    │
│              │      │                 │         │               │
│ Supports:    │      │ Supports:       │         │ Supports:     │
│ XL430, XM540 │      │ Maxon, Faulhaber│         │ Any existing  │
│ MX, AX, RX   │      │ Beckhoff, Yaskawa│        │ ros2_control  │
└──────────────┘      └─────────────────┘         │ hardware      │
                                                   └───────────────┘
```

### 14.2 FAL Driver Interface (Abstract Base Class)

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

class ControlMode(Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    TORQUE   = "torque"
    CURRENT  = "current"

@dataclass
class JointState:
    position: float      # rad or m
    velocity: float      # rad/s or m/s
    effort:   float      # Nm or N
    temperature: float   # °C
    error_code: int      # 0 = no error

@dataclass
class JointDiagnostics:
    firmware_version: str
    motor_temperature: float
    driver_temperature: float
    input_voltage: float
    current: float
    error_flags: list[str]

class FALDriver(ABC):
    """Abstract base class for all FAL drivers."""

    @abstractmethod
    def connect(self, config: dict) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def enable(self, joint_id: int) -> bool: ...

    @abstractmethod
    def disable(self, joint_id: int) -> bool: ...

    @abstractmethod
    def set_control_mode(self, joint_id: int, mode: ControlMode) -> bool: ...

    @abstractmethod
    def write_command(self, joint_id: int, value: float) -> bool: ...

    @abstractmethod
    def read_state(self, joint_id: int) -> JointState: ...

    @abstractmethod
    def get_diagnostics(self, joint_id: int) -> JointDiagnostics: ...

    @abstractmethod
    def emergency_stop(self) -> None: ...

    @abstractmethod
    def firmware_update(self, joint_id: int, firmware_path: str) -> bool: ...
```

### 14.3 PID Auto-Tuning per Driver

Each FAL driver reports its default PID gains (from firmware) and URAF optionally runs a closed-loop step-response test to refine them:

```
Step Response Test:
1. Command step input: position Δ = 0.1 rad
2. Record response over 500ms at 1kHz
3. Estimate: rise_time, overshoot, settling_time, steady_state_error
4. Apply Ziegler-Nichols tuning rules:
   Kp = 0.6 × Ku
   Ki = 1.2 × Ku / Tu
   Kd = 0.075 × Ku × Tu
5. Update gains in controller config
6. Re-test to verify improvement
```

---

## 15. Technology Stack & Dependency Map

### 15.1 Core Platform

| Component        | Technology                     | Version | License    |
| ---------------- | ------------------------------ | ------- | ---------- |
| OS               | Ubuntu 24.04 LTS               | 24.04   | GPL        |
| ROS              | ROS2 Jazzy Jalisco             | 0.1     | Apache 2.0 |
| Motion Planning  | MoveIt2                        | 2.x     | BSD        |
| Hardware Control | ros2_control                   | 4.x     | Apache 2.0 |
| URDF Processing  | urdfdom, urdf_parser           | latest  | BSD        |
| Kinematics       | TRAC-IK, BioIK, KDL, Pinocchio | latest  | Mixed      |
| Trajectory       | Ruckig                         | 0.14    | MIT        |
| Simulation       | Gazebo Harmonic                | 8.x     | Apache 2.0 |
| Simulation Alt   | MuJoCo                         | 3.x     | Apache 2.0 |
| Visualization    | RViz2                          | latest  | BSD        |

### 15.2 Perception Stack

| Component           | Technology                     | Version | License        |
| ------------------- | ------------------------------ | ------- | -------------- |
| Camera (RealSense)  | librealsense2 + realsense2_ros | 2.55+   | Apache 2.0     |
| Camera (ZED)        | ZED SDK + zed_ros2_wrapper     | 4.x     | MIT            |
| Camera (Orbbec)     | OrbbecSDK + orbbec_ros2        | latest  | Apache 2.0     |
| Object Detection    | Ultralytics YOLO v11           | 11.x    | AGPL-3.0       |
| Segmentation        | SAM2 (Meta)                    | 2.x     | Apache 2.0     |
| Pose Estimation     | FoundationPose (NVIDIA)        | latest  | NVIDIA         |
| Pose Estimation Alt | MegaPose                       | latest  | BSD            |
| Grasp Planning      | AnyGrasp                       | latest  | Non-commercial |
| Grasp Planning Alt  | GraspNet                       | latest  | MIT            |
| 3D Mapping          | Octomap                        | 1.10    | BSD            |
| 3D Mapping Alt      | Voxblox                        | latest  | BSD            |
| Calibration         | hand_eye_calibration           | latest  | BSD            |

### 15.3 AI / LLM Stack

| Component             | Technology             | Purpose                      |
| --------------------- | ---------------------- | ---------------------------- |
| Local LLM             | Ollama + Llama 3.1 70B | Orchestration, config repair |
| Remote LLM (optional) | OpenAI GPT-4o API      | Higher capability fallback   |
| Speech-to-Text        | OpenAI Whisper (local) | Voice commands               |
| Embedding Model       | sentence-transformers  | Config similarity search     |
| Agent Framework       | Custom (ROS2-native)   | Agent orchestration          |
| Vector DB             | ChromaDB               | Config history search        |

### 15.4 Infrastructure

| Component        | Technology              | Purpose                     |
| ---------------- | ----------------------- | --------------------------- |
| Containerization | Docker + docker-compose | Reproducible environment    |
| Config Store     | Git (libgit2)           | Version control for configs |
| State Manager    | Redis 7                 | Shared agent state          |
| Agent Bus        | ROS2 + Redis Pub/Sub    | Dual-channel comms          |
| GUI Backend      | FastAPI (Python)        | REST API for GUI            |
| GUI Frontend     | React + Tailwind        | Web-based wizard            |
| 3D Viewport      | three.js / Babylon.js   | Live 3D robot view          |
| CI/CD            | GitHub Actions          | Config validation           |

---

## 16. Data Flow & Inter-Agent Communication

### 16.1 Complete System Data Flow

```
USER INPUT (GUI / YAML / NL)
         │
         ▼
MASTER ORCHESTRATOR
         │ Dispatches discovery pipeline
         ▼
HARDWARE DISCOVERY AGENT ──► FIRMWARE DETECTION AGENT
         │                              │
         ▼                              ▼
  HardwareProfile                 FAL Driver Selected
         │                              │
         └──────────────┬───────────────┘
                        ▼
             GEOMETRY EXTRACTION AGENT
             (motion tests / YAML / URDF)
                        │
                        ▼ GeometryProfile
             URDF GENERATOR AGENT ──► check_urdf
                        │
                        ▼ URDF + SRDF
           ┌────────────┴────────────┐
           ▼                         ▼
  KINEMATICS AGENT         SRDF GENERATOR AGENT
  (benchmark solvers)      (collision matrix)
           │                         │
           └────────────┬────────────┘
                        ▼
           MOVEIT2 CONFIGURATOR AGENT
           + ROS2_CONTROL CONFIGURATOR
                        │
                        ▼
               SIM VALIDATION (Gazebo)
                        │
                   Pass?  No ──► AI CONFIG AGENT ──► retry
                        │
                       Yes
                        │
                        ▼
              VISION AGENT
              (detect cameras)
                        │
                        ▼
              CALIBRATION AGENT
              (hand-eye, TCP, joints)
                        │
                        ▼
            PERCEPTION INTELLIGENCE
            + MAPPING AGENT
                        │
                        ▼
              TASK EXECUTION AGENT
              (pick, place, follow)
                        │
                        ▼
                 OPERATIONAL
              (continuous monitoring
               by SELF-HEALING AGENT)
```

### 16.2 ROS2 Topic Architecture

```
/uraf/
├── /agent_bus/events           # Agent coordination events
├── /agent_bus/confirmations    # Human-in-loop confirmations
├── /discovery/hardware_profile  # HardwareProfile message
├── /discovery/firmware_info     # Detected firmware details
├── /model/urdf_ready            # Signal: URDF generated
├── /calibration/status         # Calibration progress
├── /calibration/results        # Calibration parameters
├── /perception/detected_objects # Object detections + poses
├── /perception/workspace_scene  # Semantic workspace model
├── /safety/estop               # Emergency stop command
├── /safety/status              # System safety status
├── /task/command               # Task command input (NL or primitive)
├── /task/status                # Task execution status
├── /health/metrics             # System health metrics
└── /diagnostics               # All diagnostic data
```

---

## 17. GUI Wizard — Full Specification

### 17.1 GUI Architecture

The URAF GUI is a browser-based application (runs at `http://localhost:8080`) built on React with a FastAPI backend. It provides both a guided wizard mode and an expert dashboard mode.

### 17.2 Wizard Pages

#### Page 1: Welcome & Mode Selection

```
╔════════════════════════════════════════════╗
║  URAF — Universal Robot Arm Foundation     ║
║  v1.0                                      ║
║                                            ║
║  Select Setup Mode:                        ║
║                                            ║
║  ● Auto Setup (Recommended)               ║
║    Let URAF do everything automatically    ║
║                                            ║
║  ○ Guided Wizard                           ║
║    Answer questions step by step           ║
║                                            ║
║  ○ Expert Mode                             ║
║    Full manual configuration control       ║
║                                            ║
║            [Continue →]                    ║
╚════════════════════════════════════════════╝
```

#### Page 2: Hardware Connection

```
╔════════════════════════════════════════════╗
║  Connect Your Robot                        ║
║                                            ║
║  Detected Interfaces:                      ║
║  ✓ CAN bus (can0) — Active               ║
║  ✓ USB Serial (/dev/ttyUSB0) — Active    ║
║  ✗ EtherCAT — Not found                  ║
║  ✓ ROS2 Environment — No existing robot  ║
║                                            ║
║  [Scan Again]  [Use These]                 ║
║                                            ║
║  Manual override:                          ║
║  Interface type: [CAN ▼] Device: [can0]   ║
╚════════════════════════════════════════════╝
```

#### Page 3: Robot Discovery Progress

```
╔════════════════════════════════════════════╗
║  Discovering Your Robot...                 ║
║                                            ║
║  ✓ Scanning CAN bus...                    ║
║    Found 6 nodes (joints detected: 6)     ║
║  ✓ Identifying firmware...                ║
║    CANOpen CiA 402 (Maxon EPOS4)          ║
║  ○ Reading joint capabilities...          ║
║  ○ Checking Community Library...          ║
║  ○ Inferring geometry...                  ║
║                                            ║
║  ████████████░░░░░░  65%                  ║
╚════════════════════════════════════════════╝
```

#### Page 4: Confirmation Dialogs (Human-in-the-Loop)

```
╔════════════════════════════════════════════╗
║  ⚠ Confirmation Required                   ║
║                                            ║
║  I detected the following robot structure: ║
║                                            ║
║  Joints: 6 (revolute)                      ║
║  Interface: CAN / CANOpen                  ║
║  Encoders: Absolute                        ║
║  Control modes: Position, Velocity, Torque ║
║                                            ║
║  Estimated geometry:                       ║
║  L1=150mm  L2=250mm  L3=220mm             ║
║  L4=195mm  L5=120mm  Wrist=60mm           ║
║  Confidence: 84%                           ║
║                                            ║
║  Is joint 3 rotation inverted? [Yes] [No] ║
║                                            ║
║  [Accept All] [Modify Values] [Learn More] ║
╚════════════════════════════════════════════╝
```

#### Page 5: Live 3D Visualization

```
╔════════════════════════════════════════════╗
║  Live Robot Model                  ● RViz  ║
║                                            ║
║  ┌──────────────────────────────────┐      ║
║  │        3D Robot Render           │      ║
║  │     (three.js + URDF loader)     │      ║
║  │                                  │      ║
║  │    [Interactive: drag to rotate] │      ║
║  └──────────────────────────────────┘      ║
║                                            ║
║  Joint Values:                             ║
║  J1: ██████░░  45.2°                       ║
║  J2: ████░░░░  32.1°                       ║
║  J3: ██░░░░░░  15.0°                       ║
║                                            ║
║  [Jog Mode]  [Home]  [Run Test Motion]     ║
╚════════════════════════════════════════════╝
```

#### Page 6: Calibration Wizard

```
╔════════════════════════════════════════════╗
║  Calibration                               ║
║                                            ║
║  Step 1/4: Print the calibration pattern   ║
║  [Download PDF]                            ║
║                                            ║
║  Step 2/4: Mount pattern in view of camera ║
║  Camera feed: [LIVE VIEW]                  ║
║  Pattern detected: ✓                       ║
║                                            ║
║  Step 3/4: Running hand-eye calibration... ║
║  Poses collected: 12/15                    ║
║  ████████░░  80%                           ║
║                                            ║
║  Estimated accuracy: ± 3.2mm              ║
╚════════════════════════════════════════════╝
```

### 17.3 Expert Dashboard Features

- Live TF tree visualization
- Real-time joint state plots
- Controller performance graphs
- Camera feed viewer
- Config file editor with syntax highlighting
- Agent status dashboard
- Log viewer with LLM-powered analysis
- Calibration quality monitor
- Firmware updater
- Plugin manager
- Community Library browser

---

## 18. Digital Twin Subsystem

### 18.1 Goals

1. Maintain a perfect digital replica of the physical robot state at all times
2. Use the twin for pre-execution motion validation
3. Enable offline development and testing without hardware
4. Sync twin state bidirectionally with physical robot
5. Support multiple simulator backends

### 18.2 Simulator Backend Support

| Simulator       | Use Case                | Integration       | Features                     |
| --------------- | ----------------------- | ----------------- | ---------------------------- |
| RViz2           | Visualization only      | Native ROS2       | TF, URDF, markers            |
| Gazebo Harmonic | Full physics simulation | gz_ros2_control   | Collision, sensors, dynamics |
| MuJoCo          | High-fidelity dynamics  | mujoco_ros2       | Contact-rich manipulation    |
| Isaac Sim       | GPU-accelerated sim     | isaac_ros2_bridge | Photo-realistic, RL training |
| PyBullet        | Lightweight Python sim  | pybullet_ros2     | Fast, easy scripting         |

### 18.3 MJCF/SDF Auto-Generation

The Digital Twin Agent converts the generated URDF to the target simulator's format:

```python
class DigitalTwinAgent:
    def generate_gazebo_model(self, urdf: str) -> str:
        return ros2_gz.urdf_to_sdf(urdf, physics_params={
            "step_size": 0.001,
            "real_time_factor": 1.0,
        })

    def generate_mujoco_model(self, urdf: str) -> str:
        return mujoco.MjModel.from_xml_string(
            urdf_to_mjcf(urdf, contact_params={"friction": 0.8})
        )
```

### 18.4 Pre-Execution Validation Protocol

Before any physical motion:

```
MOTION REQUEST (MoveIt2 plan)
           │
           ▼
  DIGITAL TWIN EXECUTION (Gazebo/MuJoCo)
           │
     ┌─────▼─────┐
     │ Checks:   │
     │ Collision │
     │ Limits    │
     │ Dynamics  │
     │ Stability │
     └─────┬─────┘
           │
      Pass?  No ──► Reject + reason → user
           │
          Yes
           │
           ▼
  PHYSICAL EXECUTION
```

---

## 19. Safety Architecture

### 19.1 Safety Integrity Level

URAF targets **SIL 2** equivalent safety for collaborative robot applications, with provisions for integration into certified safety PLCs for industrial SIL 3 deployments.

### 19.2 Safety Functions

| ID    | Safety Function                                  | Response Time      | Action                                      |
| ----- | ------------------------------------------------ | ------------------ | ------------------------------------------- |
| SF-01 | Joint position limit violation                   | < 10ms             | Controlled deceleration stop                |
| SF-02 | Joint velocity limit violation                   | < 5ms              | Immediate velocity clamp                    |
| SF-03 | Self-collision imminent                          | < 50ms             | Trajectory abort + retreat                  |
| SF-04 | External collision imminent (from occupancy map) | < 100ms            | Velocity reduction                          |
| SF-05 | Torque/current overload                          | < 5ms (FAL layer)  | Disable actuator                            |
| SF-06 | Motor temperature critical                       | < 1s               | Controlled stop, hold position              |
| SF-07 | Controller watchdog timeout                      | < 100ms            | Emergency stop                              |
| SF-08 | Software E-stop command                          | < 50ms             | Controlled deceleration                     |
| SF-09 | Hardware E-stop signal                           | < 10ms (FAL layer) | Immediate power cut or regen                |
| SF-10 | Workspace boundary approach                      | Continuous         | Velocity reduction proportional to distance |
| SF-11 | Loss of camera/sensor data                       | < 500ms            | Freeze motion (if in visual servo mode)     |
| SF-12 | Gripper position error                           | < 100ms            | Open gripper (drop safe)                    |

### 19.3 Safe Startup Sequence

```
POWER ON
    │
    ▼
FAL health check (all joints respond)
    │
    ▼
Joint homing (if encoders are incremental)
    │
    ▼
Position validation (joints within limits)
    │
    ▼
Brake release (if applicable)
    │
    ▼
Controller enable (low-velocity mode)
    │
    ▼
Move to home position (10% max velocity)
    │
    ▼
Safety systems armed
    │
    ▼
OPERATIONAL
```

---

## 20. Plugin System Architecture

### 20.1 Plugin Types

| Plugin Type      | Interface                    | Examples                  |
| ---------------- | ---------------------------- | ------------------------- |
| FAL Driver       | `FALDriver` base class       | New firmware support      |
| IK Solver        | `KinematicsPlugin` (MoveIt2) | Custom analytical solvers |
| Planner          | `PlanningPlugin` (MoveIt2)   | Domain-specific planners  |
| Camera Driver    | `CameraPlugin`               | New camera hardware       |
| Gripper Driver   | `GripperPlugin`              | Custom end-effectors      |
| Perception Model | `PerceptionPlugin`           | New object detectors      |
| Calibration      | `CalibrationPlugin`          | Custom cal. algorithms    |
| Task Primitive   | `PrimitivePlugin`            | New task building blocks  |

### 20.2 Plugin Development Interface

```python
# Example: Adding a new FAL driver as a plugin

# 1. Implement the FALDriver interface
class MyCustomRobotFALDriver(FALDriver):
    def connect(self, config): ...
    def write_command(self, joint_id, value): ...
    # ... implement all abstract methods

# 2. Create plugin manifest
# plugins/my_custom_robot/plugin.yaml
plugin_info:
  name: "my_custom_robot"
  type: "fal_driver"
  version: "1.0.0"
  author: "Your Name"
  description: "Driver for MyCustomRobot"
  entry_point: "my_custom_robot.driver:MyCustomRobotFALDriver"
  detection:
    method: "usb_vid_pid"
    vid: "0x1234"
    pid: "0x5678"

# 3. Install plugin
uraf plugin install ./plugins/my_custom_robot/
# → Plugin registered, driver loaded, detection pattern added
```

### 20.3 Plugin Registry

```
~/.uraf/plugins/
├── registry.json              # Index of all installed plugins
├── my_custom_robot/
│   ├── plugin.yaml
│   └── my_custom_robot/
│       └── driver.py
└── custom_ik_solver/
    ├── plugin.yaml
    └── custom_ik/
        └── solver.cpp
```

---

## 21. Community Robot Library (CRL)

### 21.1 Goals

1. Provide a curated, tested library of robot profiles that URAF users can share
2. Enable zero-effort setup for supported robots (< 5 minutes from connect to operational)
3. Allow community contribution of new robot profiles
4. Maintain quality control via automated testing of submitted profiles

### 21.2 Robot Fingerprinting

Each unique robot configuration is identified by a fingerprint:

```python
def compute_robot_fingerprint(hardware: HardwareProfile, geometry: GeometryProfile) -> str:
    """
    Creates a stable, reproducible hash of the robot's physical identity.
    Does NOT include calibration data (which is per-unit, not per-model).
    """
    identity = {
        "dof": hardware.joints.count,
        "joint_types": [j.type for j in hardware.joints],
        "firmware_type": hardware.firmware_type,
        "link_lengths_approx": [round(l, 2) for l in geometry.link_lengths],
        "dh_params_approx": geometry.dh_parameters_rounded(precision=2),
    }
    return "sha256:" + sha256(json.dumps(identity, sort_keys=True)).hexdigest()[:16]
```

### 21.3 Profile Structure

```yaml
# community_profile_ur5e.yaml (example)
crl:
  profile_id: "ur5e_v3"
  robot_name: "Universal Robots UR5e"
  manufacturer: "Universal Robots"
  fingerprint: "sha256:a4f7e9b2c1d3..."
  version: "2.0"
  test_status: "verified" # verified | community | experimental
  test_date: "2024-01-01"
  contributor: "community"
  downloads: 1543

  hardware:
    dof: 6
    joint_types: [revolute, revolute, revolute, revolute, revolute, revolute]
    firmware: "polyscope_5.x"
    interface: "ros2_control"

  geometry:
    # Full DH parameters, link lengths, joint limits
    # (pre-verified to match manufacturer datasheet)

  moveit2:
    recommended_ik_solver: "trac_ik"
    recommended_planner: "ompl/RRTConnect"
    known_issues: []

  calibration:
    tcp_offset_standard: [0, 0, 0.1, 0, 0, 0]
    joint_zero_procedure: "manual_markings"
```

---

## 22. Configuration Schema Reference

### 22.1 Complete Robot YAML Schema

See Section 6.2.3 for the master schema. Additional detail on specific subsections:

#### Joint Configuration

```yaml
robot:
  joints:
    - name: "joint_1"
      type: "revolute" # revolute | prismatic | continuous | fixed
      axis: [0, 0, 1] # rotation/translation axis in parent frame
      limits:
        position:
          lower: -3.14159 # rad (or meters for prismatic)
          upper: 3.14159
        velocity: 2.094 # rad/s (or m/s)
        effort: 150.0 # Nm (or N)
        acceleration: 4.0 # rad/s² (optional)
        jerk: 40.0 # rad/s³ (optional, for Ruckig)
      dynamics:
        damping: 0.0
        friction: 0.0
      hardware:
        node_id: 1 # CAN node ID or serial address
        encoder_resolution: 4096 # counts per revolution
        gear_ratio: 100.0 # motor-to-joint ratio
        direction: 1 # 1 or -1 (inversion)
```

#### Controller Tuning

```yaml
control:
  joints:
    - name: "joint_1"
      pid:
        p: 100.0
        i: 0.5
        d: 5.0
        i_clamp: 10.0
        antiwindup: true
      feedforward:
        velocity: 0.0
        acceleration: 0.0
```

---

## 23. Testing Strategy

### 23.1 Test Pyramid

```
                 ╱───────────────╲
                ╱  E2E Tests      ╲        (5%)
               ╱  Physical robot   ╲
              ╱   real hardware     ╲
             ╱─────────────────────╲
            ╱  Integration Tests    ╲      (25%)
           ╱   Gazebo + MuJoCo sim   ╲
          ╱─────────────────────────╲
         ╱     Unit Tests            ╲    (70%)
        ╱      Mock hardware          ╲
       ╱        Agent logic            ╲
      ╱──────────────────────────────╲
```

### 23.2 Test Categories

**Unit Tests (70%):**

- Agent logic with mocked hardware interfaces
- URDF generation from geometry fixtures
- IK solver output correctness
- Trajectory generation profiles
- Config store commit/rollback
- Agent message schema validation
- Safety limit enforcement

**Integration Tests (25%):**

- Full discovery → model → planning pipeline in Gazebo
- Calibration routines with simulated cameras
- Pick-and-place with simulated objects
- Multi-agent coordination under load
- Self-healing agent response to injected failures
- Plugin loading and unloading

**End-to-End Tests (5%):**

- 6-DOF arm: full setup under 30 minutes from cold
- 3-DOF arm: full setup under 15 minutes
- CAN bus robot: full setup under 30 minutes
- ROS2 existing robot: integration under 10 minutes
- Dual-arm collaborative task

### 23.3 Reference Robot Test Suite

The following robots are used as the primary validation set:

| Robot                         | DOF | Interface        | Status                 |
| ----------------------------- | --- | ---------------- | ---------------------- |
| Universal Robots UR5e         | 6   | ros2_control     | Target (high priority) |
| Franka Research 3             | 7   | libfranka / ROS2 | Target (high priority) |
| Dynamixel X-series custom arm | 6   | Serial           | Target                 |
| ODrive-based custom arm       | 4   | CAN              | Target                 |
| Unitree Z1                    | 6   | CAN              | Target                 |
| OpenManipulator-X             | 4   | Serial           | Target                 |
| Custom SCARA                  | 4   | CANOpen          | Stretch                |
| KUKA iiwa 7                   | 7   | ROS2             | Stretch                |

---

## 24. Success Metrics & KPIs

### 24.1 Setup Time KPIs

| Robot Scenario               | Target Setup Time | Measurement                                      |
| ---------------------------- | ----------------- | ------------------------------------------------ |
| Known robot (in CRL)         | < 5 minutes       | Time from first GUI click to MoveIt2 operational |
| Unknown 6-DOF with camera    | < 45 minutes      | With all calibration                             |
| Unknown 6-DOF without camera | < 30 minutes      | Geometry estimated from motion                   |
| Unknown 3-DOF simple arm     | < 15 minutes      |                                                  |
| ROS2 robot integration       | < 10 minutes      | Existing URDF + controllers                      |
| Dual-arm setup               | < 60 minutes      | Both arms + shared scene                         |

### 24.2 Accuracy KPIs

| Metric                        | Target                   | Measurement Method             |
| ----------------------------- | ------------------------ | ------------------------------ |
| Geometry estimation accuracy  | < 5% error vs CAD        | Compare to manufacturer spec   |
| IK success rate               | ≥ 98% on reachable poses | 1000-pose benchmark            |
| IK solve time (p99)           | < 20ms                   | Benchmark suite                |
| TCP calibration accuracy      | < 3mm                    | Repeatability test (20 points) |
| Hand-eye calibration accuracy | < 5mm                    | Checkerboard reprojection      |
| Object detection mAP          | ≥ 85%                    | YCB benchmark                  |
| Grasp success rate            | ≥ 80%                    | 100-trial physical test        |

### 24.3 System Reliability KPIs

| Metric                                | Target            |
| ------------------------------------- | ----------------- |
| Setup success rate (known hardware)   | ≥ 95%             |
| Setup success rate (unknown hardware) | ≥ 80%             |
| Mean time to first motion             | < 30 min          |
| Continuous uptime (production)        | ≥ 99.5% over 72h  |
| Self-healing recovery rate            | ≥ 85% of failures |
| Config rollback time                  | < 30 seconds      |

### 24.4 User Experience KPIs

| Metric                                           | Target         |
| ------------------------------------------------ | -------------- |
| Setup without reading docs (new user)            | < 90 minutes   |
| Number of questions asked during setup (average) | ≤ 5            |
| GUI setup completion rate                        | ≥ 90% of users |
| Natural language command accuracy                | ≥ 90%          |
| User satisfaction score (surveys)                | ≥ 4.2/5.0      |

---

## 25. Risk Analysis & Mitigation

### 25.1 Technical Risks

| Risk                                              | Probability | Impact   | Mitigation                                                        |
| ------------------------------------------------- | ----------- | -------- | ----------------------------------------------------------------- |
| Geometry estimation fails for non-standard robots | Medium      | High     | Fallback to manual YAML input; clear UI guidance                  |
| IK fails on unusual morphologies                  | Low         | High     | Always maintain KDL as final fallback; TRAC-IK handles most cases |
| CAN bus protocol variations break discovery       | Medium      | High     | Passive-listen + ML classifier for unknown protocols              |
| URDF validation passes but sim crashes            | Low         | Medium   | Redundant validation: urdf_parser + Gazebo load test              |
| Camera calibration converges to wrong solution    | Medium      | Medium   | Reprojection error threshold + user verification step             |
| FAL latency too high for real-time control        | Low         | Critical | Strict < 2ms requirement; benchmarks in CI/CD                     |
| LLM generates invalid config repairs              | Medium      | Medium   | Always validate in sim before applying; confidence gating         |
| Safety layer fails to stop in time                | Very Low    | Critical | Multi-layer safety (firmware + controller + supervisor)           |

### 25.2 Integration Risks

| Risk                                      | Probability | Impact | Mitigation                                              |
| ----------------------------------------- | ----------- | ------ | ------------------------------------------------------- |
| MoveIt2 API changes break configurator    | Medium      | Medium | Pin MoveIt2 version; CI tests against tagged releases   |
| Robot firmware update breaks FAL driver   | Low         | High   | Version-lock detection; driver versioning in registry   |
| ROS2 middleware DDS issues in multi-robot | Medium      | Medium | Test with FastDDS and Cyclone DDS; set QoS correctly    |
| GPU not available, perception fails       | Medium      | Medium | CPU fallback for all perception (slower but functional) |

### 25.3 Safety Risks

| Risk                                       | Probability | Impact   | Mitigation                                                          |
| ------------------------------------------ | ----------- | -------- | ------------------------------------------------------------------- |
| Robot moves before calibration complete    | Very Low    | Critical | State machine prevents motion commands until OPERATIONAL state      |
| Wrong joint limit direction causes crash   | Low         | High     | Sim validation + slow first motion at 10% velocity                  |
| Collision check misses fast-moving objects | Low         | High     | Conservative collision margins; velocity-scaled bounding boxes      |
| AI generates dangerous motion plan         | Very Low    | Critical | All plans validated in sim; safety layer enforces limits regardless |

---

## 26. Stretch Goals Roadmap

### 26.1 URAF v2.0 — Learning & Adaptation

**Timeline:** 6 months post v1.0

| Feature                            | Description                                                   | Key Technology             |
| ---------------------------------- | ------------------------------------------------------------- | -------------------------- |
| Learning-Based IK                  | Neural network IK trained on robot's own motion data          | PyTorch, RealNVP flows     |
| Reinforcement Learning Integration | Train policies in Isaac Sim, deploy to real robot             | Isaac Lab, RL_Games        |
| Sim-to-Real Transfer Agent         | Automatically minimize sim-to-real gap using real data        | Domain randomization       |
| Skill Library                      | Reusable motion skills (screw-in, wipe, pour) that generalize | Behavior cloning           |
| Auto-PID Tuning (RL)               | Use RL to find optimal PID gains per robot                    | Bayesian optimization + RL |

### 26.2 URAF v3.0 — Language & Multi-Agent

**Timeline:** 12 months post v1.0

| Feature                           | Description                                                 | Key Technology           |
| --------------------------------- | ----------------------------------------------------------- | ------------------------ |
| VLA Integration                   | Vision-Language-Action models for open-vocab task execution | OpenVLA, π0, RoboVLMs    |
| Natural Language Task Programming | "Pick up everything that's red and put it in the bin"       | GPT-4o + SAM2 + planning |
| LLM-Based Debugging               | "Why did my robot fail?" → full root-cause analysis         | RAG over log files + LLM |
| Multi-Agent Task Planning         | Multiple robots collaborate on complex tasks                | MARL, centralized critic |
| Foundation Models for Grasp       | Zero-shot grasp for never-seen objects                      | GR-2, UniDexGrasp++      |

### 26.3 URAF v4.0 — Scale & Cloud

**Timeline:** 18 months post v1.0

| Feature                            | Description                                        | Key Technology              |
| ---------------------------------- | -------------------------------------------------- | --------------------------- |
| Cloud Fleet Management             | Manage 10+ robots from a central dashboard         | Kubernetes, ROS2 over VPN   |
| Digital Twin Cloud Sync            | Twins run in cloud, policies deployed to edge      | AWS RoboMaker / Azure       |
| Automatic Firmware Generation      | AI generates firmware for unknown microcontrollers | CodeGen LLMs + HAL          |
| Humanoid Support                   | Extend URAF to bipedal humanoid bodies             | Whole-body control          |
| Autonomous Recovery After Failures | Robot diagnoses its own mechanical failures        | Force-torque analysis + LLM |
| Self-Healing Configurations        | Automatic detection and repair of config drift     | Anomaly detection + RAG     |

### 26.4 Stretch Goal: Robot Capability Discovery

A future agent that can _discover what a robot can do_ without being told:

```
CAPABILITY DISCOVERY PROTOCOL:
1. Move each joint through full range → map workspace
2. Mount tool → try to grasp objects → learn grasp capabilities
3. Apply forces to environment → characterize payload capacity
4. Time motions → characterize maximum speed/acceleration
5. Run endurance test → characterize thermal limits
6. Output: RobotCapabilityProfile (autonomously generated spec sheet)
```

---

## 27. Project Timeline Summary

```
QUARTER 1 (Months 1-3)
├── Month 1: Phase 0 — Foundation Infrastructure
│   ├── Workspace setup, Docker, Agent Bus
│   ├── Config Store, State Manager
│   └── CI/CD pipeline
│
├── Month 1-2: Phase 1 — Robot Discovery Layer
│   ├── Hardware Discovery Agent (serial, CAN, EtherCAT)
│   ├── Firmware Detection Agent (Dynamixel, ODrive, VESC, CANOpen)
│   ├── ROS2 Discovery Agent
│   └── End-Effector Detection Agent
│
└── Month 2-3: Phase 2 — Model Generation Layer
    ├── Geometry Extraction Agent
    ├── URDF Generator Agent
    ├── SRDF Generator Agent
    └── Kinematics Agent (benchmark + selection)

QUARTER 2 (Months 4-6)
├── Month 4-5: Phase 3 — Control & Planning Layer
│   ├── MoveIt2 Configurator Agent
│   ├── ros2_control Configurator Agent
│   ├── Trajectory Generation Agent
│   └── Safety Agent
│
└── Month 5-6: Phase 4 — Perception & Mapping
    ├── Vision Agent (camera detection, calibration)
    ├── Mapping Agent (Octomap, Voxblox)
    └── Self-Calibration Agent (TCP, hand-eye, joints)

QUARTER 3 (Months 7-9)
├── Month 7-8: Phase 5 — Intelligence Layer
│   ├── Perception Intelligence (YOLO, SAM2, FoundationPose)
│   ├── Grasp Planning Agent (AnyGrasp, GraspNet)
│   ├── Task Execution Agent
│   ├── Natural Language Interface
│   └── AI Configuration Agent
│
└── Month 8-9: GUI Wizard (full implementation)
    ├── Web-based wizard
    ├── 3D visualization
    └── Expert dashboard

QUARTER 4 (Months 10-12)
├── Month 10-11: Phase 6 — Self-Healing & Production
│   ├── Self-Healing Agent
│   ├── Learning Agent
│   ├── Multi-Robot Coordinator
│   └── Community Robot Library (launch)
│
├── Month 11-12: Integration & Validation
│   ├── Full test suite on reference robots
│   ├── Performance benchmarking
│   ├── Documentation
│   └── Community beta program
│
└── Month 12: v1.0 Release
    ├── Open-source release (GitHub)
    ├── Docker Hub images
    ├── Documentation site
    ├── Community Robot Library (initial profiles)
    └── Video demonstrations
```

---

## Appendix A: CLI Reference

```bash
# Install URAF
./install.sh

# Run full auto-setup
uraf setup --auto

# Run guided wizard
uraf setup --guided

# Discover hardware only
uraf discover

# Generate URDF from existing hardware profile
uraf generate-urdf --profile hardware_profile.yaml

# Run calibration
uraf calibrate --type full
uraf calibrate --type tcp
uraf calibrate --type hand-eye
uraf calibrate --type joint-zeros

# Start URAF (after setup)
uraf start

# Check system status
uraf status

# Rollback configuration
uraf rollback                    # Last working state
uraf rollback --to cfg_abc123   # Specific commit

# Plugin management
uraf plugin install ./my_plugin/
uraf plugin list
uraf plugin remove my_plugin

# Community library
uraf crl search "ur5"
uraf crl download ur5e_v3
uraf crl upload                  # Share current robot profile

# Natural language interface
uraf nl "move to home position"
uraf nl "pick up the red cube on the left"
```

---

## Appendix B: Environment Variables

```bash
URAF_CONFIG_PATH=~/.uraf/configs       # Config store location
URAF_PLUGIN_PATH=~/.uraf/plugins       # Plugin directory
URAF_LOG_LEVEL=INFO                     # DEBUG | INFO | WARN | ERROR
URAF_LLM_BACKEND=ollama                 # ollama | openai | anthropic
URAF_LLM_MODEL=llama3.1:70b            # Model identifier
URAF_OPENAI_API_KEY=sk-...              # If using OpenAI backend
URAF_GPU_ENABLED=true                   # Enable GPU acceleration
URAF_CRL_ENDPOINT=https://crl.uraf.io  # Community library endpoint
URAF_SIM_BACKEND=gazebo                 # gazebo | mujoco | rviz_only
URAF_SAFETY_MODE=standard              # standard | strict | relaxed (dev only)
URAF_CONFIDENCE_THRESHOLD_AUTO=0.85    # Min confidence for auto-execution
URAF_CONFIDENCE_THRESHOLD_PROPOSE=0.60 # Min confidence for proposal
ROS_DOMAIN_ID=0                        # ROS2 domain
```

---

_End of URAF AI Agentic Implementation Plan v1.0_
_Document generated by URAF project team_
_Version: 1.0 | Classification: Open_
