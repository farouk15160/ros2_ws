# Launch File Architecture

## Overview

The Visiona robot system uses a modular launch architecture with clear separation of concerns.

---

## Launch Files

### 1. **spawn_visiona.launch.py** (Main Launch File)
**Purpose:** Complete robot system with all hardware and control logic

**Contains:**
- Robot State Publisher
- ROS2 Control (controller_manager, ros2_control_node)
- Joint Controllers (joint_state_broadcaster, joint_trajectory_controller)
- Web GUI
- Camera driver (optional)
- OctoMap 3D mapping (optional)
- Basic RViz visualization (optional)
- LLM/VLA control system (optional)

**Usage:**
```bash
# Basic robot
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real

# With all features
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true \
  llm:=true
```

---

### 2. **visiona_moveit.launch.py** (MoveIt Components Only)
**Purpose:** MoveIt motion planning components ONLY

**Contains:**
- move_group (planning server)
- MoveIt RViz interface

**Does NOT contain:**
- Robot hardware
- Controllers
- Camera
- Any robot logic

**Usage:**
```bash
# Standalone (robot must already be running!)
ros2 launch visiona_bridge visiona_moveit.launch.py

# Better: Use spawn_visiona with viz:=moveit
ros2 launch visiona_bridge spawn_visiona.launch.py viz:=moveit
```

---

### 3. **llm_control.launch.py** (LLM/VLA Components Only)
**Purpose:** Natural language control system

**Contains:**
- LLM Task Planner
- VLA Action Generator
- Visual Servoing Controller
- Task Executor

**Usage:**
```bash
# Standalone
ros2 launch visiona_bridge llm_control.launch.py language:=en

# Better: Use spawn_visiona with llm:=true
ros2 launch visiona_bridge spawn_visiona.launch.py llm:=true
```

---

## Architecture Diagram

```
spawn_visiona.launch.py (Main - All robot logic)
│
├─ Robot State Publisher
├─ ROS2 Control
│  ├─ controller_manager
│  ├─ joint_state_broadcaster
│  └─ joint_trajectory_controller
│
├─ Web GUI
├─ Camera driver (if camera:=true)
├─ OctoMap (if camera:=true)
│
├─ Visualization (controlled by viz parameter)
│  ├─ viz:=rviz  → Basic RViz
│  ├─ viz:=moveit → visiona_moveit.launch.py ─┐
│  └─ viz:=off   → No visualization           │
│                                              │
└─ Optional Features                          │
   └─ llm:=true → llm_control.launch.py       │
                                               │
                                               ▼
                        visiona_moveit.launch.py (MoveIt only)
                        │
                        ├─ move_group (planning server)
                        └─ MoveIt RViz interface
```

---

## Design Principles

### ✅ spawn_visiona.launch.py
- **ALL robot hardware and control logic**
- Self-contained robot system
- Can run standalone
- Optionally includes other launch files

### ✅ visiona_moveit.launch.py
- **ONLY MoveIt planning components**
- No robot hardware
- Expects robot to already be running
- Included by spawn_visiona when viz:=moveit

### ✅ llm_control.launch.py
- **ONLY LLM/VLA components**
- No robot hardware
- Expects robot to already be running
- Included by spawn_visiona when llm:=true

---

## Common Usage Patterns

### Pattern 1: Basic Robot
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real
```
Gets: Robot + Web GUI + Basic RViz

### Pattern 2: Robot + MoveIt Planning
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true
```
Gets: Robot + Web GUI + Camera + OctoMap + MoveIt + MoveIt RViz

### Pattern 3: Robot + LLM Control
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=rviz \
  camera:=true \
  llm:=true
```
Gets: Robot + Web GUI + Camera + Basic RViz + LLM/VLA system

### Pattern 4: Everything
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true \
  mapping:=high \
  llm:=true \
  language:=en
```
Gets: Full system with all features

---

## Summary

**One command, one launch file:** `spawn_visiona.launch.py`

All other launch files are modular components that get included as needed.

**Clean separation:**
- `spawn_visiona.launch.py` = Robot logic
- `visiona_moveit.launch.py` = MoveIt logic only
- `llm_control.launch.py` = LLM/VLA logic only
