# Visiona Robot - Complete Launch Guide

## Main Launch File: spawn_visiona.launch.py

This is your **all-in-one launch file** for the entire robot system.

---

## Launch Examples

### 1. Basic Robot (RViz only)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real
```

### 2. **Robot + MoveIt Planning** ⭐
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true
```
This launches **everything**: robot, camera, OctoMap, MoveIt, and RViz!

### 3. Robot + LLM Control (English)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=rviz \
  camera:=true \
  llm:=true \
  language:=en
```

### 4. **Full System** (MoveIt + LLM + Camera)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true \
  mapping:=high \
  llm:=true \
  language:=en
```

### 5. Headless Mode (No Visualization)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=off \
  camera:=true
```

---

## All Parameters

| Parameter | Options | Default | Description |
|-----------|---------|---------|-------------|
| **mode** | `real`, `sim`, `gazebo` | `real` | Robot control mode |
| **viz** | `rviz`, `moveit`, `off` | `rviz` | **Visualization mode** |
| **gui** | `true`, `false` | `true` | Launch web GUI |
| **camera** | `true`, `false` | `false` | Launch camera driver |
| **mapping** | `low`, `high` | `low` | OctoMap resolution |
| **llm** | `true`, `false` | `false` | Enable LLM control |
| **language** | `en`, `de` | `en` | LLM language |

---

## What Gets Launched?

### Always:
- Robot State Publisher
- ROS2 Control
- Joint Controllers
- Web GUI (unless `gui:=false`)

### When `camera:=true`:
- Camera driver (ascamera_hp60c)
- OctoMap 3D mapping

### When `viz:=rviz`:
- Basic RViz visualization

### When `viz:=moveit`:
- **Full MoveIt stack**
- MoveIt RViz interface
- Motion planning
- Trajectory execution

### When `llm:=true`:
- LLM Task Planner
- VLA Action Generator (stub)
- Visual Servoing Controller (stub)
- Task Executor

---

## Quick Commands

```bash
# Typical usage - Robot with MoveIt
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real viz:=moveit camera:=true

# Send LLM command
ros2 topic pub --once /llm/command std_msgs/String "data: 'Pick up the red cube'"

# Test LLM interactively
python3 ~/ros2_ws/src/visiona_bridge/scripts/test_llm.py
```

---

## One Launch File, Everything Included! ✨

No need for separate launch commands - `spawn_visiona.launch.py` handles it all!
