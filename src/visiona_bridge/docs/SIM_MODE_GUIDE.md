# Running Visiona in Simulation Mode

## Sim Mode vs Real Mode

The Visiona robot has multiple modes:

| Mode | Description | Hardware | Use Case |
|------|-------------|----------|----------|
| **real** | Real robot hardware | Required | Production use |
| **sim** | Software simulation | Not required | Testing without robot |
| **gazebo** | Gazebo physics sim | Not required | Full 3D simulation |

---

## Launch in Sim Mode

### Basic Sim Mode
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  viz:=rviz
```

### Sim Mode with LLM/VLA
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  viz:=rviz \
  llm:=true \
  language:=en
```

### Sim Mode with MoveIt
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  viz:=moveit
```

### Full Sim (No Camera)
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  viz:=moveit \
  llm:=true
```

---

## What Happens in Sim Mode?

### ✅ Works:
- Robot visualization in RViz
- Joint state publishing
- MoveIt planning
- LLM task decomposition
- Web GUI control
- All ROS2 topics

### ❌ Doesn't Work:
- Camera (no real camera in sim)
- OctoMap (requires camera)
- Real motor control
- Current sensing

### ⚠️ Limited:
- VLA (needs camera images - use test images instead)
- Visual servoing (needs camera - will use stub)

---

## Testing LLM/VLA in Sim

Since VLA needs camera images, you have two options:

### Option 1: Test with Static Images
```bash
# Launch in sim mode
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  llm:=true

# Publish test image
ros2 run image_publisher image_publisher_node \
  /path/to/test/image.jpg \
  --ros-args -r image_raw:=/ascamera_hp60c/camera_publisher/rgb0/image
```

### Option 2: Use Webcam as Fake Camera
```bash
# Install usb_cam
sudo apt install ros-humble-usb-cam

# Launch robot in sim
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim llm:=true &

# Launch webcam (separate terminal)
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/ascamera_hp60c/camera_publisher/rgb0/image \
  -r camera_info:=/ascamera_hp60c/camera_publisher/rgb0/camera_info
```

---

## Sim Mode Advantages

1. **No Hardware Required** - Test software without robot
2. **Safe Testing** - Can't damage robot
3. **Faster Development** - No need to connect to robot
4. **Reproducible** - Same initial state every time
5. **CI/CD Friendly** - Can run automated tests

---

## Example Workflow

```bash
# Terminal 1: Launch simulation
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  viz:=rviz \
  llm:=true

# Terminal 2: Test LLM
python3 ~/ros2_ws/src/visiona_bridge/scripts/test_llm.py

# Send commands
> Move to home position
> Pick up the object
```

---

## Switching Between Modes

```bash
# Real robot
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real camera:=true llm:=true

# Simulation
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim llm:=true

# Gazebo (if configured)
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=gazebo llm:=true
```

---

## Troubleshooting Sim Mode

### Robot not moving in RViz
- Check `/joint_states` topic: `ros2 topic echo /joint_states`
- Verify sim mode is active: Check terminal output

### LLM not responding
- Ensure virtual environment is activated
- Check LLM logs for errors
- Verify model file exists

### No visualization
- Check `viz` parameter (should be `rviz` or `moveit`)
- Ensure RViz launched successfully

---

## Quick Test Commands

```bash
# Check system status
ros2 topic list
ros2 node list

# Test joint movement (sim mode)
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint'], position: [0.0]}"

# Test LLM
ros2 topic pub --once /llm/command std_msgs/String "data: 'Move to home'"

# Monitor tasks
ros2 topic echo /llm/tasks
```

---

**Sim mode is perfect for testing LLM/VLA logic before deploying to real hardware!** 🎮
