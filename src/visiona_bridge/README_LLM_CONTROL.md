# Running the LLM/VLA Control System

## Quick Start

### Build and Source
```bash
cd ~/ros2_ws
colcon build --packages-select visiona_bridge
source install/setup.bash
```

### Launch Options

The main launch file `spawn_visiona.launch.py` now supports:

#### 1. **Basic Robot (RViz only)**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=rviz
```

#### 2. **With MoveIt Planning**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true
```

#### 3. **With LLM Control (English)**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=rviz \
  camera:=true \
  llm:=true \
  language:=en
```

#### 4. **With LLM Control (German)**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  camera:=true \
  llm:=true \
  language:=de
```

#### 5. **Headless (No Visualization)**
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=off \
  llm:=true
```

---

## Launch Parameters

| Parameter | Options | Default | Description |
|-----------|---------|---------|-------------|
| `mode` | `real`, `sim`, `gazebo` | `real` | Robot control mode |
| `viz` | `rviz`, `moveit`, `off` | `rviz` | Visualization mode |
| `gui` | `true`, `false` | `true` | Launch web GUI |
| `camera` | `true`, `false` | `false` | Launch camera driver |
| `mapping` | `low`, `high` | `low` | OctoMap quality |
| `llm` | `true`, `false` | `false` | Enable LLM control |
| `language` | `en`, `de` | `en` | LLM language |

---

## Sending Commands

### Method 1: Test Script
```bash
# Interactive mode
python3 ~/ros2_ws/src/visiona_bridge/scripts/test_llm.py

# Single command
python3 ~/ros2_ws/src/visiona_bridge/scripts/test_llm.py "Pick up the red cube"
```

### Method 2: ROS Topic
```bash
ros2 topic pub --once /llm/command std_msgs/String \
  "data: 'Move to home position'"
```

### Method 3: Web GUI (Coming in Week 5)
The web GUI will have a natural language input field.

---

## Example Commands

**English:**
- "Pick up the blue object"
- "Move to home position"
- "Place the cube in the box"
- "Scan the workspace"
- "Grasp the red block and move it to the left"

**German:**
- "Nimm den roten Würfel auf"
- "Gehe zur Startposition"
- "Lege den Würfel in die Box"
- "Untersuche den Arbeitsbereich"
- "Greife den blauen Block und bewege ihn nach rechts"

---

## Monitoring

```bash
# Watch LLM task planning output
ros2 topic echo /llm/tasks

# Monitor LLM status
ros2 topic echo /llm/status

# Watch task executor
ros2 topic echo /task_executor/status

# Visual servoing status
ros2 topic echo /visual_servo/status
```

---

## Current Status (Week 1-2)

✅ **Working:**
- Package structure
- Configuration files
- LLM task planner (requires model download)
- Stub nodes (VLA, executor, visual servo)
- Launch file integration
- Bilingual prompts (English/German)

⏳ **To Do (requires model setup):**
- Run `./scripts/setup_models.sh` to download LLM
- Model takes ~6GB disk space
- First inference will be slow (loading model)

🚧 **In Progress (Weeks 3-6):**
- Full VLA implementation with camera
- Visual servoing control loop
- Task executor orchestration
- Web GUI integration

---

## Troubleshooting

### "llama-cpp-python not installed"
```bash
# Install with CUDA support for Jetson
CMAKE_ARGS="-DLLAMA_CUBLAS=on" pip install llama-cpp-python --no-cache-dir
```

### "Model path not found"
```bash
# Run the setup script
cd ~/ros2_ws/src/visiona_bridge
./scripts/setup_models.sh

# Then update config/llm_config.yaml with the model path
```

### Launch fails with "Package not found"
```bash
# Rebuild workspace
cd ~/ros2_ws
colcon build --packages-select visiona_bridge
source install/setup.bash
```

---

## Next Steps

1. **Week 1:** Run model setup script
2. **Week 2:** Test LLM task decomposition
3. **Week 3:** Implement VLA vision integration
4. **Week 4:** Add visual servoing control
5. **Week 5:** Connect full pipeline
6. **Week 6:** Test and refine

See [implementation_plan.md](../../.gemini/antigravity/brain/*/implementation_plan.md) for full details.
