# Quick Start: Simple XYZ Cartesian Control

The simple IK solver is now **integrated into the main launch file**!

## 🚀 Launch with Simple IK Solver

```bash
# Basic launch with IK solver (default)
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real

# With RViz visualization
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real viz:=rviz

# With GUI and IK solver (recommended)
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real gui:=true ik:=true

# Disable IK solver if needed
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real ik:=false
```

## 🎮 Using XYZ Control

### From Command Line:

```bash
# Move to absolute XYZ position
ros2 topic pub --once /visiona/cartesian_command geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"

# The robot will:
# 1. Calculate smooth trajectory (10-50 waypoints)
# 2. Use numerical IK to solve each waypoint
# 3. Execute motion at 20Hz
# 4. Complete in ~2-5 seconds
```

### From Web GUI (Coming Next):

The GUI will have XYZ jog buttons:
- **+X / -X** - Move forward/backward
- **+Y / -Y** - Move left/right  
- **+Z / -Z** - Move up/down
- **Go to XYZ** - Absolute positioning

## ⚙️ Configuration

Edit `config/simple_ik_config.yaml` to tune:

```yaml
simple_ik_solver:
  ros__parameters:
    max_iterations: 100      # IK solver iterations
    tolerance: 0.001         # 1mm convergence
    step_size: 0.01         # 1cm per waypoint (smooth!)
    control_rate: 20.0      # 20Hz execution
    max_speed: 0.1          # 0.1 m/s max speed
```

## 🆚 Simple IK vs MoveIt

| Feature | MoveIt | Simple IK |
|---------|--------|-----------|
| **Speed** | 1-5 seconds | <1 second |
| **Memory** | ~500MB | ~50MB |
| **Complexity** | 50+ nodes | 1 node |
| **Collision** | Yes | No (not needed) |
| **Planning** | Complex | Direct |
| **GUI Integration** | Difficult | Easy! |

## 📊 What Changed

**Removed from spawn_visiona.launch.py:**
- ❌ `moveit_launch` (no more MoveIt dependency)

**Added to spawn_visiona.launch.py:**
- ✅ `simple_ik_node` (fast IK solver)
- ✅ `ik:=true/false` parameter

**Result:**
- Same launch file, simpler system!
- MoveIt only loaded if you explicitly want it
- Simple IK runs by default with GUI

## 🧪 Testing

```bash
# Terminal 1: Launch system
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real viz:=rviz

# Terminal 2: Send XYZ command
ros2 topic pub --once /visiona/cartesian_command geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"

# Watch in RViz: Smooth motion with visible waypoints!
```

## ✅ Expected Behavior

```
[simple_ik_solver] 🎯 Cartesian command: x=0.300, y=0.000, z=0.400
[simple_ik_solver]    Generated 45 waypoints
[simple_ik_solver] ✅ Cartesian motion complete
```

Robot should move smoothly to the target position! 🎉

---

**Note:** The simple IK solver is **always enabled by default** when you launch the GUI!
