# Cartesian Control for Visiona Robot

## Current Status

The Visiona robot has **two different Cartesian control approaches**:

### 1. **MoveIt Cartesian Controller** (Existing)
**Location:** `scripts/cartesian_controller.py`  
**Status:** ✅ Working (but has latency)  
**Method:** Uses MoveIt's `move_group` action for planning

**Usage:**
```bash
# Launch robot with MoveIt
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real viz:=moveit

# Send Cartesian command
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.2}, 
          orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

**Topics:**
- **Subscribe:** `/visiona/target_pose` (geometry_msgs/PoseStamped)
- **Publish:** `/visiona/current_pose` (geometry_msgs/PoseStamped)

**How it works:**
1. Receives target Cartesian pose
2. Sends to MoveIt for planning
3. MoveIt plans trajectory avoiding collisions
4. Executes trajectory via `joint_trajectory_controller`

**Pros:**
- ✅ Collision avoidance with OctoMap
- ✅ Proven and reliable
- ✅ Full MoveIt planning capabilities

**Cons:**
- ❌ High latency (~2-5 seconds per move)
- ❌ Not suitable for reactive control
- ❌ Requires MoveIt to be running

---

### 2. **Visual Servoing Controller** (New - In Development)
**Location:** `visiona_bridge/visual_servoing/visual_servo_node.py`  
**Status:** 🚧 Stub (Week 4 implementation)  
**Method:** Direct velocity control with visual feedback

**Usage (When Implemented):**
```bash
# Launch with visual servoing
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  camera:=true \
  llm:=true

# Send goal pose
ros2 topic pub --once /visual_servo/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.2}, 
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**Topics:**
- **Subscribe:** `/visual_servo/goal` (geometry_msgs/PoseStamped)
- **Publish:** `/visual_servo/status` (std_msgs/String)
- **Publish:** `/joint_targets` (sensor_msgs/JointState) - velocity commands

**How it will work:**
1. Receives target Cartesian pose
2. Continuously tracks error in Cartesian space
3. Computes velocity commands (PBVS controller)
4. Sends joint velocities directly to robot
5. Closes loop at 25Hz for reactive control

**Pros:**
- ✅ Low latency (~40ms control loop)
- ✅ Reactive to environment changes
- ✅ Suitable for LLM/VLA real-time control

**Cons:**
- ⚠️ Not yet implemented (stub only)
- ⚠️ Requires inverse Jacobian
- ⚠️ No automatic collision avoidance (yet)

---

## Which Should You Use?

### For Testing LLM (Now):
Use **MoveIt Cartesian Controller**:
```bash
# 1. Launch with MoveIt
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim viz:=moveit llm:=true

# 2. In another terminal, run cartesian controller
ros2 run visiona_bridge cartesian_controller.py

# 3. Send LLM command
ros2 topic pub --once /llm/command std_msgs/String \
  "data: 'Move to position x=0.3 y=0.0 z=0.2'"
```

**Note:** The LLM will decompose this into tasks, but you'll need to manually publish to `/visiona/target_pose` for now since the full pipeline isn't connected yet.

### For Future (Week 5-6):
Use **Visual Servoing** when fully implemented:
- Full LLM → VLA → Visual Servo pipeline
- Real-time reactive control
- Camera-guided movements

---

## Quick Command Examples

### MoveIt Cartesian (Works Now)
```bash
# Move to specific XYZ
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.25, y: 0.1, z: 0.3}, 
          orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

# Check current pose
ros2 topic echo /visiona/current_pose
```

### Joint Control (Also Works Now)
```bash
# Direct joint command
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 
           'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'],
   position: [1.57, 1.57, 1.57, 1.57, 1.57, 0.26]}"
```

---

## Integration with LLM/VLA System

### Current Implementation (Weeks 1-2):
```
User Command → LLM → Task Sequence (JSON) → [Manual execution needed]
```

### Target Implementation (Week 5):
```
User Command → LLM → Task Sequence → Task Executor
                                          ↓
                                    VLA (for object poses)
                                          ↓
                                  Visual Servoing (execution)
                                          ↓
                                      Robot Moves!
```

---

## Testing Cartesian Control Now

### Method 1: MoveIt (Recommended for now)
```bash
# Terminal 1: Launch
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim viz:=moveit

# Terminal 2: Run cartesian controller
ros2 run visiona_bridge cartesian_controller.py

# Terminal 3: Send poses
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.2}, 
          orientation: {w: 1.0}}}"
```

### Method 2: Web GUI (Interactive)
```bash
# Launch with GUI
ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real viz:=moveit gui:=true

# Open browser: http://localhost:5000
# Use GUI to command positions
```

---

## Converting LLM Commands to Cartesian Poses

The LLM can decompose commands like:
```
"Move to position x=0.3 y=0.0 z=0.2"
```

Into task JSON:
```json
{
  "tasks": [
    {
      "action": "move_to",
      "target": "position",
      "parameters": {
        "x": 0.3,
        "y": 0.0,
        "z": 0.2
      }
    }
  ]
}
```

But you currently need to manually convert this to the `/visiona/target_pose` message until Week 5 when `task_executor` is fully implemented.

---

## Summary

**✅ YES, Cartesian X/Y/Z control works!**

**Options:**
1. **MoveIt Cartesian Controller** - Works now, use `/visiona/target_pose`
2. **Visual Servo Controller** - Coming in Week 4-5
3. **Web GUI** - Interactive control, works now

**For LLM Testing:**
- LLM decomposes commands ✅
- Manual pose publishing needed (for now) ⚠️
- Full automation coming in Week 5 🚧

Use MoveIt for testing now, visual servoing will be ready for production later!
