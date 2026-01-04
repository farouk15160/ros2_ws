# Troubleshooting Guide - Common Issues

## Issue 1: MoveIt Error "Cannot find planning configuration for group 'visiona_arm'"

**Error:**
```
[ERROR] [moveit.ompl_planning.planning_context_manager]: Cannot find planning configuration for group 'visiona_arm'
```

**Cause:** The MoveIt configuration uses group name `arm`, not `visiona_arm`.

**Fix:**
The cartesian_controller.py has been updated. If you still see this:
```bash
# Check your SRDF file to confirm group name
cat ~/ros2_ws/src/visiona_moveit_config/config/visiona.srdf | grep "group name"

# Should show: <group name="arm">
```

If using the controller manually:
```bash
ros2 run visiona_bridge cartesian_controller.py --ros-args -p group_name:=arm
```

---

## Issue 2: Self-Collision "right_finger_link" and "left_finger_link"

**Error:**
```
[INFO] [moveit_collision_detection_fcl.collision_common]: Found a contact between 'right_finger_link' and 'left_finger_link'
[INFO] [moveit_ros.fix_start_state_collision]: Start state appears to be in collision
```

**Cause:** Gripper fingers are modeled as colliding when closed.

**Fix Option 1:** Disable self-collision for gripper (Recommended)
```bash
# Edit SRDF
nano ~/ros2_ws/src/visiona_moveit_config/config/visiona.srdf

# Add to <robot> section:
<disable_collisions link1="left_finger_link" link2="right_finger_link" reason="Adjacent"/>
```

**Fix Option 2:** Open gripper before planning
```bash
# Command gripper to open
ros2 topic pub --once /gripper_command std_msgs/Float32 "data: 60.0"
```

---

## Issue 3: Web GUI Crash in Sim Mode - "AttributeError: robot_publishers"

**Error:**
```
AttributeError: 'RobotArmBridge' object has no attribute 'robot_publishers'
```

**Cause:** Missing initialization in bridge_node.py for sim mode.

**Workaround:** Launch without GUI in sim mode:
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=sim \
  viz:=moveit \
  gui:=false
```

**Or use real mode** (hardware not required for testing with mock_generic_system):
```bash
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit
```

---

## Issue 4: LLM Not Loading

**Error:**
```
[ERROR] [llm_task_planner]: ❌ LLM not loaded, cannot plan tasks
```

**Fix:**
```bash
# Ensure venv has llama-cpp-python
source ~/.venv/llm/bin/activate
pip list | grep llama-cpp-python

# If not installed:
CMAKE_ARGS="-DGGML_CUDA=on" pip install llama-cpp-python --no-cache-dir
```

---

## Quick Workaround for Testing NOW

### Use Real Mode (Not Sim)
```bash
# Terminal 1: Launch (real mode works even without hardware connected)
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  gui:=false

# Terminal 2: Run cartesian controller
ros2 run visiona_bridge cartesian_controller.py

# Terminal 3: Send pose
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.3}, 
          orientation: {w: 1.0}}}"
```

**Note:** You might see serial port errors, but MoveIt planning will still work!

---

## Checking System Status

```bash
# Check if move_group is running
ros2 node list | grep move_group

# Check available action servers
ros2 action list

# Should see: /move_action

# Check joint states
ros2 topic echo /joint_states

# Check MoveIt planning groups
ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces "{}"
```

---

## Complete Working Example

```bash
# 1. Kill any existing launches (Ctrl+C)

# 2. Launch fresh
ros2 launch visiona_bridge spawn_visiona.launch.py \
  mode:=real \
  viz:=moveit \
  gui:=false

# 3. Wait for "You can start planning now!" message

# 4. In new terminal:
ros2 run visiona_bridge cartesian_controller.py

# 5. Send command
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.25, y: 0.1, z: 0.25}, 
          orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

---

## Expected Behavior

✅ **Success looks like:**
```
[INFO] [cartesian_controller]: Received Target Pose: 0.25, 0.10, 0.25
[INFO] [cartesian_controller]: Sending MoveGroup Goal...
[INFO] [cartesian_controller]: Goal accepted. Executing...
[INFO] [move_group]: Planning successful
[INFO] [cartesian_controller]: Result Error Code: 1  # SUCCESS
```

❌ **Failure looks like:**
```
[INFO] [cartesian_controller]: Result Error Code: 99999  # FAILURE
```

Common failure codes:
- `99999` - Planning failed
- `-1` - Aborted
- `-2` - Preempted

---

## Summary

**For immediate testing:**
1. Use `mode:=real` (not sim)
2. Set `gui:=false`
3. Fix group name to `arm`
4. Disable gripper self-collision

These are temporary workarounds. The full system will be properly integrated in Week 5!
