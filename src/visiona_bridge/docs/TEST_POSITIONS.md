# How to Test Multiple Positions

## Quick Test Script

I've created an automated test script that will test multiple positions for you!

### Run the Test:

```bash
# Make sure your launch is running in another terminal:
# ros2 launch visiona_bridge spawn_visiona.launch.py mode:=real viz:=moveit gui:=false

# Also run cartesian controller in another terminal:
# ros2 run visiona_bridge cartesian_controller.py

# Then run the test script:
python3 ~/ros2_ws/src/visiona_bridge/scripts/test_positions.py
```

### What It Tests:

**Test 1: Joint Control (5 positions)**
1. Home Position - `[0.0, 1.57, 1.57, 1.57, 0.0, 0.26]`
2. Up Position - `[0.0, 1.2, 1.5, 1.57, 1.57, 0.26]`
3. Side Position - `[1.57, 1.2, 1.5, 1.57, 1.57, 0.26]`
4. Forward Reach - `[0.0, 0.8, 1.2, 1.5, 1.57, 0.26]`
5. Back to Home

**Test 2: Cartesian Control (5 positions)**
1. Straight Up - `x=0.0, y=0.0, z=0.5`
2. Forward and Up - `x=0.3, y=0.0, z=0.4`
3. To the Right - `x=0.2, y=0.2, z=0.35`
4. To the Left - `x=0.2, y=-0.2, z=0.35`
5. Close and High - `x=0.15, y=0.0, z=0.45`

---

## Manual Testing (If You Prefer)

### Test Joint Positions:

```bash
# Home
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'], position: [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]}"

# Up Position
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'], position: [0.0, 1.2, 1.5, 1.57, 1.57, 0.26]}"

# Side (90° rotation)
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'], position: [1.57, 1.2, 1.5, 1.57, 1.57, 0.26]}"

# Forward Reach
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'], position: [0.0, 0.8, 1.2, 1.5, 1.57, 0.26]}"
```

### Test Cartesian Positions:

```bash
# Straight Up
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"

# Forward
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.0, z: 0.4}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

# Right
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: 0.2, z: 0.35}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

# Left
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: -0.2, z: 0.35}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

---

## Expected Results:

### ✅ Joint Control:
- Should work **100%** - direct motor commands
- Robot moves immediately
- No planning required

### ⚠️ Cartesian Control:
- **May fail** if pose is unreachable or in collision
- Requires MoveIt planning
- Success code: `1`, Failure code: `99999`
- Watch cartesian_controller terminal for results

---

## Tips:

1. **Always wait 5 seconds** between joint commands
2. **Start from a good pose** before Cartesian moves
3. **Use RViz** to visualize planned trajectories
4. **Check logs** in cartesian_controller terminal

---

## What You'll Learn:

- ✅ Joint control is **reliable and fast**
- ⚠️ Cartesian control **depends on reachability**
- 🎯 Some poses work, some don't (normal for MoveIt)
- 🤖 Your robot hardware is **working perfectly**!

Run the test script and watch your robot move! 🚀
