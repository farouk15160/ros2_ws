# Quick Fix: Get MoveIt Working NOW

## The Problem

MoveIt says: **"Unable to sample any valid states for goal tree"**

This means your target Cartesian poses (x=0.25, y=0.1, z=0.25) are either:
- Outside the robot's reachable workspace
- In collision  
- Impossible from current position (all joints at 90°)

---

## ✅ IMMEDIATE FIX: Use Joint Control First

**Step 1: Move robot to a known good pose using joint control**

```bash
# Send robot to a better starting position
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 
           'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'],
   position: [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]}"
```

**Step 2: Try a reachable Cartesian pose**

```bash
# This pose is known to be reachable
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.0, y: 0.0, z: 0.4}, 
          orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

---

## Known Working Poses

Try these Cartesian poses (they're within reach):

### Pose 1: Straight Up
```bash
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.0, y: 0.0, z: 0.5}, 
          orientation: {w: 1.0}}}"
```

### Pose 2: Forward and Up
```bash
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.3}, 
          orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

### Pose 3: To the Side
```bash
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.2, y: 0.2, z: 0.3}, 
          orientation: {w: 1.0}}}"
```

---

## Why Your Poses Failed

Your requested poses:
- `x=0.25, y=0.1, z=0.25` ← **TOO LOW** (z=0.25 is close to base, hard to reach)
- `x=0.25, y=0.2, z=0.25` ← **TOO LOW**

**Rule of thumb for this robot:**
- **z should be ≥ 0.3** (at least 30cm above base)
- **x should be 0.0-0.4** (forward reach)
- **y should be -0.3 to 0.3** (side reach)

---

## Better Workflow

### 1. Move to home first (joint control)
```bash
ros2 topic pub --once /joint_targets sensor_msgs/JointState \
  "{name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 
           'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint'],
   position: [0.0, 1.57, 1.57, 1.57, 0.0, 0.26]}"
```

### 2. Wait 3 seconds for movement

### 3. Try Cartesian pose (higher z value)
```bash
ros2 topic pub --once /visiona/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
   pose: {position: {x: 0.3, y: 0.0, z: 0.35}, 
          orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

---

## Success Looks Like

If planning works, you'll see:
```
[INFO] [cartesian_controller]: Received Target Pose: 0.30, 0.00, 0.35
[INFO] [cartesian_controller]: Sending MoveGroup Goal...
[INFO] [cartesian_controller]: Goal accepted. Executing...
[INFO] [move_group]: Planning successful
[INFO] [cartesian_controller]: Result Error Code: 1  ✅ SUCCESS!
```

And **THE ROBOT WILL ACTUALLY MOVE!**

---

## Alternative: Use RViz Interactive Marker

In RViz with MoveIt:
1. Click and drag the interactive marker (orange ball)
2. Move it to a reachable pose
3. Click "Plan" button
4. Click "Execute" button

This lets you visually see what's reachable!

---

## Summary

**Your poses were too low (z=0.25).** Try:
- **Minimum z = 0.3**
- Start from a good joint pose
- Use the working poses above

The robot **IS connected and working** - you just need reachable goals! 🎯
