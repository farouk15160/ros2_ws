# ROS2 Topics Reference - Visiona Robot

Complete reference of all ROS2 topics available in the Visiona robot system.

---

## Robot Control Topics

### `/joint_states` 
**Type:** `sensor_msgs/JointState`  
**Publisher:** `robot_state_publisher`, `web_gui_node`  
**Description:** Current joint positions, velocities, and efforts for all 6 joints

**Payload Example:**
```yaml
header:
  stamp: {sec: 1767521000, nanosec: 0}
  frame_id: ''
name: ['base_link_joint', 'link_1_shoulder_joint', 'link_2_elbow_joint', 
       'link_3_wrist_joint', 'link_3_wrist_to_gripper_base_joint', 'gripper_joint']
position: [1.57, 1.57, 1.57, 1.57, 1.57, 0.0]  # radians
velocity: []
effort: []
```

### `/joint_targets`
**Type:** `sensor_msgs/JointState`  
**Publisher:** `web_gui_node`, `visual_servo_node`  
**Description:** Target joint positions to command the robot

**Payload Example:**
```yaml
header:
  stamp: {sec: 1767521000, nanosec: 0}
name: ['base_link_joint', 'link_1_shoulder_joint', ...]
position: [1.57, 1.57, 1.57, 1.57, 1.57, 0.26]
```

### `/joint_trajectory_controller/joint_trajectory`
**Type:** `trajectory_msgs/JointTrajectory`  
**Subscriber:** `joint_trajectory_controller`  
**Description:** Execute a full joint trajectory (for MoveIt)

**Payload Example:**
```yaml
joint_names: ['base_link_joint', 'link_1_shoulder_joint', ...]
points:
  - positions: [1.57, 1.57, 1.57, 1.57, 1.57, 0.0]
    time_from_start: {sec: 0, nanosec: 0}
  - positions: [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
    time_from_start: {sec: 2, nanosec: 0}
```

### `/joint_trajectory_controller/state`
**Type:** `control_msgs/JointTrajectoryControllerState`  
**Publisher:** `joint_trajectory_controller`  
**Description:** Current state of the trajectory controller

---

## Gripper & Hardware Topics

### `/gripper_command`
**Type:** `std_msgs/Float32`  
**Subscriber:** `web_gui_node`  
**Description:** Gripper opening width in degrees (0-120°)

**Payload Example:**
```yaml
data: 60.0  # degrees
```

### `/gripper_current`
**Type:** `std_msgs/Float32`  
**Publisher:** `web_gui_node`  
**Description:** Current gripper motor current in mA

### `/main_current`
**Type:** `std_msgs/Float32`  
**Publisher:** `web_gui_node`  
**Description:** Main motor current in Amps

### `/set_speed_factor`
**Type:** `std_msgs/Float32`  
**Subscriber:** `web_gui_node`  
**Description:** Set robot speed scaling factor (0.0-1.0)

**Payload Example:**
```yaml
data: 0.5  # 50% speed
```

### `/set_fan_speed`
**Type:** `std_msgs/Int32`  
**Subscriber:** `web_gui_node`  
**Description:** Set cooling fan speed (0-255)

---

## LLM/VLA Control Topics

### `/llm/command` ⭐
**Type:** `std_msgs/String`  
**Subscriber:** `llm_task_planner`  
**Description:** Send natural language commands to the robot

**Payload Example:**
```yaml
data: "Pick up the red cube"
```

### `/llm/tasks`
**Type:** `std_msgs/String`  
**Publisher:** `llm_task_planner`  
**Description:** JSON string of decomposed task sequence from LLM

**Payload Example:**
```yaml
data: '[{"action": "move_to", "target": "red cube", "parameters": {}}, 
        {"action": "grasp", "target": "red cube", "parameters": {}}]'
```

### `/llm/status`
**Type:** `std_msgs/String`  
**Publisher:** `llm_task_planner`  
**Description:** Current status of LLM task planner

**Payload Example:**
```yaml
data: "Ready (2 tasks planned)"
```

### `/task_executor/status`
**Type:** `std_msgs/String`  
**Publisher:** `task_executor`  
**Description:** Current status of task execution

### `/vla/task_description`
**Type:** `std_msgs/String`  
**Subscriber:** `vla_action_generator` (stub)  
**Description:** Task description for VLA visual grounding

### `/vla/action_goal`
**Type:** `geometry_msgs/PoseStamped`  
**Publisher:** `vla_action_generator` (stub)  
**Description:** 3D pose goal generated from visual grounding

---

## Visual Servoing Topics

### `/visual_servo/goal`
**Type:** `geometry_msgs/PoseStamped`  
**Subscriber:** `visual_servo_node` (stub)  
**Description:** Target pose for visual servoing controller

**Payload Example:**
```yaml
header:
  frame_id: "world"
pose:
  position: {x: 0.3, y: 0.0, z: 0.2}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

### `/visual_servo/status`
**Type:** `std_msgs/String`  
**Publisher:** `visual_servo_node` (stub)  
**Description:** Status of visual servoing controller

---

## Camera Topics

### `/ascamera_hp60c/camera_publisher/rgb0/image`
**Type:** `sensor_msgs/Image`  
**Publisher:** `ascamera_node`  
**Description:** RGB camera image (640x480 @ 25fps)

**Payload:** Raw image data in RGB8 format

### `/ascamera_hp60c/camera_publisher/rgb0/camera_info`
**Type:** `sensor_msgs/CameraInfo`  
**Publisher:** `ascamera_node`  
**Description:** RGB camera calibration parameters

**Payload Example:**
```yaml
width: 640
height: 480
K: [595.111, 0.0, 335.371,
    0.0, 594.785, 239.169,
    0.0, 0.0, 1.0]
```

### `/ascamera_hp60c/camera_publisher/depth0/image_raw`
**Type:** `sensor_msgs/Image`  
**Publisher:** `ascamera_node`  
**Description:** Raw depth image (640x480)

### `/ascamera_hp60c/camera_publisher/depth0/points`
**Type:** `sensor_msgs/PointCloud2`  
**Publisher:** `ascamera_node`  
**Description:** 3D point cloud from depth camera

### `/ascamera_hp60c/camera_publisher/depth0/camera_info`
**Type:** `sensor_msgs/CameraInfo`  
**Publisher:** `ascamera_node`  
**Description:** Depth camera calibration

---

## OctoMap Topics

### `/octomap_binary`
**Type:** `octomap_msgs/Octomap`  
**Publisher:** `octomap_server_node`  
**Description:** Binary octree representation of 3D environment

### `/octomap_full`
**Type:** `octomap_msgs/Octomap`  
**Publisher:** `octomap_server_node`  
**Description:** Full octree with all voxel data

### `/octomap_point_cloud_centers`
**Type:** `sensor_msgs/PointCloud2`  
**Publisher:** `octomap_server_node`  
**Description:** Centers of occupied voxels

### `/occupied_cells_vis_array`
**Type:** `visualization_msgs/MarkerArray`  
**Publisher:** `octomap_server_node`  
**Description:** Visualization of occupied cells in RViz

### `/free_cells_vis_array`
**Type:** `visualization_msgs/MarkerArray`  
**Publisher:** `octomap_server_node`  
**Description:** Visualization of free (empty) cells

### `/projected_map`
**Type:** `nav_msgs/OccupancyGrid`  
**Publisher:** `octomap_server_node`  
**Description:** 2D projection of octree for navigation

---

## Cartesian Control Topics (MoveIt Integration)

### `/visiona/current_pose`
**Type:** `geometry_msgs/PoseStamped`  
**Publisher:** `cartesian_controller` (if running)  
**Description:** Current end-effector Cartesian pose

**Payload Example:**
```yaml
header:
  frame_id: "world"
pose:
  position: {x: 0.25, y: 0.0, z: 0.3}
  orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
```

### `/visiona/target_pose`
**Type:** `geometry_msgs/PoseStamped`  
**Subscriber:** `cartesian_controller`  
**Description:** Target Cartesian pose for MoveIt planning

---

## Safety & Calibration Topics

### `/set_collision_threshold`
**Type:** `std_msgs/Float32`  
**Subscriber:** `web_gui_node`  
**Description:** Set collision detection current threshold

### `/set_collision_dev_threshold`
**Type:** `std_msgs/Float32`  
**Subscriber:** `web_gui_node`  
**Description:** Set collision deviation threshold

### `/set_max_limits`
**Type:** `std_msgs/Float32MultiArray`  
**Subscriber:** `web_gui_node`  
**Description:** Set maximum joint angle limits

### `/set_min_limits`
**Type:** `std_msgs/Float32MultiArray`  
**Subscriber:** `web_gui_node`  
**Description:** Set minimum joint angle limits

---

## RViz Interaction Topics

### `/clicked_point`
**Type:** `geometry_msgs/PointStamped`  
**Publisher:** RViz  
**Description:** Point clicked in RViz 3D view

### `/goal_pose`
**Type:** `geometry_msgs/PoseStamped`  
**Publisher:** RViz  
**Description:** Goal pose set via RViz 2D Nav Goal tool

### `/initialpose`
**Type:** `geometry_msgs/PoseWithCovarianceStamped`  
**Publisher:** RViz  
**Description:** Initial pose estimate from RViz

---

## Transform Topics

### `/tf`
**Type:** `tf2_msgs/TFMessage`  
**Publisher:** `robot_state_publisher`, `static_transform_publisher`  
**Description:** Dynamic coordinate frame transforms

### `/tf_static`
**Type:** `tf2_msgs/TFMessage`  
**Publisher:** `robot_state_publisher`, `static_transform_publisher`  
**Description:** Static coordinate frame transforms

**Key Frames:**
- `world` → `base` → `base_link` → link chain → `gripper_base`
- `camera_link` → `ascamera_hp60c_ascamera_0`

---

## System Topics

### `/robot_description`
**Type:** `std_msgs/String`  
**Publisher:** `robot_state_publisher`  
**Description:** URDF robot model as XML string

### `/parameter_events`
**Type:** `rcl_interfaces/ParameterEvent`  
**Publisher:** All nodes  
**Description:** ROS2 parameter change events

### `/rosout`
**Type:** `rcl_interfaces/Log`  
**Publisher:** All nodes  
**Description:** Logging messages from all ROS2 nodes

---

## Quick Command Reference

```bash
# Send LLM command
ros2 topic pub --once /llm/command std_msgs/String "data: 'Pick up the red cube'"

# Watch LLM output
ros2 topic echo /llm/tasks

# Monitor robot status
ros2 topic echo /joint_states

# Command gripper
ros2 topic pub --once /gripper_command std_msgs/Float32 "data: 60.0"

# Set speed
ros2 topic pub --once /set_speed_factor std_msgs/Float32 "data: 0.5"

# View camera
ros2 run image_view image_view --ros-args -r image:=/ascamera_hp60c/camera_publisher/rgb0/image

# List all topics
ros2 topic list

# Get topic info
ros2 topic info /llm/command

# Get topic rate
ros2 topic hz /joint_states
```

---

## Topic Diagram

```
Natural Language Command Flow:
  User → /llm/command → llm_task_planner → /llm/tasks → task_executor
                                        ↓
                                   /llm/status

Visual Servoing Flow:
  vla_action_generator → /vla/action_goal → visual_servo_node → /joint_targets → web_gui_node → Robot

Robot State Flow:
  Robot → web_gui_node → /joint_states → robot_state_publisher → /tf

Camera Flow:
  ascamera_node → /rgb0/image, /depth0/points → octomap_server_node → /octomap_binary
```

---

**Last Updated:** 2026-01-04  
**System Version:** Visiona Bridge v5.0 with LLM/VLA Control
