# Visiona AI-Powered Robotic Arm Workspace

This repository contains the complete software suite for the Visiona intelligent 6-axis robot arm. It bridges the gap between low-level FreeRTOS hardware control, ROS 2 middleware, complex kinematics, and high-level perception (vision) systems.

The architecture is highly modular, split between a highly responsive embedded firmware layer and an intelligent Python/ROS2 orchestrator capable of smooth path planning, dynamic obstacle avoidance, and real-time point-cloud mapping.

---

## 🏗️ System Architecture

The workspace is structured into two core components that communicate seamlessly over a robust serial protocol:

1. **`visiona_firmware` (Low-Level Control)**: Runs on an ESP32 micro-controller using FreeRTOS. It is responsible for hardware-level motion interpolation, stepper/PWM signal generation, safety bounds, and current sensing for force feedback.
2. **`visiona_bridge` (High-Level AI & Kinematics)**: Runs on the host PC (or Jetson). Contains the ROS 2 node orchestrator (`bridge_node.py`), the Inverse Kinematics solver (`simple_ik_solver.py`), and the vision point-cloud accumulator (`colored_map_node.py`).

### Data Flow Overview

```text
[Camera/RGB-D] -> /colored_map (Point Cloud) -> [Simple IK Solver (Collision Check)]
                                                       ^
[Target XYZ] ------------------------------------------|
                                                       v
                   [Minimum Jerk Trajectory] -> [Bridge / Safety Validator]
                                                       | (Serial Protocol / 0xA5 Packets)
                                                       v
                                            [ESP32 FreeRTOS Firmware] 
                                            -> IIR Filter Interpolation -> [Servos/Steppers]
```

---

## 💻 Low-Level Firmware Details (ESP32 / FreeRTOS)

The embedded firmware (`visiona_firmware/firmware/src/`) guarantees smooth execution, real-time safety, and deterministic timing, built completely on FreeRTOS tasks.

### 1. The Motion Interpolator (`motion.cpp`)
Instead of simply setting servo positions, the firmware employs a **100Hz tight-loop IIR Filter** to ensure ultra-smooth motion and eliminate hardware jerks.

**Filtering Math & Logic:**
```cpp
current_angle += (target_angle - current_angle) * alpha
```
- **Dynamic Alpha Calculation:** The `alpha` value determines the filter's responsiveness and is calculated inversely to the requested speed factor (`alpha = 6.0 / speed_factor`). 
- **Speeds constraints:** For extremely slow speeds (e.g., speed=500), `alpha` drops as low as `0.002`, executing an incredibly slow, step-less easing curve.
- **Soft-Start:** On boot (`main.cpp`), the ESP32 gradually pre-loads the instantaneous PWM pulses to the Servos before fully engaging the OE (Output Enable) pin, preventing violent robotic twitching on startup.

### 2. Force Feedback & Gripper Control
The firmware integrates an INA219/ACS712 current sensor. In `motion.cpp`, if the torque (current draw in mA) on the gripper servo exceeds the defined `g_grip_target_current_mA`, the target angle is immediately overridden to the current angle:
```cpp
if (gripper_current > target_mA) { target_angle = current_angle; }
```
This serves as an effective, sensor-less "grip-strength" limiter for picking up fragile objects.

### 3. Packet Protocol (`serial_protocol.cpp`)
Communication uses a structured byte payload starting with `0xA5` (Header), followed by a Command ID (`M`, `H`, `G`, `E`, `K`), array of 6 floats (angles), speed, and gripper thresholds, concluded by an XOR checksum to reject noise.

---

## 🧠 Kinematics & Path Planning (`visiona_bridge`)

The high-level logic resides in Python nodes that use mathematical optimizations to solve Cartesian target points.

### 1. Inverse Kinematics (Damped Least Squares)
The `simple_ik_solver.py` converts 3D world coordinates (X, Y, Z) to joint angles using the **Damped Least Squares (Levenberg-Marquardt)** method, which robustly handles singularities where traditional Jacobian inverses fail.

**Algorithm Math:**
Instead of `Δθ = J⁻¹ * Δx`, it computes:
```math
J_{dls} = J^T (J \cdot J^T + \lambda^2 I)^{-1}
\theta_{next} = \theta_{current} + \alpha (J_{dls} \cdot \Delta x)
```
- $J$ is the numeric Jacobian matrix.
- $\lambda$ (damping factor) = `0.05`. Ensures the matrix is always invertible, trading absolute accuracy near singularities for stability.
- Contains safety limits checking: workspace boundary limits, minimum radius to base ($r > 0.12m$), and elbow folding bounds (`elbow_angle < 315°`).

### 2. Minimum Jerk Trajectory Generation
To move from `joint_A` to `joint_B` in joint space seamlessly, the IK solver calculates a time-scaled Minimum Jerk Polynomial:
```math
s(t) = 10t^3 - 15t^4 + 6t^5
```
Where $t$ goes from `0.0` to `1.0`. This ensures that acceleration and velocity start and end at absolutely `0`, producing zero mechanical shock when the robot begins or stops a movement.

### 3. Dynamic Obstacle Avoidance (Octomap)
The IK Solver actively limits paths utilizing an Octomap voxel grid. For validation, it leverages Forward Kinematics (DH Parameters) to compute the 3D position of every joint segment. It then samples points at `3cm` intervals along these limb-segments and checks distance against occupied voxels. If an obstacle is detected mid-movement in the trajectory, the node issues an **EMERGENCY STOP**.

---

## 👁️ Vision & Perception (`colored_map_node.py`)

The vision system bridges 2D RGB-D camera feeds into actionable 3D robotic space.

### TF2 Colored Voxel Mapping
The `colored_map_node` subscribes to the raw depth/RGB point cloud (`/ascamera_hp60c...`) and processes the data:
1. **Coordinate Transformation:** Retrieves `TransformListener` transforms to convert frames from `camera_link` to the global `world` base frame using Quaternion rotation matrices.
2. **Voxel Accumulation:** Groups `(x, y, z)` floats into discrete `2cm` spatial bins. 
3. **Running Averages:** If multiple camera frames observe the same voxel over time, it performs a running average of both the physical coordinate data and the `uint32` RGB colors:
   ```python
   new_rgb = (old_rgb * count + frame_rgb) / (count + 1)
   ```
4. **FIFO Capping:** Holds up to 500,000 points in memory, ensuring the robot constantly maintains an up-to-date, real-world understanding of its surroundings for the IK solver to negotiate.

---

## 🚀 Usage & Deployment

### Dependencies
- **Firmware**: PlatformIO, Adafruit PWMServoDriver, Adafruit INA219.
- **ROS 2 Bridge**: ROS 2 Humble, `tf2_ros`, `octomap_msgs`, Flask, SocketIO, NumPy.

### Quick Start
1. **Flash Firmware**: Navigate to `visiona_firmware/firmware/` and use PlatformIO to flash the ESP32.
2. **Launch Bridge**:
```bash
source install/setup.bash
ros2 run visiona_bridge visiona_bridge_with_gui
```
3. **Launch IK and Vision**:
```bash
ros2 run visiona_bridge simple_ik_solver
ros2 run visiona_bridge colored_map_accumulator
```
