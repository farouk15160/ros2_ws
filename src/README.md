# 🦾 Visiona Robotic System - Deep Technical Architecture

Welcome to the internal engineering documentation for the **Visiona ecosystem**. This document serves as a deep dive into the mathematical, logical, and architectural decisions underlying the integration of a 6-DOF robotic arm with dual-core ESP32 safety hardware, custom Python ROS 2 bridges, NumPy-based non-linear Inverse Kinematics, and localized VLA (Voice-Language-Action) intelligence.

---

## 📑 Technical Architecture Overview

The system operates across three primary conceptual nodes:
1. **The Physical Node (SentinelArm-6 Firmware):** A dual-core ESP32 Dev Module running FreeRTOS with hard real-time safety constraints.
2. **The Bridging & Kinematics Node (`visiona_bridge`):** A Python-based ROS 2 Humble package that handles physical-to-logical translation, fast IK solving, and Web GUI (Flask/Socket.IO) teleoperation.
3. **The Perception & Intelligence Node (`visiona_vision` / `llm_control`):** An orchestrated stack comprising ORB-SLAM2, Octomap Server, and a localized `llama-cpp-python` VLA agent.

---

## 1. Low-Level Firmware: SentinelArm-6 (C++ / FreeRTOS)

**Repository Context:** `visiona_firmware/firmware/src/`

The firmware eschews a single Arduino `loop()` in favor of deterministic **FreeRTOS task scheduling** across the ESP32's two Xtensa cores.

### A. Dual-Core Task Allocation
- **Core 0 (Monitoring & Safety Task):** 
  Runs `task_monitoring` at high frequency (pinning). It acts as the ultimate physical safety net.
  - **Sensors:** Reads external current via an ACS712 (analog) for the main system, and an INA219 (I2C) specifically for the gripper.
  - **IIR Filtering:** Raw ADC current readings are smoothed using an Infinite Impulse Response (IIR) filter: `g_avgCurrent_A = (g_avgCurrent_A * 0.9) + (g_mainCurrent_A * 0.1)`.
  - **Trigger Logic:** Implements both an *Absolute Threshold* (e.g., > 3.0A triggers an instantaneous E-Stop via `perform_safe_shutdown()`) and a *Deviation Threshold* (detecting sudden spikes > 0.4A that deviate from the IIR running average). Crucially, it employs a `DEVIATION_GRACE_PERIOD_MS` to mask the massive inductive inrush currents that occur when 6 servos start accelerating simultaneously from a dead stop.
  
- **Core 1 (Motion & Serial Parsing Tasks):**
  - **Command Parser:** Unpacks generic binary structs incoming at 921600 Baud on the hardware UART.
  - **Interpolation Engine:** Runs `task_motion_interpolator` at exactly 100Hz.

### B. The Motion Interpolation Engine
Rather than relying on the ROS layer to stream 100Hz micro-steps (which is highly vulnerable to serial jitter), the ROS bridge simply sends the *final* Cartesian target joint configurations. The ESP32 handles the physical easing.

* **Dynamic Alpha Calculation:** To allow variable "speed" commands to dictate motion curves smoothly, the firmware computes an exponential tracking `alpha`. 
  Because `$Speed Factor \propto \Delta \theta / \text{Time}$`, the code derives an adaptive smoothing factor. For fast moves, `alpha` approaches `0.3`, allowing the joint to track the target tightly. For smooth, cinematic moves (speed index ~500), `alpha` shrinks to ~`0.002`, meaning `current_angle += (target - current) * 0.002` operates like a heavily damped spring, erasing mechanical jitter.

### C. The Binary Serial Protocol
The bridging node and ESP32 communicate via a packed binary `struct`.
* **Command Packet (`<BB8f`):** [Header `0xA5`] + [Char ID (e.g., 'M' for Move)] + [6x 32-bit floats for Joint Angles] + [32-bit float Speed] + [32-bit float Gripper Current]. Total: 34 Bytes. Checked via XOR parity.

---

## 2. Core ROS 2 Bridge & Kinematics

**Repository Context:** `visiona_bridge/visiona_bridge/`

### A. Modular Node Architecture (`bridge_node.py`)
The main Python node strictly adheres to MVC separation:
- **`hardware/`**: Contains `VirtualHardware` state tracking and `SerialInterface` utilizing `pyserial` and `struct.pack/unpack` to interface with the ESP32.
- **`ros2_interface/`**: Contains pure `rclpy.Publisher` and `Subscriber` classes (isolated from business logic) translating Python dictionaries into `sensor_msgs/JointState` or `geometry_msgs/PoseStamped` structures, and publishing the `tf2` transforms from `world` -> `base_link` -> `gripper`.

### B. The Simple IK Solver (`simple_ik_solver.py`)
This is the mathematical core of the bridge. To avoid the ~2.5 second latency of MoveIt (which uses OMPL tree-search), Visiona uses a direct Numpy-based Non-linear solver.

#### Mathematics of the IK Engine
1. **Forward Kinematics (FK):** Implements standard Denavit-Hartenberg (DH) parameters generating 4x4 Homogeneous Transformation Matrices for the 4 primary joints: `$T = Rot_z(\theta) \cdot Trans_z(d) \cdot Trans_x(a) \cdot Rot_x(\alpha)$`.
2. **Numeric Jacobian ($J$):** Rather than deriving the complex analytical partial derivatives, the solver uses computational limits: `$J_{ij} = [FK(\theta + \epsilon) - FK(\theta)]_i / \epsilon$` creating a 3x4 Jacobian mapping joint velocities to Cartesian XYZ velocities.
3. **Damped Least Squares (DLS):** A standard Jacobian Pseudoinverse `$J^+ = J^T (J J^T)^{-1}$` becomes mathematically unstable (approaching infinite joint velocities) when the arm nears a "singularity" (e.g., fully outstretched). 
   To solve this, Visiona uses Levenberg-Marquardt (DLS):
   `$\Delta \theta = J^T (J J^T + \lambda^2 I)^{-1} \Delta X$`
   Where `$\lambda=0.05$` is the damping factor. This artificially limits the math from producing exploding joint velocities at the cost of slight precision near edges.

#### Joint Space Minimum Jerk Trajectories
Once the target joints $\theta_{target}$ are found, we do not interpolate linearly. Linear interpolation causes infinite acceleration (jerk) at $t=0$ and $t=1$, damaging the physical gears.
Instead, we apply a **5th-Order Polynomial**:
`$s(t) = 10t^3 - 15t^4 + 6t^5$` where $t \in [0, 1]$.
This ensures continuous position, zero velocity, and zero acceleration at the start and end of the motion.

#### 360-Degree Angle Wrapping Mitigation
A common robotic flaw: If the continuous base joint is currently at `3.0 rads`, and the IK determines the best solution is `-3.2 rads`, a naive controller will spin the robot `-6.2 rads` backward. 
The solver intercepts this by normalizing the $\Delta \theta$:
`$\Delta\theta_0 = ((\Delta\theta_0 + \pi) \bmod 2\pi) - \pi$`. This restricts the move to the absolute shortest geometric path $\in (-\pi, \pi]$.

---

## 3. Perception, SLAM, & Collision Validation

### A. The Sensory Array
Operates an **ASCamera HP60C** utilizing the `ros2_orbslam` package to generate feature-matched odometry (via ORB Vocabulary matching) and aligned depth point clouds `/depth0/points`.
*(Note: As ORB-SLAM2 occasionally experiences memory allocation seg-faults in dense ROS 2 environments, `rtabmap-ros` is recommended as the production substitute).*

### B. OctoMap Occupancy Grids
The massive point clouds are injected into the OctoMap server, generating 3D voxel matrices. The bridge param `mapping:=high` instantiates 1.5cm voxels, whereas `mapping:=low` generates faster 2.0cm voxels.

### C. Whole-Arm Collision Math
Before executing a Minimum-Jerk trajectory, the IK solver interrogates the `octomap_msgs/Octomap`. 
It calculates the exact 3D Cartesian line-segment for every physical link of the arm using the DH FK. It then discretizes these segments into `3cm` sample points. 
If *any* sampled point along the arm lies inside an occupied voxel index in the OctoMap HashSet, the system flags the physical bounds as hazardous and aborts the Move. (It re-checks this dynamically every 5 control ticks during active movement to prevent moving into a newly introduced obstacle).

---

## 4. The Agentic VLA Stack (Voice-Language-Action)

**Repository Context:** `visiona_bridge/llm_control.launch.py`

The system abandons hard-coded behavior trees for an LLM-driven planner.

- **The Natural Language Substrate:** `llama-cpp-python` runs a local quantized model, listening to `/llm/command` (e.g., *"Hebe die Taste auf dem Tisch auf"*). 
- **Task Decomposition:** The LLM is system-prompted to output strict JSON arrays: `[{"action": "move_to", "target": "Taste", "params": {}}, {"action": "grasp"}]`.
- **Visual Grounding (VLA Stub):** Translates semantic nouns (e.g., "Taste") into numerical XYZ coordinates using bounding-box centroids from the RGB stream, mapped to depth via the depth camera.
- **Continuous Visual Servoing (PBVS):** Because the target object might map to `(0.32, -0.1, 0.05)`, but the user nudges the object, a high-frequency (40ms) PBVS (Position Based Visual Servoing) node skips the heavy IK path-planner. It continuously calculates the Cartesian error matrix $(X_{target} - X_{current})$, dynamically re-computing Joint velocities via the Jacobian to strictly track the object until the grasp executes.

---

## 5. Web Teleoperation (Digital Twin)

**Repository Context:** `visiona_bridge/gui/`

A Flask Application running a `gunicorn`/`eventlet` backend.
- **Three.js Digital Twin:** WebGL renders a URDF-equivalent mesh of the arm in the browser. 
- **WebSocket Streaming:** The Python Bridge node catches `/joint_states` and pushes them at 20Hz via `socketio.emit()`. The frontend maps these generic float arrays into rotation matrices applied to the `Three.js` Object3D bones, resulting in zero-latency mirrored movements.
- **EEPROM Modification Route:** Sliders on the frontend send REST callbacks to update the hard `SERVOS_MIN` / `SERVOS_MAX` limits, which are formatted into `ConfigPacket ('C')` arrays and saved permanently into the ESP32's onboard Flash NVRAM.

---
**Maintained by:** Farouk Jamali & Antigravity AI  
**Last Updated:** February 2026
