# AI-Powered Robotic Arm Workspace

This repository contains the complete software suite for an intelligent, vision-enabled 6-axis robot arm. The goal of this workspace is to bridge the gap between low-level hardware control and high-level AI-driven tasks.

It integrates real-time computer vision (YOLO) on an embedded platform (Jetson Nano) with a robust web control interface, enabling the robot to perceive its environment and perform autonomous "smart" movements.



---

## 🚀 Core Projects

This workspace is a monorepo containing several key projects:

* **`/robot-control-interface`**: The primary control station. This project includes:
    * A **Python Backend** (Flask/Socket.IO) that serves as the main API for the robot.
    * A **Web UI** (HTML/JS/three.js) for real-time 3D visualization, manual joint control, and a pose sequencer.

* **`/vision-cv`**: The "eyes" of the robot. This project contains all computer vision logic:
    * **Camera Integration**: Scripts for capturing and processing feeds from USB or CSI cameras.
    * **YOLO Object Detection**: Pre-trained and custom-trained YOLO models for identifying and locating objects, tools, or hands in the robot's workspace.

* **`/jetson-ai-core`**: The "brain" of the robot, designed to run on a Jetson Nano.
    * **Smart Movement**: Takes 2D pixel coordinates from the `/vision-cv` module, performs coordinate transformation (e.g., pixel-to-world), and calculates the 3D world coordinates for the robot.
    * **Task Logic**: Contains high-level AI scripts that define autonomous tasks like "pick and place all red blocks" or "follow my hand." This is where "other AI features" like path planning or gesture recognition would live.

---

## 🛠️ System Architecture

The projects are designed to work together in a coordinated system:

1.  A **Camera** streams its video feed directly to the **Jetson Nano**.
2.  The `/vision-cv` project (running on the Jetson) processes this feed with **YOLO** to identify objects and their pixel locations.
3.  The `/jetson-ai-core` module receives these locations, calculates the target 3D (X,Y,Z) world coordinates, and determines the correct action.
4.  The Jetson then sends high-level commands (e.g., `GOTO_XYZ(x, y, z)`) to the `/robot-control-interface` **Backend** (running on the Jetson or a host PC).
5.  The **Backend** translates this command into a series of low-level joint movements and streams them to the physical **Robot Arm**.
6.  The **Web UI** can be opened on any device (laptop, tablet) to monitor the robot's 3D model, view logs, or override the AI and take manual control.



[Image of a system architecture diagram]


---

## ⚡ Key Technologies

* **AI / CV**: Python, PyTorch / TensorFlow, YOLO (v5/v8), OpenCV
* **Hardware**: Jetson Nano, 6-Axis Robot Arm, 2D/3D Camera
* **Control & Backend**: Python, Flask, Socket.IO, `pyserial`
* **Frontend**: HTML5, CSS3, JavaScript (ES6+), three.js

---

## 🏁 Getting Started

This is a multi-project workspace. Each sub-directory (e.g., `/robot-control-interface`, `/vision-cv`) is a standalone project with its own `README.md` and dependencies.

1.  **Clone this repository:**
    ```bash
    git clone [https://github.com/your-username/your-repo-name.git](https://github.com/your-username/your-repo-name.git)
    cd your-repo-name
    ```

2.  **Explore a project:**
    ```bash
    cd robot-control-interface
    ```

3.  **Follow the setup instructions:**
    * Read the `README.md` inside that specific project's folder.
    * Install its `requirements.txt`.
    * Follow its specific instructions for hardware setup (e.g., connecting the arm, setting up the Jetson Nano, downloading YOLO weights).
