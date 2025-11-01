# Robot Arm Control V4.3

A comprehensive, browser-based interface for controlling a 6-axis robot arm. This UI provides real-time 3D visualization, manual controls, a pose sequencer, and deep configuration settings. It's designed to communicate with a Python backend (via Socket.IO and REST API) to control physical hardware or run in a full simulation mode.

![3D Model Calibration Interface](image_d93ad6.png)

## ✨ Features

* **Real-time 3D View:** A `three.js` scene that accurately visualizes the robot's pose, updated live from the controller.
* **Tabbed Interface:**
    * **🤖 Control:** Manual sliders for all 6 joints, speed control, gripper commands, and fan control.
    * **📊 Sequencer:** Create, play, stop, and clear sequences of saved poses. Sequences can be saved to and loaded from the server as JSON.
    * **⚙️ Settings:** Configure low-level controller parameters like joint limits and collision thresholds, and save them to the MCU.
* **Live Status Panel:** Monitors connection status (MCU or Simulation), Emergency Stop state, motor currents, and current joint angles.
* **Saved Positions:** Save the robot's current pose with a name, then return to it later.
* **Fullscreen Jog Mode:** Enter a fullscreen 3D view with overlay buttons for jogging each joint, ideal for tablets or touch displays.
* **3D Model Calibration:** A visual editor in the Settings tab to fine-tune the position (XYZ) and rotation (RPY) offsets for each joint's 3D model, allowing you to perfectly match the visual representation to your physical hardware.

## 🚀 Tech Stack

* **Frontend:** HTML5, CSS3, vanilla JavaScript (ES6 Module)
* **3D Rendering:** `three.js`
* **Real-time Communication:** `socket.io` (Client)

## 🖥️ Backend (Assumed)

This frontend is designed to work with a backend (likely Python/Flask) that provides:

1.  A **Socket.IO server** for pushing real-time `status_update` and `log_message` events to the client.
2.  A **REST API** at `/api/` to receive commands, such as:
    * `/api/send_joints`
    * `/api/home`
    * `/api/set_speed`
    * `/api/save_position`
    * `/api/play_sequence`
    * ...and many others.
3.  A `/static/models/` directory to serve the `.stl` files for the robot parts.

## 🛠️ How to Use

1.  **Start the Backend:** Run your Python (Flask/Socket.IO) server.
2.  **Serve the Frontend:** Serve the `index.html` file. A simple way is to use the Python HTTP server:
    ```bash
    python -m http.server
    ```
3.  **Open:** Navigate to `http://localhost:8000` in your browser. The interface will automatically try to connect to the WebSocket server.
