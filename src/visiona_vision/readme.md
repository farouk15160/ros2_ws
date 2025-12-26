# Setup and Run Instructions for Visiona Vision System

These instructions cover how to build and run the `ORB_SLAM2`, `ros2_orbslam`, and `ascamera` packages tailored for the Nuwa HP60C camera.

## 1. Prerequisites

Ensure you have the necessary system dependencies installed:

```bash
# Update package list
sudo apt update

# Install ROS 2 dependencies
sudo apt install -y ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-message-filters

# Install Visualization tools (optional but recommended)
sudo apt install -y ros-humble-rqt-image-view
```

## 2. Directory Structure

Assuming you have migrated the code to `~/ros2_ws/src/visiona_vision` using the provided script, your structure should look like this:

```
~/ros2_ws/
  src/
    visiona_vision/
      ORB_SLAM2/          # The core library
      ros2-ORB_SLAM2/     # The ROS 2 wrapper
      ascamera/           # The camera driver
```

## 3. Building

The build process has two parts: the C++ library and the ROS 2 packages.

### Part A: Build ORB_SLAM2 Library

This library must be built natively first.

```bash
cd ~/ros2_ws/src/visiona_vision/ORB_SLAM2
chmod +x build.sh
./build.sh
```

**Verify:** Check that `lib/libORB_SLAM2.so` exists in that directory.

### Part B: Build ROS 2 Packages

Now build the ROS 2 workspace.

```bash
cd ~/ros2_ws

# Important: Tell CMake where to find the ORB_SLAM2 library we just built
export ORB_SLAM2_ROOT_DIR=~/ros2_ws/src/visiona_vision/ORB_SLAM2

# Build the specific packages
colcon build --packages-select ascamera ros2_orbslam --symlink-install
```

## 4. Running the System

You will need **two** terminals.

### Terminal 1: Camera Driver

Start the Nuwa HP60C camera driver.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ascamera hp60c.launch.py
```

*Wait until you see log messages indicating the camera has started and is publishing topics.*

### Terminal 2: ORB_SLAM2 RGB-D

Run the SLAM node. Note the remapping arguments which link the camera topics to the SLAM node.

```bash
cd ~/ros2_ws
source install/setup.bash

# Export variable (required every time in new terminal)
export ORB_SLAM2_ROOT_DIR=~/ros2_ws/src/visiona_vision/ORB_SLAM2

# Run command
ros2 run ros2_orbslam rgbd \
    ~/ros2_ws/src/visiona_vision/ORB_SLAM2/Vocabulary/ORBvoc.txt \
    ~/ros2_ws/src/visiona_vision/ORB_SLAM2/Examples/ROS/ORB_SLAM2/NuwaHP60C.yaml \
    --ros-args \
    --remap camera/rgb/image_raw:=/ascamera_hp60c/camera_publisher/rgb0/image \
    --remap camera/depth/image_raw:=/ascamera_hp60c/camera_publisher/depth0/image_raw
```

## 5. Troubleshooting

*   **"Double free or corruption" on exit**: This is a known issue with the clean-up sequence when pressing Ctrl+C. If your trajectory is saved (look for "trajectory saved!" in the output), you can safely ignore this error.
*   **"Package not found"**: Ensure you have sourced the setup file: `source install/setup.bash`.
*   **No tracking**: Ensure the lens caps are off and the environment has enough texture/light.
*   **Trajectory File**: The `KeyFrameTrajectory.txt` will be saved in the directory where you run the command.

## 6. Visualization

To view the raw camera feed to ensure it's workng:

```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/ascamera_hp60c/camera_publisher/rgb0/image` from the dropdown.
