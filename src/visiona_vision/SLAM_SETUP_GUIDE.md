# ORB-SLAM2 and SLAM Setup Guide for Visiona Vision

This guide explains how to set up and run ORB-SLAM2 and other SLAM solutions with the Visiona robot and ASCamera depth camera.

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Environment Setup](#environment-setup)
- [ORB-SLAM2 Setup](#orb-slam2-setup)
- [Running SLAM](#running-slam)
- [Known Issues & Solutions](#known-issues--solutions)
- [Alternative SLAM Solutions](#alternative-slam-solutions)

---

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- 4GB+ RAM (8GB recommended for SLAM)
- USB 3.0 port for ASCamera

### Required Packages
```bash
sudo apt update
sudo apt install -y \
    libglew-dev \
    libpython2.7-dev \
    ffmpeg \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff5-dev \
    libopenexr-dev \
    libgtk-3-dev \
    libdc1394-dev \
    libv4l-dev \
    libboost-all-dev \
    libeigen3-dev
```

---

## Environment Setup

### 1. Pangolin Library Setup

Pangolin is required for ORB-SLAM2 visualization.

**Build Pangolin:**
```bash
cd ~/ros2_ws/src/visiona_vision/Pangolin
mkdir -p build
cd build
cmake ..
make -j4
```

**Add to system library path:**
```bash
echo "/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build" | sudo tee /etc/ld.so.conf.d/pangolin.conf
sudo ldconfig
```

**Verify installation:**
```bash
ldconfig -p | grep pangolin
# Should show: libpango_windowing.so, libpango_display.so, etc.
```

### 2. ORB-SLAM2 Library Setup

**Add ORB-SLAM2 to library path:**
```bash
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/ORB_SLAM2/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build:$LD_LIBRARY_PATH
```

**Make permanent (add to ~/.bashrc):**
```bash
echo 'export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/ORB_SLAM2/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 3. Build SLAM Packages

```bash
cd ~/ros2_ws
colcon build --packages-select yahboomcar_slam ros2_orbslam --symlink-install
source install/setup.bash
```

---

## ORB-SLAM2 Setup

### Camera Calibration Parameters

The ASCamera HP60C parameters are configured in:
```
~/ros2_ws/src/visiona_vision/yahboomcar_slam/params/rgbd.yaml
```

**Default calibration:**
```yaml
Camera.fx: 595.111
Camera.fy: 594.785
Camera.cx: 335.371
Camera.cy: 239.169

DepthMapFactor: 1000.0  # Depth scale
```

### ORB Vocabulary

The vocabulary file must exist at:
```
~/ros2_ws/install/yahboomcar_slam/share/yahboomcar_slam/params/ORBvoc.txt
```

If missing, download from ORB-SLAM2 repository.

---

## Running SLAM

### Method 1: Camera Only (Simple Test)

**Terminal 1 - Launch Camera:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py
```

**Terminal 2 - Visualize in RViz:**
```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

**In RViz:**
1. Set Fixed Frame: `ascamera_hp60c_ascamera_0`
2. Add → PointCloud2 → Topic: `/ascamera_hp60c/depth/points`
3. Add → Image → Topic: `/ascamera_hp60c/rgb/image_raw`
4. Add → TF to see coordinate frames

### Method 2: ORB-SLAM2 (Full Pipeline)

> **⚠️ WARNING:** ORB-SLAM2 is currently crashing due to segmentation fault. See [Known Issues](#known-issues--solutions).

**Step 1 - Start Camera:**
```bash
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build:$LD_LIBRARY_PATH
source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py
```

**Step 2 - Start ORB-SLAM2 (wait 5 seconds after camera):**
```bash
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/ORB_SLAM2/lib:$LD_LIBRARY_PATH
source ~/ros2_ws/install/setup.bash
ros2 launch yahboomcar_slam orbslam_base_launch.py
```

**Step 3 - Start Point Cloud Mapping (wait 10 seconds after ORB-SLAM2):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch yahboomcar_slam test_orbslam_simple.launch.py
```

**Step 4 - Visualize (Optional):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch yahboomcar_slam display_pcl_launch.py
```

### Method 3: Using Test Script

A convenience script is provided:
```bash
cd ~/ros2_ws
./test_orbslam.sh
```

This automatically:
- Sets library paths
- Launches camera
- Launches ORB-SLAM2 (with delays)
- Launches point cloud mapping

---

## Known Issues & Solutions

### Issue 1: `libpango_windowing.so.0: cannot open shared object file`

**Cause:** Pangolin not in system library path

**Solution:**
```bash
echo "/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build" | sudo tee /etc/ld.so.conf.d/pangolin.conf
sudo ldconfig
```

### Issue 2: `rgbd_pose` crashes with segmentation fault (exit code -11)

**Cause:** Multiple potential issues:
- Missing/corrupted ORBvoc.txt vocabulary file
- Incompatible camera parameters in rgbd.yaml
- Memory allocation issues in ORB-SLAM2

**Current Status:** ❌ **UNRESOLVED** - ORB-SLAM2 crashes immediately

**Attempted Solutions:**
1. Verified Pangolin installation ✓
2. Added library paths ✓
3. Fixed camera launch order ✓
4. Vocabulary file exists ✓

**Recommended Workaround:** Use rtabmap (see Alternative SLAM Solutions below)

### Issue 3: Camera opened twice - "Busy" error

**Cause:** Multiple launches trying to access same camera

**Solution:** Launch camera only once, not in both `hp60c.launch.py` AND `orbslam_base_launch.py`

**Fixed in:** `orbslam_base_launch.py` - now launches camera only once

### Issue 4: RViz "queue full" warnings

**Message:**
```
Message Filter dropping message: frame 'ascamera_hp60c_ascamera_0' at time ... for reason 'discarding message because the queue is full'
```

**Cause:** Camera publishes faster than RViz processes (normal behavior)

**Impact:** ℹ️ **INFORMATIONAL** - does not affect functionality

**Solution:** Can be safely ignored, or reduce camera FPS in camera config

### Issue 5: Missing octomap RViz plugins

**Error:**
```
The class required for this display, 'octomap_rviz_plugins/OccupancyGrid', could not be loaded
```

**Cause:** octomap_rviz_plugins not installed

**Impact:** ⚠️ **MINOR** - Only affects 3D occupancy grid visualization

**Solution (Optional):**
```bash
sudo apt install ros-humble-octomap-rviz-plugins
```

### Issue 6: Point cloud mapping crashes (exit code -11)

**Cause:** Segmentation fault in `point_cloud_mapping` node

**Possible Causes:**
- Memory issues with PCL library
- Invalid camera parameters
- Missing ORB-SLAM2 pose data

**Current Status:** ❌ **INVESTIGATING** - Dependent on ORB-SLAM2 fix

---

## Alternative SLAM Solutions

Since ORB-SLAM2 has stability issues, consider these alternatives:

### rtabmap-ros (Recommended)

**Advantages:**
- Native ROS2 support
- More stable than ORB-SLAM2
- Better documentation
- Works well with RGBD cameras
- Built-in visualization

**Installation:**
```bash
sudo apt install ros-humble-rtabmap-ros
```

**Launch:**
```bash
# Terminal 1 - Camera
ros2 launch ascamera hp60c.launch.py

# Terminal 2 - rtabmap
ros2 launch rtabmap_ros rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/ascamera_hp60c/rgb/image_raw \
    depth_topic:=/ascamera_hp60c/depth/image_raw \
    camera_info_topic:=/ascamera_hp60c/rgb/camera_info \
    approx_sync:=true
```

### SLAM Toolbox (2D SLAM)

**For 2D mapping with LiDAR or depth camera:**
```bash
sudo apt install ros-humble-slam-toolbox
```

### Google Cartographer

**For complex 3D mapping:**
```bash
sudo apt install ros-humble-cartographer-ros
```

---

## Quick Reference

### Essential Commands

**Check library paths:**
```bash
echo $LD_LIBRARY_PATH
ldconfig -p | grep pangolin
ldconfig -p | grep ORB_SLAM
```

**List camera topics:**
```bash
ros2 topic list | grep ascamera
```

**View camera info:**
```bash
ros2 topic echo /ascamera_hp60c/rgb/camera_info
```

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf
```

**Monitor point cloud:**
```bash
ros2 topic hz /ascamera_hp60c/depth/points
```

### File Locations

| Item | Path |
|------|------|
| Pangolin build | `/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build` |
| ORB-SLAM2 lib | `/home/farouk/ros2_ws/src/visiona_vision/ORB_SLAM2/lib` |
| SLAM packages | `/home/farouk/ros2_ws/src/visiona_vision/yahboomcar_slam` |
| Camera params | `~/ros2_ws/src/visiona_vision/yahboomcar_slam/params/rgbd.yaml` |
| Saved maps | `~/ros2_ws/src/visiona_vision/yahboomcar_slam/pcl/` |

---

## Troubleshooting Checklist

Before running SLAM:

- [ ] Pangolin installed and in LD_LIBRARY_PATH
- [ ] ORB-SLAM2 lib in LD_LIBRARY_PATH
- [ ] Camera launches successfully alone
- [ ] Can see point cloud in RViz
- [ ] TF tree shows camera frame
- [ ] `rgbd.yaml` parameters match camera calibration
- [ ] ORBvoc.txt exists
- [ ] No other process using camera (check with `lsusb` and `fuser`)

---

## Contributing

If you fix any of the known issues or improve the setup process, please document it here!

**Current Contributors:**
- Farouk - Initial setup and testing
- Antigravity AI - Documentation and debugging

**Last Updated:** 2026-01-03
