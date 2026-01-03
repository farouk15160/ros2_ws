#!/bin/bash
# Complete ORB-SLAM2 Point Cloud Mapping Test
# Run this script to test the entire pipeline

echo "=== ORB-SLAM2 Point Cloud Mapping Test ==="
echo ""

# Add Pangolin to library path
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/Pangolin/build:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/farouk/ros2_ws/src/visiona_vision/ORB_SLAM2/lib:$LD_LIBRARY_PATH

# Source ROS2
source /home/farouk/ros2_ws/install/setup.bash

echo "Step 1: Launching camera..."
ros2 launch ascamera hp60c.launch.py &
CAMERA_PID=$!
sleep 5

echo "Step 2: Launching ORB-SLAM2..."
ros2 launch yahboomcar_slam orbslam_base_launch.py &
ORBSLAM_PID=$!
sleep 10

echo "Step 3: Launching point cloud mapping..."
ros2 launch yahboomcar_slam test_orbslam_simple.launch.py &
PCL_PID=$!

echo ""
echo "=== All nodes launched! ==="
echo "Camera PID: $CAMERA_PID"
echo "ORB-SLAM2 PID: $ORBSLAM_PID"
echo "Point Cloud PID: $PCL_PID"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

# Wait and cleanup on exit
trap "kill $CAMERA_PID $ORBSLAM_PID $PCL_PID 2>/dev/null; exit" INT TERM
wait
