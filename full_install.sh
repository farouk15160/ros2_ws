#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status.

# ==============================================================================
# Visiona Project - Full Installation & Initialization Script
# Target System: Jetson Nano / Ubuntu 22.04 (Jammy) / ROS2 Humble
# ==============================================================================
# PREVIOUS PROBLEMS & SOLUTIONS LOG:
# 1. Start Issue: "Unable to locate package ros-humble-gazebo-ros"
#    - Cause: The binary `ros-humble-gazebo-ros` does not exist for ARM64/Jammy in standard repos.
#    - Solution: We must build `gazebo_ros_pkgs` from source.
#
# 2. Dependency Hell: "Package 'libgazebo-dev' has no installation candidate"
#    - Cause: The default `gazebo` metapackage is missing/broken in the OSRF PPA for this arch.
#             Also, `universe` repo might be missing.
#    - Solution: Explicitly enabled `universe`. Explicitly installed `gazebo11` and `libgazebo11-dev` binaries.
#
# 3. GPG Error: "NO_PUBKEY" or "curl: (23) Failure writing output"
#    - Cause: `curl | sudo tee` was flaky or permissions failed.
#    - Solution: Switched to `gpg --keyserver` + `gpg --export` which is robust.
#
# 4. Conflict: "CMake Error: Could not find package configuration file provided by gazebo"
#    - Cause: `colcon` tried to build a local clone of `gazebo-classic` (simulator) source.
#    - Solution: We removed `src/gazebo-classic` (using installed binary instead) and ONLY built `src/gazebo_ros_pkgs` (bridge).
# ==============================================================================

# --- Configuration ---
WORKSPACE_DIR=~/ros2_ws
SRC_DIR=$WORKSPACE_DIR/src
VISION_DIR=$SRC_DIR/visiona_vision
INSTALL_PREFIX=$HOME/.local
EIGEN3_CMAKE_DIR=/usr/share/eigen3/cmake

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[SETUP] $1${NC}"
}

error() {
    echo -e "${RED}[ERROR] $1${NC}"
    exit 1
}

warn() {
    echo -e "${YELLOW}[WARNING] $1${NC}"
}

log "Starting Full Installation..."
sleep 3

# ==============================================================================
# 0. Cleanup Conflicting PPAs & Old Sources
# ==============================================================================
# Problem: 'openrobotics-ubuntu-gazebo11-gz-cli-jammy' PPA installs incomplete beta versions.
if [ -f /etc/apt/sources.list.d/openrobotics-ubuntu-gazebo11-gz-cli-jammy.list ]; then
    log "Removing conflicting Gazebo PPA..."
    sudo rm /etc/apt/sources.list.d/openrobotics-ubuntu-gazebo11-gz-cli-jammy.list
fi

# Cleanup old attempted source builds that conflict with binaries
rm -rf ~/ros2_ws/src/gazebo-classic

# ==============================================================================
# 1. System Dependencies & Repositories
# ==============================================================================
log "Step 1: Installing System Dependencies..."

# A. Enable Universe (Correct step for standard libs like libgazebo-dev)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# B. Add OSRF Repository (Robust GPG Method)
# Problem: wget/curl methods failed to import key correctly.
# Solution: Use gpg keyserver directly.
log "Adding OSRF Repository..."
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo gpg --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo gpg --export D2486D2DD83DB69272AFE98867170598AF249743 | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null

sudo apt update
sudo apt install -y curl gnupg2 lsb-release

# ==============================================================================
# 2. Install Binaries (Gazebo Simulator)
# ==============================================================================
# Problem: `gazebo` virtual package missing.
# Solution: Install `gazebo11` and `libgazebo11-dev` explicitly.
log "Step 2: Installing Gazebo Binaries & ROS Dependencies..."
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-xacro \
    gazebo11 \
    libgazebo11-dev \
    ros-humble-gazebo-dev \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-dev-tools \
    python3-pip \
    python3-venv \
    libglew-dev \
    libpython3-dev \
    libopencv-dev \
    git \
    cmake \
    build-essential \
    ros-humble-moveit \
    ros-humble-octomap-server \
    ros-humble-octomap-rviz-plugins


# ==============================================================================
# 3. Clone Missing Sources (Gazebo Bridge)
# ==============================================================================
# Problem: `ros-humble-gazebo-ros-pkgs` binary missing on ARM64.
# Solution: Build it from source.
log "Step 3: Cloning Gazebo ROS Bridge (Source)..."
cd $SRC_DIR
if [ ! -d "gazebo_ros_pkgs" ]; then
    git clone -b ros2 https://github.com/ros-simulation/gazebo_ros_pkgs.git
fi

# Additional Perception Deps
if [ ! -d "vision_msgs" ]; then
    git clone -b ros2 https://github.com/ros-perception/vision_msgs.git
fi
if [ ! -d "image_common" ]; then
    git clone -b ros2 https://github.com/ros-perception/image_common.git
fi

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || warn "rosdep init failed"
fi
rosdep update || warn "rosdep update failed"

log "Installing Python Dependencies..."
pip3 install Flask Flask-SocketIO python-socketio pyserial gunicorn numpy opencv-python flask-cors platformio   

# ==============================================================================
# 4. Pangolin Installation (Manual Build)
# ==============================================================================
log "Step 4: Building Pangolin..."
mkdir -p $VISION_DIR
cd $VISION_DIR

if [ ! -d "Pangolin" ]; then
    git clone https://github.com/stevenlovegrove/Pangolin.git
fi

cd Pangolin
touch COLCON_IGNORE
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install

export CMAKE_PREFIX_PATH=$INSTALL_PREFIX:$EIGEN3_CMAKE_DIR:$CMAKE_PREFIX_PATH

# ==============================================================================
# 5. ORB_SLAM2 Setup & Patching
# ==============================================================================
log "Step 5: Setting up ORB_SLAM2..."
cd $VISION_DIR

if [ ! -d "ORB_SLAM2" ]; then
    git clone https://github.com/raulmur/ORB_SLAM2.git
fi

cd ORB_SLAM2
touch COLCON_IGNORE

log "Applying Patches..."
# Patch 1: ORBextractor.h Includes
if grep -q "opencv/cv.h" include/ORBextractor.h; then
    sed -i 's|#include <opencv/cv.h>|#include <opencv2/core/core.hpp>\n#include <opencv2/features2d/features2d.hpp>\n#include <opencv2/imgproc/imgproc.hpp>|' include/ORBextractor.h
fi

# Patch 2: PnPsolver.h Legacy Includes
if ! grep -q "opencv2/core/core_c.h" include/PnPsolver.h; then
    sed -i '/#include <opencv2\/core\/core.hpp>/a #include <opencv2/core/core_c.h>\n#include <opencv2/core/types_c.h>' include/PnPsolver.h
fi

# Patch 3: LoopClosing.h Allocator Fix
sed -i 's|std::pair<const KeyFrame\*, g2o::Sim3>|std::pair<KeyFrame* const, g2o::Sim3>|g' include/LoopClosing.h

# Patch 4: CMakeLists.txt (C++11/14 Defs & Eigen Target)
if ! grep -q "DCOMPILEDWITHC11" CMakeLists.txt; then
    sed -i '/add_definitions(-DCOMPILEDWITHC14)/a \   add_definitions(-DCOMPILEDWITHC11)' CMakeLists.txt
fi

if ! grep -q "DEFINED Eigen3::Eigen" CMakeLists.txt; then
    sed -i '/find_package(Eigen3 3.1.0 REQUIRED)/a if(NOT TARGET Eigen3::Eigen)\n  message(STATUS "Defined missing Eigen3::Eigen target")\n  add_library(Eigen3::Eigen INTERFACE IMPORTED)\n  set_target_properties(Eigen3::Eigen PROPERTIES\n    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")\nendif()' CMakeLists.txt
fi
sed -i '/${EIGEN3_LIBS}/d' CMakeLists.txt

# Patch 5: Mass Replacements
find src -name "*.cc" -print0 | xargs -0 sed -i 's/CV_REDUCE_SUM/cv::REDUCE_SUM/g'
find src -name "*.cc" -print0 | xargs -0 sed -i 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g'
find src Examples -name "*.cc" -print0 | xargs -0 sed -i 's/CV_FONT_HERSHEY_SIMPLEX/cv::FONT_HERSHEY_SIMPLEX/g; s/CV_FONT_HERSHEY_PLAIN/cv::FONT_HERSHEY_PLAIN/g; s/CV_AA/cv::LINE_AA/g; s/CV_WINDOW_AUTOSIZE/cv::WINDOW_AUTOSIZE/g; s/CV_WINDOW_KEEPRATIO/cv::WINDOW_KEEPRATIO/g'
find src Examples -name "*.cc" -print0 | xargs -0 sed -i 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g'

# ==============================================================================
# 6. ORB_SLAM2 Build
# ==============================================================================
log "Step 6: Building ORB_SLAM2 (Manual Build)..."
mkdir -p build && cd build
export CMAKE_PREFIX_PATH=$INSTALL_PREFIX:$EIGEN3_CMAKE_DIR:$CMAKE_PREFIX_PATH
cmake .. -DCMAKE_BUILD_TYPE=Release -DEigen3_DIR=$EIGEN3_CMAKE_DIR
make -j$(nproc)

log "Verifying verification artifacts..."
if [ ! -f "../lib/libORB_SLAM2.so" ]; then
    error "ORB_SLAM2 Shared Library not found!"
fi

# ==============================================================================
# 7. ROS2 Workspace Build
# ==============================================================================
log "Step 7: Building ROS2 Workspace..."
cd $WORKSPACE_DIR

echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash

# Export required variables for the build
export ORB_SLAM2_ROOT_DIR=$VISION_DIR/ORB_SLAM2
export LD_LIBRARY_PATH=$ORB_SLAM2_ROOT_DIR/lib:$ORB_SLAM2_ROOT_DIR/Thirdparty/DBoW2/lib:$ORB_SLAM2_ROOT_DIR/Thirdparty/g2o/lib:$LD_LIBRARY_PATH

echo "Running colcon build..."
colcon build --symlink-install --cmake-args \
    -DORB_SLAM2_ROOT_DIR=$ORB_SLAM2_ROOT_DIR \
    -DORB_SLAM2_DIR=$ORB_SLAM2_ROOT_DIR


echo "Updating .bashrc with colcon-argcomplete..."
if ! grep -q "colcon-argcomplete.bash" ~/.bashrc; then
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc
fi
echo "Updating .bashrc with GAZEBO_MODEL_PATH..."
if ! grep -q "GAZEBO_MODEL_PATH" ~/.bashrc; then
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix visiona_bridge)/share' >> ~/.bashrc
fi

# 1. Disable online model lookup (prevents the crash)
export GAZEBO_MODEL_DATABASE_URI=""
log "========================================================"
log " BUILD COMPLETE SUCCESSFULLY"
log "========================================================"
echo "To launch the simulation:"
echo "  1. source ~/ros2_ws/install/setup.bash"
echo "  2. ros2 launch visiona_bridge spawn_visiona.launch.py mode:=sim"
echo ""
