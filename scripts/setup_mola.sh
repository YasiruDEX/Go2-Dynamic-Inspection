#!/bin/bash
# Setup script for MOLA LiDAR Odometry workspace
set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

MOLA_WS="$HOME/ros2_mola_ws"

echo -e "${GREEN}=== MOLA Workspace Setup ===${NC}"
echo ""

# Check if MOLA workspace already exists
if [ -d "$MOLA_WS" ]; then
    echo -e "${YELLOW}MOLA workspace already exists at: $MOLA_WS${NC}"
    read -p "Do you want to remove and reinstall? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing MOLA workspace..."
        rm -rf "$MOLA_WS"
    else
        echo "Keeping existing MOLA workspace. Attempting to build..."
        cd "$MOLA_WS"
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
        echo -e "${GREEN}MOLA workspace built successfully!${NC}"
        exit 0
    fi
fi

# Create workspace directory
echo "Creating MOLA workspace at: $MOLA_WS"
mkdir -p "$MOLA_WS/src"
cd "$MOLA_WS/src"

# Clone MOLA repositories
echo ""
echo -e "${GREEN}Cloning MOLA repositories...${NC}"

# Main MOLA repository
git clone https://github.com/MOLAorg/mola.git

# MOLA common
git clone https://github.com/MOLAorg/mola_common.git

# MOLA LiDAR Odometry (the main package we need)
git clone https://github.com/MOLAorg/mola_lidar_odometry.git

# MOLA IMU preintegration
git clone https://github.com/MOLAorg/mola_imu_preintegration.git

# MOLA state estimation
git clone https://github.com/MOLAorg/mola_state_estimation.git

# MOLA loop closure
git clone https://github.com/MOLAorg/mola_sm_loop_closure.git

# MOLA test datasets (optional)
git clone https://github.com/MOLAorg/mola_test_datasets.git

# MP2P ICP (required dependency)
git clone --recursive https://github.com/MOLAorg/mp2p_icp.git

# Initialize all git submodules
echo ""
echo -e "${GREEN}Initializing git submodules...${NC}"
for dir in */; do
    echo "Checking submodules in $dir"
    cd "$dir"
    git submodule update --init --recursive
    cd ..
done

echo ""
echo -e "${GREEN}Installing system dependencies...${NC}"

# Install MOLA dependencies
sudo apt-get update
sudo apt-get install -y \
    libmrpt-dev \
    mrpt-apps \
    libeigen3-dev \
    libtbb-dev \
    libsuitesparse-dev \
    libglfw3-dev \
    libglfw3 \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-storage

# Install rosdep dependencies
cd "$MOLA_WS"
echo ""
echo -e "${GREEN}Installing ROS dependencies with rosdep...${NC}"
source /opt/ros/humble/setup.bash
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo ""
echo -e "${GREEN}Building MOLA workspace...${NC}"
echo "This may take 10-20 minutes depending on your system..."
echo ""

colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}MOLA workspace built successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Workspace location: $MOLA_WS"
    echo ""
    echo "To use MOLA, source the workspace:"
    echo "  source $MOLA_WS/install/setup.bash"
    echo ""
    echo "Or it will be automatically sourced when running scripts."
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Build failed! Please check the errors above.${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
