#!/bin/bash

# Manual Launch Script - Based on User's Custom Commands
# This script runs all components manually with user-specified parameters

set -e

cd /home/tharushi/Go2-Dynamic-Inspection

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Manual Launch - User Custom Configuration               ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Ask for map file location
echo -e "${YELLOW}Enter the full path to your myMap.mm file:${NC}"
echo -e "${YELLOW}(Press Enter for default: ~/ros2_mola_ws/myMap.mm)${NC}"
read -r MAP_FILE
MAP_FILE=${MAP_FILE:-~/ros2_mola_ws/myMap.mm}
MAP_FILE=$(eval echo $MAP_FILE)  # Expand tilde

if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
    exit 1
fi

# Ask for rosbag location
echo -e "${YELLOW}Enter the rosbag name (in ~/Go2-Dynamic-Inspection/rosbags/):${NC}"
echo -e "${YELLOW}(Press Enter for default: rosbag2_2026_03_03-15_06_01)${NC}"
read -r ROSBAG_NAME
ROSBAG_NAME=${ROSBAG_NAME:-rosbag2_2026_03_03-15_06_01}

ROSBAG_PATH="/home/tharushi/Go2-Dynamic-Inspection/rosbags/$ROSBAG_NAME"
if [ ! -d "$ROSBAG_PATH" ]; then
    echo -e "${RED}Error: Rosbag not found: $ROSBAG_PATH${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Configuration:${NC}"
echo -e "  Map file: $MAP_FILE"
echo -e "  Rosbag:   $ROSBAG_PATH"
echo ""

# Function to wait for a process to be ready
wait_for_topic() {
    local topic=$1
    local timeout=30
    local elapsed=0
    
    echo -ne "Waiting for topic $topic..."
    while ! ros2 topic list 2>/dev/null | grep -q "$topic"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ $elapsed -ge $timeout ]; then
            echo -e " ${RED}TIMEOUT${NC}"
            return 1
        fi
    done
    echo -e " ${GREEN}OK${NC}"
    return 0
}

# 1. Start MOLA with custom map
echo -e "${GREEN}[1/6] Starting MOLA with custom map...${NC}"
export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
gnome-terminal --title="MOLA Localization" -- bash -c "
    source /home/tharushi/ros2_mola_ws/install/setup.bash
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        start_active:=True \
        publish_localization_following_rep105:=False \
        start_mapping_enabled:=False \
        lidar_topic_name:=\"/livox/lidar\" \
        imu_topic_name:=\"/livox/imu\" \
        mola_tf_base_link:=\"base_link\" \
        mola_deskew_method:=\"MotionCompensationMethod::IMU\" \
        mola_initial_map_mm_file:=\"$MAP_FILE\"; exec bash
"
sleep 5

# 2. Start TF Publisher
echo -e "${GREEN}[2/6] Starting TF publisher...${NC}"
gnome-terminal --title="TF Publisher" -- bash -c "
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame; exec bash
"
sleep 2

# Wait for critical topics before starting rosbag
wait_for_topic "/lidar_odometry/pose" || echo -e "${YELLOW}Warning: MOLA may not be fully ready${NC}"

# 3. Start rosbag playback
echo -e "${GREEN}[3/6] Starting rosbag playback...${NC}"
gnome-terminal --title="Rosbag Player" -- bash -c "
    ros2 bag play $ROSBAG_PATH --loop; exec bash
"
sleep 3

# 4. Start terrain analysis
echo -e "${GREEN}[4/6] Starting terrain analysis...${NC}"
gnome-terminal --title="Terrain Analysis" -- bash -c "
    source /home/tharushi/Go2-Dynamic-Inspection/workspaces/terrain_analyzer/install/setup.bash
    ros2 launch terrain_analysis terrain_analysis.launch; exec bash
"
sleep 2

# 5. Start terrain analysis extended
echo -e "${GREEN}[5/6] Starting terrain analysis extended...${NC}"
gnome-terminal --title="Terrain Analysis Extended" -- bash -c "
    source /home/tharushi/Go2-Dynamic-Inspection/workspaces/terrain_analyzer/install/setup.bash
    ros2 launch terrain_analysis_ext terrain_analysis_ext.launch; exec bash
"
sleep 2

# 6. Start Far Planner
echo -e "${GREEN}[6/6] Starting Far Planner...${NC}"
gnome-terminal --title="Far Planner" -- bash -c "
    source /home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner/install/setup.bash
    ros2 launch far_planner far_planner.launch.py; exec bash
"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo -e "${GREEN}All components launched in separate terminals!${NC}"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Check the terminals to verify each component is running."
echo ""
echo "To monitor the system:"
echo "  ros2 topic list"
echo "  ros2 topic echo /lidar_odometry/pose"
echo "  ros2 topic hz /terrain_map"
echo ""
echo "To stop all processes:"
echo "  pkill -f 'ros2|mola|rviz|terrain|far_planner|rosbag2'"
echo ""
