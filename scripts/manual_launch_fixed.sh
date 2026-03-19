#!/bin/bash

# Manual Launch Script - FIXED VERSION
# Launches each component in a separate terminal with proper sequencing and error checking

set -e

cd /home/tharushi/Go2-Dynamic-Inspection

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Manual Launch - Each Component in Separate Terminal     ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Stop any existing processes
echo -e "${YELLOW}Stopping any existing ROS processes...${NC}"
pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2" 2>/dev/null || true
sleep 3
echo -e "${GREEN}✓ Cleanup complete${NC}"
echo ""

# Configuration
echo -e "${BLUE}Configuration:${NC}"
echo "  Map file: ~/ros2_mola_ws/myMap.mm"
echo "  Rosbag:   rosbag2_2026_03_03-15_06_01"
echo ""

# Function to wait for topic
wait_for_topic() {
    local topic=$1
    local timeout=$2
    local elapsed=0
    
    echo -ne "${YELLOW}Waiting for topic $topic...${NC}"
    while ! ros2 topic list 2>/dev/null | grep -q "^${topic}$"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ $elapsed -ge $timeout ]; then
            echo -e " ${RED}TIMEOUT${NC}"
            return 1
        fi
        echo -n "."
    done
    echo -e " ${GREEN}✓${NC}"
    return 0
}

# Function to wait for topic data
wait_for_topic_data() {
    local topic=$1
    local timeout=$2
    local elapsed=0
    
    echo -ne "${YELLOW}Waiting for data on $topic...${NC}"
    while ! timeout 1 ros2 topic hz $topic >/dev/null 2>&1; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ $elapsed -ge $timeout ]; then
            echo -e " ${RED}TIMEOUT${NC}"
            return 1
        fi
        echo -n "."
    done
    echo -e " ${GREEN}✓${NC}"
    return 0
}

# 1. Start MOLA
echo ""
echo -e "${GREEN}[1/6] Starting MOLA Localization...${NC}"
export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
gnome-terminal --title="MOLA Localization" --geometry=100x30+0+0 -- bash -c "
    echo '══════════════════════════════════════════════════════'
    echo '  MOLA LiDAR Odometry'
    echo '══════════════════════════════════════════════════════'
    cd /home/tharushi/ros2_mola_ws
    source install/setup.bash
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        start_active:=True \
        publish_localization_following_rep105:=False \
        start_mapping_enabled:=False \
        lidar_topic_name:=\"/livox/lidar\" \
        imu_topic_name:=\"/livox/imu\" \
        mola_tf_base_link:=\"base_link\" \
        mola_deskew_method:=\"MotionCompensationMethod::IMU\" \
        mola_initial_map_mm_file:=\"/home/tharushi/Downloads/ENTC_3rd_floor-20260310T074539Z-3-001/ENTC_3rd_floor/myMap.mm"
    exec bash
" &
sleep 8
echo -e "${GREEN}✓ MOLA terminal launched${NC}"

# 2. Start TF Publisher
echo ""
echo -e "${GREEN}[2/6] Starting TF Publisher...${NC}"
gnome-terminal --title="TF Publisher" --geometry=80x10+0+500 -- bash -c "
    echo '══════════════════════════════════════════════════════'
    echo '  Static TF Publisher: base_link → livox_frame'
    echo '══════════════════════════════════════════════════════'
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
    exec bash
" &
sleep 3
echo -e "${GREEN}✓ TF Publisher terminal launched${NC}"

# Wait for MOLA to be ready
wait_for_topic "/lidar_odometry/pose" 30

# 3. Start Rosbag
echo ""
echo -e "${GREEN}[3/6] Starting Rosbag Playback...${NC}"
gnome-terminal --title="Rosbag Player" --geometry=100x15+600+0 -- bash -c "
    echo '══════════════════════════════════════════════════════'
    echo '  Rosbag Player - rosbag2_2026_03_03-15_06_01'
    echo '  Playing in LOOP mode'
    echo '══════════════════════════════════════════════════════'
    cd /home/tharushi/Go2-Dynamic-Inspection/rosbags
    
    # Play the rosbag in loop mode
    while true; do
        echo ''
        echo '▶ Starting playback...'
        ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop --rate 1.0
        echo '⟳ Restarting playback...'
        sleep 1
    done
    exec bash
" &
sleep 5
echo -e "${GREEN}✓ Rosbag terminal launched${NC}"

# Wait for lidar data
wait_for_topic_data "/livox/lidar" 15

# 4. Start Terrain Analysis
echo ""
echo -e "${GREEN}[4/6] Starting Terrain Analysis...${NC}"
gnome-terminal --title="Terrain Analysis" --geometry=100x20+600+250 -- bash -c "
    echo '══════════════════════════════════════════════════════'
    echo '  Terrain Analysis'
    echo '══════════════════════════════════════════════════════'
    cd /home/tharushi/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
    source install/setup.bash
    ros2 launch terrain_analysis terrain_analysis.launch
    exec bash
" &
sleep 3
echo -e "${GREEN}✓ Terrain Analysis terminal launched${NC}"

# 5. Start Terrain Analysis Extended
echo ""
echo -e "${GREEN}[5/6] Starting Terrain Analysis Extended...${NC}"
gnome-terminal --title="Terrain Analysis Ext" --geometry=100x20+600+500 -- bash -c "
    echo '══════════════════════════════════════════════════════'
    echo '  Terrain Analysis Extended'
    echo '══════════════════════════════════════════════════════'
    cd /home/tharushi/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
    source install/setup.bash
    ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
    exec bash
" &
sleep 3
echo -e "${GREEN}✓ Terrain Analysis Ext terminal launched${NC}"

# Wait for terrain map
wait_for_topic "/terrain_map" 20
wait_for_topic_data "/terrain_map" 15

# 6. Start Far Planner
echo ""
echo -e "${GREEN}[6/6] Starting Far Planner with RViz...${NC}"
gnome-terminal --title="Far Planner + RViz" --geometry=120x30+1200+0 -- bash -c "
    echo '══════════════════════════════════════════════════════'
    echo '  Far Planner - GPU-Accelerated Path Planning'
    echo '══════════════════════════════════════════════════════'
    cd /home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner
    source install/setup.bash
    ros2 launch far_planner far_planner.launch.py
    exec bash
" &
sleep 5
echo -e "${GREEN}✓ Far Planner terminal launched${NC}"

# Final status
echo ""
echo "═══════════════════════════════════════════════════════════"
echo -e "${GREEN}✓ ALL COMPONENTS LAUNCHED!${NC}"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo -e "${BLUE}Monitor the system:${NC}"
echo "  ros2 topic list"
echo "  ros2 topic hz /terrain_map"
echo "  ros2 topic hz /lidar_odometry/pose"
echo "  ros2 topic echo /lidar_odometry/pose --once"
echo ""
echo -e "${BLUE}Check terminals for:${NC}"
echo "  1. MOLA: Should show 'Initial re-localization done'"
echo "  2. TF Publisher: Should show 'Spinning until stopped'"
echo "  3. Rosbag: Should show playback messages"
echo "  4. Terrain Analysis: Should process point clouds"
echo "  5. Terrain Ext: Should generate extended terrain maps"
echo "  6. Far Planner: Should show 'V-Graph Updated' messages"
echo ""
echo -e "${YELLOW}To stop all:${NC}"
echo "  pkill -f 'ros2|mola|rviz|terrain|far_planner|rosbag2'"
echo ""
echo -e "${GREEN}Look for RViz windows that opened automatically!${NC}"
echo ""
