#!/bin/bash

# Diagnostic Script - Check System Status
# Use this to see what's working and what's not

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  System Diagnostic - Far Planner Setup                   ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check running processes
echo "=== Running ROS Processes ==="
ps_count=$(ps aux | grep -E "ros2|mola|rviz|terrain|far_planner" | grep -v grep | wc -l)
echo -e "Active ROS processes: ${GREEN}$ps_count${NC}"
ps aux | grep -E "ros2|mola|rviz|terrain|far_planner" | grep -v grep | awk '{print $11, $12, $13, $14, $15}' | head -10
echo ""

# Check critical topics
echo "=== Topic Status ==="

check_topic_exists() {
    if ros2 topic list 2>/dev/null | grep -q "^$1\$"; then
        echo -ne "${GREEN}✓${NC} $1 exists"
        # Check if it has data
        if timeout 2 ros2 topic hz $1 >/dev/null 2>&1; then
            echo -e " ${GREEN}[HAS DATA]${NC}"
        else
            echo -e " ${RED}[NO DATA]${NC}"
        fi
    else
        echo -e "${RED}✗${NC} $1 ${RED}MISSING${NC}"
    fi
}

check_topic_exists "/livox/lidar"
check_topic_exists "/livox/imu"
check_topic_exists "/lidar_odometry/pose"
check_topic_exists "/lidar_odometry/deskewed_scan_points"
check_topic_exists "/terrain_map"
check_topic_exists "/terrain_map_ext"
echo ""

# Check specific processes
echo "=== Component Status ==="

check_process() {
    if ps aux | grep -E "$1" | grep -v grep >/dev/null; then
        echo -e "${GREEN}✓${NC} $2 is running"
    else
        echo -e "${RED}✗${NC} $2 is NOT running"
    fi
}

check_process "mola-cli" "MOLA Localization"
check_process "static_transform_publisher.*base_link.*livox_frame" "TF Publisher"
check_process "ros2 bag play.*rosbag" "Rosbag Player"
check_process "terrainAnalysis" "Terrain Analysis"
check_process "terrainAnalysisExt" "Terrain Analysis Ext"
check_process "far_planner" "Far Planner"
check_process "rviz2" "RViz"
echo ""

# Data flow check
echo "=== Data Flow Analysis ==="

if timeout 2 ros2 topic hz /livox/lidar >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Rosbag is publishing sensor data"
else
    echo -e "${RED}✗${NC} Rosbag is NOT publishing (may have finished playing)"
    echo -e "  ${YELLOW}→ Restart rosbag or use --loop flag${NC}"
fi

if timeout 2 ros2 topic hz /lidar_odometry/pose >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} MOLA is processing and publishing localization"
else
    echo -e "${RED}✗${NC} MOLA is NOT publishing localization"
    echo -e "  ${YELLOW}→ Check if TF is available and rosbag is playing${NC}"
fi

if timeout 2 ros2 topic hz /terrain_map >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Terrain Analysis is publishing maps"
else
    echo -e "${RED}✗${NC} Terrain Analysis is NOT publishing"
    echo -e "  ${YELLOW}→ Check if MOLA point clouds are available${NC}"
fi

echo ""
echo "=== Common Issues & Solutions ==="
echo ""
echo "1. ${YELLOW}Rosbag not publishing:${NC}"
echo "   → The rosbag may have finished playing"
echo "   → Solution: Restart rosbag with --loop flag"
echo ""
echo "2. ${YELLOW}MOLA not publishing:${NC}"
echo "   → TF transform may not be available"
echo "   → Solution: Start TF publisher BEFORE starting rosbag"
echo ""
echo "3. ${YELLOW}Far Planner not showing output:${NC}"
echo "   → terrain_map must have data flowing"
echo "   → Solution: Ensure complete data pipeline is working:"
echo "      Rosbag → MOLA → Terrain Analysis → Far Planner"
echo ""
echo "4. ${YELLOW}No RViz visualization:${NC}"
echo "   → RViz may not be configured correctly"
echo "   → Solution: Use the launch files which load proper configs"
echo ""
echo "=== Quick Commands ==="
echo "Monitor topics:   ros2 topic hz /terrain_map"
echo "See topic data:   ros2 topic echo /lidar_odometry/pose --once"
echo "List all topics:  ros2 topic list"
echo "Stop everything:  pkill -f 'ros2|mola|rviz|terrain|far_planner|rosbag2'"
echo ""
