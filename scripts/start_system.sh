#!/bin/bash
# Proper startup sequence for Go2 Dynamic Inspection system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  Go2 Dynamic Inspection - Proper Startup      ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════╝${NC}"
echo ""

# Step 1: Start MOLA localization first
echo -e "${GREEN}[1/6] Starting MOLA localization...${NC}"
./scripts/run_localization_pipeline.sh localization-active &
MOLA_PID=$!
sleep 5

# Step 2: Start TF publisher
echo -e "${GREEN}[2/6] Starting TF publisher...${NC}"
./scripts/run_localization_pipeline.sh tf &
TF_PID=$!
sleep 2

# Step 3: Start rosbag after MOLA is ready
echo -e "${GREEN}[3/6] Starting rosbag playback...${NC}"
./scripts/run_localization_pipeline.sh bag &
BAG_PID=$!
sleep 3

# Check if rosbag is running
if ! kill -0 $BAG_PID 2>/dev/null; then
    echo -e "${YELLOW}Warning: Rosbag may not be running${NC}"
fi

# Step 4: Start terrain analysis (needed for Far Planner)
echo -e "${GREEN}[4/6] Starting terrain analysis...${NC}"
./scripts/run_localization_pipeline.sh terrain &
TERRAIN_PID=$!
sleep 2

echo -e "${GREEN}[5/6] Starting extended terrain analysis...${NC}"
./scripts/run_localization_pipeline.sh terrain-ext &
TERRAIN_EXT_PID=$!
sleep 2

# Step 5: Check topics before starting visualization
echo -e "${GREEN}[6/6] Checking data flow...${NC}"
echo ""
echo "Available topics:"
ros2 topic list | grep -E "livox|odometry|terrain|lidar" || echo "  Waiting for topics..."
echo ""

# Wait a bit for terrain maps to be generated
sleep 3

echo -e "${GREEN}Starting Far Planner visualization...${NC}"
./scripts/run_localization_pipeline.sh far &
FAR_PID=$!

echo ""
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}System is running!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════${NC}"
echo ""
echo "Running processes:"
echo "  • MOLA:          PID $MOLA_PID"
echo "  • TF Publisher:  PID $TF_PID"
echo "  • Rosbag:        PID $BAG_PID"
echo "  • Terrain:       PID $TERRAIN_PID"
echo "  • Terrain Ext:   PID $TERRAIN_EXT_PID"
echo "  • Far Planner:   PID $FAR_PID"
echo ""
echo "To monitor topics:"
echo "  ros2 topic list"
echo "  ros2 topic echo /lidar_odometry/pose"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping all processes...${NC}"
    kill $FAR_PID $TERRAIN_EXT_PID $TERRAIN_PID $BAG_PID $TF_PID $MOLA_PID 2>/dev/null
    wait 2>/dev/null
    echo -e "${GREEN}All processes stopped.${NC}"
}

trap cleanup SIGINT SIGTERM

# Wait for all processes
wait
