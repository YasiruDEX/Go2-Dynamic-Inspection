#!/bin/bash

echo "Stopping all running ROS2 processes..."
pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2"
sleep 2

echo "Processes stopped. Now restart the system:"
echo ""
echo "  cd ~/Go2-Dynamic-Inspection"
echo "  ./scripts/simple_manual_launch.sh"
echo ""
echo "Watch for these messages (after ~5 seconds):"
echo "  '[INFO] Auto-load enabled. Will load visibility graph from: ...'"
echo "  '[INFO] Starting auto-load of visibility graph...'"
echo "  '[INFO] Successfully loaded visibility graph with XXXX nodes'"
echo ""
