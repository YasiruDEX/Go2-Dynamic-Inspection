#!/bin/bash

# Diagnostic script for Far Planner Auto-Load Issues

echo "════════════════════════════════════════════"
echo "  Far Planner Auto-Load Diagnostics"
echo "════════════════════════════════════════════"
echo ""

# Check if Far Planner node is running
echo "1. Checking if Far Planner node is running..."
FAR_NODE=$(ps aux | grep far_planner_node | grep -v grep)
if [ -z "$FAR_NODE" ]; then
    echo "   ❌ Far Planner node is NOT running!"
    echo ""
    echo "   Possible causes:"
    echo "   - Node crashed on startup"
    echo "   - Configuration error"
    echo "   - Missing dependencies"
    echo ""
else
    echo "   ✅ Far Planner node is running"
    echo "$FAR_NODE"
    echo ""
fi

# Check if RViz is running
echo "2. Checking if RViz is running..."
RVIZ=$(ps aux | grep rviz2 | grep far_planner | grep -v grep)
if [ -z "$RVIZ" ]; then
    echo "   ❌ RViz (Far Planner) is NOT running!"
else
    echo "   ✅ RViz is running"
fi
echo ""

# Check for core dumps or crashes
echo "3. Checking for recent crashes..."
if [ -f /var/crash/_opt_ros_humble_lib_far_planner_far_planner_node.*.crash ]; then
    echo "   ⚠️  Crash files found!"
    ls -lht /var/crash/*far_planner* 2>/dev/null | head -3
else
    echo "   ✅ No crash files found"
fi
echo ""

# Check graph file
echo "4. Checking visibility graph file..."
GRAPH_FILE="/home/tharushi/Go2-Dynamic-Inspection/saved_vgraphs/my_graph.vgh"
if [ -f "$GRAPH_FILE" ]; then
    echo "   ✅ Graph file exists: $(ls -lh "$GRAPH_FILE" | awk '{print $5}')"
else
    echo "   ❌ Graph file NOT found: $GRAPH_FILE"
fi
echo ""

# Check configuration
echo "5. Checking configuration..."
CONFIG="/home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner/src/far_planner/config/default.yaml"
if [ -f "$CONFIG" ]; then
    AUTOLOAD=$(grep "vgraph_autoload:" "$CONFIG" | awk '{print $2}')
    FILEPATH=$(grep "vgraph_file_path:" "$CONFIG" | awk '{print $2}' | tr -d '"')
    echo "   vgraph_autoload: $AUTOLOAD"
    echo "   vgraph_file_path: $FILEPATH"
else
    echo "   ❌ Config file not found!"
fi
echo ""

# Check ROS topics
echo "6. Checking ROS2 topics..."
if timeout 2 ros2 topic list &>/dev/null; then
    echo "   Available Far Planner topics:"
    ros2 topic list 2>/dev/null | grep -E "far_planner|graph" || echo "   ⚠️  No Far Planner topics found!"
else
    echo "   ⚠️  ROS2 daemon not responding or no topics available"
fi
echo ""

# Check logs
echo "7. Checking recent ROS2 logs..."
LOG_DIR="$HOME/.ros/log"
if [ -d "$LOG_DIR" ]; then
    LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -mmin -5 | grep far_planner | head -1)
    if [ -n "$LATEST_LOG" ]; then
        echo "   Latest log: $LATEST_LOG"
        echo "   Last 10 lines:"
        tail -10 "$LATEST_LOG" 2>/dev/null
    else
        echo "   ⚠️  No recent Far Planner logs found"
    fi
else
    echo "   ⚠️  ROS log directory not found"
fi
echo ""

echo "════════════════════════════════════════════"
echo "  Recommendations:"
echo "════════════════════════════════════════════"
echo ""

if [ -z "$FAR_NODE" ]; then
    echo "Far Planner node is not running. Check the Far Planner terminal for errors."
    echo ""
    echo "Common issues:"
    echo "  1. Workspace not sourced correctly"
    echo "  2. Graph file path is wrong in config"
    echo "  3. LoadVisibilityGraph() crashed during load"
    echo "  4. Missing transform frames (TF errors)"
    echo ""
    echo "Try running manually to see errors:"
    echo "  cd ~/Go2-Dynamic-Inspection/workspaces/far_planner"
    echo "  source install/setup.bash"
    echo "  ros2 run far_planner far_planner_node"
fi

echo ""
