#!/bin/bash

# Complete Auto-Load Test & Verification Script

echo "════════════════════════════════════════════════════════════"
echo "  Visibility Graph Auto-Load - Complete Test"
echo "════════════════════════════════════════════════════════════"
echo ""

# Step 1: Check prerequisites
echo "Step 1: Checking prerequisites..."
echo ""

GRAPH_FILE="/home/tharushi/Go2-Dynamic-Inspection/saved_vgraphs/vgraph.vgh"
if [ ! -f "$GRAPH_FILE" ]; then
    echo "❌ Graph file not found: $GRAPH_FILE"
    exit 1
fi
echo "✅ Graph file exists: $(ls -lh $GRAPH_FILE | awk '{print $5}')"

CONFIG_FILE="/home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner/src/far_planner/config/default.yaml"
AUTOLOAD=$(grep "vgraph_autoload:" "$CONFIG_FILE" | awk '{print $2}')
if [ "$AUTOLOAD" != "true" ]; then
    echo "❌ Auto-load is disabled in config"
    exit 1
fi
echo "✅ Auto-load enabled in config"

if [ ! -d "/home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner/install" ]; then
    echo "❌ Far Planner not built"
    exit 1
fi
echo "✅ Far Planner workspace built"

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Test Instructions"
echo "════════════════════════════════════════════════════════════"
echo ""

echo "1. STOP any running processes:"
echo "   pkill -f 'ros2|mola|rviz|terrain|far_planner|rosbag2'"
echo ""

echo "2. START the system:"
echo "   cd ~/Go2-Dynamic-Inspection"
echo "   ./scripts/simple_manual_launch.sh"
echo ""

echo "3. WATCH the Far Planner terminal for these messages (~5 seconds):"
echo "   ┌────────────────────────────────────────────────────────┐"
echo "   │ [INFO] Auto-load enabled. Will load visibility graph  │"
echo "   │ [INFO] Starting auto-load of visibility graph...      │"
echo "   │ [INFO] Loading XXXX nodes...                          │"
echo "   │ [INFO] Successfully loaded visibility graph           │"
echo "   │ [INFO] Updating visualization and contour extraction  │"
echo "   │ [INFO] Graph integration complete                     │"
echo "   └────────────────────────────────────────────────────────┘"
echo ""

echo "4. VERIFY in RViz (should appear within 8-10 seconds):"
echo "   ✅ White lines (visibility graph)"
echo "   ✅ Green/cyan polygons (contours/obstacles)"
echo "   ✅ Both should appear almost immediately"
echo ""

echo "5. VERIFY dynamic updates (as rosbag plays):"
echo "   ✅ Polygons should update/move"
echo "   ✅ New obstacles should appear"
echo "   ✅ Graph should grow if new areas explored"
echo ""

echo "════════════════════════════════════════════════════════════"
echo "  Expected Timeline"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "  0s      System starts"
echo "  1-4s    Components initialize"
echo "  ~5s     Auto-load triggers"
echo "  6-7s    Graph loading"
echo "  ~8s     ✨ WHITE GRAPH + POLYGONS APPEAR ✨"
echo "  8s+     Dynamic updates as rosbag plays"
echo ""

echo "════════════════════════════════════════════════════════════"
echo "  Troubleshooting"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "If graph doesn't load:"
echo "  • Check Far Planner terminal for error messages"
echo "  • Verify file path in config/default.yaml"
echo "  • Run: bash scripts/diagnose_autoload.sh"
echo ""
echo "If polygons don't appear:"
echo "  • Wait 10 seconds (polygon extraction may be slow)"
echo "  • Check RViz 'Polygons' display is enabled"
echo "  • Verify rosbag is playing (check topics)"
echo ""
echo "If updates don't work:"
echo "  • Check rosbag is in loop mode"
echo "  • Verify sensor topics are publishing"
echo "  • Run: ros2 topic hz /livox/lidar"
echo ""

echo "════════════════════════════════════════════════════════════"
echo ""
echo "Ready to test! Follow the instructions above."
echo ""
