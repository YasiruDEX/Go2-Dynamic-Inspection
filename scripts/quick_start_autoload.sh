#!/bin/bash

# Quick Start - Far Planner with Auto-Load
# Run this after the fix to test auto-loading

echo "🚀 Starting Far Planner with Auto-Load..."
echo ""
echo "Prerequisites:"
echo "  ✅ Graph saved at: saved_vgraphs/my_graph.vgh (145 KB)"
echo "  ✅ Auto-load enabled in config"
echo "  ✅ Far Planner rebuilt with fix"
echo ""

# Source workspace
cd ~/Go2-Dynamic-Inspection/workspaces/far_planner
source install/setup.bash

echo "Watch for these messages in ~2 seconds:"
echo "  '[INFO] Auto-loading visibility graph from: ...'"
echo "  '[INFO] Successfully loaded visibility graph with XXXX nodes'"
echo ""
echo "In RViz, you should see the graph immediately (white lines)"
echo ""
echo "Starting system..."
echo ""

cd ~/Go2-Dynamic-Inspection
./scripts/simple_manual_launch.sh
