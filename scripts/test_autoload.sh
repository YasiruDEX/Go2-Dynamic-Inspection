#!/bin/bash

# Test Auto-Load Functionality
# This script helps you verify that the visibility graph auto-loads on startup

echo "============================================"
echo "Testing Far Planner Auto-Load Functionality"
echo "============================================"
echo ""

# Check if graph file exists
GRAPH_FILE="/home/tharushi/Go2-Dynamic-Inspection/saved_vgraphs/vgraph.vgh"

if [ ! -f "$GRAPH_FILE" ]; then
    echo "❌ ERROR: Graph file not found at: $GRAPH_FILE"
    echo ""
    echo "Please save a graph first:"
    echo "  ./scripts/save_vgraph.sh"
    exit 1
fi

echo "✅ Graph file found: $GRAPH_FILE"
ls -lh "$GRAPH_FILE"
echo ""

# Check configuration
CONFIG_FILE="/home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner/src/far_planner/config/default.yaml"

echo "Checking configuration in: $CONFIG_FILE"
echo ""

AUTOLOAD=$(grep "vgraph_autoload:" "$CONFIG_FILE" | awk '{print $2}')
FILE_PATH=$(grep "vgraph_file_path:" "$CONFIG_FILE" | awk '{print $2}' | tr -d '"')

echo "  vgraph_autoload: $AUTOLOAD"
echo "  vgraph_file_path: $FILE_PATH"
echo ""

if [ "$AUTOLOAD" != "true" ]; then
    echo "⚠️  WARNING: Auto-load is disabled!"
    echo "   Set 'vgraph_autoload: true' in config/default.yaml"
    echo ""
fi

if [ "$FILE_PATH" != "$GRAPH_FILE" ]; then
    echo "⚠️  WARNING: File path mismatch!"
    echo "   Config points to: $FILE_PATH"
    echo "   But graph is at: $GRAPH_FILE"
    echo ""
fi

# Check if workspace is sourced
if [ -f "/home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner/install/setup.bash" ]; then
    echo "✅ Far Planner workspace built"
else
    echo "❌ ERROR: Far Planner not built!"
    echo "   Run: cd workspaces/far_planner && colcon build"
    exit 1
fi

echo ""
echo "============================================"
echo "Test Instructions:"
echo "============================================"
echo ""
echo "1. Source the workspace:"
echo "   cd ~/Go2-Dynamic-Inspection/workspaces/far_planner"
echo "   source install/setup.bash"
echo ""
echo "2. Start Far Planner and watch for auto-load message:"
echo "   cd ~/Go2-Dynamic-Inspection"
echo "   ./scripts/manual_launch.sh"
echo ""
echo "3. Look for this message in Far Planner terminal:"
echo "   'Auto-loading visibility graph from: ...'"
echo "   'Successfully loaded visibility graph with N nodes'"
echo ""
echo "4. In RViz, you should immediately see:"
echo "   - White graph lines (visibility graph)"
echo "   - No waiting for graph to build"
echo ""
echo "============================================"
echo ""
