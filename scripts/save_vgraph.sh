#!/bin/bash

# Save Visibility Graph Script
# This script publishes a message to save the current visibility graph

set -e

VGRAPH_DIR="/home/yasiru/Documents/Far_planner_test/saved_vgraphs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DEFAULT_FILE="$VGRAPH_DIR/vgraph_$TIMESTAMP.vgh"

# Get filename from argument or use default
FILENAME="${1:-$DEFAULT_FILE}"

# Ensure directory exists
mkdir -p "$VGRAPH_DIR"

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Save Far Planner Visibility Graph                       ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Saving visibility graph to: $FILENAME"
echo ""

# Check if Far Planner is running
if ! ros2 node list | grep -q "far_planner"; then
    echo "ERROR: Far Planner is not running!"
    echo "Please start Far Planner first with:"
    echo "  ./scripts/manual_launch_simple.sh"
    exit 1
fi

# Publish save command
ros2 topic pub --once /save_file_dir std_msgs/msg/String "{data: '$FILENAME'}"

echo ""
echo "✓ Save command sent!"
echo ""
echo "The visibility graph will be saved to:"
echo "  $FILENAME"
echo ""
echo "To load this graph on startup, update the config file:"
echo "  workspaces/far_planner/src/far_planner/config/default.yaml"
echo ""
echo "Set:"
echo "  vgraph_autoload: true"
echo "  vgraph_file_path: \"$FILENAME\""
echo ""
