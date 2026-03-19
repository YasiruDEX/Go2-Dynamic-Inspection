#!/bin/bash

# Load Visibility Graph Script
# This script publishes a message to load a saved visibility graph

set -e

VGRAPH_DIR="/home/tharushi/Go2-Dynamic-Inspection/saved_vgraphs"

# Get filename from argument
if [ -z "$1" ]; then
    echo "Usage: $0 <vgraph_file.vgh>"
    echo ""
    echo "Available visibility graphs:"
    if [ -d "$VGRAPH_DIR" ] && [ "$(ls -A $VGRAPH_DIR/*.vgh 2>/dev/null)" ]; then
        ls -lh "$VGRAPH_DIR"/*.vgh
    else
        echo "  No saved graphs found in $VGRAPH_DIR"
    fi
    exit 1
fi

FILENAME="$1"

# Check if file exists
if [ ! -f "$FILENAME" ]; then
    # Try with directory prefix
    if [ -f "$VGRAPH_DIR/$FILENAME" ]; then
        FILENAME="$VGRAPH_DIR/$FILENAME"
    else
        echo "ERROR: File not found: $FILENAME"
        exit 1
    fi
fi

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Load Far Planner Visibility Graph                       ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Loading visibility graph from: $FILENAME"
echo "File size: $(du -h "$FILENAME" | cut -f1)"
echo ""

# Check if Far Planner is running
if ! ros2 node list | grep -q "far_planner"; then
    echo "ERROR: Far Planner is not running!"
    echo "Please start Far Planner first with:"
    echo "  ./scripts/manual_launch_simple.sh"
    exit 1
fi

# Publish load command
ros2 topic pub --once /read_file_dir std_msgs/msg/String "{data: '$FILENAME'}"

echo ""
echo "✓ Load command sent!"
echo ""
echo "The visibility graph is being loaded..."
echo "Check Far Planner terminal for status messages"
echo ""
