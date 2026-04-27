#!/bin/bash

# Load Visibility Graph Script
# Usage:
#   ./scripts/load_vgraph.sh <vgraph_file.vgh>
#
# This script:
#  1) Detects FAR planner node name (/far_planner preferred)
#  2) Sets parameter vgraph_file_path on that node
#  3) Verifies it was set (prevents silently using default.yaml path)
#  4) Calls /load_visibility_graph (std_srvs/Trigger)
#     (falls back to topic /read_file_dir if service isn't available)

set -e

VGRAPH_DIR="/home/tharushi/Go2-Dynamic-Inspection/saved_vgraphs"

detect_far_node() {
    if ros2 node list 2>/dev/null | grep -qx "/far_planner"; then
        echo "/far_planner"
        return 0
    fi
    # Fallback for older naming
    if ros2 node list 2>/dev/null | grep -qx "/far_planner_node"; then
        echo "/far_planner_node"
        return 0
    fi
    return 1
}

if [ -z "${1:-}" ]; then
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

if [ ! -f "$FILENAME" ]; then
    if [ -f "$VGRAPH_DIR/$FILENAME" ]; then
        FILENAME="$VGRAPH_DIR/$FILENAME"
    else
        echo "ERROR: File not found: $FILENAME"
        exit 1
    fi
fi

FAR_NODE_NAME="$(detect_far_node)" || true
if [ -z "${FAR_NODE_NAME}" ]; then
    echo "ERROR: Far Planner is not running!"
    echo "Please start Far Planner first with:"
    echo "  ./scripts/launch_far_planner_only.sh"
    exit 1
fi

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Load Far Planner Visibility Graph                       ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Loading visibility graph from: $FILENAME"
echo "File size: $(du -h "$FILENAME" | cut -f1)"
echo "Target node: ${FAR_NODE_NAME}"
echo ""

echo "Setting ${FAR_NODE_NAME} parameter vgraph_file_path..."
ros2 param set "${FAR_NODE_NAME}" vgraph_file_path "${FILENAME}" >/dev/null

SET_VAL="$(ros2 param get "${FAR_NODE_NAME}" vgraph_file_path 2>/dev/null | sed -n 's/^String value is: //p')"
if [ "${SET_VAL}" != "${FILENAME}" ]; then
    echo "ERROR: vgraph_file_path did not take effect on ${FAR_NODE_NAME}."
    echo "Expected: ${FILENAME}"
    echo "Got:      ${SET_VAL}"
    echo "Aborting without calling /load_visibility_graph."
    exit 1
fi

echo "Calling /load_visibility_graph service..."
if ros2 service list 2>/dev/null | grep -qx "/load_visibility_graph"; then
    ros2 service call /load_visibility_graph std_srvs/srv/Trigger "{}"
else
    echo "WARNING: /load_visibility_graph service not found. Falling back to topic /read_file_dir."
    ros2 topic pub --once /read_file_dir std_msgs/msg/String "{data: '$FILENAME'}" >/dev/null
    echo "(Topic sent)"
fi

echo ""
echo "✓ Load command sent!"
echo "Check Far Planner terminal for status messages"
echo ""
