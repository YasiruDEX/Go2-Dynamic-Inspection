#!/bin/bash
# =============================================================================
# Real Robot Pipeline Launch Script
# =============================================================================
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
#
# Launches the complete autonomy pipeline for real robot operation.
#
# Launch Sequence:
#   T=0s:  Foxglove Bridge (visualization)
#   T=5s:  DLIO (LiDAR-Inertial Odometry)
#   T=15s: Open3D SLAM (3D mapping)
#   T=20s: Vehicle Simulator (motion planning interface)
#   T=25s: Far Planner (global path planning)
#
# Usage:
#   ./scripts/launch.sh              # Launch with default settings
#   ./scripts/launch.sh --mapping    # Launch mapping-only pipeline
#   ./scripts/launch.sh --dev        # Launch development pipeline
# =============================================================================

set -e  # Exit on any error

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Default launch file
LAUNCH_FILE="pipeline_real.launch.py"

# Parse command line arguments
case "$1" in
    --mapping|-m)
        LAUNCH_FILE="pipeline_mapping.launch.py"
        ;;
    --dev|-d)
        LAUNCH_FILE="pipeline_dev.launch.py"
        ;;
    --sim|-s)
        echo "For simulation, use ./scripts/sim.sh instead"
        exit 1
        ;;
    --help|-h)
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "Options:"
        echo "  --mapping, -m   Launch mapping-only pipeline (DLIO + Open3D SLAM)"
        echo "  --dev, -d       Launch development pipeline (no Far Planner)"
        echo "  --help, -h      Show this help message"
        exit 0
        ;;
esac

# -----------------------------------------------------------------------------
# Source Workspaces
# -----------------------------------------------------------------------------
echo "=== Real Robot Pipeline Launch ==="
echo "Sourcing workspace environments..."

# Source workspaces in dependency order
WORKSPACES=(
    "autonomous_exploration"
    "far_planner"
    "dlio"
    "open3d_slam_ws"
    "pipeline_launcher"
)

for ws in "${WORKSPACES[@]}"; do
    ws_setup="$WORKSPACE_ROOT/workspaces/$ws/install/setup.bash"
    if [[ -f "$ws_setup" ]]; then
        source "$ws_setup"
        echo "  ✓ $ws"
    else
        echo "  ⚠ $ws (not found, skipping)"
    fi
done

echo ""

# -----------------------------------------------------------------------------
# Launch Pipeline
# -----------------------------------------------------------------------------
echo "=== Launching Pipeline ==="
echo "Launch file: $LAUNCH_FILE"
echo ""
echo "Component startup sequence:"
echo "  T=0s:  Foxglove Bridge"
echo "  T=5s:  DLIO"
echo "  T=15s: Open3D SLAM"
echo "  T=20s: Vehicle Simulator"
echo "  T=25s: Far Planner"
echo ""

# Launch the pipeline in background
ros2 launch pipeline_launcher "$LAUNCH_FILE" &
PIPELINE_PID=$!

# -----------------------------------------------------------------------------
# Static Transforms
# -----------------------------------------------------------------------------
sleep 2  # Wait for nodes to initialize

echo "Publishing static transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 livox_frame sensor &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map_o3d map &

echo ""
echo "Pipeline launched successfully. Press Ctrl+C to stop."

# Wait for pipeline to finish
wait $PIPELINE_PID
