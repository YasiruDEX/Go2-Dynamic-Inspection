#!/bin/bash
# =============================================================================
# Simulation Pipeline Launch Script
# =============================================================================
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
#
# Launches the complete autonomy pipeline in Gazebo simulation.
#
# Launch Sequence:
#   1. Gazebo simulation with robot (waits for initialization)
#   2. Pipeline launcher with simulation configuration
#   3. Static transforms for sensor frames
#
# Usage:
#   ./scripts/sim.sh                           # Default world
#   ./scripts/sim.sh --world /path/to/world    # Custom world file
#   ./scripts/sim.sh --no-gazebo               # Skip Gazebo (for external sim)
# =============================================================================

set -e  # Exit on any error

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Default values
WORLD_FILE="/home/yasiru/world.world"
GAZEBO_WAIT_TIME=15
SKIP_GAZEBO=false
LAUNCH_FILE="pipeline_simulation.launch.py"

# Process IDs for cleanup
GAZEBO_PID=""
PIPELINE_PID=""

# -----------------------------------------------------------------------------
# Argument Parsing
# -----------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --world|-w)
            WORLD_FILE="$2"
            shift 2
            ;;
        --no-gazebo)
            SKIP_GAZEBO=true
            shift
            ;;
        --wait-time)
            GAZEBO_WAIT_TIME="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --world, -w FILE    World file for Gazebo (default: $WORLD_FILE)"
            echo "  --no-gazebo         Skip Gazebo launch (for external simulation)"
            echo "  --wait-time SECS    Gazebo initialization wait time (default: 15)"
            echo "  --help, -h          Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# -----------------------------------------------------------------------------
# Cleanup Handler
# -----------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "Shutting down..."
    [[ -n "$PIPELINE_PID" ]] && kill $PIPELINE_PID 2>/dev/null || true
    [[ -n "$GAZEBO_PID" ]] && kill $GAZEBO_PID 2>/dev/null || true
    # Kill any remaining background jobs
    jobs -p | xargs -r kill 2>/dev/null || true
    echo "Cleanup complete."
    exit 0
}
trap cleanup SIGINT SIGTERM

# -----------------------------------------------------------------------------
# Source Workspaces
# -----------------------------------------------------------------------------
echo "=== Simulation Pipeline Launch ==="
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

# Source CUDA setup for DLIO if available
CUDA_SETUP="$WORKSPACE_ROOT/workspaces/dlio/setup_cuda12.sh"
if [[ -f "$CUDA_SETUP" ]]; then
    source "$CUDA_SETUP"
    echo "  ✓ CUDA environment"
fi

echo ""

# -----------------------------------------------------------------------------
# Launch Gazebo
# -----------------------------------------------------------------------------
if [[ "$SKIP_GAZEBO" == false ]]; then
    echo "=== Starting Gazebo Simulation ==="
    echo "World file: $WORLD_FILE"
    echo ""

    if [[ ! -f "$WORLD_FILE" ]]; then
        echo "WARNING: World file not found: $WORLD_FILE"
        echo "Launching with default world..."
        ros2 launch go2_config gazebo_velodyne.launch.py &
    else
        ros2 launch go2_config gazebo_velodyne.launch.py world:="$WORLD_FILE" &
    fi
    GAZEBO_PID=$!

    echo "Waiting for Gazebo initialization (${GAZEBO_WAIT_TIME}s)..."
    sleep $GAZEBO_WAIT_TIME

    # Verify Gazebo is running
    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        echo "ERROR: Gazebo failed to start!"
        exit 1
    fi
    echo "  ✓ Gazebo running"
fi

# -----------------------------------------------------------------------------
# Launch Pipeline
# -----------------------------------------------------------------------------
echo ""
echo "=== Starting Pipeline ==="
echo "Launch file: $LAUNCH_FILE"
echo ""
echo "Component startup sequence:"
echo "  T=0s:  Foxglove Bridge"
echo "  T=5s:  DLIO"
echo "  T=15s: Open3D SLAM"
echo "  T=20s: Vehicle Simulator"
echo "  T=25s: Far Planner"
echo ""

ros2 launch pipeline_launcher "$LAUNCH_FILE" &
PIPELINE_PID=$!

# -----------------------------------------------------------------------------
# Static Transforms
# -----------------------------------------------------------------------------
sleep 3

echo "Publishing static transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 livox_frame sensor &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map_o3d map &

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Simulation launched successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Press Ctrl+C to stop all nodes."
echo ""

# Wait for processes
wait
