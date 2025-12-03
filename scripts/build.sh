#!/bin/bash
# =============================================================================
# Workspace Build Script
# =============================================================================
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
#
# Builds all ROS 2 workspaces in dependency order.
#
# Usage:
#   ./scripts/build.sh              # Build all workspaces
#   ./scripts/build.sh --clean      # Clean build (remove build/install/log)
#   ./scripts/build.sh --parallel   # Build with parallel jobs
#   ./scripts/build.sh <workspace>  # Build specific workspace only
# =============================================================================

set -e  # Exit on any error

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Build configuration
CLEAN_BUILD=false
PARALLEL_JOBS=""
SPECIFIC_WORKSPACE=""

# Workspaces in dependency order
ALL_WORKSPACES=(
    "autonomous_exploration"
    "dlio"
    "open3d_slam_ws"
    "far_planner"
    "pipeline_launcher"
)

# -----------------------------------------------------------------------------
# Argument Parsing
# -----------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean|-c)
            CLEAN_BUILD=true
            shift
            ;;
        --parallel|-p)
            PARALLEL_JOBS="--parallel-workers $(nproc)"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS] [WORKSPACE]"
            echo ""
            echo "Options:"
            echo "  --clean, -c     Clean build (removes build/install/log directories)"
            echo "  --parallel, -p  Enable parallel compilation"
            echo "  --help, -h      Show this help message"
            echo ""
            echo "Available workspaces:"
            for ws in "${ALL_WORKSPACES[@]}"; do
                echo "  - $ws"
            done
            exit 0
            ;;
        *)
            SPECIFIC_WORKSPACE="$1"
            shift
            ;;
    esac
done

# Determine which workspaces to build
if [[ -n "$SPECIFIC_WORKSPACE" ]]; then
    WORKSPACES=("$SPECIFIC_WORKSPACE")
else
    WORKSPACES=("${ALL_WORKSPACES[@]}")
fi

# -----------------------------------------------------------------------------
# Build Function
# -----------------------------------------------------------------------------
build_workspace() {
    local workspace="$1"
    local workspace_path="$WORKSPACE_ROOT/workspaces/$workspace"

    if [[ ! -d "$workspace_path" ]]; then
        echo "  ⚠ Workspace not found: $workspace_path"
        return 0
    fi

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Building: $workspace"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    cd "$workspace_path"

    # Clean build if requested
    if [[ "$CLEAN_BUILD" == true ]]; then
        echo "Cleaning build artifacts..."
        rm -rf build install log
    fi

    # Build with release configuration
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        $PARALLEL_JOBS

    echo "  ✓ $workspace built successfully"

    # Source the workspace
    if [[ -f "install/setup.bash" ]]; then
        source install/setup.bash
    fi
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
echo "=== Workspace Build ==="
echo "Workspace root: $WORKSPACE_ROOT"
echo "Clean build: $CLEAN_BUILD"
echo "Parallel: ${PARALLEL_JOBS:-disabled}"

# Source ROS 2 environment
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble environment sourced"
else
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Build each workspace
for workspace in "${WORKSPACES[@]}"; do
    build_workspace "$workspace"
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Build Complete"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps:"
echo "  Launch real robot:  ./scripts/launch.sh"
echo "  Launch simulation:  ./scripts/sim.sh"
