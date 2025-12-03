#!/bin/bash
# =============================================================================
# Development Environment Setup Script
# =============================================================================
# Copyright 2025 Quadruped-dyn-insp
# SPDX-License-Identifier: Apache-2.0
#
# Sets up the complete development environment including:
#   - System dependencies
#   - ROS 2 dependencies
#   - Workspace dependencies via rosdep
#
# Usage:
#   ./scripts/setup.sh              # Full setup
#   ./scripts/setup.sh --deps-only  # Only install dependencies
#   ./scripts/setup.sh --rosdep     # Only run rosdep
# =============================================================================

set -e  # Exit on any error

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Parse arguments
DEPS_ONLY=false
ROSDEP_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --deps-only)
            DEPS_ONLY=true
            shift
            ;;
        --rosdep)
            ROSDEP_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --deps-only  Only install system dependencies"
            echo "  --rosdep     Only run rosdep for workspace dependencies"
            echo "  --help, -h   Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# -----------------------------------------------------------------------------
# Functions
# -----------------------------------------------------------------------------
install_system_deps() {
    echo ""
    echo "Installing system dependencies..."
    sudo apt update
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        libpcl-dev \
        libgoogle-glog-dev \
        libgflags-dev
    echo "  ✓ System dependencies installed"
}

install_ros_deps() {
    echo ""
    echo "Installing ROS 2 dependencies..."
    sudo apt install -y \
        ros-humble-pcl-ros \
        ros-humble-pcl-conversions \
        ros-humble-vision-opencv \
        ros-humble-image-transport \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-foxglove-bridge
    echo "  ✓ ROS 2 dependencies installed"
}

run_rosdep() {
    echo ""
    echo "Running rosdep..."

    # Initialize rosdep if needed
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        echo "  Initializing rosdep..."
        sudo rosdep init || true
    fi

    rosdep update

    # Install workspace dependencies
    cd "$WORKSPACE_ROOT"
    for ws_dir in workspaces/*/; do
        if [[ -d "$ws_dir/src" ]]; then
            ws_name=$(basename "$ws_dir")
            echo "  Installing dependencies for $ws_name..."
            cd "$ws_dir"
            rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
            cd "$WORKSPACE_ROOT"
        fi
    done
    echo "  ✓ Workspace dependencies installed"
}

make_scripts_executable() {
    echo ""
    echo "Making scripts executable..."
    chmod +x "$WORKSPACE_ROOT/scripts/"*.sh
    echo "  ✓ Scripts ready"
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
echo "=== Development Environment Setup ==="
echo "Workspace root: $WORKSPACE_ROOT"

# Check ROS 2 installation
if ! command -v ros2 &> /dev/null; then
    echo ""
    echo "ERROR: ROS 2 not found!"
    echo "Please install ROS 2 Humble first:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi
echo "✓ ROS 2 found: $(ros2 --version 2>/dev/null | head -1)"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
echo "✓ ROS 2 environment sourced"

# Run requested operations
if [[ "$ROSDEP_ONLY" == true ]]; then
    run_rosdep
elif [[ "$DEPS_ONLY" == true ]]; then
    install_system_deps
    install_ros_deps
else
    install_system_deps
    install_ros_deps
    run_rosdep
    make_scripts_executable
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Setup Complete"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps:"
echo "  1. Build workspaces:  ./scripts/build.sh"
echo "  2. Launch pipeline:   ./scripts/launch.sh"
