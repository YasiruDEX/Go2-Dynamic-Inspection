#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

source_if_exists() {
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        source "$setup_file"
        echo "  ✓ ${setup_file}"
    else
        echo "  ⚠ Missing: ${setup_file}"
    fi
}

echo "Sourcing ROS and workspaces..."
source_if_exists "/opt/ros/humble/setup.bash"

# Local project workspaces
source_if_exists "$WORKSPACE_ROOT/workspaces/terrain_analyzer/install/setup.bash"
source_if_exists "$WORKSPACE_ROOT/workspaces/far_planner/install/setup.bash"
source_if_exists "$WORKSPACE_ROOT/workspaces/local_planner/install/setup.bash"

# External MOLA workspace used in your commands
source_if_exists "$WORKSPACE_ROOT/workspaces/mola_lo/install/setup.bash"

echo "Workspace sourcing complete."
