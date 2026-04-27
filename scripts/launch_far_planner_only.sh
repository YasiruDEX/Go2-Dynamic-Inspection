#!/usr/bin/env bash
# =============================================================================
# Far Planner Only Launch Script
# =============================================================================
# Launches only the Far Planner node/launch file (no mapping, no vehicle sim).
# Intended for workflows where upstream mapping/localization is already running.
#
# Usage:
#   ./scripts/launch_far_planner_only.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== Far Planner Only ==="

echo "Sourcing workspace environments..."

# Colcon-generated setup files may reference variables like COLCON_TRACE that
# are intentionally unset. Avoid crashing the launcher because of that.
_had_nounset=0
case "$-" in
  *u*) _had_nounset=1; set +u ;;
esac

# Best-effort: source the repo's install space if present.
if [[ -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE_ROOT/install/setup.bash"
fi

# Also source any stacked workspaces that the repo might have.
if [[ -f "$WORKSPACE_ROOT/workspaces/far_planner/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE_ROOT/workspaces/far_planner/install/setup.bash"
fi

if [[ "$_had_nounset" -eq 1 ]]; then
  set -u
fi

echo "Launching far planner..."

# Launch the far planner directly.
exec ros2 launch far_planner far_planner.launch.py
