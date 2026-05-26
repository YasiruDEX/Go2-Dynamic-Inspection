#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

USE_RELEASE=false
PARALLEL_WORKERS=1

usage() {
    echo "Usage: $0 [--release] [--parallel-workers N]"
    echo ""
    echo "Default per-workspace command:"
    echo "  colcon build --symlink-install"
    echo ""
    echo "Options:"
    echo "  --release              Use Release build type"
    echo "  --parallel-workers N   Worker count in --release mode (default: 1)"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --release)
            USE_RELEASE=true
            shift
            ;;
        --parallel-workers)
            PARALLEL_WORKERS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

build_workspace() {
    local ws_path="$1"
    local ws_name
    ws_name="$(basename "$ws_path")"

    [[ -d "$ws_path/src" ]] || return 0

    if ! find "$ws_path/src" -name package.xml -type f | grep -q .; then
        return 0
    fi

    echo ""
    echo "=== Building: $ws_name ==="
    echo "Path: $ws_path"

    pushd "$ws_path" >/dev/null
    if [[ "$USE_RELEASE" == true ]]; then
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --parallel-workers "$PARALLEL_WORKERS"
    else
        colcon build --symlink-install
    fi
    popd >/dev/null
}

echo "Discovering colcon workspaces..."

# 1) All workspaces under ROOT/workspaces/*
if [[ -d "$ROOT_DIR/workspaces" ]]; then
    while IFS= read -r -d '' ws; do
        build_workspace "$ws"
    done < <(find "$ROOT_DIR/workspaces" -mindepth 1 -maxdepth 1 -type d -print0 | sort -z)
fi

# 2) Any top-level workspace folders with src/ + package.xml
while IFS= read -r -d '' top_ws; do
    # Skip scripts folder parent itself and avoid double-building workspaces/
    [[ "$top_ws" == "$ROOT_DIR/workspaces" ]] && continue
    build_workspace "$top_ws"
done < <(find "$ROOT_DIR" -mindepth 1 -maxdepth 1 -type d -print0 | sort -z)

echo ""
echo "All detected workspaces built."
