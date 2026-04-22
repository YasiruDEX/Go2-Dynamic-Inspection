#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

source "$SCRIPT_DIR/source_workspaces.sh"

usage() {
    echo "Usage: $0 [localization|localization-active|tf|bag|terrain|terrain-ext|far|local|foxglove|all|all-active]"
    echo ""
    echo "Commands:"
    echo "  localization             Run MOLA localization (start_active:=False)"
    echo "  localization-active      Run MOLA localization (start_active:=True)"
    echo "  tf                       Publish base_link -> livox_frame static transform"
    echo "  bag [bag_dir]            Play rosbag (default: rosbags/rosbag2_2026_03_03-15_06_01)"
    echo "  terrain                  Launch terrain_analysis"
    echo "  terrain-ext              Launch terrain_analysis_ext"
    echo "  far                      Launch far_planner"
    echo "  local                    Launch local_planner"
    echo "  foxglove                 Start Foxglove bridge node"
    echo "  dlio                     Launch DLIO odometry and mapping (start_dlio:=false)"
    echo "  all                      Launch localization + terrain + terrain-ext + far + local + tf + foxglove"
    echo "  all-active               Same as 'all' but start localization active and play default bag"
}

run_localization() {
    local active="$1"
    export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
    cd "$HOME/ros2_mola_ws"
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        start_active:="$active" \
        publish_localization_following_rep105:=False \
        start_mapping_enabled:=False \
        lidar_topic_name:="/livox/lidar" \
        imu_topic_name:="/livox/imu" \
        mola_tf_base_link:="base_link" \
        mola_deskew_method:="MotionCompensationMethod::IMU" \
        mola_initial_map_mm_file:="/home/yasiru/Documents/Far_planner_test/maps/Inspection/myMap.mm"
}

# run_localization() {
#     local active="$1"
#     export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
#     cd "$HOME/ros2_mola_ws"
#     ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
#         start_active:="$active" \
#         publish_localization_following_rep105:=False \
#         start_mapping_enabled:=False \
#         lidar_topic_name:="/points_raw_decoded" \
#         imu_topic_name:="/livox/imu" \
#         mola_tf_base_link:="base_link" \
#         mola_deskew_method:="MotionCompensationMethod::IMU" \
#         mola_initial_map_mm_file:="/home/yasiru/Documents/Far_planner_test/maps/Inspection/myMap.mm"
# }

run_tf() {
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
}

run_bag() {
    local bag_dir="${1:-/home/yasiru/Documents/rosbag2_2026_03_26-18_18_08}"
    ros2 bag play "$bag_dir" --loop
}

run_terrain() {
    ros2 launch terrain_analysis terrain_analysis.launch
}

run_terrain_ext() {
    ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
}

run_far() {
    ros2 launch far_planner far_planner.launch.py \
        vgraph_autoload:=false \
        vgraph_file_path:='/home/yasiru/Documents/Far_planner_test/saved_vgraphs/vgrap.vgh'
}

run_local() {
    ros2 launch local_planner local_planner.launch
}

run_foxglove() {
    # start the Foxglove bridge; the package provides a node named
    # "foxglove_bridge" which opens a websocket on port 8765 by default.
    ros2 run foxglove_bridge foxglove_bridge
}

run_rviz() {
    cd "$WORKSPACE_ROOT/workspaces/far_planner"
    source install/setup.bash
    rviz2 -d /home/yasiru/Downloads/far_planner.rviz
}

run_dlio() {
    cd "$WORKSPACE_ROOT/workspaces/dlio"
    source install/setup.bash
    ros2 launch direct_lidar_inertial_odometry dlio_mapping.launch.py start_dlio:=false
}

cleanup() {
    echo ""
    echo "Stopping launched processes..."
    jobs -p | xargs -r kill
    pkill -f foxglove_bridge 2>/dev/null || true
}

cmd="${1:-all}"
case "$cmd" in
    localization)
        run_localization "False"
        ;;
    localization-active)
        run_localization "True"
        ;;
    dlio)
        run_dlio
        ;;
    foxglove)
        run_foxglove
        ;;
    bag)
        run_bag "$2"
        ;;
    terrain)
        run_terrain &
        run_terrain_ext
        ;;
    terrain-ext)
        run_terrain_ext
        ;;
    far)
        run_far
        ;;
    local)
        run_local
        ;;
    all)
        trap cleanup SIGINT SIGTERM
        run_localization "True" &
        sleep 2
        run_tf &
        sleep 1
        run_terrain &
        run_terrain_ext &
        run_far &
        run_local &
        run_foxglove &
        wait
        ;;
    all-bag)
        trap cleanup SIGINT SIGTERM
        run_localization "True" &
        sleep 2
        run_tf &
        sleep 1
        run_terrain &
        run_terrain_ext &
        run_far &
        run_foxglove &
        run_local &
        run_bag &
        wait
        ;;
     all-real)
        trap cleanup SIGINT SIGTERM
        run_localization "False" &
        sleep 2
        run_tf &
        sleep 1
        run_far &
        run_rviz &
        wait
        ;;
    all-no-far)
        trap cleanup SIGINT SIGTERM
        run_localization "False" &
        sleep 2
        run_tf &
        sleep 1
        run_rviz &
        wait
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        echo "Unknown command: $cmd"
        usage
        exit 1
        ;;
esac
