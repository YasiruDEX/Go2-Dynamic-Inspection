#!/bin/bash

# Simple Manual Launch - Each Component in Separate Terminal
# Each terminal sources ROS2 explicitly before running

cd /home/tharushi/Go2-Dynamic-Inspection

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Launching Components in Separate Terminals              ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Stop any existing processes (but NOT ros2 daemon)
echo "Stopping existing processes..."
pkill -f "mola-cli|mola_launcher" 2>/dev/null
pkill -f "far_planner/far_planner" 2>/dev/null
pkill -f "terrainAnalysis" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "ros2 bag play" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "visualize_vgh_rviz" 2>/dev/null
sleep 3
echo "✓ Cleanup complete"
echo ""

# 1. Start MOLA
echo "[1/6] Launching MOLA..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  MOLA LiDAR Odometry"
    echo "════════════════════════════════════════"
    export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
    cd /home/tharushi/ros2_mola_ws
    source install/setup.bash
    echo "Starting MOLA with myMap.mm..."
    ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        start_active:=True \
        publish_localization_following_rep105:=False \
        start_mapping_enabled:=False \
        lidar_topic_name:="/livox/lidar" \
        imu_topic_name:="/livox/imu" \
        mola_tf_base_link:="base_link" \
        mola_deskew_method:="MotionCompensationMethod::IMU" \
        mola_initial_map_mm_file:="/home/tharushi/ros2_mola_ws/myMap.mm"
    exec bash
' &
sleep 12
echo "✓ MOLA launched"

# 2. Start TF Publisher  
echo "[2/6] Launching TF Publisher..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  TF Publisher: base_link → livox_frame"
    echo "════════════════════════════════════════"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
    exec bash
' &
sleep 3
echo "✓ TF Publisher launched"

# 3. Start Rosbag
echo "[3/6] Launching Rosbag (with --loop)..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  Rosbag Player - LOOPING MODE"
    echo "  rosbag2_2026_03_03-15_06_01"
    echo "════════════════════════════════════════"
    cd /home/tharushi/Go2-Dynamic-Inspection/rosbags
    echo "▶ Starting playback in loop mode..."
    ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop --rate 1.0
    exec bash
' &
sleep 5
echo "✓ Rosbag launched"

# 4. Start Terrain Analysis
echo "[4/6] Launching Terrain Analysis..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  Terrain Analysis"
    echo "════════════════════════════════════════"
    cd /home/tharushi/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
    source install/setup.bash
    ros2 launch terrain_analysis terrain_analysis.launch
    exec bash
' &
sleep 3
echo "✓ Terrain Analysis launched"

# 5. Start Terrain Analysis Extended
echo "[5/6] Launching Terrain Analysis Extended..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  Terrain Analysis Extended"
    echo "════════════════════════════════════════"
    cd /home/tharushi/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
    source install/setup.bash
    ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
    exec bash
' &
sleep 3
echo "✓ Terrain Analysis Ext launched"

# 6. Start Far Planner
echo "[6/7] Launching Far Planner..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  Far Planner + RViz"
    echo "════════════════════════════════════════"
    cd /home/tharushi/Go2-Dynamic-Inspection/workspaces/far_planner
    source install/setup.bash
    echo "Starting Far Planner..."
    ros2 launch far_planner far_planner.launch.py
    exec bash
' &
sleep 5
echo "✓ Far Planner launched"

# 7. Start VGH Visualizer (university2.vgh → RViz markers)
echo "[7/7] Launching VGH Graph Visualizer (university2.vgh)..."
gnome-terminal -- bash -c '
    source /opt/ros/humble/setup.bash
    echo "════════════════════════════════════════"
    echo "  VGH Graph Visualizer"
    echo "  university2.vgh → /vgh_graph/* topics"
    echo "════════════════════════════════════════"
    echo "Waiting 5s for Far Planner to initialise..."
    sleep 5
    echo "Publishing visibility graph markers..."
    python3 /home/tharushi/Go2-Dynamic-Inspection/scripts/visualize_vgh_rviz.py \
        /home/tharushi/Documents/university2.vgh
    exec bash
' &
sleep 2
echo "✓ VGH Visualizer launched"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "✓ ALL 7 TERMINALS OPENED!"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Look for 7 new terminal windows on your screen!"
echo ""
echo "Expected terminals:"
echo "  1. MOLA LiDAR Odometry (with RViz)"
echo "  2. TF Publisher"
echo "  3. Rosbag Player (looping)"
echo "  4. Terrain Analysis"
echo "  5. Terrain Analysis Extended"
echo "  6. Far Planner (with RViz)"
echo "  7. VGH Graph Visualizer (university2.vgh)"
echo ""
echo "VGH graph topics published on:"
echo "  /vgh_graph/nodes          - all nodes (colour-coded)"
echo "  /vgh_graph/nav_edges      - navigation edges (gray)"
echo "  /vgh_graph/poly_edges     - polygon edges (cyan)"
echo "  /vgh_graph/contour_edges  - contour edges (green)"
echo "  /vgh_graph/frontier_nodes - frontier nodes (red)"
echo "  /vgh_graph/navpoint_nodes - navpoint nodes (orange)"
echo ""
echo "In the Far Planner RViz, add MarkerArray displays for the"
echo "  /vgh_graph/* topics, or open the pre-built config with:"
echo "  rviz2 -d /home/tharushi/Go2-Dynamic-Inspection/scripts/university2_vgh.rviz"
echo ""
echo "You should also see 2 RViz windows open automatically:"
echo "  - MOLA RViz (localization visualization)"
echo "  - Far Planner RViz (planning visualization)"
echo ""
echo "To check if everything is working:"
echo "  ros2 topic hz /terrain_map"
echo "  ros2 topic hz /lidar_odometry/pose"
echo ""
echo "To stop all processes:"
echo "  pkill -f 'ros2|mola|rviz|terrain|far_planner|rosbag2|visualize_vgh'"
echo ""
