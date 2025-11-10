#!/bin/bash

# Pipeline Launch Script
# This script sources the required setup files and launches the complete pipeline

echo "=== Pipeline Launch Script ==="
echo "Sourcing required setup files..."

# Source the autonomous exploration workspace
echo "Sourcing autonomous_exploration workspace setup..."
source ~/Documents/Far_planner_test/workspaces/autonomous_exploration/install/setup.sh

# Source the far_planner workspace
echo "Sourcing far_planner workspace setup..."
source ~/Documents/Far_planner_test/workspaces/far_planner/install/setup.sh

# Source the fastlio2 workspace
echo "Sourcing fastlio2 workspace setup..."
source ~/Documents/Far_planner_test/workspaces/fastlio2/install/setup.sh

# Source the dlio workspace 
echo "Sourcing dlio workspace setup..."
source ~/Documents/Far_planner_test/workspaces/dlio/install/setup.sh

echo "Sourcing open3d slam workspace setup..."
source ~/Documents/Far_planner_test/workspaces/open3d_slam_ws/install/setup.sh


# Source the pipeline_launcher workspace
echo "Sourcing pipeline_launcher workspace setup..."
source ~/Documents/Far_planner_test/workspaces/pipeline_launcher/install/setup.sh

echo "All setup files sourced successfully!"
echo ""
echo "=== Launching Pipeline ==="
echo "Starting the complete pipeline with the following sequence:"
echo "  T=0s: fast_lio mapping starts"
echo "  T=3s: vehicle_simulator starts"  
echo "  T=6s: far_planner starts"
echo ""

# Launch the pipeline
# ros2 launch go2_config gazebo_velodyne.launch.py &
ros2 launch go2_config gazebo_velodyne.launch.py world:=/home/yasiru/simple_env_1.world
ros2 launch pipeline_launcher pipeline_gazebo.launch.py &


# --- Publish static transforms ---
sleep 2  # Small delay to ensure ROS nodes are up

echo "Publishing static transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 livox_frame sensor &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map_o3d map &


wait  # Wait for both background processes to finish
