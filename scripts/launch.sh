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
ros2 launch pipeline_launcher pipeline.launch.py & ros2 bag play ~/Documents/Far_planner_test/rosbags/rosbag_004

