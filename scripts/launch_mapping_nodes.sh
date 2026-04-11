#!/bin/bash

# Define the setup commands to source the workspaces
SETUP_CMDS="cd /home/yasiru/Documents/Far_planner_test/workspaces/mola_lo && source install/setup.bash && cd /home/yasiru/Documents/Far_planner_test/workspaces/map_creator_ws && source install/setup.bash"

# Launch rosbag_recorder in a new gnome-terminal window
gnome-terminal --window --title="rosbag_recorder" -- bash -c "$SETUP_CMDS && ros2 run rosbag_recorder recorder_node; exec bash"

# Launch map_creator in a new gnome-terminal window
gnome-terminal --window --title="map_creator" -- bash -c "$SETUP_CMDS && ros2 run map_creator map_creator_node; exec bash"

echo "Launched recorder_node and map_creator_node in separate terminals."
