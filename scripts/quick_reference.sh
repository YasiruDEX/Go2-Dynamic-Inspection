#!/bin/bash

# Quick Reference - Far Planner System Commands

cat << 'EOF'

╔═══════════════════════════════════════════════════════════════════╗
║            GO2 FAR PLANNER - QUICK REFERENCE                      ║
╚═══════════════════════════════════════════════════════════════════╝

┌─────────────────────────────────────────────────────────────────┐
│ LAUNCH OPTIONS                                                  │
└─────────────────────────────────────────────────────────────────┘

1. MANUAL LAUNCH (Separate Terminals) - RECOMMENDED FOR DEBUGGING
   ./scripts/manual_launch_fixed.sh

   ✓ Each component in its own terminal window
   ✓ See output from each component
   ✓ Automatic rosbag looping
   ✓ Proper startup sequence

2. AUTOMATED LAUNCH (All in One)
   ./scripts/start_system.sh

   ✓ Quick to start
   ✓ Managed startup sequence
   ✓ All output in one place

3. MANUAL COMMANDS (Type Everything Yourself)
   See: WHY_NO_FAR_PLANNER_OUTPUT.md
   
   ⚠ Must follow exact order!
   ⚠ Must use --loop flag for rosbag!
   ⚠ Must wait for each component!

┌─────────────────────────────────────────────────────────────────┐
│ DIAGNOSTIC COMMANDS                                             │
└─────────────────────────────────────────────────────────────────┘

Check System Status:
  ./scripts/diagnose_system.sh

Monitor Data Flow:
  ros2 topic hz /terrain_map
  ros2 topic hz /lidar_odometry/pose
  ros2 topic hz /livox/lidar

See Topic Data:
  ros2 topic echo /lidar_odometry/pose --once
  ros2 topic echo /terrain_map --once

List All Topics:
  ros2 topic list

Count Running Processes:
  ps aux | grep -E "ros2|mola|rviz|terrain|far_planner" | grep -v grep | wc -l

┌─────────────────────────────────────────────────────────────────┐
│ STOP COMMANDS                                                   │
└─────────────────────────────────────────────────────────────────┘

Stop Everything:
  pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2"

Stop Specific Components:
  pkill -f "far_planner"        # Stop Far Planner only
  pkill -f "mola"               # Stop MOLA only
  pkill -f "rosbag2"            # Stop rosbag only
  pkill -f "terrainAnalysis"    # Stop terrain analysis

┌─────────────────────────────────────────────────────────────────┐
│ TROUBLESHOOTING                                                 │
└─────────────────────────────────────────────────────────────────┘

Problem: No output from Far Planner
Solution: Check data pipeline with diagnose_system.sh
  → Make sure rosbag is using --loop flag
  → Verify MOLA is publishing (/lidar_odometry/pose)
  → Verify terrain maps are being published (/terrain_map)

Problem: Rosbag stops publishing
Solution: Restart with --loop flag
  cd ~/Go2-Dynamic-Inspection/rosbags
  ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop

Problem: MOLA not processing
Solution: Make sure TF publisher started BEFORE rosbag
  1. Start MOLA
  2. Start TF publisher
  3. Wait 2 seconds
  4. Start rosbag

Problem: Terrain Analysis crashes (exit code -6)
Solution: This is a known issue, restart it:
  cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
  source install/setup.bash
  ros2 launch terrain_analysis_ext terrain_analysis_ext.launch

┌─────────────────────────────────────────────────────────────────┐
│ MANUAL LAUNCH - CORRECT ORDER                                  │
└─────────────────────────────────────────────────────────────────┘

Terminal 1: MOLA
  export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
  cd ~/ros2_mola_ws && source install/setup.bash
  ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
    start_active:=True \
    mola_initial_map_mm_file:="$(pwd)/myMap.mm" \
    ... (see full command in WHY_NO_FAR_PLANNER_OUTPUT.md)

Terminal 2: TF Publisher (WAIT 8 sec after MOLA)
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame

Terminal 3: Rosbag (WAIT 2 sec after TF)
  cd ~/Go2-Dynamic-Inspection/rosbags
  ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop

Terminal 4: Terrain Analysis (WAIT 5 sec after rosbag)
  cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
  source install/setup.bash
  ros2 launch terrain_analysis terrain_analysis.launch

Terminal 5: Terrain Analysis Ext (WAIT 2 sec)
  cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
  source install/setup.bash
  ros2 launch terrain_analysis_ext terrain_analysis_ext.launch

Terminal 6: Far Planner (WAIT 2 sec)
  cd ~/Go2-Dynamic-Inspection/workspaces/far_planner
  source install/setup.bash
  ros2 launch far_planner far_planner.launch.py

┌─────────────────────────────────────────────────────────────────┐
│ EXPECTED OUTPUT - Far Planner Working                          │
└─────────────────────────────────────────────────────────────────┘

In Far Planner terminal you should see:
  ✓ "FAR Planner Initiated Complete"
  ✓ "V-Graph Initialized"
  ✓ "V-Graph Updated. Number of global vertices: XXX"
  ✓ "FARMaster: dynamic obstacle detected"
  ✓ "Global V-Graph Updated"
  ✓ Numbers increasing over time

In RViz windows (2 windows):
  ✓ MOLA RViz: Point clouds, localization
  ✓ Far Planner RViz: Terrain maps, graphs, paths

┌─────────────────────────────────────────────────────────────────┐
│ FILES & DOCUMENTATION                                           │
└─────────────────────────────────────────────────────────────────┘

Scripts:
  ./scripts/manual_launch_fixed.sh    ← Use this for manual launch!
  ./scripts/start_system.sh           ← Use this for quick automated launch
  ./scripts/diagnose_system.sh        ← Use this to check what's wrong

Documentation:
  ./WHY_NO_FAR_PLANNER_OUTPUT.md      ← Read this for detailed explanation
  ./README.md                         ← Project documentation

This File:
  ./scripts/quick_reference.sh        ← You're reading it now!

┌─────────────────────────────────────────────────────────────────┐
│ QUICK START (EASIEST WAY)                                      │
└─────────────────────────────────────────────────────────────────┘

cd ~/Go2-Dynamic-Inspection
./scripts/manual_launch_fixed.sh

Wait for all terminals to open, then check RViz windows!

═══════════════════════════════════════════════════════════════════

EOF
