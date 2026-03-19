# Running the Far Planner System - Complete Guide

## Overview

This system integrates multiple components for autonomous navigation:
- **MOLA**: LiDAR-based localization and odometry
- **Terrain Analysis**: Point cloud processing and terrain mapping
- **Far Planner**: GPU-accelerated path planning with visibility graphs

## Quick Start

### Option 1: Automated Launch in Separate Terminals (RECOMMENDED)

```bash
cd ~/Go2-Dynamic-Inspection
./scripts/manual_launch_simple.sh
```

This script:
- ✅ Stops any existing ROS processes
- ✅ Launches each component in its own terminal window
- ✅ Starts components in the correct order
- ✅ Plays rosbag in infinite loop mode
- ✅ Opens 2 RViz windows for visualization

**What you'll see:**
- 6 new terminal windows will open (one for each component)
- 2 RViz windows will open automatically
- Each terminal shows the output of its component

### Option 2: Automated Launch in Single Terminal

```bash
cd ~/Go2-Dynamic-Inspection
./scripts/start_system.sh
```

- ✅ Quick to start
- ✅ All output in one terminal
- ⚠️ Less visibility into individual components

## System Architecture

### Data Flow

```
Rosbag (Sensor Data)
    ↓
    ├─→ /livox/lidar (LiDAR point clouds)
    └─→ /livox/imu (IMU data)
    ↓
MOLA Localization
    ↓
    ├─→ /lidar_odometry/pose (Robot position)
    ├─→ /lidar_odometry/deskewed_scan_points (Processed point clouds)
    └─→ /lidar_odometry/localmap_points (Local map)
    ↓
Terrain Analysis
    ↓
    ├─→ /terrain_map (Terrain elevation map)
    └─→ /terrain_map_ext (Extended terrain features)
    ↓
Far Planner
    ↓
    ├─→ /decoded_vgraph (Visibility graph)
    ├─→ /free_paths (Planned paths)
    └─→ RViz Visualization
```

### Component Startup Order

**CRITICAL: Components must start in this exact order!**

1. **MOLA Localization** (First)
   - Initializes the localization system
   - Loads the pre-built map
   - Waits for sensor data

2. **TF Publisher** (Second)
   - Publishes static transform: `base_link` → `livox_frame`
   - Required for MOLA to process LiDAR data

3. **Rosbag Player** (Third)
   - Plays recorded sensor data
   - **MUST use `--loop` flag** to continuously replay data
   - Without loop, data stops and entire pipeline breaks

4. **Terrain Analysis** (Fourth)
   - Processes point clouds from MOLA
   - Generates terrain elevation maps

5. **Terrain Analysis Extended** (Fifth)
   - Adds additional terrain features
   - Slope analysis, traversability

6. **Far Planner** (Sixth)
   - Receives terrain maps
   - Builds visibility graph
   - Plans collision-free paths
   - Opens RViz for visualization

## Components in Detail

### 1. MOLA (Modular Optimization framework for Localization and mApping)

**Location:** `~/ros2_mola_ws`

**Purpose:** Performs LiDAR-based localization and odometry

**Command:**
```bash
export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
cd ~/ros2_mola_ws
source install/setup.bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
    start_active:=True \
    publish_localization_following_rep105:=False \
    start_mapping_enabled:=False \
    lidar_topic_name:="/livox/lidar" \
    imu_topic_name:="/livox/imu" \
    mola_tf_base_link:="base_link" \
    mola_deskew_method:="MotionCompensationMethod::IMU" \
    mola_initial_map_mm_file:="$(pwd)/myMap.mm"
```

**Key Parameters:**
- `start_active:=True` - Start processing immediately
- `start_mapping_enabled:=False` - Use pre-built map (localization only)
- `mola_initial_map_mm_file` - Path to pre-built map file
- `mola_deskew_method` - Use IMU for motion compensation

**Published Topics:**
- `/lidar_odometry/pose` - Robot pose (10 Hz)
- `/lidar_odometry/deskewed_scan_points` - Motion-compensated point clouds
- `/lidar_odometry/localmap_points` - Local map around robot

**Expected Output:**
```
Initial re-localization done with pose: (x,y,z,yaw,pitch,roll)=...
```

### 2. TF Publisher

**Purpose:** Provides coordinate frame transformations

**Command:**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
```

**Transform:** `base_link` → `livox_frame` (identity transform)

**Why it's needed:** MOLA needs this transform to process LiDAR data correctly

### 3. Rosbag Player

**Location:** `~/Go2-Dynamic-Inspection/rosbags/rosbag2_2026_03_03-15_06_01/`

**Purpose:** Replays recorded sensor data

**Command:**
```bash
cd ~/Go2-Dynamic-Inspection/rosbags
ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop
```

**CRITICAL FLAGS:**
- `--loop` - Continuously replay the bag file (REQUIRED!)

**Published Topics:**
- `/livox/lidar` - LiDAR point clouds (~10 Hz)
- `/livox/imu` - IMU measurements (~200 Hz)

**Common Issue:**
❌ **Without `--loop` flag:** Rosbag finishes playing → No sensor data → MOLA stops → Terrain Analysis stops → Far Planner has no data!

### 4. Terrain Analysis

**Location:** `~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer`

**Purpose:** Processes point clouds into terrain elevation maps

**Command:**
```bash
cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
source install/setup.bash
ros2 launch terrain_analysis terrain_analysis.launch
```

**Subscribed Topics:**
- `/lidar_odometry/deskewed_scan_points`
- `/lidar_odometry/pose`

**Published Topics:**
- `/terrain_map` - Terrain elevation grid map

### 5. Terrain Analysis Extended

**Location:** `~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer`

**Purpose:** Adds slope and traversability analysis

**Command:**
```bash
cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
source install/setup.bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
```

**Published Topics:**
- `/terrain_map_ext` - Extended terrain features

**Known Issue:** May crash with exit code -6 on shutdown (harmless)

### 6. Far Planner

**Location:** `~/Go2-Dynamic-Inspection/workspaces/far_planner`

**Purpose:** GPU-accelerated visibility-graph-based path planning

**Command:**
```bash
cd ~/Go2-Dynamic-Inspection/workspaces/far_planner
source install/setup.bash
ros2 launch far_planner far_planner.launch.py
```

**Subscribed Topics:**
- `/terrain_map` - Terrain elevation map
- `/terrain_map_ext` - Extended terrain features
- `/goal_point` - Goal position (from RViz plugin)

**Published Topics:**
- `/decoded_vgraph` - Visibility graph for visualization
- `/free_paths` - Planned collision-free paths
- `/FAR_*_debug` - Various debug visualization topics

**Expected Output:**
```
[far_planner-1] FAR Planner Initiated Complete
[far_planner-1] V-Graph Initialized
[far_planner-1] V-Graph Updated. Number of global vertices: 281
[far_planner-1] FARMaster: dynamic obstacle detected, size: 222
[far_planner-1] Global V-Graph Updated. Number of global vertices: 450
[far_planner-1] Total V-Graph Update Time: 211.37ms
```

**RViz Visualization:**
- Terrain elevation maps (colorized by height)
- Visibility graph (white lines connecting vertices)
- Planned paths (colored trajectory)
- Obstacle detection (red areas)

## Diagnostic Tools

### Check System Status

```bash
cd ~/Go2-Dynamic-Inspection
./scripts/diagnose_system.sh
```

**Output shows:**
- ✅ Running processes
- ✅ Topic availability
- ✅ Data flow status
- ⚠️ Common issues and solutions

### Monitor Topics

**Check if topics are publishing:**
```bash
ros2 topic hz /terrain_map          # Should show ~1-2 Hz
ros2 topic hz /lidar_odometry/pose  # Should show ~10 Hz
ros2 topic hz /livox/lidar          # Should show ~10 Hz
```

**View topic data:**
```bash
ros2 topic echo /lidar_odometry/pose --once
ros2 topic echo /terrain_map --once
```

**List all topics:**
```bash
ros2 topic list
```

### Check Running Processes

```bash
ps aux | grep -E "mola|terrain|far_planner|rosbag" | grep -v grep
```

## Troubleshooting

### Problem: No output from Far Planner

**Symptoms:**
- Far Planner terminal shows initialization but no "V-Graph Updated" messages
- RViz shows empty scene

**Diagnosis:**
```bash
./scripts/diagnose_system.sh
```

**Common Causes:**

1. **Rosbag not using `--loop` flag**
   ```bash
   # Check if rosbag is publishing
   ros2 topic hz /livox/lidar
   
   # If timeout, restart rosbag:
   pkill -f rosbag
   cd ~/Go2-Dynamic-Inspection/rosbags
   ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop
   ```

2. **MOLA not processing**
   ```bash
   # Check MOLA output
   ros2 topic hz /lidar_odometry/pose
   
   # If timeout, check TF publisher started before rosbag
   # Restart in correct order: MOLA → TF → Rosbag
   ```

3. **Terrain Analysis not publishing**
   ```bash
   # Check terrain maps
   ros2 topic hz /terrain_map
   
   # If timeout, check terrain analysis process is running
   ps aux | grep terrainAnalysis
   ```

### Problem: Components started in wrong order

**Solution:** Stop everything and restart

```bash
# Stop all processes
pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2"

# Wait for cleanup
sleep 3

# Restart with automated script
./scripts/manual_launch_simple.sh
```

### Problem: Terrain Analysis Extended crashes (exit code -6)

**Symptoms:**
```
[ERROR] [terrainAnalysisExt-1]: process has died [pid XXX, exit code -6]
```

**Solution:** This is a known issue on shutdown. Restart it:

```bash
cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
source install/setup.bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
```

### Problem: RViz doesn't show anything

**Possible Causes:**

1. **Topics not publishing** - Check with `ros2 topic hz`
2. **RViz configuration not loaded** - The launch files should load configs automatically
3. **Coordinate frames not available** - Check TF publisher is running

**Solution:**
- Verify all components are running and publishing data
- Check RViz left panel: Displays should show topic names in green
- If topics are red, they're not available

### Problem: MOLA shows transform errors

**Symptoms:**
```
[WARN] Could not transform point cloud from livox_frame to base_link
```

**Solution:** TF publisher wasn't running when MOLA started

```bash
# Stop MOLA and rosbag
pkill -f "mola|rosbag"

# Restart TF publisher
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame &

# Wait 2 seconds, then restart MOLA and rosbag
```

## Configuration Files

### Map Files

**Location:** `~/ros2_mola_ws/myMap.mm`

**Alternative map:** `/home/tharushi/Downloads/ENTC_3rd_floor-20260310T074539Z-3-001/ENTC_3rd_floor/myMap.mm`

**Format:** MOLA proprietary format (.mm file)

**Size:** ~92 MB

### Rosbag Files

**Primary:** `~/Go2-Dynamic-Inspection/rosbags/rosbag2_2026_03_03-15_06_01/`
- Size: 2.8 GB
- Format: MCAP
- Duration: ~several minutes
- Topics: `/livox/lidar`, `/livox/imu`

**Alternative:** `~/Go2-Dynamic-Inspection/rosbags/rosbags-20260310T064339Z-3-002/rosbags/rosbag_003`

### RViz Configurations

**MOLA RViz:** Loaded automatically by `ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py`

**Far Planner RViz:** Loaded automatically by `ros2 launch far_planner far_planner.launch.py`

## Available Scripts

### Launch Scripts

| Script | Purpose | Use Case |
|--------|---------|----------|
| `./scripts/manual_launch_simple.sh` | Launch in separate terminals | **RECOMMENDED** - Best for debugging |
| `./scripts/start_system.sh` | Automated launch (single terminal) | Quick testing |
| `./scripts/manual_launch.sh` | Interactive manual launch | Custom configuration |

### Utility Scripts

| Script | Purpose |
|--------|---------|
| `./scripts/diagnose_system.sh` | System diagnostics |
| `./scripts/quick_reference.sh` | Command quick reference |
| `./scripts/build_all.sh` | Build all workspaces |
| `./scripts/source_workspaces.sh` | Source all workspaces |
| `./scripts/update.sh` | Update repository from remote |

### Pipeline Scripts

| Script | Purpose |
|--------|---------|
| `./scripts/run_localization_pipeline.sh` | Component launcher (used by other scripts) |
| `./scripts/pipeline.sh` | Legacy pipeline script |

## Stop Commands

### Stop Everything

```bash
pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2"
```

### Stop Specific Components

```bash
pkill -f "far_planner"        # Stop Far Planner only
pkill -f "mola"               # Stop MOLA only
pkill -f "rosbag2"            # Stop rosbag only
pkill -f "terrainAnalysis"    # Stop terrain analysis only
```

## Expected Behavior - System Working Correctly

### Terminal Outputs

**MOLA Terminal:**
```
[15:33:42.9008] Initial re-localization done with pose: (x,y,z,yaw,pitch,roll)=(0.0000,0.0000,0.0000,0.00deg,0.00deg,0.00deg)
```

**TF Publisher Terminal:**
```
[INFO] [static_transform_publisher]: Spinning until stopped - publishing transform
translation: ('0.000000', '0.000000', '0.000000')
rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
from 'base_link' to 'livox_frame'
```

**Rosbag Terminal:**
```
[INFO] [rosbag2_player]: Set rate to 1
[INFO] [rosbag2_player]: Adding keyboard callbacks.
```

**Terrain Analysis Terminal:**
```
Processing point clouds...
Publishing terrain maps...
```

**Far Planner Terminal:**
```
[far_planner-1] FAR Planner Initiated Complete
[far_planner-1] V-Graph Initialized
[far_planner-1] MH: Global Cloud Map Grid Initialized.
[far_planner-1] [INFO] DG: One trajectory node has been created.
[far_planner-1] V-Graph Updated. Number of global vertices: 281
[far_planner-1] [WARN] FARMaster: dynamic obstacle detected, size: 222
[far_planner-1] Global V-Graph Updated. Number of global vertices: 450
[far_planner-1] Total V-Graph Update Time: 211.37ms
```

### RViz Visualization

**MOLA RViz Window:**
- Point cloud display (white/colored points)
- Robot trajectory (path history)
- Current pose (coordinate axes)
- Local map (accumulated points)

**Far Planner RViz Window:**
- Terrain elevation map (color-coded by height)
- Visibility graph (white lines connecting vertices)
- Planned paths (colored trajectories)
- Obstacle regions (red areas)
- Free space (green/blue areas)

### Topic Statistics

```bash
$ ros2 topic hz /livox/lidar
average rate: 10.023
        min: 0.095s max: 0.105s std dev: 0.00234s window: 100

$ ros2 topic hz /lidar_odometry/pose
average rate: 10.015
        min: 0.098s max: 0.102s std dev: 0.00123s window: 100

$ ros2 topic hz /terrain_map
average rate: 1.523
        min: 0.620s max: 0.680s std dev: 0.01456s window: 10
```

## Performance Notes

### GPU Acceleration

Far Planner uses **GPU acceleration** for visibility graph computation.

**Requirements:**
- NVIDIA GPU with CUDA support
- CUDA toolkit installed
- Proper GPU drivers

**Performance:**
- V-Graph update: 200-500ms (typical)
- Path search: <1ms
- Total planning cycle: ~300ms

### CPU Usage

Typical CPU usage (8-core system):
- MOLA: 100-150% (1-1.5 cores)
- Terrain Analysis: 50-80% (0.5-0.8 cores)
- Far Planner: 80-120% (0.8-1.2 cores)
- Total: ~300-350% (3-3.5 cores)

### Memory Usage

Typical memory consumption:
- MOLA: ~500 MB (with loaded map)
- Terrain Analysis: ~200 MB
- Far Planner: ~400 MB
- Total: ~1.5 GB

## Advanced Usage

### Using a Different Map

Edit the launch script or run manually with different map path:

```bash
cd ~/ros2_mola_ws
source install/setup.bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
    start_active:=True \
    mola_initial_map_mm_file:="/path/to/your/custom_map.mm" \
    ...other parameters...
```

### Using a Different Rosbag

Edit the script or run manually:

```bash
cd ~/Go2-Dynamic-Inspection/rosbags
ros2 bag play your_custom_rosbag/ --loop
```

### Recording New Data

To create your own rosbag:

```bash
ros2 bag record /livox/lidar /livox/imu -o my_new_rosbag
```

### Building the Map

To create a new map with MOLA, enable mapping mode:

```bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
    start_active:=True \
    start_mapping_enabled:=True \
    ...other parameters...
```

Map will be saved when you stop MOLA.

## References

### Documentation Files

- `WHY_NO_FAR_PLANNER_OUTPUT.md` - Troubleshooting guide for Far Planner
- `scripts/quick_reference.sh` - Command quick reference
- `README.md` - Project overview
- `docs/setup/installation.md` - Installation instructions

### External Documentation

- [MOLA Documentation](https://docs.mola-slam.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Far Planner Paper](https://arxiv.org/abs/2103.09601)

## Quick Reference Card

```bash
# Launch system (separate terminals)
./scripts/manual_launch_simple.sh

# Check system status
./scripts/diagnose_system.sh

# Monitor data flow
ros2 topic hz /terrain_map
ros2 topic hz /lidar_odometry/pose

# Stop everything
pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2"

# Quick reference
./scripts/quick_reference.sh
```

---

**Last Updated:** March 10, 2026  
**System Version:** ROS2 Humble Hawksbill  
**MOLA Version:** Latest from source (~/ros2_mola_ws)  
**Far Planner:** GPU-accelerated version
