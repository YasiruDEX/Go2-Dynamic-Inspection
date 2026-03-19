# Why You Can't See Far Planner Output - SOLUTION

## THE PROBLEM

When you run components in separate terminals manually, you're experiencing **NO OUTPUT** from Far Planner because the **data pipeline is broken**.

### Root Cause:
**The rosbag finishes playing and stops publishing sensor data!**

Without continuous sensor data:
```
❌ Rosbag stops → No /livox/lidar data
   ↓
❌ MOLA can't process → No /lidar_odometry/pose
   ↓
❌ No point clouds → Terrain Analysis has nothing to process
   ↓
❌ No /terrain_map → Far Planner has NOTHING to visualize!
```

## THE SOLUTION

### Option 1: Use the Fixed Manual Launch Script (RECOMMENDED)

```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/manual_launch_fixed.sh
```

**What this script does:**
- ✅ Launches each component in a **separate terminal window**
- ✅ Uses **proper startup sequence** (MOLA → TF → Rosbag → Terrain → Far Planner)
- ✅ Plays rosbag in **infinite loop mode** (never stops!)
- ✅ Waits for each component to be ready before starting the next
- ✅ Shows you exactly what's happening in each terminal

### Option 2: Manual Commands (If you insist on typing everything)

**CRITICAL: You MUST run commands in THIS EXACT ORDER:**

#### Terminal 1 - MOLA (Start FIRST!)
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

**WAIT** until you see: `Initial re-localization done`

#### Terminal 2 - TF Publisher (Start SECOND!)
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
```

**WAIT** 2-3 seconds for TF to be available

#### Terminal 3 - Rosbag (Start THIRD!) **MUST USE --loop!**
```bash
cd ~/Go2-Dynamic-Inspection/rosbags

# THIS IS CRITICAL - USE --loop FLAG!
ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop
```

**VERIFY** the rosbag is publishing:
```bash
# In another terminal:
ros2 topic hz /livox/lidar
# Should show ~10 Hz
```

#### Terminal 4 - Terrain Analysis
```bash
cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
source install/setup.bash
ros2 launch terrain_analysis terrain_analysis.launch
```

#### Terminal 5 - Terrain Analysis Extended
```bash
cd ~/Go2-Dynamic-Inspection/workspaces/terrain_analyzer
source install/setup.bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
```

#### Terminal 6 - Far Planner
```bash
cd ~/Go2-Dynamic-Inspection/workspaces/far_planner
source install/setup.bash
ros2 launch far_planner far_planner.launch.py
```

**NOW you should see:**
- Far Planner terminal showing: `V-Graph Updated. Number of global vertices: XXX`
- RViz window with terrain maps and planning visualization

## DIAGNOSTIC - Check if it's Working

```bash
cd ~/Go2-Dynamic-Inspection
./scripts/diagnose_system.sh
```

Look for:
- ✅ All topics should show `[HAS DATA]`
- ✅ "Rosbag is publishing sensor data"
- ✅ "MOLA is processing and publishing localization"
- ✅ "Terrain Analysis is publishing maps"

## COMMON MISTAKES

### ❌ Mistake #1: Rosbag without --loop flag
```bash
# WRONG:
ros2 bag play rosbag2_2026_03_03-15_06_01/

# RIGHT:
ros2 bag play rosbag2_2026_03_03-15_06_01/ --loop
```

### ❌ Mistake #2: Starting rosbag before TF publisher
```
WRONG ORDER:
1. MOLA
2. Rosbag ← TOO EARLY!
3. TF Publisher ← TOO LATE!

RIGHT ORDER:
1. MOLA
2. TF Publisher
3. Rosbag
```

### ❌ Mistake #3: Not waiting for components to initialize
Each component needs time to start up. Wait at least:
- MOLA: 8 seconds
- TF: 2 seconds
- After rosbag starts: 5 seconds before terrain analysis

## QUICK COMMANDS

### See Far Planner output in terminal:
Far Planner shows output in its own terminal window. Look for lines like:
```
[far_planner-1] V-Graph Updated. Number of global vertices: 1639
[far_planner-1] Total V-Graph Update Time: 211.37ms
[far_planner-1] FARMaster: dynamic obstacle detected, size: 222
```

### Monitor topics:
```bash
ros2 topic hz /terrain_map          # Should show ~1-2 Hz
ros2 topic hz /lidar_odometry/pose  # Should show ~10 Hz
ros2 topic echo /lidar_odometry/pose --once  # See actual data
```

### Stop everything:
```bash
pkill -f "ros2|mola|rviz|terrain|far_planner|rosbag2"
```

## FILES CREATED FOR YOU

1. **`./scripts/manual_launch_fixed.sh`**
   - Launches everything in separate terminals
   - Handles all timing and sequencing automatically
   - **USE THIS!**

2. **`./scripts/diagnose_system.sh`**
   - Shows what's working and what's not
   - Identifies data flow problems
   - Run this if something isn't working

3. **`./scripts/start_system.sh`**
   - Automated launch (all in one terminal)
   - Good for quick testing
   - Less visibility into individual components

## WHY THE AUTOMATED SCRIPT WORKED

The automated `start_system.sh` worked because:
1. ✅ It started components in the correct order
2. ✅ It used `--loop` flag for rosbag
3. ✅ It waited for topics to be ready
4. ✅ MOLA was already running when rosbag started publishing

When you run manually, if you forget any of these, the pipeline breaks!

## EXPECTED BEHAVIOR

When working correctly, you should see:

**MOLA Terminal:**
```
Initial re-localization done with pose: (x,y,z,yaw,pitch,roll)=...
```

**Rosbag Terminal:**
```
[INFO] [rosbag2_player]: Set rate to 1
```

**Terrain Analysis Terminal:**
```
Processing point clouds...
```

**Far Planner Terminal:**
```
[far_planner-1] FAR Planner Initiated Complete
[far_planner-1] V-Graph Updated. Number of global vertices: 281
[far_planner-1] FARMaster: dynamic obstacle detected, size: 222
[far_planner-1] Global V-Graph Updated. Number of global vertices: 450
```

**RViz Windows (2 windows should open):**
1. MOLA RViz - Shows localization and mapping
2. Far Planner RViz - Shows terrain maps, visibility graphs, paths

---

**Bottom Line:** Use `./scripts/manual_launch_fixed.sh` - it does everything correctly!
