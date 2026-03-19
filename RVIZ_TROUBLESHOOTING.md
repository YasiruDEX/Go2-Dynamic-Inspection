# Far Planner RViz Troubleshooting Guide

## Why RViz Shows Nothing

The Far Planner RViz is empty because the **data pipeline isn't running**. Here's what needs to happen:

```
Rosbag → MOLA → Point Clouds → Terrain Analysis → Terrain Maps → Far Planner → RViz
```

## ✅ Correct Startup Sequence

### Option 1: Automated (Recommended)
```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/start_system.sh
```

### Option 2: Manual (4 Terminals)

**Terminal 1 - Rosbag:**
```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/run_localization_pipeline.sh bag
```

**Terminal 2 - MOLA Localization (wait 3 seconds after Terminal 1):**
```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/run_localization_pipeline.sh localization-active
```

**Terminal 3 - TF Publisher (wait 5 seconds after Terminal 2):**
```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/run_localization_pipeline.sh tf
```

**Terminal 4 - Far Planner (wait 3 seconds after Terminal 3):**
```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/run_localization_pipeline.sh far
```

## 🔍 Verify Data Flow

After starting, check if topics are publishing:

```bash
# List all topics
ros2 topic list

# Check if these exist:
# /livox/lidar              ← From rosbag
# /livox/imu                ← From rosbag  
# /lidar_odometry/pose      ← From MOLA
# /lidar_odometry/deskewed_scan_points ← From MOLA

# Monitor odometry
ros2 topic echo /lidar_odometry/pose --once

# Check point cloud rate
ros2 topic hz /livox/lidar
```

## ❌ Why "all" Command Fails

The `./scripts/run_localization_pipeline.sh all` command DOES NOT start the rosbag, so:
- ❌ No sensor data (LiDAR/IMU)
- ❌ MOLA has no data to process
- ❌ Terrain analysis crashes (no point clouds)
- ❌ Far planner has no terrain maps
- ❌ RViz shows nothing

## 🎯 What Should You See in RViz

When working correctly, Far Planner RViz should show:

1. **Point Cloud** - Deskewed LiDAR points from MOLA
2. **Terrain Map** - Traversability grid (green=safe, red=obstacles)
3. **Robot Pose** - Current position (from odometry)
4. **Planned Path** - Navigation path (if goal is set)
5. **TF Frames** - Coordinate frame transforms

## 🔧 Common Issues

### Issue: "Terrain analysis crashes immediately"
**Cause:** No point cloud data coming from MOLA  
**Solution:** Start rosbag first, then MOLA, wait for data to flow

### Issue: "MOLA shows no output"
**Cause:** No rosbag playing  
**Solution:** Start rosbag before MOLA

### Issue: "RViz is completely black/empty"
**Cause:** No topics being published  
**Solution:** Follow the correct startup sequence above

### Issue: "Topics exist but RViz still empty"
**Cause:** RViz visualization settings not configured  
**Solution:** 
- Click "Add" in RViz
- Add "PointCloud2" display
- Set topic to `/lidar_odometry/deskewed_scan_points`
- Add "Map" display for terrain maps
- Add "TF" display to see frames

## 📊 Expected Topic Rates

When system is working:
- `/livox/lidar`: ~10 Hz
- `/livox/imu`: ~200 Hz
- `/lidar_odometry/pose`: ~10 Hz
- `/terrain_map`: ~2-5 Hz

## 🚀 Quick Test

```bash
# Start everything
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/start_system.sh

# In another terminal, after 10 seconds:
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic echo /lidar_odometry/pose --once
```

If you see data on these topics, RViz should show the visualization.
