# Go2 Dynamic Inspection - Quick Start Guide

## ✅ Setup Complete!

All workspaces have been built successfully:
- terrain_analyzer
- far_planner
- local_planner
- go2_webrtc_bridge
- MOLA (ros2_mola_ws)

## 📖 How to Use

### 1. Play Rosbag Only
```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/run_localization_pipeline.sh bag
```

### 2. Run MOLA Localization (Odometry Mode)
```bash
./scripts/run_localization_pipeline.sh localization
```
**Note:** Currently runs in odometry-only mode. To use localization with a map, create a map first (see below).

### 3. Run Terrain Analysis
```bash
./scripts/run_localization_pipeline.sh terrain
```

### 4. Run Far Planner
```bash
./scripts/run_localization_pipeline.sh far
```

### 5. Run Everything Together
```bash
./scripts/run_localization_pipeline.sh all
```

## 🗺️ Creating a Map (Optional)

To use MOLA in localization mode, you first need to create a map:

```bash
cd /home/tharushi/Go2-Dynamic-Inspection
./scripts/run_mapping.sh mola-gui /path/to/your/rosbag
```

Available rosbags:
- rosbag2_2026_03_03-15_06_01
- rosbag2_2026_03_03-14_46_17  
- rosbag_003
- rosbag_01

Example:
```bash
./scripts/run_mapping.sh mola-gui rosbags/rosbags-20260310T064339Z-3-002/rosbags/rosbag2_2026_03_03-15_06_01
```

This will create `myMap.simplemap` which can be converted to `myMap.mm` for localization.

## 🔧 Troubleshooting

### Issue: Terrain Analysis or Local Planner crashes
**Cause:** Missing sensor data from rosbag or incorrect topic names.

**Solution:** 
1. Check rosbag topics: `ros2 bag info <rosbag_path>`
2. Verify topic names match in launch files
3. Ensure rosbag is playing before launching other nodes

### Issue: MOLA crashes immediately
**Cause:** Missing map file or incorrect parameters.

**Solution:**
- The script now automatically runs in odometry mode if no map exists
- Create a map first using `./scripts/run_mapping.sh`

### Issue: "Package not found" errors
**Solution:** Rebuild the workspace:
```bash
./scripts/build_all.sh --release --parallel-workers 4
```

## 📁 Available Rosbags

Located in: `/home/tharushi/Go2-Dynamic-Inspection/rosbags/rosbags-20260310T064339Z-3-002/rosbags/`

- `rosbag2_2026_03_03-15_06_01/`
- `rosbag2_2026_03_03-14_46_17/`
- `rosbag_003/`
- `rosbag_01/`

## 🚀 Next Steps

1. **Test rosbag playback:**
   ```bash
   ./scripts/run_localization_pipeline.sh bag
   ```

2. **Run MOLA odometry:**
   ```bash
   ./scripts/run_localization_pipeline.sh localization-active
   ```

3. **Create a map (if needed):**
   ```bash
   ./scripts/run_mapping.sh mola-gui rosbags/rosbags-20260310T064339Z-3-002/rosbags/rosbag2_2026_03_03-15_06_01
   ```

4. **Run the full pipeline:**
   ```bash
   ./scripts/run_localization_pipeline.sh all
   ```

## 📝 Notes

- The `pipeline_launcher` workspace is missing but optional
- MOLA runs in odometry-only mode until a map is created
- All packages are built in Release mode for optimal performance
- Use `--parallel-workers` flag for faster builds

## 🔄 Updating the Repository

When the remote repo is updated:
```bash
./scripts/update.sh
```

This script will:
- Stash local changes
- Pull latest updates
- Restore your changes
- Offer to rebuild workspaces
