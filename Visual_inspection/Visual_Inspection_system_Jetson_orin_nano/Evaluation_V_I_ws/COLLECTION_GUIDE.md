# Data Collection Guide

---

## Terminal Commands

### Start Pipeline (every session)

```bash
# T1
source /opt/ros/humble/setup.bash && source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node

# T2
source /opt/ros/humble/setup.bash && source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node

# T3
source /opt/ros/humble/setup.bash && source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server

# T4 — collection script
python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
```

### ROS Topic Monitor Commands

```bash
# Check all active topics
ros2 topic list

# Watch camera feed is running (should publish at 30Hz)
ros2 topic hz /visual_inspection/insta360/image_raw

# Watch Logitech camera
ros2 topic hz /visual_inspection/logitech/image_raw

# Watch YOLO detections
ros2 topic echo /visual_inspection/detections

# Watch current system status
ros2 topic echo /visual_inspection/status

# Watch IBVS error live
ros2 topic echo /visual_inspection/ibvs_error

# Check action server is ready
ros2 action list

# Open RViz to see camera feed
rviz2
# Then: Add → By Topic → /visual_inspection/debug → Image
```

### Check Collected Images

```bash
# Count all collected images
find ~/Documents/Visual_Inspection_ws/evaluation/ -name "*.jpg" | wc -l

# See folder breakdown
find ~/Documents/Visual_Inspection_ws/evaluation/ -name "*.jpg" | \
  sed 's|/[^/]*\.jpg||' | sort | uniq -c | sort -rn

# Check latest capture
ls -lt ~/Documents/Visual_Inspection_ws/captures/inspection/ | head -5

# View capture log
cat ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
```

### SCP Images to Laptop

```bash
# Run on LAPTOP after each session
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/

# Or just the log
scp rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/capture_log.csv
```

---

## Script Features

Press `s` in the main menu to see a **progress bar** showing how many images in each folder.

All sessions let you:
- Type **any angle** (0 to 180°, any direction)
- Type **any exact distance** you measured (e.g. 1.2m, 2.4m)
- Type **how many images** you want at that position
- Type **any occlusion %** you applied
- Decide when you're done — `q` goes back to menu, `y` continues

---

## What to Collect Per Session

### Session 1 — Reference
- Pick object → robot at 1m, 0° → press ENTER
- 5 images per object (fire_ext, gauge, door)

### Session 2 — Angle
- Pick ANY angle (0°, 15°, 30°, 45°, 60°, 90° etc.)
- Pick L or R direction (covers full 180°)
- Type actual measured distance (can vary ±20cm per position)
- 10 images per position
- Keep going: change angle → type new angle → more images

### Session 3 — Distance
- Type exact measured distance (0.5m, 1.0m, 1.5m, 2.0m, 2.5m, 3.0m, 4.0m)
- 10 images per distance
- Keep going: new distance → more images

### Session 4 — Gauge Ground Truth
- Type true reading → 3 images → type next reading → repeat
- Skip if no gauge equipment (q to exit)

### Session 5 — VLM Images
- Pick scenario (fire_ext pass/fail, exit pass/fail, door, cylinder)
- Type caption after each capture (1 sentence)
- 10 images per scenario

### Session 6 — Occlusion
- Type exact % you covered (0, 10, 25, 50, 75, 90...)
- 10 images per level

### Session 7 — Multi-object
- Pick scene type → 5 images per scene

---

## Checklist

```
[ ] Reference:      fire_ext (5)  gauge (5)  door (5)
[ ] Angle:          0° 15°L 15°R 30°L 30°R 45°L 45°R  (10 each)
[ ] Angle extra:    60°L 60°R 90°L 90°R  (10 each, if time)
[ ] Angle vertical: up down  (5-10 each)
[ ] Distance:       1m 1.5m 2m 2.5m 3m 3.5m 4m  (10 each)
[ ] Gauge GT:       5 reading positions × 3  (skip if no equipment)
[ ] VLM:            fire_ext pass (10)  fail (10)
[ ] VLM:            exit pass (10)  fail (10)
[ ] VLM:            door pass (10)
[ ] Occlusion:      0% 25% 50% 75% (10 each)
[ ] Multi-object:   2 same class, mixed class  (5 each)
```
