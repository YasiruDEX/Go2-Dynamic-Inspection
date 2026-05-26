# Visual Inspection System — Demo & Run Commands
**Jetson Orin Nano | Ubuntu 22.04 | ROS2 Humble**

SSH into Jetson first (from laptop):
```bash
ssh rgen@192.168.8.181
# password: ffddffdd
```

---

## QUICK REFERENCE — What mode to use

| Mode | When to use | What you see |
|------|-------------|--------------|
| Headed Python | Demo to someone physically at the Jetson screen / VNC | Live camera window with bounding boxes |
| Headless Python | SSH terminal only, no display | FPS + IBVS error numbers in terminal |
| ROS2 + RViz | Full ROS integration demo | Topics + RViz2 debug view from laptop |
| Dataset collection | Evaluation data capture session | Pipeline auto-runs, images auto-logged |
| YOLO training data | Collect raw images for retraining | Both cameras preview side-by-side |

---

## 1. PYTHON PIPELINE — HEADED MODE (visual window via VNC)

**Use:** Connect to Jetson via VNC first, then open a terminal on the Jetson desktop.

```bash
# Activate venv
source ~/Documents/Visual_Inspection_ws/venv/bin/activate

# Run headed pipeline
cd ~/Documents/Visual_Inspection_ws/jetson_workspace
python3 ibvs_headed.py
```

**Optional shortcut (also frees cameras + sets max Jetson performance):**
```bash
cd ~/Documents/Visual_Inspection_ws/jetson_workspace
./run.sh
```

**What to watch:**
- Live side-by-side window: Insta360 (left) + Logitech (right)
- Green bounding boxes on detected objects (Insta360 view)
- Blue IBVS correction arrow + target crosshair (Logitech view)
- Status bar: `DETECTING → COARSE → IBVS → CAPTURING`
- IBVS pixel error displayed in real time (target < 10 px)

---

## 2. PYTHON PIPELINE — HEADLESS MODE (terminal only, SSH)

**Use:** When demonstrating over SSH without a screen.

```bash
# Activate venv
source ~/Documents/Visual_Inspection_ws/venv/bin/activate

# Run headless pipeline
cd ~/Documents/Visual_Inspection_ws/jetson_workspace
python3 ibvs_headless.py
```

**Optional shortcut:**
```bash
cd ~/Documents/Visual_Inspection_ws/jetson_workspace
./run.sh headless
```

**What to watch in terminal:**
```
[Insta360] fire_extinguisher ID=1  cx=312  cy=180  zone=FRONT
[COARSE]   pan=87  tilt=94  (polynomial mapping)
[IBVS]     obj=1   err=142px  →  84px  →  31px  →  9.3px  CONVERGED
[CAPTURE]  4 images saved  →  captures/inspection/20260317_163000/
[RESULT]   success=True | objects_found=1 | objects_inspected=1
```

---

## 3. ROS2 PIPELINE — 3 terminals + optional RViz

### Terminal 1 — Camera node

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node
```

**Expected output:**
```
[INFO] Camera node started
[INFO] Insta360  → /dev/video2  640x360 @ 30 Hz
[INFO] Logitech  → /dev/video0  640x480 @ 30 Hz
```

### Terminal 2 — Servo node

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node
```

**Expected output:**
```
[INFO] Servo node started — Arduino on /dev/ttyACM0
[INFO] Home position: pan=90  tilt=90
```

### Terminal 3 — IBVS action server

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server
```

**Expected output:**
```
[INFO] IBVS action server ready
[INFO] Waiting for goals...
```

### Terminal 4 — Send inspection goal (triggers the pipeline)

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py
```

**Expected output:**
```
[IBVS] obj=1  err=8.4px  .......................
RESULT: success=True | objects_found=1 | objects_inspected=1
```

---

## 4. MONITOR ROS TOPICS (while pipeline is running)

Open extra terminals alongside the 3 node terminals:

```bash
source /opt/ros/humble/setup.bash

# Live status (IDLE / DETECTING / COARSE / IBVS / CAPTURING)
ros2 topic echo /visual_inspection/status

# Live IBVS pixel error (x, y, magnitude) — watch this converge
ros2 topic echo /visual_inspection/ibvs_error

# Detection results (class, track_id, zone FRONT/BACK, bounding box)
ros2 topic echo /visual_inspection/detections

# Pan/tilt servo commands being sent to Arduino
ros2 topic echo /servo/pan_tilt

# List ALL active topics
ros2 topic list

# Check publishing rate (e.g. camera at 30Hz?)
ros2 topic hz /visual_inspection/insta360/image_raw
ros2 topic hz /visual_inspection/logitech/image_raw
```

---

## 5. RVIZ2 — Live camera debug view (from laptop)

RViz runs on your **laptop**, not Jetson. Set up ROS2 network between laptop and Jetson first:

**On Jetson** (in each terminal, before sourcing ROS):
```bash
export ROS_DOMAIN_ID=42
```

**On laptop:**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

# Launch RViz2
rviz2
```

**In RViz2 — Add these displays:**
1. Click `Add` → `By topic` → `/visual_inspection/debug` → `Image`
   - Shows: side-by-side Insta360 + Logitech with bounding boxes + IBVS arrow
2. Click `Add` → `By topic` → `/visual_inspection/insta360/image_raw` → `Image`
3. Click `Add` → `By topic` → `/visual_inspection/logitech/image_raw` → `Image`

**Save the RViz config:**
```bash
# Save once set up — loads it next time
rviz2 -d ~/Documents/Visual_Inspection_ws/inspection_ws/inspection.rviz
```

---

## 6. EVALUATION DATASET COLLECTION

**Runs on Jetson — requires ROS2 nodes running in T1, T2, T3 (see Section 3)**

```bash
# Terminal 4 — start collection script
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
source ~/Documents/Visual_Inspection_ws/venv/bin/activate
python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
```

**Menu options:**
```
  1: Reference images         ← clean baseline captures at 1m, head-on
  2: Angle evaluation         ← any angle (0°, 15°, 30°, 45°...)
  3: Distance evaluation      ← any distance (0.75m, 1m, 1.5m, 2m...)
  4: Gauge ground truth       ← known gauge reading for MAE/RMSE
  5: VLM PASS/FAIL images     ← blocked/unblocked scenarios
  6: Occlusion evaluation     ← 0%, 25%, 50%, 75% covered
  7: Multi-object scenes      ← 2-3 objects in same frame
  s: Show collection status   ← image counts per folder
  q: Quit
```

**What happens automatically for each capture:**
1. Pipeline runs (Insta360 → YOLO → coarse → IBVS → capture)
2. IBVS convergence time + final pixel error parsed from output
3. Image copied to the correct evaluation subfolder
4. Row logged to `capture_log.csv`

**Check collected data:**
```bash
ls ~/Documents/Visual_Inspection_ws/evaluation/
cat ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
```

**Image output location on Jetson:**
```
~/Documents/Visual_Inspection_ws/evaluation/
  reference/          ← reference images
  distance_eval/1m/   ← distance test images
  distance_eval/2m/   ← ...etc
  angle_eval/...
  occlusion/...
```

---

## 7. YOLO RETRAINING DATASET COLLECTION

**No ROS needed — runs standalone on Jetson Jetson desktop / VNC**

```bash
source ~/Documents/Visual_Inspection_ws/venv/bin/activate
cd ~/Documents/Visual_Inspection_ws
python3 collect_yolo_dataset.py
```

**Controls in the camera preview window:**

| Key | Action |
|-----|--------|
| `←` `→` | Pan servo left / right |
| `↑` `↓` | Tilt servo up / down |
| `SPACE` | Capture both cameras simultaneously |
| `+` / `-` | Step size 1°–15° per keypress |
| `h` | Home (pan=90°, tilt=90°) |
| `c` | Change class |
| `q` | Quit |

**Classes:** `1` = fire_extinguisher  `2` = door  `3` = gauge

**Image output on Jetson:**
```
~/Documents/Visual_Inspection_ws/yolo_dataset/
  fire_extinguisher/
    insta360/   img_0001.jpg  img_0002.jpg  ...
    logitech/   img_0001.jpg  img_0002.jpg  ...
  door/
    insta360/   ...
    logitech/   ...
  gauge/
    insta360/   ...
    logitech/   ...
```

---

## 8. SYNC DATA FROM JETSON TO LAPTOP

Run on **laptop** (not on Jetson):

```bash
bash /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/sync_from_jetson.sh
```

**What it pulls:**

| Jetson folder | Local destination |
|---------------|-------------------|
| `evaluation/` | `Evaluation_V_I_ws/eval_dataset/` |
| `captures/` | `data/captures_from_jetson/` |
| `yolo_dataset/` | `data/yolo_dataset/` |

**Or pull individually:**
```bash
# Evaluation dataset only
rsync -avz rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/

# YOLO training data only
rsync -avz rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/yolo_dataset/ \
    /home/dinethra/Jetson_orin_nano/data/yolo_dataset/
```

---

## 9. TROUBLESHOOTING

### Cameras not found / "camera in use"
```bash
# Find what process is holding the camera
fuser /dev/video0 /dev/video1 /dev/video2 /dev/video3

# Kill the blocking process(es)
kill -9 <PID>

# Or kill all known pipeline processes at once
pkill -9 -f "ibvs_headed\|ibvs_headless\|ibvs_pipeline\|camera_node\|collect_yolo"
```

### Arduino / servo not responding
```bash
# Check Arduino is connected
ls /dev/ttyACM0 /dev/arduino

# Grant access
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/arduino

# Test serial communication
python3 ~/Documents/Visual_Inspection_ws/test_scripts/04_test_arduino_serial.py
```

### ROS2 build after code changes
```bash
# Always build WITHOUT venv active
deactivate
cd ~/Documents/Visual_Inspection_ws/inspection_ws
colcon build --packages-select visual_inspection_ros
source install/setup.bash
```

### Check if ROS2 nodes are running
```bash
ros2 node list
# Expected:
# /camera_node
# /servo_node
# /ibvs_action_server
```

### Jetson performance mode (max speed)
```bash
sudo nvpmodel -m 0         # max power mode
sudo jetson_clocks         # lock clocks to max
