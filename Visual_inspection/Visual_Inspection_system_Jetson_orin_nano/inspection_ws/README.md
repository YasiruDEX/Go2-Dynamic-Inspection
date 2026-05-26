# Visual Inspection System — ROS2 Package
**Stack:** ROS2 Humble · Python 3.10 · YOLO11 TensorRT · ByteTrack · paho-mqtt · py_trees  
**Hardware:** Jetson Orin Nano · Insta360 camera · Logitech camera · Arduino (pan-tilt servos)

---

## Quick Start (Jetson — 4 terminals)

> Run every command on **Jetson (rgen@ubuntu)** unless marked LAPTOP.

```bash
# ── Paste this source block at the top of EVERY new terminal ──────────────
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
```

### Terminal 1 — Camera publisher
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node
```
✅ Ready when you see: `Camera publisher running at 30 Hz`

### Terminal 2 — Servo controller
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node
```
✅ Ready when you see: `Servo node ready`

### Terminal 3 — IBVS Action Server (main pipeline)
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server
```
✅ Ready when you see:
```
Action server ready at /visual_inspection/inspect_objects
MQTT broker: demo.thingsboard.io:1883
```

### Terminal 4 — Test (replaces Behaviour Tree until BT is connected)
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py
```

### Terminal 5 — RViz2 (live debug view)
```bash
source /opt/ros/humble/setup.bash
rviz2
# In RViz2: Add → By Topic → /visual_inspection/debug → Image → OK
```

---

## Package Structure

```
inspection_ws/
├── README.md                            ← this file
├── PLAN.md                              ← original development plan
│
├── visual_inspection_interfaces/        ← ROS2 custom message/action package
│   └── action/
│       └── InspectObjects.action        ← action definition (see below)
│
└── visual_inspection_ros/               ← ROS2 node package
    ├── package.xml
    ├── setup.py
    └── visual_inspection_ros/
        ├── camera_node.py               ← Node 1: publish both cameras
        ├── servo_node.py                ← Node 2: Arduino servo control
        ├── ibvs_action_server.py        ← Node 3: full inspection pipeline
        └── bt_nodes/
            ├── __init__.py
            └── inspection_bt_nodes.py  ← BT leaf nodes (for Ravith)
```

---

## Action Interface — InspectObjects.action

```
# ── GOAL (what you send to start inspection) ─────────────────────────────────
int32  max_objects       # 0 = inspect all detected front objects, N = first N only
bool   return_home       # true = servos return to 90,90 after done
string location_label    # label from BT e.g. "gauge_room_A" / "unknown"
bool   overview_only     # true = just capture Insta360 snapshots (no IBVS)
int32  overview_count    # how many Insta360 images (used when overview_only=true)

# ── RESULT (what you get back) ────────────────────────────────────────────────
bool   success
int32  objects_inspected  # how many objects were fully centered + captured
int32  objects_found      # total detected on Insta360 (front + back)
bool   object_in_back     # true → BT must rotate robot 180° and retry
string failed_reason      # "": success | "no_detection" | "ibvs_timeout" |
                          # "logi_no_detection" | "all_in_back"

# ── FEEDBACK (streamed while running) ────────────────────────────────────────
string  current_step      # "detecting" / "coarse" / "ibvs" / "capturing" / "overview"
int32   current_object    # which object is being processed
float32 ibvs_error_px     # live pixel error during IBVS centering
```

---

## All ROS2 Topics

### Published by us
| Topic | Type | Description |
|-------|------|-------------|
| `/visual_inspection/insta360/image_raw` | `sensor_msgs/Image` | Insta360 raw feed (360°) |
| `/visual_inspection/logitech/image_raw` | `sensor_msgs/Image` | Logitech raw feed (close-up) |
| `/visual_inspection/debug` | `sensor_msgs/Image` | **Combined debug view** — side-by-side Insta360 + Logitech with bounding boxes, class names, IBVS arrow, mode, FPS |
| `/visual_inspection/status` | `std_msgs/String` | Current mode: `IDLE` / `DETECTING` / `COARSE` / `IBVS` / `CAPTURING` / `OVERVIEW` |
| `/visual_inspection/ibvs_error` | `geometry_msgs/Point` | IBVS pixel error: x=ex, y=ey, z=total magnitude |
| `/visual_inspection/detections` | `std_msgs/String` | JSON list of detected objects with class, confidence, track_id, front/back zone |

### Subscribed by us
| Topic | Type | Description |
|-------|------|-------------|
| `/servo/pan_tilt` | `std_msgs/Int16MultiArray` | `[tilt, pan]` servo command (0-180°) |

### Actions
| Action | Type | Description |
|--------|------|-------------|
| `/visual_inspection/inspect_objects` | `InspectObjects` | Full pipeline: detect → coarse → IBVS → capture → MQTT |

---

## How to Check Topics (Monitoring Commands)

```bash
# List all active topics
ros2 topic list

# Check cameras are publishing
ros2 topic hz /visual_inspection/insta360/image_raw
ros2 topic hz /visual_inspection/logitech/image_raw

# Watch current pipeline status
ros2 topic echo /visual_inspection/status

# Watch IBVS pixel error (live)
ros2 topic echo /visual_inspection/ibvs_error

# Watch YOLO detections (JSON with class names)
ros2 topic echo /visual_inspection/detections

# Check servo is receiving commands
ros2 topic echo /servo/pan_tilt

# Send servo home manually
ros2 topic pub --once /servo/pan_tilt std_msgs/Int16MultiArray "{data: [90, 90]}"

# List action servers
ros2 action list
ros2 action info /visual_inspection/inspect_objects
```

---

## Inspection Pipeline — How It Works

```
BT / test_script
     │
     ▼  Goal {max_objects, return_home, location_label, overview_only}
┌─────────────────────────────────────────────────────────┐
│  ibvs_action_server                                     │
│                                                         │
│  if overview_only=True:                                 │
│    → Capture N Insta360 images with YOLO drawn          │
│    → Save to captures/overview/SESSION/                 │
│    → MQTT publish with location_label                   │
│    → Return result (SUCCESS)                            │
│                                                         │
│  else (full inspection):                                │
│    Stage 1  DETECTING (up to 20s)                       │
│      YOLO on Insta360 → ByteTrack assigns stable IDs    │
│      Split: cy < 200px = FRONT, cy > 200px = BACK       │
│                                                         │
│    if all_in_back → signal BT to rotate 180°            │
│                                                         │
│    Stage 2  For each FRONT object (by track ID):        │
│      COARSE: polynomial formula → servo angles          │
│      Wait 2s for servo to settle                        │
│                                                         │
│      IBVS: YOLO on Logitech → PID servo loop            │
│        until err < 10px or 40s timeout                  │
│        feedback: ibvs_error_px every tick               │
│                                                         │
│      CAPTURE (after 10s autofocus wait):               │
│        4x Logitech ROI images (focused close-up)        │
│        1x Insta360 overview image (with YOLO boxes)     │
│        → saves to captures/inspection/SESSION/CLASS/    │
│        → MQTT publish with class_name to ThingsBoard    │
│                                                         │
│    Return result {success, objects_found, object_in_back}│
└─────────────────────────────────────────────────────────┘
```

---

## Capture Folder Structure

```
~/Documents/Visual_Inspection_ws/captures/
│
├── inspection/                    ← Full IBVS ROI captures
│   └── 20260309_120000/           ← Session ID (timestamp when inspection started)
│       ├── fire_extinguisher/
│       │   └── instance_1/        ← instance_N if multiple same-class objects
│       │       ├── img_01.jpg     ← Logitech close-up (focused, post-10s autofocus)
│       │       ├── img_02.jpg
│       │       ├── img_03.jpg
│       │       ├── img_04.jpg
│       │       └── overview_unknown_01.jpg  ← Insta360 with YOLO boxes
│       └── gauge/
│           └── instance_1/...
│
└── overview/                      ← Insta360-only snapshots for VLM
    └── 20260309_120500/
        ├── gauge_room_A_01.jpg    ← Labelled by location_label from BT
        └── gauge_room_A_02.jpg
```

**Check what's been captured:**
```bash
ls -lR ~/Documents/Visual_Inspection_ws/captures/
```

---

## Test Commands (Without Behaviour Tree)

### Full inspection (most common test)
```bash
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py
```

### Single manual goal (terminal)
```bash
ros2 action send_goal /visual_inspection/inspect_objects \
  visual_inspection_interfaces/action/InspectObjects \
  "{max_objects: 0, return_home: true, location_label: 'unknown', overview_only: false, overview_count: 2}"
```

### Overview-only mode (BT requests 360 snapshot for VLM)
```bash
ros2 action send_goal /visual_inspection/inspect_objects \
  visual_inspection_interfaces/action/InspectObjects \
  "{max_objects: 0, return_home: false, location_label: 'gauge_room_A', overview_only: true, overview_count: 2}"
```

### Run the BT tree directly
```bash
pip install py_trees   # first time only
ros2 run visual_inspection_ros run_inspection_bt
```

---

## Behaviour Tree Integration (for Ravith)

### BT Leaf Nodes — Import These

```python
from visual_inspection_ros.bt_nodes.inspection_bt_nodes import (
    InspectObjectsAction,   # Action: runs full IBVS inspection pipeline
    CaptureOverviewAction,  # Action: captures Insta360 360° snapshot for VLM
    CheckObjectInBack,      # Condition: checks if object was in back zone
)
```

### Node 1: InspectObjectsAction

```python
InspectObjectsAction(
    name='Inspect',
    node=ros_node,          # your rclpy Node instance
    max_objects=0,          # 0 = inspect all, N = first N
    return_home=True,       # servos go home after done
    location_label='unknown'  # or known label from BT blackboard
)
```

**Blackboard Writes (after completion):**
| Key | Type | Description |
|-----|------|-------------|
| `inspection_success` | bool | True if ≥1 object inspected |
| `objects_found` | int | Total detected on Insta360 |
| `objects_inspected` | int | Successfully centered + captured |
| `object_in_back` | bool | True → robot needs 180° rotation |
| `failed_reason` | str | `""` / `"no_detection"` / `"ibvs_timeout"` / `"all_in_back"` |
| `ibvs_error_px` | float | Last IBVS pixel error |

**Status:**
| Condition | py_trees Status |
|-----------|-----------------|
| ≥1 object inspected | `SUCCESS` |
| IBVS failed / no detection | `FAILURE` |
| Still running | `RUNNING` |

---

### Node 2: CaptureOverviewAction

```python
CaptureOverviewAction(
    name='CaptureOverview',
    node=ros_node,
    location_label='gauge_room_A',   # from BT blackboard
    overview_count=2                  # number of 360 images
)
```

**Blackboard Writes:** `overview_success` (bool)

Use this when BT arrives at a location and wants a 360° snapshot sent to VLM **without** running IBVS.

---

### Node 3: CheckObjectInBack (Condition)

```python
CheckObjectInBack(name='IsObjectInBack')
```

Returns `SUCCESS` if `object_in_back=True` on blackboard → BT should rotate robot 180° and retry.

---

### Example BT Structure

```
Sequence  (InspectionSequence)
├── CaptureOverviewAction          ← 360 snapshot for VLM (optional)
│
└── Selector  (TryBothSides)
    │
    ├── Sequence  (FrontSide)
    │   └── InspectObjectsAction   ← Try front side
    │       (SUCCESS → done)
    │       (FAILURE failed_reason=="all_in_back" → fall through)
    │
    └── Sequence  (BackSide)
        ├── CheckObjectInBack      ← Confirm back objects exist
        ├── RotateRobot180         ← YOUR BT node (rotate robot)
        └── InspectObjectsAction   ← Inspect again after rotation
```

### BT Decision Table

| `success` | `object_in_back` | `failed_reason` | BT Action |
|-----------|-----------------|-----------------|-----------|
| `True` | `False` | `""` | ✅ All done, continue mission |
| `True` | `True` | `""` | ✅ Front done, rotate 180° and reinspect back |
| `False` | `True` | `"all_in_back"` | 🔄 Rotate 180° then retry |
| `False` | `False` | `"no_detection"` | ➡️ Move to next position |
| `False` | `False` | `"ibvs_timeout"` | ⚠️ IBVS failed, skip or alert |

---

## MQTT Configuration

**File:** `~/Documents/Visual_Inspection_ws/config/mqtt_config.yaml`

```yaml
broker:       demo.thingsboard.io   # ThingsBoard cloud
port:         1883
access_token: 34bbvq0ix4u2licucuq0  # ThingsBoard device access token
topic:        v1/devices/me/telemetry
jpeg_quality: 85
capture_dir:  ~/Documents/Visual_Inspection_ws/captures
```

**ThingsBoard:** https://demo.thingsboard.io  
Login → Devices → "inspection" → Latest Telemetry

**MQTT payload per image:**
```json
{
    "session":     "20260309_120000",
    "object_id":   1,
    "class_name":  "fire_extinguisher",
    "image_idx":   1,
    "total":       5,
    "timestamp":   1741500000.0,
    "image_b64":   "..."
}
```

---

## Configuration — Key Constants (ibvs_action_server.py)

| Constant | Default | Description |
|----------|---------|-------------|
| `INSTA_SEARCH_TIMEOUT` | 20s | Max wait for object detection on Insta360 |
| `IBVS_TOTAL_TIMEOUT` | 40s | Max time for IBVS centering per object |
| `IBVS_TOL_PX` | 10px | Error threshold to consider "centered" |
| `FOCUS_WAIT` | 10s | Autofocus wait after IBVS before capture |
| `IMAGES_PER_OBJ` | 4 | Logitech ROI images per object |
| `FRONT_Y_MAX` | 200px | Insta360 y-threshold: above=BACK, below=FRONT |
| `TILT_REVERSED` | True | Flip tilt direction (hardware-specific) |
| `KEEP_LOCAL` | True | Always keep images locally (dataset mode) |
| `CONF_INSTA` | 0.5 | YOLO confidence for Insta360 detection |
| `CONF_IBVS` | 0.3 | YOLO confidence for Logitech IBVS |

---

## Rebuild After Code Changes

```bash
# Only needed when InspectObjects.action file is changed:
cd ~/Documents/Visual_Inspection_ws/inspection_ws
rm -rf build/visual_inspection_interfaces install/visual_inspection_interfaces
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_inspection_interfaces visual_inspection_ros
source install/setup.bash

# For Python file changes only (ibvs_action_server.py etc.) — NO rebuild needed:
# Just SCP the file from laptop and restart the node
pkill -f ibvs_action_server && sleep 1
ros2 run visual_inspection_ros ibvs_action_server
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `ImportError: cannot import name 'InspectObjects'` | Run `colcon build` and `source install/setup.bash` |
| `ValueError: too many values to unpack` | SCP latest ibvs_action_server.py |
| `ModuleNotFoundError: No module named 'em'` | Don't activate `.venv` before `colcon build` on laptop |
| No camera feed | Check `/dev/insta360` and `/dev/logitech` exist: `ls /dev/insta360` |
| Servo not moving | Check Arduino connected: `ls /dev/ttyUSB*` |
| IBVS not converging (timeout 40s) | Move object closer, check Logitech camera focused |
| MQTT no telemetry on ThingsBoard | Check `mqtt_config.yaml` access token; check MQTT broker reachable |
| `failed_reason: ""` with status=6 | Crash in execute_callback — check Terminal 3 for traceback |
| Images blurry | Increase `FOCUS_WAIT` (currently 10s) in ibvs_action_server.py |

---

## YOLO Model

- **Path:** `~/Documents/Visual_Inspection_ws/weights/yolo11n.engine`
- **Format:** TensorRT FP16 (accelerated on Jetson GPU) ← loads first
- **Fallback:** `.pt` PyTorch file if `.engine` not found
- **Classes detected:** door, gauge, fire_extinguisher (trained classes)
- **Tracking:** ByteTrack (`model.track(persist=True, tracker='bytetrack.yaml')`)

---

*Last updated: 2026-03-09*
