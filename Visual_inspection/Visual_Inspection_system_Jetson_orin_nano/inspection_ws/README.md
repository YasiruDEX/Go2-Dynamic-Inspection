# Visual Inspection System — ROS2 Package
**Stack:** ROS2 Humble · Python 3.10 · YOLOv26s TensorRT · ByteTrack · paho-mqtt · py_trees
**Hardware:** Jetson Orin Nano · Insta360 camera · Logitech camera · Arduino (pan-tilt servos)
*Updated: 2026-04-20 — Migrated from ROS2 Action → ROS2 Service*

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

### Terminal 3 — Inspection Service (main pipeline)
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros inspection_service
```
✅ Ready when you see:
```
Inspection service ready: /visual_inspection/inspect
```

### Terminal 4 — Test (simulates Behaviour Tree)
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object fire_extinguisher
```

### Terminal 5 — RViz2 (optional debug view)
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
├── visual_inspection_interfaces/        ← ROS2 custom message/service/action package
│   ├── action/
│   │   └── InspectObjects.action        ← legacy action (kept)
│   └── srv/
│       └── Inspect.srv                  ← ✅ ACTIVE service definition
│
└── visual_inspection_ros/               ← ROS2 node package
    ├── package.xml
    ├── setup.py
    └── visual_inspection_ros/
        ├── camera_node.py               ← Node 1: publish both cameras
        ├── servo_node.py                ← Node 2: Arduino servo control
        ├── ibvs_action_server.py        ← Node 3: legacy action server (base class)
        ├── inspection_service.py        ← Node 4: ✅ ACTIVE service server
        └── bt_nodes/
            └── inspection_bt_nodes.py  ← BT leaf nodes
```

---

## Service Interface — Inspect.srv

```
# ── REQUEST ───────────────────────────────────────────────────────────────
string  target_object    # "fire_extinguisher" | "door" | "person" | "gauge"
                         # "unknown" | "main_cylinder"  → overview-only
                         # "" or "any"                  → detect all classes
string  location_label   # Location from BT e.g. "engine_room_A"
int32   max_objects      # 0 = inspect all, N = first N
bool    return_home      # true = servo returns to 90°,90° when done

# ── RESPONSE ──────────────────────────────────────────────────────────────
bool    success               # true if ≥1 object inspected
string  status                # "ok" | "no_detection" | "ibvs_timeout" |
                              # "all_in_back" | "no_frames" | "busy"
int32   objects_found         # total detected on Insta360
int32   objects_inspected     # successfully centred + captured
bool    object_in_back        # true → BT must rotate robot 180° and retry
string[] image_paths          # absolute paths to all saved images
string  info                  # human-readable summary
```

### YOLO Model Classes (yolov26s.engine)

| ID | YOLO Name | BT `target_object` | Confidence |
|---|---|---|---|
| 0 | `door` | `"door"` | 0.5 |
| 1 | `extinguisher` | `"fire_extinguisher"`, `"extinguisher"` | 0.5 |
| 2 | `gauge` | `"gauge"`, `"pressure_gauge"` | **0.3** |
| 3 | `person` | `"person"`, `"people"` | 0.5 |

*`unknown` and `main_cylinder` → overview-only (not YOLO classes)*

---

## All ROS2 Topics

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/visual_inspection/insta360/image_raw` | `sensor_msgs/Image` | Insta360 raw feed (360°) |
| `/visual_inspection/logitech/image_raw` | `sensor_msgs/Image` | Logitech raw feed (close-up) |
| `/visual_inspection/debug` | `sensor_msgs/Image` | Combined debug view (optional) |
| `/visual_inspection/status` | `std_msgs/String` | `IDLE`/`DETECTING`/`COARSE`/`IBVS`/`CAPTURING`/`OVERVIEW`/`SWEEP` |
| `/visual_inspection/ibvs_error` | `geometry_msgs/Point` | IBVS pixel error |
| `/visual_inspection/detections` | `std_msgs/String` | JSON detected objects |

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/servo/pan_tilt` | `std_msgs/Int16MultiArray` | `[tilt, pan]` servo command |

### Services (active)
| Service | Type | Description |
|--------|------|-------------|
| `/visual_inspection/inspect` | `Inspect.srv` | ✅ Full pipeline — request/response |

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

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# 1. Fire extinguisher
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object fire_extinguisher

# 2. Door
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object door

# 3. Person
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object person

# 4. Gauge (with sweep fallback)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object gauge

# 5. Unknown (overview only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object unknown

# 6. Main cylinder (overview only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object main_cylinder

# 7. All classes
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py
```

### Check service manually (ros2 CLI)
```bash
ros2 service list
ros2 service call /visual_inspection/inspect \
  visual_inspection_interfaces/srv/Inspect \
  "{target_object: 'gauge', location_label: 'test', max_objects: 0, return_home: true}"
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

## Configuration — Key Constants (inspection_service.py / ibvs_action_server.py)

| Constant | Default | Description |
|----------|---------|-------------|
| `INSTA_SEARCH_TIMEOUT` | 20s | Max wait for object on Insta360 |
| `GAUGE_INSTA_TIMEOUT` | 8s | Gauge-specific shorter timeout before sweep |
| `IBVS_TOTAL_TIMEOUT` | 40s | Max IBVS centering time |
| `IBVS_TOL_PX` | 10px | Centred threshold |
| `FOCUS_WAIT` | 10s | Autofocus wait before capture |
| `IMAGES_PER_OBJ` | 3 | Logitech ROI images per object |
| `FRONT_Y_MAX` | 200px | Insta360 front/back split line |
| `TILT_REVERSED` | True | Flip tilt (hardware-specific) |
| `KEEP_LOCAL` | True | Always keep images locally |
| `CLASS_CONF['gauge']` | 0.3 | Lower threshold for gauge |
| `CLASS_CONF['*']` | 0.5 | Threshold for all other classes |
| `SWEEP_TILTS` | [20,50,80] | Gauge sweep tilt positions |
| `SWEEP_PAN_RANGE` | (20,160) | Gauge sweep pan range |

---

## Rebuild After Code Changes

```bash
# Full rebuild (after changing .srv or .action files):
cd ~/Documents/Visual_Inspection_ws/inspection_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_inspection_interfaces visual_inspection_ros
source install/setup.bash

# Python-only changes (inspection_service.py, ibvs_action_server.py):
# Just SCP the file and restart — no rebuild needed
pkill -f inspection_service && sleep 1
ros2 run visual_inspection_ros inspection_service
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `cannot import name 'Inspect'` | Run `colcon build` and `source install/setup.bash` |
| `Service not available` | Check `inspection_service` node is running |
| `No module named 'torch'` | Run `ln -sf ~/Documents/Visual_Inspection_ws/venv/lib/python3.10/site-packages/torch ~/.local/lib/python3.10/site-packages/torch` |
| No camera feed | Check `/dev/insta360` and `/dev/logitech` exist |
| Servo not moving | Check Arduino: `ls /dev/ttyUSB*` |
| IBVS timeout | Move object closer, check Logitech focus |
| Gauge never found | Sweep scan runs automatically — check logs for `SWEEP` |
| Images blurry | Increase `FOCUS_WAIT` (currently 10s) |

---

## YOLO Model

- **Path:** `~/Documents/Visual_Inspection_ws/weights/yolov26s.engine`
- **Format:** TensorRT FP16 (Jetson GPU)
- **Classes:** `door`(0), `extinguisher`(1), `gauge`(2), `person`(3)
- **Tracking:** ByteTrack

---

*Last updated: 2026-04-20*
