# Visual Inspection — ROS2 Integration + Behaviour Tree Plan

**Stack:** ROS2 Humble + py_trees_ros + Python  
**Existing:** Working ibvs_pipeline.py on Jetson (DO NOT break this)  
**Goal:** Wrap existing pipeline into ROS2 nodes → connect to main BT

---

## 📁 Current Jetson Workspace (flat, working)

```
~/Documents/Visual_Inspection_ws/
├── venv/                    ← existing, keep
├── weights/yolo11n.engine   ← existing TRT engine, keep
├── ibvs_pipeline.py         ← existing working pipeline, keep
├── ibvs_headless.py         ← existing launcher, keep
├── ibvs_headed.py           ← existing launcher, keep
├── calibration_config.py    ← existing, keep
├── config/                  ← existing camera calibration, keep
└── inspection_ws/           ← NEW folder SCP'd from laptop
    └── (our new ROS2 code)
```

**Rule:** Never modify existing files. New ROS2 nodes import from them.

---

## ✅ Step 0 (DONE) — Python Pipeline Working

- ✅ Insta360 YOLO detection (TensorRT FP16, ~30 FPS)
- ✅ COARSE positioning (cubic formula → servo)
- ✅ FINE centering (IBVS, <10px error)
- ✅ Camera auto-detection (udev symlinks)
- ✅ Arduino auto-detection (USB vendor ID)

---

## 🔄 Step 1 — ROS2 Conversion (DO THIS FIRST)

Convert the working Python pipeline into ROS2 nodes so the BT person can connect.

### 1a. ROS2 Package Structure

```
inspection_ws/
├── README.md                    ← for your friend
├── PLAN.md                      ← this file
├── setup_ros.sh                 ← install script for Jetson
│
└── visual_inspection_ros/       ← ROS2 package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── visual_inspection_ros
    ├── action/
    │   └── InspectObjects.action  ← custom ROS2 action definition
    ├── msg/
    │   └── InspectionResult.msg   ← custom message
    └── visual_inspection_ros/
        ├── __init__.py
        ├── camera_node.py           ← Node 1: publish camera feeds
        ├── servo_node.py            ← Node 2: Arduino servo control
        ├── ibvs_action_server.py    ← Node 3: IBVS as Action Server
        ├── inspection_node.py       ← Node 4: full inspection coordinator
        └── bt_nodes/
            ├── __init__.py
            ├── detect_bt_node.py    ← BT node: Insta360 detect
            ├── coarse_bt_node.py    ← BT node: coarse position
            ├── ibvs_bt_node.py      ← BT node: IBVS center
            ├── capture_bt_node.py   ← BT node: capture 4 images
            └── mqtt_bt_node.py      ← BT node: publish to MQTT
```

---

### 1b. ROS2 Topics and Actions We Expose

**Topics (published by us):**

| Topic | Type | Description |
|-------|------|-------------|
| `/visual_inspection/insta360/image_raw` | `sensor_msgs/Image` | Insta360 camera feed |
| `/visual_inspection/logitech/image_raw` | `sensor_msgs/Image` | Logitech camera feed |
| `/visual_inspection/status` | `std_msgs/String` | Pipeline status (IDLE/COARSE/FINE/CAPTURING) |
| `/visual_inspection/ibvs_error` | `geometry_msgs/Point` | Current IBVS pixel error (x, y) |
| `/visual_inspection/detections` | `std_msgs/String` | JSON of detected objects |

**Topics (subscribed by us):**

| Topic | Type | Description |
|-------|------|-------------|
| `/servo/pan_tilt` | `std_msgs/Int16MultiArray` | Direct servo command [tilt, pan] |

**Actions (we are the server, BT calls us):**

```
Action: /visual_inspection/inspect_objects
Type:   visual_inspection_ros/action/InspectObjects

Goal:
  int32 max_objects        # 0 = inspect all detected, N = inspect only N
  bool  return_home        # true = servo returns 90,90 after done

Result:
  bool   success
  int32  objects_inspected
  string failed_reason     # "no_detection" / "ibvs_timeout" / "mqtt_error" / ""

Feedback:
  string current_step      # "detecting" / "coarse" / "ibvs" / "capturing" / "publishing"
  int32  current_object    # which object number currently processing
  float  ibvs_error_px     # current pixel error
```

---

### 1c. The 4 ROS2 Nodes

#### Node 1: `camera_node.py`
```
Name: /visual_inspection/camera_publisher
Publishes:
  /visual_inspection/insta360/image_raw
  /visual_inspection/logitech/image_raw
Frequency: ~30 Hz
Uses: cv2.VideoCapture with existing udev symlinks (/dev/insta360, /dev/logitech)
```

#### Node 2: `servo_node.py`
```
Name: /visual_inspection/servo_controller
Subscribes: /servo/pan_tilt  [tilt, pan] as Int16MultiArray
Action: writes to Arduino serial (/dev/arduino)
Uses: existing find_arduino() from ibvs_pipeline.py
```

#### Node 3: `ibvs_action_server.py`
```
Name: /visual_inspection/ibvs_server
Action Server: /visual_inspection/inspect_objects
When called:
  1. Runs YOLO on Insta360 frame (using existing TRT model)
  2. COARSE: cubic formula → publishes to /servo/pan_tilt
  3. FINE: IBVS loop → publishes to /servo/pan_tilt
  4. CAPTURE: 4 images → publishes via MQTT
  5. Sends result back to BT
Publishes feedback: step-by-step progress
```

#### Node 4: `inspection_node.py` (optional orchestrator)
```
Name: /visual_inspection/inspection_coordinator
Manages: multi-object queue, ByteTracker IDs, inspected list
```

---

## 🌳 Step 2 — Behaviour Tree Integration (py_trees_ros)

**Your friend builds the main BT. You provide BT leaf nodes.**

### BT Nodes We Provide

Your friend adds these leaf nodes to the main tree:

```python
# They import from our package:
from visual_inspection_ros.bt_nodes import (
    DetectObjectsNode,     # Condition: checks if YOLO sees objects
    InspectObjectsNode,    # Action: runs full inspection (wraps ROS2 Action)
)
```

### What Goes Inside the BT (our section)

```
[BT Section: Visual Inspection]  ← your friend places this in the main tree
│
└── Sequence
    ├── Condition: IsRobotStationary      ← from navigation BT (not us)
    │
    ├── Action: InspectObjects             ← OUR action node
    │     (internally does COARSE+IBVS+CAPTURE+MQTT for all objects)
    │     SUCCESS → BT continues
    │     FAILURE(no_detection) → BT handles (rotate robot & retry)
    │     FAILURE(ibvs_timeout) → BT handles (skip or alert)
    │
    └── Action: PublishInspectionSummary   ← OUR node (optional)
```

### Our BT Leaf Node Classes

```python
# detect_bt_node.py
class DetectObjectsNode(py_trees.behaviour.Behaviour):
    """Condition: Returns SUCCESS if YOLO detects objects on Insta360"""
    # Stores detected objects in BT Blackboard for next nodes

# ibvs_bt_node.py  
class InspectObjectsNode(py_trees_ros.action_clients.Fromaction):
    """Action: Calls /visual_inspection/inspect_objects ROS2 action"""
    # Wraps ROS2 action as py_trees node
    # Handles RUNNING/SUCCESS/FAILURE states
```

---

## 📋 Implementation Order

```
Week 1: ROS2 Conversion
├── [ ] Create ROS2 package (visual_inspection_ros)
├── [ ] camera_node.py — publish both cameras as ROS2 topics
├── [ ] servo_node.py  — Arduino control via ROS2 topic
├── [ ] ibvs_action_server.py — IBVS as ROS2 Action Server
└── [ ] Test: ros2 action send_goal /visual_inspection/inspect_objects ...

Week 2: Multi-Object + Capture
├── [ ] ByteTracker integration (Ultralytics model.track() with persist=True)
├── [ ] 4-image capture with IBVS re-centering between captures
├── [ ] Save images + metadata JSON to capture/ folder
└── [ ] Test multiple object scenario

Week 3: MQTT
├── [ ] MQTTPublisher class (paho-mqtt, already in existing venv)
├── [ ] Publish 4 images + metadata per object
├── [ ] Connect to broker (need broker IP from team)
└── [ ] Test end-to-end image delivery

Week 4: BT Nodes
├── [ ] DetectObjectsNode (py_trees Condition)
├── [ ] InspectObjectsNode (py_trees_ros Action client)
├── [ ] Share node classes with BT person
└── [ ] Integration test with main BT
```

---

## 🛠️ Setup on Jetson

```bash
# 1. Source existing venv (has all dependencies)
source ~/Documents/Visual_Inspection_ws/venv/bin/activate

# 2. Source ROS2 Humble
source /opt/ros/humble/setup.bash

# 3. Build our package
cd ~/Documents/Visual_Inspection_ws/inspection_ws
colcon build --packages-select visual_inspection_ros
source install/setup.bash

# 4. Run nodes
ros2 run visual_inspection_ros camera_node
ros2 run visual_inspection_ros ibvs_action_server

# 5. Test action from terminal (instead of BT)
ros2 action send_goal /visual_inspection/inspect_objects \
  visual_inspection_ros/action/InspectObjects \
  "{max_objects: 0, return_home: true}"
```

---

## ❓ Open Questions (confirm with team)

1. **MQTT broker IP/hostname** — needed for mqtt_bt_node.py
2. **Gazebo** — for simulation testing? which robot model?
3. **Shared BT blackboard keys** — what key names does main BT use for robot state?
4. **py_trees_ros version** — `pip install py_trees_ros` or ROS apt package?
5. **How to trigger us** — does BT call our action, or does main BT read our topic?
