# Weekly Progress Report - Visual Inspection System (Week 4)
# Period: 4 March – 10 March 2026

---

## Slide 1: Last Week Recap — ROS2 Conversion This Week

○ Previous weeks completed: TensorRT acceleration, two-stage IBVS pipeline, udev camera detection — all working and benchmarked

○ **This week's focus: ROS2 conversion** — wrapping the working pipeline into ROS2 nodes so Behaviour Tree integration can happen

○ All items from PLAN.md now completed:

| Plan Item | Status |
|-----------|--------|
| camera_node.py — both cameras published | ✅ Done |
| servo_node.py — Arduino control | ✅ Done |
| ibvs_action_server.py — IBVS as Action Server | ✅ Done |
| ByteTrack multi-object integration | ✅ Done |
| 4-image capture per object | ✅ Done |
| MQTT publisher (images + metadata) | ✅ Done |
| Front/back zone detection + object_in_back signal | ✅ Done |
| BT leaf node classes provided | ✅ Done |
| README for BT person | ✅ Done |

**Script for Slide 1:**
"You already know the IBVS pipeline from last week — TensorRT at 30 FPS, two-stage centering, reliable camera detection. This week I converted all of that into ROS2. The reason is the Behaviour Tree integration — the navigation team needs standard ROS2 interfaces to call our inspection system. Every item on the plan we created for this phase is now done."

---

## Slide 2: ROS2 Nodes Built + Topics

**Three nodes created — each with a clear single responsibility:**

```
Node 1: camera_node
  Publishes → /visual_inspection/insta360/image_raw  (sensor_msgs/Image, 30 Hz)
             → /visual_inspection/logitech/image_raw  (sensor_msgs/Image, 30 Hz)

Node 2: servo_node
  Subscribes → /servo/pan_tilt  (std_msgs/Int16MultiArray: [tilt, pan])
  Writes    → Arduino serial (/dev/arduino) → physical pan-tilt servos

Node 3: ibvs_action_server
  Action    → /visual_inspection/inspect_objects  (InspectObjects action)
  Publishes → /visual_inspection/debug       (combined side-by-side camera view)
            → /visual_inspection/status      (IDLE/DETECTING/COARSE/IBVS/CAPTURING)
            → /visual_inspection/ibvs_error  (live pixel error: x, y, magnitude)
            → /visual_inspection/detections  (JSON: objects, class, track_id, zone)
```

**Camera access method:** OpenCV `cv2.VideoCapture` with Linux V4L2 backend
- Camera nodes subscribe to `/insta360/image_raw` and `/logitech/image_raw`
- Initially had difficulty opening cameras inside ROS2 nodes — solved by reusing the previously implemented camera access method (V4L2 with udev symlinks) adapted into a ROS2 publisher node

*(Insert: RViz2 screenshot showing /visual_inspection/debug topic — side-by-side camera feed with bounding boxes)*

**Script for Slide 2:**
"Three nodes, each doing one job. The camera node reads both cameras and publishes as standard ROS2 image topics. It uses V4L2 through OpenCV — we had previously implemented a reliable camera detection method using udev symlinks, so I adapted that approach into the ROS2 publisher. The servo node is just a bridge — it receives ROS2 messages and forwards them to Arduino over serial. The IBVS action server is the main node — it contains the entire inspection pipeline and communicates with the BT via a ROS2 Action. There are also 4 status topics so any external monitor or BT node can observe what's happening in real time."

---

## Slide 3: Full Inspection Pipeline — Demo

**Inspection flow:**

```
DETECTING (Insta360, up to 20s)
  → ByteTrack assigns stable IDs to each object
  → Split: cy < 200px = FRONT objects | cy > 200px = BACK objects

COARSE (per front object)
  → Polynomial formula: (cx_insta, cy_insta) → (pan_angle, tilt_angle)
  → Servo moves to rough position (2s settle)

IBVS (per front object, up to 40s)
  → YOLO on Logitech → PID loop until error < 10px
  → Feedback to BT: ibvs_error_px every tick

CAPTURE (after 10s autofocus wait)
  → 4 Logitech ROI images + 1 Insta360 overview image
  → MQTT publish to ThingsBoard + save locally
```

*(Insert: terminal screenshot showing IBVS error reducing → from ~150px to 8.4px)*

*(Insert: RViz2 /visual_inspection/debug screenshot — bounding box with class name, ID, IBVS arrow, mode overlay)*

*(Insert: full inspection run video clip — servo moving, IBVS converging, capture)*

**Test script (replaces BT until integration):**
```bash
python3 test_scripts/test_full_pipeline.py
# Output:
# [IBVS] obj=1  IBVS err= 8.4px  .............................................
# RESULT: success=True | objects_found=1 | objects_inspected=1
# [BT] -> SUCCESS: All front objects inspected.
```

**Script for Slide 3:**
"This is the full pipeline running end to end. You can see in the terminal screenshot that the IBVS error starts around 150 pixels when the object first appears on the Logitech camera, and reduces down to around 8 pixels over several seconds — that's the PID loop doing its job. The RViz2 screenshot shows the combined debug view — left side is Insta360 with the front-back boundary line visible, right side is Logitech with the bounding box centred and the blue IBVS arrow showing the correction direction. The mode overlay shows IBVS, then switches to CAPTURING when converged. Since we don't have the BT connected yet, I built a test script that runs the same goal-feedback-result flow that the BT would use."

---

## Slide 4: Key Technical Features Added

**1. ByteTrack Multi-Object Tracking:**
- `model.track(persist=True, tracker='bytetrack.yaml')` from Ultralytics
- Each detected object gets a stable integer ID across frames (ID38 stays ID38)
- Multiple same-class objects (e.g., 3 fire extinguishers) processed in consistent track ID order
- Label shown on bounding box: `[fire_extinguisher] ID38 0.82`

**2. Front / Back Zone Detection:**
- Insta360 equirectangular output: top half = FRONT robot direction, bottom half = BACK
- Threshold at y=200px — objects above are FRONT (green boxes), below are BACK (orange boxes)
- If all objects in BACK: result returns `object_in_back=True, failed_reason="all_in_back"`
- BT rotates robot 180° and calls inspection again

**3. BT Leaf Nodes Provided (`bt_nodes/inspection_bt_nodes.py`):**
```python
from visual_inspection_ros.bt_nodes.inspection_bt_nodes import (
    InspectObjectsAction,   # wraps full pipeline as py_trees Action
    CaptureOverviewAction,  # quick 360° snapshot for VLM (no IBVS)
    CheckObjectInBack,      # Condition: reads object_in_back from blackboard
)
```

**BT Decision Table:**
| `success` | `object_in_back` | `failed_reason` | BT Action |
|-----------|-----------------|-----------------|-----------|
| True | False | `""` | All done, continue |
| True/False | True | `""`/`"all_in_back"` | Rotate 180° → retry |
| False | False | `"no_detection"` | Move to next position |
| False | False | `"ibvs_timeout"` | Log alert, skip |

**Script for Slide 4:**
"Three key technical features. ByteTrack gives us stable object IDs so we can handle multiple objects in one scene without confusion — the system always processes them in the same order. The front-back zone detection means the system can tell the BT when objects are behind the robot. And the BT leaf node classes mean Ravith can get the full inspection working with essentially a few lines of Python — import three classes, place them in the tree, done. The CaptureOverview node is also new — it lets the BT quickly grab a 360-degree snapshot for the VLM pipeline without moving the servos at all."

---

## Slide 5: MQTT Integration + Dataset Captures

**ThingsBoard MQTT working:**
- Broker: `demo.thingsboard.io:1883`
- Auth: device access token (ThingsBoard standard: username=token, password=empty)
- Each image sent as JSON with: session ID, class_name, object_id, base64 JPEG

*(Insert: ThingsBoard screenshot showing "inspection" device Latest Telemetry with class_name, message, session, timestamp keys)*

**Local capture folder (all images kept for dataset — `KEEP_LOCAL=True`):**
```
captures/
├── inspection/                  ← Full IBVS captures (YOLO ROI + overview)
│   └── 20260309_120000/         ← Session = inspection timestamp
│       └── fire_extinguisher/   ← Auto-labelled from YOLO class name
│           └── instance_1/
│               ├── img_01.jpg   ← Logitech (after 10s autofocus wait)
│               ├── img_02.jpg
│               ├── img_03.jpg
│               ├── img_04.jpg
│               └── overview_01.jpg  ← Insta360 with YOLO boxes
│
└── overview/                    ← Insta360-only snapshots for VLM
    └── 20260309_121000/
        ├── gauge_room_A_01.jpg
        └── gauge_room_A_02.jpg
```

**Initial dataset collected this week:**

*(Insert: screenshot of captures/ folder showing collected images)*

*(Insert: sample captured ROI images — gauge, fire extinguisher, door — showing image quality after 10s autofocus)*

- Images auto-labelled by YOLO class → folder is already structured for training
- 10-second autofocus wait added → significantly sharper gauge images vs without wait

**Script for Slide 5:**
"MQTT is working — you can see the ThingsBoard screenshot showing telemetry arriving from our device. Class name, session ID, and timestamp keys are visible. Each image is base64 encoded into a JSON payload. We also started collecting an initial dataset this week using the pipeline itself. The capture folder structure automatically labels images by YOLO class name — so every image already knows if it's a gauge, fire extinguisher, or door without any manual labeling. The 10-second autofocus wait before capture makes a meaningful difference for gauge images — the text on the gauge is actually readable in the captured images."

---

## Slide 6: Blockers Encountered + Solutions

**Blocker 1: Camera access inside ROS2 nodes**
- Problem: Initial approach to open cameras directly inside ROS2 nodes failed — device access conflicts
- Solution: **Reused previously implemented camera access method** (V4L2 via OpenCV with udev symlinks) and adapted it into a ROS2 publisher node. Separate camera_node publishes topics → ibvs_action_server only subscribes
- Result: Clean separation, cameras reliably accessible

**Blocker 2: `ValueError: too many values to unpack` — IBVS crash**
- Problem: `_detect_insta()` updated to return 4 values but one call inside `_ibvs()` still expected 3 → exception silently caught by action framework → `failed_reason=""` with status=ABORTED
- How found: Checked Terminal 3 (action server log) for traceback — error visible there
- Fix: `_, _, insta_dbg = ...` → `_, _, _, insta_dbg = ...` (one underscore added)

**Blocker 3: `UnboundLocalError: front_dets referenced before assignment`**
- Problem: During refactoring to add `overview_only` mode, accidentally deleted the `_search_insta()` call from the full inspection path
- Fix: Restored `front_dets, back_dets = self._search_insta(goal_handle)` in the correct location

**Blocker 4: MQTT images not arriving on ThingsBoard**
- Root cause: `paho-mqtt` installed in `.venv` Python only — ROS2 action server uses **system Python** → `import paho` silently failed (caught by `except Exception`)
- Fix: `pip3 install paho-mqtt` on system Python (without activating venv)
- Lesson: ROS2 nodes always use system Python, never the venv

**Blocker 5: `colcon build` fails with `ModuleNotFoundError: No module named 'em'`**
- Root cause: Building with `.venv` active overrides system Python used by `rosidl_adapter`
- Fix: Never activate `.venv` before `colcon build` — build only runs on Jetson, never laptop

**Blocker 6: paho-mqtt 1.x `wait_for_publish(timeout=5)` crashes**
- Problem: ThingsBoard paho on Jetson is version 1.x — `timeout` argument not available
- Fix: Added try/except fallback → `result.wait_for_publish()` (no timeout arg for 1.x)

**Script for Slide 6:**
"Six blockers this week — I'll go through the most educational ones. The camera access issue was about separation of concerns. Rather than fighting with direct camera access inside the action server, I used the camera architecture we previously implemented — a separate publisher node that opens the cameras, and the action server just subscribes. This is actually better design anyway. The MQTT silence was the trickiest — the action server was failing silently because paho-mqtt wasn't installed for the system Python. All my testing with the venv worked fine, but ROS2 runs on system Python. From now on any library that a ROS2 node needs must be installed with system pip3."

---

## Slide 7: Next Week — Dataset Collection + Evaluation

**Goal:** Collect structured dataset for all 3 YOLO classes using the pipeline

| Class | Target images | Method |
|-------|--------------|--------|
| `gauge` | 100+ | Various distances, angles, lighting conditions |
| `fire_extinguisher` | 100+ | Various backgrounds, partial occlusion cases |
| `door` | 50+ | Different door types, open/closed, indoor lighting |

**Collection method:**
1. Terminal 1-3: start ROS2 nodes
2. Terminal 4: run `test_full_pipeline.py` — pipeline automatically captures, labels, saves to `captures/inspection/`
3. Images already labeled by class → ready for YOLO fine-tuning after collection

**Evaluation plan:**
- Measure IBVS convergence time at different distances (1m, 2m, 3m)
- Measure pixel error at convergence (target: < 10px)
- Measure capture quality score (can a human read the gauge? 1-5)
- Test front/back detection with objects in back zone → confirm BT signal works
- Test multi-object scenario (2-3 objects in same scene)

**BT integration (with Ravith):**
- Share `inspection_bt_nodes.py` + README instructions
- Agree on blackboard key names
- Integration test: BT calls action → front object → success → BT calls again → back object → rotate 180° → retry

**Script for Slide 7:**
"Next week has two parallel tasks. I'll be systematically collecting data using the pipeline — the folder structure already handles the labeling automatically so I just run the pipeline in front of each object type and it saves labeled images. The evaluation work will measure convergence time at different distances, error at convergence, and image quality for gauge readability. In parallel, Ravith will start connecting our BT leaf nodes into the main tree. The key test we want to do together is the back-side scenario — put an object behind the robot, confirm the system signals object_in_back=True, confirm the BT rotation happens, then inspect again. That end-to-end test will validate the full integration."

---

## Technical Explanations (for your notes)

**1. Why ROS2 Action (not Topic or Service)?**
- Topic: fire-and-forget, no response → can't get result or feedback
- Service: synchronous, one response only, no streaming → BT hangs for 60 seconds waiting
- Action: long-running, streaming feedback, structured result, fully cancellable → perfect for inspection

**2. Why camera_node is separate from ibvs_action_server:**
- Cameras must publish at 30 Hz continuously even when no inspection is running
- If cameras were inside the action server, they'd only run during a goal
- Separation also allows other nodes (e.g., a monitoring dashboard) to subscribe to camera feeds independently

**3. How ByteTrack handles multiple objects:**
- Frame N: detects objects → assigns IDs using IoU matching with previous frame
- High confidence detections: tracked immediately
- Low confidence: buffered, given ID only if reappears consistently
- Result: even if same-class objects move slightly between frames, IDs stay stable

**4. Why YOLO class name = folder name = MQTT tag:**
- The YOLO model already knows what it detected: `results.names[int(box.cls[0])]`
- No extra classification step needed — YOLO output IS the label
- Folder: `captures/inspection/SESSION/fire_extinguisher/instance_1/img_01.jpg`
- MQTT payload: `"class_name": "fire_extinguisher"`
- Everything consistently labeled from the single YOLO detection step

**5. Front/Back zone — equirectangular projection:**
- Insta360 outputs a flat equirectangular image of the 360° sphere
- Physical interpretation: top rows of image = forward/front direction, bottom rows = backward/back
- A gauge at y=100 (in a 480px tall image) is physically in front of the robot
- A fire extinguisher at y=350 is physically behind the robot
- This lets us make a front/back decision using just a y-threshold with no 3D geometry

**6. ThingsBoard MQTT — why base64 for images:**
- ThingsBoard telemetry is designed for scalar values (temperature, pressure, etc.)
- Images don't have a native MQTT type — base64 encoding converts binary image to a JSON-safe string
- Limitation: ThingsBoard stores it as a string value, not a viewable image
- Future: use ThingsBoard's HTTP API to upload images as proper attachments, or use a dedicated image server

---

*Report prepared by: Dinethra | 2026-03-10*
