# Behaviour Tree Integration Guide
**Visual Inspection System — Jetson Orin Nano**

---

## 1. Overview

The visual inspection pipeline uses **two ROS2 Services** that the BT calls in sequence:

| Step | Service | What it does |
|---|---|---|
| 1 | `/visual_inspection/inspect` | Runs full inspection — detects, focuses, captures images, saves to Jetson disk |
| 2 | `/visual_inspection/upload_images` | Reads those saved files, HTTP-POSTs them to your laptop on the same WiFi network |

> **The BT always calls Step 1 first, then Step 2.** If Step 2 (upload) fails, the BT should retry Step 2 — no need to redo the inspection.

**IMPORTANT FOR BT DEVELOPERS:** Both are ROS2 **Services** (not Actions). The BT calls each one and simply waits for a response. There is no continuous feedback stream.

---

## 2. Full BT Flow (Inspect → Upload)

```
BT navigates robot to waypoint (Nav2)
   │
   ▼
BT calls /visual_inspection/inspect
   │   sends: target_object="gauge", location_label="engine_room_A"
   │
   ▼
Jetson runs inspection pipeline:
   Insta360 detects object → Coarse servo move → IBVS fine focus → Capture 3 images
   Images + metadata.json saved on Jetson disk
   │
   ▼
Response back to BT:
   success=True, image_paths=["/path/img_01.jpg", ...], status="ok"
   │
   ▼
BT calls /visual_inspection/upload_images
   │   sends: image_paths=[...], session_label="engine_room_A"
   │
   ▼
Jetson reads files, HTTP-POSTs them to laptop at 192.168.8.244:8888
   │
   ▼
Laptop saves:  received_captures/engine_room_A/gauge/instance_1/
               ├── img_01_conf0.85.jpg
               ├── img_02_conf0.85.jpg
               ├── img_03_conf0.85.jpg
               └── metadata.json
   │
   ▼
Response back to BT:
   success=True → BT continues to next waypoint
   success=False → BT retries upload (no need to re-inspect)
```

---

## 3. Service 1 — Inspect

**Service**: `/visual_inspection/inspect`  
**Type**: `visual_inspection_interfaces/srv/Inspect`

### Request (BT → Jetson)

| Field | Type | What to send |
|---|---|---|
| `target_object` | `string` | What to inspect: `"fire_extinguisher"`, `"door"`, `"person"`, `"gauge"`, `"unknown"` |
| `location_label` | `string` | Location from BT map (e.g. `"engine_room_A"`). Used in folder names for tracking. |
| `max_objects` | `int32` | `0` = inspect all found objects of that type |
| `return_home` | `bool` | `true` = camera resets to center after done |

### Response (Jetson → BT)

| Field | Type | What BT receives |
|---|---|---|
| `success` | `bool` | `true` if ≥1 object was captured |
| `status` | `string` | `"ok"` / `"no_detection"` / `"ibvs_timeout"` / `"all_in_back"` / `"no_frames"` / `"busy"` |
| `objects_found` | `int32` | Objects spotted by Insta360 wide camera |
| `objects_inspected` | `int32` | Objects successfully focused + captured by Logitech |
| `object_in_back` | `bool` | If `true` → rotate robot 180° and retry |
| `image_paths` | `string[]` | Absolute paths to saved images on Jetson disk. Pass these to the upload service. |
| `info` | `string` | Human-readable summary |

### Status Values

| `status` | What BT should do |
|---|---|
| `"ok"` | Proceed to upload |
| `"no_detection"` | Nothing found → navigate to next waypoint |
| `"ibvs_timeout"` | IBVS didn't converge → move robot closer and retry |
| `"all_in_back"` | Object is behind robot → rotate 180° and retry |
| `"no_frames"` | Camera failure — check hardware |
| `"busy"` | Another inspection running — wait and retry |

---

## 4. Service 2 — Upload Images to Laptop

**Service**: `/visual_inspection/upload_images`  
**Type**: `visual_inspection_interfaces/srv/UploadImages`

The Jetson reads each file from the paths the BT gives it, then HTTP-POSTs them to your laptop.

### Request (BT → Jetson)

| Field | Type | What to send |
|---|---|---|
| `image_paths` | `string[]` | The `image_paths` received from the inspect response |
| `session_label` | `string` | Same `location_label` used in inspect — used as top folder on laptop |

### Response (Jetson → BT)

| Field | Type | What BT receives |
|---|---|---|
| `success` | `bool` | `true` if all files uploaded to laptop |
| `info` | `string` | Human-readable result or error message |
| `uploaded_count` | `int32` | How many files were sent |

### About location_label / session tracking

The `session_label` you send here (same as `location_label` in inspect) becomes the top-level folder on the laptop:

```
received_captures/
└── engine_room_A/         ← your location_label / session_label
    └── gauge/
        └── instance_1/
            ├── img_01_conf0.85.jpg
            └── metadata.json   ← contains: confidence, ibvs_time_s, final_error_px, etc.
```

This is how you track which images came from which location.

---

## 5. Target Object Classes

| `target_object` | Mode | Pipeline |
|---|---|---|
| `"fire_extinguisher"` | **Full IBVS** | Insta360 detect → coarse pan-tilt → Logitech IBVS → 3 images |
| `"extinguisher"` | **Full IBVS** | Same (alias) |
| `"door"` | **Full IBVS** | Same |
| `"person"` | **Full IBVS** | Same |
| `"gauge"` | **Full IBVS + Sweep** | Same + serpentine sweep if Insta360 misses |
| `"unknown"` | **Overview only** | Servo home → Insta360 raw + 1 Logitech image |
| `"main_cylinder"` | **Overview only** | Same |

### Confidence Thresholds (per class)

| Class | Threshold |
|---|---|
| `fire_extinguisher` | > 0.5 |
| `door` | > 0.5 |
| `person` | > 0.6 |
| `gauge` | > 0.3 (visually weaker) |

---

## 6. metadata.json — What is inside each captured folder

Every `instance_N/` folder contains a `metadata.json` alongside the images:

```json
{
  "class": "gauge",
  "instance_id": 1,
  "confidence": 0.8887,
  "session": "20260424_120000",
  "location_label": "engine_room_A",
  "num_images": 3,
  "camera": "logitech",
  "ibvs_converged": true,
  "ibvs_error_px": 7.62,
  "ibvs_time_s": 4.626,
  "ibvs_iterations": 45,
  "ibvs_fps": 9.7,
  "coarse_time_s": 2.004,
  "initial_ibvs_error_px": 203.06,
  "pipeline_time_s": 18.4
}
```

---

## 7. C++ BehaviorTree Node Examples (BehaviorTree.CPP)

### Inspect Node

```cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "visual_inspection_interfaces/srv/inspect.hpp"

class InspectObjectNode : public BT::CoroActionNode
{
public:
    InspectObjectNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config)
    {
        node_   = rclcpp::Node::make_shared("bt_inspect_client");
        client_ = node_->create_client<visual_inspection_interfaces::srv::Inspect>(
                      "/visual_inspection/inspect");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("target_object"),
            BT::InputPort<std::string>("location_label"),
            BT::OutputPort<std::vector<std::string>>("image_paths")  // pass to upload node
        };
    }

    BT::NodeStatus tick() override
    {
        std::string target_obj, location;
        getInput("target_object",  target_obj);
        getInput("location_label", location);

        auto req = std::make_shared<visual_inspection_interfaces::srv::Inspect::Request>();
        req->target_object  = target_obj;
        req->location_label = location;
        req->max_objects    = 0;
        req->return_home    = true;

        auto future = client_->async_send_request(req);
        while (rclcpp::ok() &&
               future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            setStatusRunningAndYield();
        }

        auto res = future.get();
        if (res->success) {
            setOutput("image_paths", res->image_paths);  // forward to upload node
            return BT::NodeStatus::SUCCESS;
        } else if (res->object_in_back) {
            // BT should rotate robot 180° and retry
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<visual_inspection_interfaces::srv::Inspect>::SharedPtr client_;
};
```

### Upload Node

```cpp
#include "visual_inspection_interfaces/srv/upload_images.hpp"

class UploadImagesNode : public BT::CoroActionNode
{
public:
    UploadImagesNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config)
    {
        node_   = rclcpp::Node::make_shared("bt_upload_client");
        client_ = node_->create_client<visual_inspection_interfaces::srv::UploadImages>(
                      "/visual_inspection/upload_images");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<std::string>>("image_paths"),  // from inspect node
            BT::InputPort<std::string>("session_label")
        };
    }

    BT::NodeStatus tick() override
    {
        std::vector<std::string> paths;
        std::string label;
        getInput("image_paths",  paths);
        getInput("session_label", label);

        auto req = std::make_shared<visual_inspection_interfaces::srv::UploadImages::Request>();
        req->image_paths   = paths;
        req->session_label = label;

        auto future = client_->async_send_request(req);
        while (rclcpp::ok() &&
               future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            setStatusRunningAndYield();
        }

        auto res = future.get();
        return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<visual_inspection_interfaces::srv::UploadImages>::SharedPtr client_;
};
```

---

## 8. BT XML Example (Navigate → Inspect → Upload)

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="InspectAndUpload">

      <!-- 1. Nav2 drives robot to waypoint -->
      <NavigateToPose pose="engine_room_A_waypoint" />

      <!-- 2. Visual inspection: saves images to Jetson disk, returns paths -->
      <InspectObjectNode
          target_object="gauge"
          location_label="engine_room_A"
          image_paths="{captured_paths}" />

      <!-- 3. Upload those images to laptop via HTTP -->
      <RetryUntilSuccessful num_attempts="3">
        <UploadImagesNode
            image_paths="{captured_paths}"
            session_label="engine_room_A" />
      </RetryUntilSuccessful>

    </Sequence>
  </BehaviorTree>
</root>
```

> `{captured_paths}` is a BT Blackboard variable — the inspect node writes to it, the upload node reads from it.

---

## 9. Running Everything

### On Jetson (4 terminals)

```bash
# ── Paste in EVERY terminal ──────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# Terminal 1 — Cameras
ros2 run visual_inspection_ros camera_node

# Terminal 2 — Servo controller
ros2 run visual_inspection_ros servo_node

# Terminal 3 — Inspection service (main pipeline)
ros2 run visual_inspection_ros inspection_service

# Terminal 4 — Image uploader (sends to laptop after inspection)
ros2 run visual_inspection_ros image_uploader
# Laptop IP is already set to 192.168.8.244:8888
# Override if needed: --ros-args -p laptop_url:=http://<NEW_IP>:8888/upload
```

### On Laptop (1 terminal, connected to YasiruDEX WiFi)

```bash
pip install flask   # only needed once
cd /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws
python3 laptop_receiver.py
# Starts listening on port 8888
# Saves received files to: received_captures/<location_label>/<object>/<instance>/
```

---

## 10. Testing — Simulate the Full BT Flow

Run this **instead of** `test_inspection_service.py` when you want to test the full pipeline including HTTP upload:

```bash
# On Jetson — after all 4 services are running:
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# Test gauge inspection + upload to laptop
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_bt_full_flow.py \
    --object gauge --location engine_room_A

# Test fire extinguisher
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_bt_full_flow.py \
    --object fire_extinguisher --location corridor_B

# What you should see:
# ── STEP 1: Visual Inspection ──
#   ✓ Inspection complete — image_paths=[...]
# ── STEP 2: Upload Images to Laptop ──
#   ✓ Upload complete — files at received_captures/engine_room_A/...
# ── BT RESULT ──
#   ✓ BT node → SUCCESS
```

### Quick single-step tests (inspection only, no upload)

```bash
# Inspect only — does NOT upload to laptop
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object gauge
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object fire_extinguisher
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object door
```

---

## 11. Monitoring Topics (Debugging only)

| Topic | Type | Description |
|---|---|---|
| `/visual_inspection/status` | `String` | `IDLE` / `DETECTING` / `COARSE` / `IBVS` / `CAPTURING` / `SWEEP` |
| `/visual_inspection/debug` | `Image` | Side-by-side Insta360 + Logitech (view in RViz2) |
| `/visual_inspection/ibvs_error` | `Point` | x/y pixel error during IBVS |
| `/visual_inspection/detections` | `String` | JSON of current detected objects |
| `/servo/pan_tilt` | `Int16MultiArray` | `[tilt, pan]` servo angles |

> Skip these topics during real deployment to save CPU/GPU resources.

---

## 12. File Locations

| File | Purpose |
|---|---|
| `visual_inspection_interfaces/srv/Inspect.srv` | Service 1 definition |
| `visual_inspection_interfaces/srv/UploadImages.srv` | Service 2 definition |
| `visual_inspection_ros/inspection_service.py` | Service 1 server (Jetson) |
| `visual_inspection_ros/image_uploader.py` | Service 2 server (Jetson) |
| `Evaluation_V_I_ws/laptop_receiver.py` | HTTP receiver (runs on laptop) |
| `test_scripts/test_bt_full_flow.py` | Full BT flow test (inspect + upload) |
| `test_scripts/test_inspection_service.py` | Inspect-only test |
