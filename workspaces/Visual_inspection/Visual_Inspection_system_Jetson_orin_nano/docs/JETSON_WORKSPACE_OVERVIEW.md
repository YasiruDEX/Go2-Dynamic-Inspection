# Jetson Workspace Overview
*Last updated: 2026-04-17*

---

## Connection Details
- **Jetson IP**: `rgen@192.168.8.181`
- **SSH**: `ssh rgen@192.168.8.181`
- **SCP**: `sshpass -p 'ffddffdd' scp <file> rgen@192.168.8.181:<dest>`
- **Jetson user home**: `/home/rgen/`

---

## Laptop (Dev Machine) → Jetson Folder Mapping

| Laptop (`/home/dinethra/Jetson_orin_nano/`) | Jetson (`/home/rgen/Documents/Visual_Inspection_ws/`) |
|---|---|
| `inspection_ws/` | `inspection_ws/` (ROS2 workspace — the active one) |
| `jetson_workspace/` | Root of `Visual_Inspection_ws/` (standalone scripts) |
| `weights/` | `weights/` |
| `tools/` | `tools/` |
| `test_scripts/` | `test_scripts/` |
| `tests/` | `tests/` |

---

## Jetson Side Structure (`~/Documents/Visual_Inspection_ws/`)

### 🔴 ROOT (Standalone — older style, still used)
```
Visual_Inspection_ws/
├── ibvs_pipeline.py          ← Main IBVS standalone script
├── ibvs_headed.py            ← With display
├── ibvs_headless.py          ← No display (production)
├── calibration_config.py     ← Insta360 pan/tilt calibration polynomials
├── test_calibration_live.py  ← Live calibration test
├── export_onnx.py            ← Export .pt → .onnx
├── export_trt.py             ← Export .onnx → .engine
├── run.sh                    ← Quick run script
├── QUICK_REFERENCE.md        ← Command cheatsheet
├── SETUP.md                  ← Full setup guide
├── requirements.txt
├── venv/                     ← Python virtualenv (activate before running standalone)
├── config/
│   └── logitech_intrinsics.yaml
├── tests/                    ← Test scripts (standalone)
├── test_scripts/             ← More test scripts
├── tools/                    ← Utility scripts
├── captures/                 ← Captured images
├── docs/                     ← Documentation
├── evaluation/               ← Evaluation scripts
├── yolo_dataset/             ← Training dataset
└── arduino/                  ← Arduino firmware
```

### 🟢 WEIGHTS (shared by both systems)
```
weights/
├── yolo11n.pt                ← YOLO11n original model
├── yolo11n.onnx              ← YOLO11n ONNX export
├── yolo11n.engine            ← YOLO11n TensorRT engine ✅ (in use by ROS2)
├── yolov26s.pt               ← NEW custom trained model (just transferred)
├── yolov26s.onnx             ← ONNX export of new model
└── yolov26s.engine           ← TensorRT engine of new model ✅
```

### 🔵 INSPECTION_WS (ROS2 — the active/clean workspace)
```
inspection_ws/
├── visual_inspection_ros/    ← Main ROS2 package
│   ├── ibvs_action_server.py ← Main pipeline (IBVS + YOLO + MQTT)
│   ├── camera_node/          ← Camera publishers
│   ├── servo_node/           ← Servo control
│   └── action/               ← ROS2 action definitions
├── visual_inspection_interfaces/  ← Custom ROS2 message types
├── build/                    ← colcon build output
├── install/                  ← colcon install output
├── log/                      ← ROS2 logs
├── pipeline/                 ← Pipeline config
├── mqtt/                     ← MQTT config
├── behaviour_tree/           ← BT nodes
├── docs/                     ← ROS2 workspace docs
└── tests/                    ← ROS2 tests
```

---

## Engine Path Used in ROS2 Pipeline
```python
# ibvs_action_server.py line 100:
ENGINE_PATH = '~/Documents/Visual_Inspection_ws/weights/yolo11n.engine'
```
To switch to new model, change to `weights/yolov26s.engine`

---

## How to Run on Jetson

### Standalone (no ROS2)
```bash
cd ~/Documents/Visual_Inspection_ws
source venv/bin/activate
python3 ibvs_pipeline.py
```

### ROS2 Pipeline
```bash
# Terminal 1 — cameras
ros2 run visual_inspection_ros camera_node

# Terminal 2 — servos
ros2 run visual_inspection_ros servo_node

# Terminal 3 — main IBVS action server
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server
```

### Rebuild ROS2 after code changes
```bash
cd ~/Documents/Visual_Inspection_ws/inspection_ws
colcon build --packages-select visual_inspection_ros
source install/setup.bash
```

---

## Sync from Laptop to Jetson
```bash
# Send a specific file
sshpass -p 'ffddffdd' scp <local_file> rgen@192.168.8.181:<jetson_path>

# Example: sync ibvs_action_server.py
sshpass -p 'ffddffdd' scp \
  /home/dinethra/Jetson_orin_nano/inspection_ws/visual_inspection_ros/visual_inspection_ros/ibvs_action_server.py \
  rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/inspection_ws/visual_inspection_ros/visual_inspection_ros/ibvs_action_server.py
```
