# Visual Inspection System

Research-grade visual inspection system using YOLOv11 + TensorRT on Jetson Orin Nano with panoramic detection and pan-tilt-zoom control.

## Project Structure

```
Jetson_orin_nano/
├── visual_inspection_ws/     # ROS 2 workspace (for future integration)
│   └── src/
├── test_scripts/             # Step-by-step test scripts
│   ├── 00_check_devices.sh   # Detect cameras and Arduino
│   ├── 01_test_cameras.py    # Test camera feeds
│   ├── 02_yolo_cpu.py        # YOLO detection (CPU)
│   ├── 03_yolo_tensorrt.py   # YOLO detection (TensorRT)
│   └── ...
├── weights/                  # YOLO model weights
│   ├── README.md
│   └── yolo11n.pt           # (Place your weight file here)
├── config/                   # Configuration files
│   └── camera_calibration.yaml
└── STEP_BY_STEP_GUIDE.md    # Development guide
```

## System Architecture

**Two-Stage Coarse-to-Fine Active Vision:**

1. **Coarse Localization**: Insta360 dual 180° view → YOLOv11n-TensorRT → Hemisphere mapping → Pan-tilt rough positioning
2. **Fine Refinement**: Logitech C920 → IBVS (Image-Based Visual Servoing) → Centered high-res ROI capture

## Hardware Requirements

- **Jetson Orin Nano** (or Linux laptop for development)
- **Insta360 Camera** (USB, dual 180° view output)
- **Logitech C920** webcam (on pan-tilt mechanism)
- **Arduino** (servo control via USB serial)
- **Pan-Tilt Mechanism** (servos on pins 9 & 10)

## Quick Start - Step by Step

### Step 0: Check Connected Devices

First, identify all your devices:

```bash
cd /home/dinethra/Jetson_orin_nano

# Run device detection script
./test_scripts/00_check_devices.sh
```

This will show you:
- Which cameras are `/dev/video0`, `/dev/video1`, etc.
- Where Arduino is connected (`/dev/ttyUSB0` or `/dev/ttyACM0`)
- Available camera resolutions

**Update `config/camera_calibration.yaml`** with the correct device indices.

---

### Step 1: Test Camera Feeds

Open cameras and verify feeds:

```bash
# Test both cameras simultaneously
python3 test_scripts/01_test_cameras.py --insta 0 --logitech 1

# Or test individually
python3 test_scripts/01_test_cameras.py --mode insta --insta 0
python3 test_scripts/01_test_cameras.py --mode logitech --logitech 1
```

**What you should see:**
- **Insta360**: Dual 180° view (top half = front, bottom half = back) with yellow separator line
- **Logitech**: Normal webcam view
- Real-time FPS display
- Resolution information

**Controls:**
- `q` - Quit
- `s` - Save screenshot

---

### Step 2: YOLO Detection (CPU Mode)

Place your `yolo11n.pt` weight file in the `weights/` directory, then:

```bash
# Coming next: YOLO detection on camera feed
python3 test_scripts/02_yolo_cpu.py
```

This will show:
- Live detection bounding boxes
- Class labels (fire_extinguisher, door, gauges)
- Confidence scores
- FPS (CPU inference speed)

---

### Step 3: TensorRT Acceleration

Export model to TensorRT and compare performance:

```bash
# Coming next: Export to TensorRT engine
python3 test_scripts/03_export_tensorrt.py

# Run accelerated detection
python3 test_scripts/04_yolo_tensorrt.py
```

You'll see significant FPS improvement (typically 3-5x faster).

---

### Step 4 and Beyond

- Hemisphere-to-PTZ mapping
- Arduino serial communication
- IBVS refinement loop
- Full integration

Follow **`STEP_BY_STEP_GUIDE.md`** for complete progression.

---

## Dependencies

### Python Packages (Offline Installation Available)

```bash
pip install opencv-python numpy pyyaml pyserial ultralytics
```

For offline installation, use pre-downloaded wheel files (instructions in guide).

### System Requirements

- Python 3.8+
- OpenCV 4.x
- CUDA 11.x (for TensorRT on Jetson)
- ROS 2 Humble (optional, for service integration)

---

## Configuration

All configuration is in `config/camera_calibration.yaml`:

- Camera device indices
- Resolutions
- Logitech intrinsic calibration parameters
- Arduino serial port
- Servo limits
- PTZ mapping offsets

Update these values as you progress through the steps.

---

## Detected Object Classes

Currently trained to detect:
- `fire_extinguisher`
- `door`
- `gauges`

Additional classes can be added by retraining the model.

---

## Research Background

This system implements:
- **Image-Based Visual Servoing (IBVS)** - Chaumette & Hutchinson, 2006
- **Panoramic-to-PTZ Mapping** - Virtual PTZ architectures
- **Coarse-to-Fine Active Vision** - Wide FOV search + narrow FOV inspection

The dual-hemisphere mapping approach handles vendor-processed Insta360 output without requiring raw fisheye calibration.

---

## Troubleshooting

### Camera not found
```bash
# Check if camera is detected
v4l2-ctl --list-devices

# Try different index
python3 test_scripts/01_test_cameras.py --insta 2
```

### Arduino not connecting
```bash
# Check available ports
ls /dev/ttyUSB* /dev/ttyACM*

# Give yourself permission
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### Low FPS
- Ensure TensorRT is properly installed
- Check if using CUDA-enabled OpenCV
- Reduce camera resolution if needed

---

## Next Steps

1. ✅ Run device detection script
2. ✅ Test camera feeds
3. ⏳ Add YOLO weights and test detection
4. ⏳ Export to TensorRT
5. ⏳ Implement hemisphere mapping
6. ⏳ Integrate Arduino control
7. ⏳ Build IBVS refinement loop
8. ⏳ Create ROS 2 nodes (optional)

---

## License & Citation

Research project for visual inspection using active camera control.

**Key References:**
- Chaumette & Hutchinson (2006) - Visual Servo Control
- Ultralytics YOLOv11 - https://docs.ultralytics.com

---

## Contact

For questions or issues, refer to the detailed guides in:
- `STEP_BY_STEP_GUIDE.md` - Development progression
- `implementation_plan.md` - Technical architecture details
