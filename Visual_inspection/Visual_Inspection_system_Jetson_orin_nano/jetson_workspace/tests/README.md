# Test Scripts - README

## Overview

This folder contains test scripts to verify each component of the IBVS system works correctly on Jetson Orin Nano.

## Test Scripts

### 01_check_cameras.py
**Purpose:** Detect and list all video devices
**Usage:**
```bash
python3 tests/01_check_cameras.py
```
**What it does:**
- Scans `/sys/class/video4linux/` for video devices
- Lists device names and indices
- Shows which devices belong to which camera

**Expected output:**
```
📹 /dev/video0
   Name: Insta360 X3: Insta360 X3

📹 /dev/video2
   Name: HD Pro Webcam C920
```

---

### 02_test_camera_feeds.py
**Purpose:** Open and display camera feeds
**Usage:**
```bash
python3 tests/02_test_camera_feeds.py
```
**What it does:**
- Finds Insta360 and Logitech cameras
- Opens both cameras
- Displays live feeds in separate windows
- Shows frame counter

**Controls:**
- Press `q` to quit
- Press `s` to save snapshots

**Expected behavior:**
- Two windows appear showing live video
- Frame counter updates
- No lag or freezing

---

### 03_test_arduino.py
**Purpose:** Test Arduino connection and servo control
**Usage:**
```bash
python3 tests/03_test_arduino.py
```
**What it does:**
- Searches for Arduino on common ports
- Opens serial connection
- Sends test servo commands
- Moves servos through predefined positions

**Expected behavior:**
- Arduino found at `/dev/ttyACM0`
- Servos move: Center → Left → Right → Up → Down → Center
- Each movement takes ~1 second

**Troubleshooting:**
```bash
# If permission denied:
sudo chmod 666 /dev/ttyACM0

# Or permanently:
sudo usermod -aG dialout $USER
# Then logout/login
```

---

### 04_test_yolo.py
**Purpose:** Test YOLO object detection on camera feeds
**Usage:**
```bash
python3 tests/04_test_yolo.py
```
**What it does:**
- Loads YOLO model
- Opens both cameras
- Runs object detection on both feeds
- Draws bounding boxes around detected objects

**Controls:**
- Press `q` to quit

**Expected behavior:**
- YOLO model loads successfully
- Green boxes appear around detected objects
- Labels show class name and confidence
- Frame counter updates

**Performance:**
- Should run at 20-30 FPS on Jetson Orin Nano
- Detection latency: 30-50ms per frame

---

## Running All Tests

### Quick Test Sequence

```bash
cd ~/jetson_workspace
source venv/bin/activate

# 1. Check cameras are detected
python3 tests/01_check_cameras.py

# 2. Test camera feeds open correctly
python3 tests/02_test_camera_feeds.py

# 3. Test Arduino connection
python3 tests/03_test_arduino.py

# 4. Test YOLO detection
python3 tests/04_test_yolo.py
```

### Automated Test Script

```bash
#!/bin/bash
# Run all tests in sequence

echo "Running test suite..."

python3 tests/01_check_cameras.py
if [ $? -ne 0 ]; then
    echo "❌ Camera detection failed"
    exit 1
fi

python3 tests/03_test_arduino.py
if [ $? -ne 0 ]; then
    echo "❌ Arduino test failed"
    exit 1
fi

echo "✅ All automated tests passed!"
echo "Run manual tests:"
echo "  python3 tests/02_test_camera_feeds.py"
echo "  python3 tests/04_test_yolo.py"
```

---

## Troubleshooting

### Camera Issues

**Problem:** No cameras detected
```bash
# Check USB connections
lsusb

# Check video devices
ls -l /dev/video*

# Reload camera driver
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```

**Problem:** Permission denied
```bash
sudo usermod -aG video $USER
# Logout and login
```

**Problem:** Camera opens but no frames
```bash
# Try different index
python3 -c "import cv2; cap=cv2.VideoCapture(1); print(cap.read())"
```

---

### Arduino Issues

**Problem:** Arduino not found
```bash
# Check if connected
ls /dev/ttyACM*

# Check dmesg
dmesg | grep tty

# Try different port
python3 -c "import serial; serial.Serial('/dev/ttyUSB0', 9600)"
```

**Problem:** Permission denied
```bash
# Quick fix
sudo chmod 666 /dev/ttyACM0

# Permanent fix
sudo usermod -aG dialout $USER
# Logout and login
```

---

### YOLO Issues

**Problem:** Model not found
```bash
# Check if file exists
ls -lh weights/yolo11n.pt

# Download if missing
cd weights
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
```

**Problem:** Out of memory
```bash
# Check memory
free -h

# Increase swap
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**Problem:** Slow inference
```bash
# Set max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Check GPU usage
sudo tegrastats
```

---

## Expected Results Summary

| Test | Expected Result | Time |
|------|----------------|------|
| 01_check_cameras.py | Lists 4-6 video devices | <1s |
| 02_test_camera_feeds.py | Two windows, live video | Manual |
| 03_test_arduino.py | Servos move through 6 positions | ~6s |
| 04_test_yolo.py | Detections with bounding boxes | Manual |

---

## Next Steps

After all tests pass:

1. Run calibration test:
   ```bash
   python3 test_calibration_live.py
   ```

2. Run full IBVS pipeline:
   ```bash
   python3 ibvs_pipeline.py
   ```

3. Tune parameters if needed (see `docs/IBVS_TUNING_GUIDE.md`)

---

**All tests passing? You're ready to run the full system! 🎯**
