# Jetson Orin Nano — IBVS Visual Servoing System

## 📁 Workspace Structure

```
jetson_workspace/
├── ibvs_pipeline.py              # Main IBVS pipeline
├── run.sh                        # ← USE THIS to start/stop the pipeline
├── test_calibration_live.py      # Calibration validation script
├── calibration_config.py         # Insta360 calibration math
├── requirements.txt              # Python dependencies
├── config/
│   └── logitech_intrinsics.yaml  # Logitech C920 calibration
├── weights/
│   ├── yolo11n.pt                # YOLO PyTorch weights
│   └── yolo11n.engine            # TensorRT engine (generated on Jetson)
├── arduino/
│   └── pan_tilt_control.ino      # Arduino servo control sketch
└── docs/
    ├── OPTIMIZATION_PLAN.md      # Performance optimization roadmap
    ├── IBVS_TUNING_GUIDE.md      # PID tuning guide
    └── IBVS_SYSTEM_SUMMARY.md    # System overview
```

---

## 🚀 How to Run the Pipeline

### ⚡ Always use `run.sh` — it handles everything automatically

```bash
cd ~/Documents/Visual_Inspection_ws
source venv/bin/activate

# Make it executable (one time only)
chmod +x run.sh
```

---

## 🖥️ Headed vs Headless Mode

| Feature | Headed Mode | Headless Mode |
|---------|-------------|---------------|
| Camera window | ✅ Visible (via VNC) | ❌ No window |
| FPS in terminal | ✅ Every second | ✅ Every second |
| cv2.imshow | ✅ Yes | ❌ No |
| Speed | Slightly slower (drawing cost) | 🔥 Maximum speed |
| When to use | Debugging, tuning | Production, deployment |
| Display needed | Yes (VNC or monitor) | No display needed |

---

## ▶️ Run Commands

### Headless Mode (recommended for production)
```bash
# Start
./run.sh headless

# OR manually:
pkill -f ibvs_pipeline.py && sleep 2
python3 ibvs_pipeline.py --headless
```

**Terminal output every second:**
```
[FPS] 30.0 fps  |  State: COARSE  |  Pan=90°  Tilt=90°
[FPS] 29.9 fps  |  State: FINE    |  Pan=95°  Tilt=88°
✓ CENTERED after 18 iterations (error=6.8px)
```

---

### Headed Mode (use for debugging via VNC)
```bash
# Start
./run.sh

# OR manually:
pkill -f ibvs_pipeline.py && sleep 2
DISPLAY=:1 python3 ibvs_pipeline.py
```

Shows a live camera window with both Insta360 and Logitech feeds side by side.
Press `q` in the window to quit, or `Ctrl+C` in terminal.

---

## 🛑 Stop the Pipeline

```bash
# Method 1 — cleanest (sends proper shutdown signal)
Ctrl+C

# Method 2 — if stuck in background or unresponsive
pkill -f ibvs_pipeline.py

# Method 2a — force kill if pkill doesn't work
pkill -9 -f ibvs_pipeline.py
sleep 2    # wait for cameras to fully release
```

> ⚠️ **IMPORTANT:** Always kill the pipeline before starting a new instance.
> If cameras are busy, wait 2-3 seconds after killing before restarting.

---

## 📊 Performance (after optimizations)

| Metric | Before | After |
|--------|--------|-------|
| FPS (COARSE) | ~3 | **~30 FPS** |
| FPS (FINE/IBVS) | ~3 | **~5-15 FPS** (has servo wait) |
| YOLO backend | PyTorch CPU | **TensorRT GPU FP16** |
| Camera format | YUYV | **MJPEG** (less USB bandwidth) |

> **Why FINE mode is slower?** In IBVS mode, the pipeline waits for the servo
> to physically move before reading the next frame. This is correct behavior —
> reading without waiting would give stale position data.

---

## 🔧 One-Time Setup (Jetson)

### Install udev rules (permanent device symlinks)
Cameras and Arduino get **fixed paths** regardless of USB device order:

```bash
sudo tee /etc/udev/rules.d/99-visual-inspection-cameras.rules << 'EOF'
# Logitech C920 — VID=046d PID=08e5
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="08e5", ATTR{index}=="0", SYMLINK+="logitech"

# Insta360 X3 — VID=2e1a PID=00c1
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2e1a", ATTRS{idProduct}=="00c1", ATTR{index}=="0", SYMLINK+="insta360"

# Arduino Uno — VID=2341
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="arduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="arduino"
EOF

sudo udevadm control --reload-rules && sudo udevadm trigger
sleep 2
ls -la /dev/insta360 /dev/logitech /dev/arduino
```

After this, plugging/unplugging anything (joystick, hub, etc.) will never break detection.

---

### Generate TensorRT Engine (one-time, ~7 min)
```bash
# Link system TensorRT to venv (if not done already)
echo "/usr/lib/python3/dist-packages" >> venv/lib/python3.10/site-packages/system_packages.pth
echo "/usr/lib/python3.10/dist-packages" >> venv/lib/python3.10/site-packages/system_packages.pth

# Export
python3 -c "
from ultralytics import YOLO
model = YOLO('weights/yolo11n.pt')
model.export(format='engine', device=0, half=True, imgsz=640, workspace=4)
print('Done! weights/yolo11n.engine ready')
"
```

> The `.engine` file is Jetson-specific. Do NOT copy it to other machines.

---

## 🐛 Troubleshooting

### "Camera not found" / "can't open camera by index"
```bash
# 1. Kill any stuck instances
pkill -9 -f ibvs_pipeline.py && sleep 3

# 2. Check cameras are detected
ls -la /dev/insta360 /dev/logitech

# 3. Check what video devices exist
v4l2-ctl --list-devices

# 4. Restart
./run.sh headless
```

### Arduino not connecting
```bash
# Check what's detected
ls /dev/ttyACM* /dev/arduino

# Fix permissions
sudo chmod 666 /dev/ttyACM0
# OR use udev rules (see One-Time Setup above)
```

### Headed mode — window not showing
```bash
# Set display correctly for VNC
DISPLAY=:1 python3 ibvs_pipeline.py

# OR use run.sh which sets DISPLAY automatically
./run.sh
```

### FPS is low (< 15)
```bash
# Enable max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Verify TensorRT engine is loaded (check startup output):
# ✅ TensorRT GPU inference active (FP16)   ← good
# 🔍 TensorRT engine not found              ← regenerate engine
```

---

## 📝 Update Pipeline from Laptop

```bash
# On LAPTOP:
scp /home/dinethra/Jetson_orin_nano/jetson_workspace/ibvs_pipeline.py \
    rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/

# On Jetson — restart to pick up changes:
./run.sh headless
```

---

## 🔄 Optimization Progress

| Phase | Status | Impact |
|-------|--------|--------|
| ✅ Phase 1 — TensorRT FP16 | Complete | 3 → 30 FPS |
| ✅ Phase 2 — MJPEG camera mode | Complete | Lower USB bandwidth |
| ⏳ Phase 3 — Arduino C++ PID | Planned | Faster servo response in FINE |
| ⏳ Phase 4 — IBVS Y-axis fix | Planned | Fix bounding box drift |

See `docs/OPTIMIZATION_PLAN.md` for full details.

---

**GitHub:** [DinethraDivanjana2001/Visual_Inspection_system_Jetson_orin_nano](https://github.com/DinethraDivanjana2001/Visual_Inspection_system_Jetson_orin_nano)
