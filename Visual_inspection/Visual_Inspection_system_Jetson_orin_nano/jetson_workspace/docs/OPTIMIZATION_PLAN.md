# IBVS Pipeline Optimization Plan
## Visual Inspection System — Jetson Orin Nano

---

## Current Status

| Metric | Current | Target |
|--------|---------|--------|
| Pipeline FPS | ~3 FPS | ~25+ FPS |
| YOLO backend | PyTorch (CPU) | TensorRT (GPU, FP16) |
| Camera format | YUYV (uncompressed) | MJPEG (compressed → faster) |
| PID location | Python on Jetson | C++ on Arduino |
| IBVS Y-axis error | Bounding box drifts down | Corrected with `cy` offset |

---

## 🔧 Pre-Steps (Run Once Before Any Phase)

### Enable Jetson Max Performance Mode
Run these on the **Jetson** before any testing:

```bash
# Set maximum power mode (all CPU + GPU cores ON)
sudo nvpmodel -m 0

# Set all cores to maximum frequency
sudo jetson_clocks

# Verify power mode
sudo nvpmodel -q

# Optional: Install jtop to monitor GPU/CPU/RAM live
sudo pip install jetson-stats
sudo reboot
jtop   # run after reboot
```

> ⚠️ These settings reset on reboot. Add to `/etc/rc.local` for permanent effect.

---

## Phase 1 — TensorRT Acceleration (HIGHEST IMPACT)

### Why
- YOLO currently runs on **CPU** via PyTorch → very slow on Jetson
- TensorRT runs on **GPU** → 3-5x faster on Jetson Orin Nano
- Expected: **3 FPS → 15-25 FPS** (pipeline total)

### Benchmark Reference (Ultralytics Official — Orin Nano Super, similar to Orin Nano)

| Format | Inference Time | Speed vs PyTorch |
|--------|---------------|-----------------|
| PyTorch | 13.70 ms/img | 1x (baseline) |
| TorchScript | 13.69 ms/img | ~1x |
| ONNX | 14.47 ms/img | ~1x |
| **TensorRT FP32** | **7.44 ms/img** | **1.8x** |
| **TensorRT FP16** | **4.53 ms/img** | **3x** |
| **TensorRT INT8** | **3.70 ms/img** | **3.7x** |

### Step 1.1 — Check JetPack Version

```bash
# On Jetson
cat /etc/nv_tegra_release
# OR
dpkg -l | grep nvidia-jetpack
```

Expected output: `JetPack 6.x` (Ubuntu 22.04, Python 3.10)

### Step 1.2 — Install Required Packages

```bash
cd ~/Documents/Visual_Inspection_ws
source venv/bin/activate

# Install ultralytics with export dependencies
pip install "ultralytics[export]"

# Install correct PyTorch wheel for JetPack 6.1 + ARM64
# (The default pip torch doesn't support Jetson GPU properly)
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl

# Install cuSPARSELt (fixes torch dependency issue)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev

# Install onnxruntime-gpu for ARM64 (JetPack 6, Python 3.10)
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl
```

### Step 1.3 — Export YOLO to TensorRT

```bash
cd ~/Documents/Visual_Inspection_ws
source venv/bin/activate

python3 -c "
from ultralytics import YOLO

model = YOLO('weights/yolo11n.pt')

# Export with FP16 (best speed/accuracy balance for Orin Nano)
model.export(
    format='engine',
    device=0,        # GPU 0
    half=True,       # FP16 — best for Orin Nano
    imgsz=640,
    workspace=4      # GB of GPU memory to use
)
print('TensorRT engine exported to weights/yolo11n.engine')
"
```

> ⏱️ This takes **10-20 minutes** on first run (TensorRT compilation).
> The `.engine` file is Jetson-specific — do NOT copy it to other machines.

### Step 1.4 — Verify TensorRT Engine Works

```bash
python3 -c "
from ultralytics import YOLO
import time

model = YOLO('weights/yolo11n.engine')

# Warmup
for _ in range(3):
    model('boats.jpg', verbose=False)

# Benchmark
start = time.time()
for _ in range(30):
    model('boats.jpg', verbose=False)
elapsed = time.time() - start

print(f'TensorRT FPS: {30/elapsed:.1f}')
print(f'Per frame: {elapsed/30*1000:.1f} ms')
"
```

### Step 1.5 — Update Pipeline to Use TensorRT

In `jetson_workspace/ibvs_pipeline.py`, change:
```python
# Change this line in Config class:
YOLO_MODEL = "weights/yolo11n.pt"
# TO:
YOLO_MODEL = "weights/yolo11n.engine"
```

The pipeline automatically uses GPU when `.engine` is loaded — no other changes needed.

### Step 1.6 — Test and Push

```bash
# On Jetson
python3 ibvs_pipeline.py --headless
# Verify: [FPS] should now show 15-25+

# On laptop — push to GitHub
cd /home/dinethra/Jetson_orin_nano
git add jetson_workspace/ibvs_pipeline.py
git commit -m "Phase 1: Switch to TensorRT FP16 engine for YOLO inference"
git push
```

---

## Phase 2 — Camera Optimization (MJPEG + Threading)

### Why
- Default camera format is **YUYV** (raw uncompressed): 640×480 × 2 bytes = 614KB per frame USB transfer
- **MJPEG** mode: ~30-50KB per frame (10x less USB bandwidth)
- **Threading**: camera reads happen in background, pipeline never waits for camera

### Expected Improvement: +3-8 FPS

### Step 2.1 — Add MJPEG Mode to Camera Open

In `jetson_workspace/ibvs_pipeline.py`, update camera initialization:

```python
def open_camera_mjpeg(idx, width, height):
    """Open camera in MJPEG mode for faster USB transfer"""
    cap = cv2.VideoCapture(idx)
    # Force MJPEG format (much faster than YUYV)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimum buffer = always fresh frame
    return cap
```

### Step 2.2 — Add Camera Thread Class

```python
import threading

class CameraReader:
    """Non-blocking camera reader — always has latest frame ready"""
    def __init__(self, idx, width, height):
        self.cap = open_camera_mjpeg(idx, width, height)
        self.frame = None
        self.ret = False
        self.lock = threading.Lock()
        self._running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def release(self):
        self._running = False
        self.cap.release()
```

### Step 2.3 — Test and Push

```bash
git add jetson_workspace/ibvs_pipeline.py
git commit -m "Phase 2: MJPEG camera mode + threaded capture for lower latency"
git push
```

---

## Phase 3 — Arduino PID Offload (Lower Servo Latency)

### Why
- Currently: Python calculates PID → sends angle to Arduino → Arduino moves servo
- Python serial roundtrip adds 10-50ms delay per servo command
- **Better**: Jetson sends pixel error `(ex, ey)` → Arduino does PID in C++ → moves servo directly
- C++ PID loop can run at 1000Hz vs Python's ~30Hz

### Architecture Change

```
BEFORE:
Jetson (Python) → YOLO → PID calc → Serial angle (90,95) → Arduino → Servo

AFTER:
Jetson (Python) → YOLO → Serial error (ex=15,ey=-8) → Arduino (C++ PID) → Servo
```

### Step 3.1 — Update Arduino Sketch

In `jetson_workspace/arduino/pan_tilt_control.ino`, add PID loop:

```cpp
// Receive: "ex,ey\n" (pixel errors from Jetson)
// Arduino does PID internally and moves servos

float kp = 0.12, ki = 0.002, kd = 0.02;
float integral_pan = 0, integral_tilt = 0;
float prev_err_pan = 0, prev_err_tilt = 0;
unsigned long last_time = 0;

void loop() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        int comma = data.indexOf(',');
        if (comma > 0) {
            float ex = data.substring(0, comma).toFloat();
            float ey = data.substring(comma+1).toFloat();

            unsigned long now = millis();
            float dt = (now - last_time) / 1000.0;
            last_time = now;

            // PAN PID
            integral_pan += ex * dt;
            integral_pan = constrain(integral_pan, -50, 50);
            float d_pan = (ex - prev_err_pan) / dt;
            float delta_pan = -(kp*ex + ki*integral_pan + kd*d_pan);
            prev_err_pan = ex;

            // TILT PID
            integral_tilt += ey * dt;
            integral_tilt = constrain(integral_tilt, -50, 50);
            float d_tilt = (ey - prev_err_tilt) / dt;
            float delta_tilt = -(kp*ey + ki*integral_tilt + kd*d_tilt);
            prev_err_tilt = ey;

            // Apply corrections
            pan_angle = constrain(pan_angle + delta_pan, 0, 180);
            tilt_angle = constrain(tilt_angle + delta_tilt, 20, 160);

            pan_servo.write(pan_angle);
            tilt_servo.write(180 - tilt_angle);
        }
    }
}
```

### Step 3.2 — Update Python Pipeline (Send error instead of angle)

```python
# In FINE stage, instead of:
cmd = f"{inverted_tilt},{new_pan}\n"

# Send pixel errors:
if arduino:
    cmd = f"{int(e_x)},{int(e_y)}\n"
    arduino.write(cmd.encode())
```

### Step 3.3 — Upload to Arduino and Test

```bash
# Upload new sketch via Arduino IDE on dev machine
# OR use arduino-cli on Jetson if available
arduino-cli compile --fqbn arduino:avr:uno jetson_workspace/arduino/pan_tilt_control/
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno jetson_workspace/arduino/pan_tilt_control/
```

### Step 3.4 — Test and Push

```bash
git add jetson_workspace/arduino/ jetson_workspace/ibvs_pipeline.py
git commit -m "Phase 3: Arduino PID offload — Jetson sends error, Arduino does C++ PID"
git push
```

---

## Phase 4 — IBVS Bug Fixes and Tuning

### Bug 1: YOLO Bounding Box Shifted Down in Y-axis

**Root cause**: The camera `cy` (optical center Y) may not match the actual principal point,
OR the camera is mounted slightly tilted downward.

**Fix**: Add a Y-axis correction offset to the IBVS target:

```python
# In Config class, add:
IBVS_Y_OFFSET = 20  # pixels — positive shifts target UP, tune this value

# In IBVSController.compute_error():
def compute_error(self, detected_point):
    cx_detected, cy_detected = detected_point
    e_x = cx_detected - self.cx
    e_y = cy_detected - (self.cy - self.config.IBVS_Y_OFFSET)  # apply offset
    return e_x, e_y
```

**Tune `IBVS_Y_OFFSET`**:
- Run pipeline
- If box still below center → increase offset
- If box above center → decrease offset

### Bug 2: Slow IBVS Convergence (Arrow Too Large)

**Fix**: Allow simultaneous pan+tilt when error is large (remove sequential-only restriction for coarse errors):

```python
# In compute_servo_correction(), change:
PAN_THRESHOLD = 2.0  # degrees

# Allow tilt to move even when pan is large (just slower)
if abs(error_pan) > PAN_THRESHOLD:
    error_tilt *= 0.3   # Allow 30% tilt correction while panning
    # Don't fully freeze tilt
```

**Also increase KP for faster initial response**:
```python
IBVS_KP_PAN  = 0.18   # was 0.12 — faster response
IBVS_KP_TILT = 0.18   # was 0.12
```

### Bug 3: YOLO Only on Relevant Camera per Stage

**Optimization**: In COARSE stage, only run YOLO on Insta360. In FINE stage, only run on Logitech.
This halves the YOLO inference load:

```python
# Already done in pipeline — verify this is implemented correctly
if state == "COARSE":
    results = model(frame_insta, ...)   # ← only Insta360
elif state == "FINE":
    results = model(frame_logi, ...)    # ← only Logitech
```

### Step 4 — Test and Push

```bash
git add jetson_workspace/ibvs_pipeline.py
git commit -m "Phase 4: Fix Y-axis IBVS offset, tune PID gains for faster convergence"
git push
```

---

## Summary Timeline

```
Week 1:  Phase 1 (TensorRT)  → 3 FPS → ~20 FPS
Week 1:  Phase 2 (Camera)    → +3-5 FPS additional
Week 2:  Phase 3 (Arduino)   → Lower servo latency, same FPS but smoother
Week 2:  Phase 4 (IBVS fixes)→ Accurate centering, no Y-axis drift
```

## Final Expected Performance

| Metric | Before | After All Phases |
|--------|--------|-----------------|
| FPS | ~3 | ~25-30 |
| Servo latency | 50ms (Python) | ~5ms (Arduino C++) |
| Y-axis drift | Yes | Fixed |
| IBVS convergence | Slow (sequential) | Faster (simultaneous) |
| YOLO device | CPU | GPU (TensorRT FP16) |

---

## Notes

- Each phase is **independent** — implement and test one at a time
- Always test on Jetson before pushing to GitHub
- The TensorRT `.engine` file is **Jetson-specific** — do not commit it to GitHub
  (add `*.engine` to `.gitignore`)
- Reference: [Ultralytics YOLO on NVIDIA Jetson](https://docs.ultralytics.com/guides/nvidia-jetson/)
