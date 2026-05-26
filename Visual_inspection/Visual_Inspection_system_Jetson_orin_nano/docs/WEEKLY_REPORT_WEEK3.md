# Weekly Progress Report - Visual Inspection System (Week 3)
# Period: Feb 17 – Feb 24, 2026

---

## Slide 1: Last Week Recap - Full Pipeline on Jetson + IBVS Working

○ Completed full two-stage pipeline integration on Jetson Orin Nano (JetPack 6.1)

○ Both cameras (Insta360 + Logitech C920) and Arduino servo controller connected and verified

○ IBVS pipeline running end-to-end: COARSE stage (Insta360 → servo move) + FINE stage (Logitech IBVS centering)

○ Initial frame rate: only **~3 FPS** — too slow for real-time visual servoing

○ Identified 4 optimization targets: GPU inference, camera input, servo latency, bounding box drift

**Script for Slide 1:**
"Last week I completed the full pipeline integration on the Jetson Orin Nano. Both stages are working — the Insta360 detects the object, moves the servo to point the Logitech camera, then IBVS kicks in for precise centering. The problem was performance — we were only getting about 3 frames per second. That's because YOLO was running on CPU. I also noticed a bounding box drift issue in the Y-axis direction. This week I solved both of these problems."

---

## Slide 2: Phase 1 — TensorRT GPU Acceleration (10× Speedup)

○ Identified Jetson as running **JetPack 6.1** (R36.4.3) with TensorRT 10.3.0 pre-installed

○ Exported YOLO11n model to **TensorRT FP16 engine** format:
  - Command: `model.export(format='engine', device=0, half=True, imgsz=640)`
  - Export time: ~7 minutes on Jetson GPU
  - Engine size: 8.4 MB (vs 5.4 MB PyTorch weights)

○ Result: **3 FPS → 30 FPS** (10× improvement)

○ Pipeline auto-selects TensorRT engine if available, falls back to PyTorch if not

```
Before (PyTorch CPU):    ~3 FPS
After  (TensorRT FP16): ~30 FPS  ← 10× faster
```

*(Insert terminal screenshot showing [FPS] 30.0 fps output)*

**Script for Slide 2:**
"The first optimization was TensorRT GPU acceleration. Jetson has NVIDIA's TensorRT already installed as part of JetPack. I exported the YOLO model to TensorRT FP16 format — this took about 7 minutes on the Jetson GPU but you only do it once. The result was dramatic: from 3 frames per second to 30 frames per second. That's 10 times faster, just by using the GPU properly. The pipeline automatically detects if the TensorRT engine file exists and uses it; otherwise it falls back to the standard PyTorch model."

---

## Slide 3: Phase 2 — Reliable Device Detection with udev Rules

○ **Problem discovered:** Plugging in the Logitech F710 joystick shifted USB device numbers — cameras and Arduino would randomly change from `/dev/video0` to `/dev/video2` etc., breaking the pipeline

○ **Root cause:** Linux enumerates USB devices in connection order — any new device shifts existing numbering

○ **Solution: udev rules** — permanent symlinks regardless of USB device order:

```
/dev/insta360  →  always points to Insta360 X3     (VID=2e1a, PID=00c1)
/dev/logitech  →  always points to Logitech C920    (VID=046d, PID=08e5)
/dev/arduino   →  always points to Arduino Uno      (VID=2341)
```

○ **3-Layer Camera Detection** (in code, at startup):
  1. udev symlink (fastest — fixed path)
  2. USB Vendor ID scan via sysfs (reliable fallback)
  3. Name-based search (original fallback)

○ **Arduino auto-detection** (`find_arduino()`) scans all `/dev/ttyACM*` ports by USB vendor ID — works regardless of joystick connection

*(Insert diagram: USB device numbering before vs after udev rules)*

**Script for Slide 3:**
"We had a reliability problem. Every time the joystick was connected, the USB device numbers would shift and the pipeline would fail to find the cameras. This is a fundamental Linux behavior — devices are numbered in the order they connect. The solution was udev rules: permanent symbolic links that always point to the right device regardless of what else is plugged in. Now /dev/insta360 always points to the Insta360, /dev/logitech always points to the Logitech, and /dev/arduino always points to the Arduino. No matter what USB devices are connected. In the code, I also added a 3-layer detection: first try the udev symlink, then scan by USB vendor ID, then fall back to name search. And the Arduino is also auto-detected by its USB vendor ID at startup."

---

## Slide 4: Headless vs Headed Modes + Run Scripts

○ Implemented two operating modes for the pipeline:

| Feature | Headed Mode | Headless Mode |
|---------|-------------|---------------|
| Camera window | ✅ Live dual-camera view (via VNC) | ❌ No window |
| FPS in terminal | ✅ Every second | ✅ Every second |
| Speed | Slightly slower (drawing overhead) | 🔥 Maximum speed |
| Use case | Debugging, tuning | Deployment, production |

○ Created two separate scripts:
  - `ibvs_headed.py` — for debugging with VNC camera view
  - `ibvs_headless.py` — for production (maximum performance)

○ Created `run.sh` launcher that automatically:
  - Kills any stuck previous instances (releases cameras + serial port)
  - Sets `DISPLAY=:1` for VNC
  - Enables max Jetson performance (`nvpmodel -m 0`, `jetson_clocks`)
  - Grants serial port permissions

○ **FPS meaning:**
  - COARSE mode: ~30 FPS (full pipeline speed with TensorRT)
  - FINE mode: ~15-20 FPS (includes servo settling delay — this is correct behavior)

**Script for Slide 4:**
"I also added two operating modes. Headed mode shows a live camera window with both cameras side by side — useful for debugging and tuning. Headless mode skips all display operations for maximum speed — this is what we'd use for production deployment. In both modes, FPS is printed to the terminal every second so you always know performance. You'll notice the FPS drops in FINE mode compared to COARSE — that's expected and correct. In FINE mode we have to wait after each servo command for the physical servo to settle before reading the next frame. If we don't wait, we'd be correcting based on an outdated position."

---

## Slide 5: Phase 4 — Y-Axis Bounding Box Drift Fix

○ **Problem:** Bounding box appeared shifted downward from the actual object position in the Logitech IBVS view

○ **Root cause — coordinate space mismatch:**

```
YOLO ran on:        frame_logi       (640 × 480 pixels)
Results drawn on:   frame_logi_disp  (640 × 360 pixels)
                                            ↑
                                     DIFFERENT HEIGHT — no scaling!
```

○ **Effect:** Object at y=300 in 480p space drawn at y=300 in 360p frame → shifted 75px too low
  - The worse the object position (lower in frame), the larger the visual shift

○ **Fix applied:**
  1. Run YOLO on `frame_logi_display` (640×360) — all coordinates in display space, no conversion needed
  2. Scale optical center cy from 480p → 360p: `cy = 257.50 × (360/480) = 193.1`
  3. All display drawing coordinates now exactly match what you see

○ IBVS error computation also updated to use 360p-space cy → more accurate error signal

*(Insert before/after screenshots showing bbox position correction)*

**Script for Slide 5:**
"This was an important bug to fix. The bounding box was appearing lower than the actual object — this is called a coordinate space mismatch. YOLO was running on the full 480 pixel height frame, but the results were being drawn on a 360 pixel display frame without any scaling. So coordinates from the 480p world were being plotted incorrectly on the 360p display. The fix was to run YOLO directly on the display-sized frame. Everything is now in the same coordinate space. I also had to scale the camera's principal point cy proportionally — from the calibrated 480p value down to the 360p equivalent. This also improved the IBVS error signal accuracy."

---

## Slide 6: Current System Performance Summary

```
┌────────────────────────────────────────────────────────┐
│              OPTIMIZATION RESULTS                       │
├──────────────────┬──────────────┬──────────────────────┤
│ Metric           │ Before       │ After                 │
├──────────────────┼──────────────┼──────────────────────┤
│ FPS (COARSE)     │ ~3 FPS       │ ~30 FPS (10×)        │
│ FPS (FINE/IBVS)  │ ~3 FPS       │ ~15-20 FPS           │
│ YOLO Backend     │ PyTorch CPU  │ TensorRT GPU FP16     │
│ Camera Detection │ Fixed index  │ udev symlinks + VID   │
│ BBox Y drift     │ ~50-75px off │ Fixed (360p space)    │
│ Device stability │ Breaks w/USB │ Stable always         │
└──────────────────┴──────────────┴──────────────────────┘
```

○ Pipeline running reliably on Jetson Orin Nano at ~20 FPS during IBVS

○ Both fire extinguisher and gauge centering confirmed working

○ Code versioned on GitHub with full documentation

*(Insert video of full pipeline running — COARSE → FINE centering)*

**Script for Slide 6:**
"Here's the summary of this week's results. We went from 3 FPS to 30 FPS. Camera and Arduino detection is now reliable regardless of what other USB devices are connected. The bounding box drift is fixed. The pipeline runs stably and can be started with a single command. Both fire extinguisher and gauge detection and centering are confirmed working."

---

## Slide 7: Next Week

○ **Phase 3: PID Tuning** — Systematic gain tuning for faster IBVS convergence
  - Current convergence: 3-8 seconds with oscillation
  - Target: <3 seconds smooth convergence
  - Method: Tune Kp, Ki, Kd individually per axis

○ **Phase 3: Arduino C++ PID offload** — Move PID computation from Jetson Python to Arduino C++
  - Benefit: Servo commands sent at ~1000 Hz instead of 15-20 Hz
  - Expected improvement: Faster, smoother servo response in FINE mode

○ **Final validation dataset** — Collect performance data with optimized system
  - Convergence time measurement
  - Accuracy measurement (pixels from center at convergence)
  - Range testing (1m, 2m, 3m, 4m, 5m)

○ **System evaluation** — Full end-to-end benchmark on real inspection objects

**Script for Slide 7:**
"For next week, there are two main goals. First, PID tuning — the IBVS converges but it takes 3 to 8 seconds and sometimes oscillates. I want to get that below 3 seconds with smooth movement. Second, I want to offload the PID computation to the Arduino. Right now, every servo command goes through Python on Jetson — that's slow. If the Arduino runs the PID itself, servo updates happen much faster. I also plan to run the full validation test — measuring convergence time and accuracy at different distances — so we have solid performance numbers for the evaluation."

---

## Technical Explanations (For Your Study)

**1. Why TensorRT is So Much Faster:**
- PyTorch runs at full 32-bit float precision on CPU → slow on embedded hardware
- TensorRT FP16 compiles the neural network specifically for the Jetson GPU
- It fuses layers, eliminates redundant operations, and uses GPU tensor cores
- Result: same accuracy, 10× the speed

**2. Why USB Device Numbers Shift:**
- Linux assigns /dev/video0, /dev/video1, etc. in the order devices connect at boot
- If you add a joystick, it might get video0 and push cameras to video1, video2, etc.
- udev rules use hardware identifiers (USB Vendor ID + Product ID) — these never change
- So the symlink always points to the right physical device

**3. What is USB Vendor ID?**
- Every USB device has a 4-digit Vendor ID (VID) and Product ID (PID) burned into hardware
- Insta360: VID=2e1a, PID=00c1
- Logitech C920: VID=046d, PID=08e5
- Arduino Uno: VID=2341
- These IDs never change regardless of which USB port or what order you plug devices in

**4. Headed vs Headless — Why Headless is Faster:**
- cv2.imshow requires compositing, memory copies, and display rendering
- These add ~2-3ms per frame (not much, but significant at 30 FPS)
- More importantly, headless mode skips all bounding box DRAWING operations
- The YOLO inference and servo control logic are identical in both modes

**5. The Coordinate Space Bug Explained:**
```
480p frame (y=0 to 480):
  Object at y=350 — that's 73% down the frame

360p display (y=0 to 360):
  73% down would be y=262
  But the code drew it at y=350 (out of 360!) → clipped/shifted way down

Fix: Run YOLO on 360p frame directly → coordinates are already correct
```

**6. Why cy Must Be Scaled:**
- Kalibr calibration was done on the 640×480 Logitech image → cy=257.50
- That means the optical axis crosses at y≈257 in a 480-pixel-tall image
- In a 360-pixel-tall image, that same physical optical axis is at: 257.50 × (360/480) = 193.1
- Using the wrong cy means the IBVS thinks the image center is lower than it is → incorrect error signal

**7. IBVS FPS in FINE Mode:**
- FINE mode runs at ~15-20 FPS (not 30 FPS)
- `time.sleep(IBVS_SERVO_DELAY)` is called after every servo command
- This wait is REQUIRED — servo needs time to physically move before the next frame is useful
- Reading a frame before servo settles would give a stale position → wrong correction → oscillation
- This is correct behavior, not a bug
