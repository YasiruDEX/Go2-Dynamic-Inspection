# Weekly Progress Report — Visual Inspection System (Week 5)
# Period: 11 March – 17 March 2026

---

## Slide 1: Last Week Recap + This Week Focus

**Last week completed (Week 4):**
- Full ROS2 conversion done — camera_node, servo_node, ibvs_action_server
- MQTT integration working with ThingsBoard
- Behaviour Tree leaf nodes provided to Ravith
- Pipeline running end-to-end

**This week's focus: Dataset Collection + Evaluation**

| Task | Status |
|------|--------|
| Research evaluation methods for this type of pipeline | ✅ Done |
| Design flexible data collection script | ✅ Done |
| Collect evaluation dataset (angle, distance, occlusion, reference) | ✅ Partial |
| Identify & fix YOLO detection degradation after ROS conversion | ✅ Root cause found |
| Collect YOLO retraining dataset (fire ext, door, gauge) | ✅ Started |

**Script for Slide 1:**
"Last week we had the full ROS2 pipeline working. This week the focus shifted to evaluation — but before collecting data, I first did research on what evaluation methods are appropriate for a system like this, then built the tooling to actually collect the data. We hit some blockers along the way which I'll go through."

---

## Slide 2: Evaluation Method Research

**Key question:** How do you evaluate a visual inspection pipeline for a research paper?

The system has three separate pipelines — each needs different metrics:

| Pipeline | What to measure | Metrics |
|----------|----------------|---------|
| **ROI Capture** (Jetson) | Does it reliably center and capture the object? | IBVS convergence time, final pixel error, convergence rate, FPS |
| **Gauge Reading** (Server) | Is the analog reading numerically accurate? | MAE, RMSE, R², reading error distribution |
| **VLM Decision** (Server) | Are PASS/FAIL decisions correct? | Accuracy, Precision/Recall, F1, BERTScore, LLM-as-judge |

**Robustness test conditions designed:**

| Variable | Levels tested |
|----------|--------------|
| Horizontal angle | 0°, 15°L, 30°L, 15°R, 30°R, 45°L, 45°R |
| Distance | 0.75m, 1m, 1.25m, 1.5m, 2m, 3m, 4m |
| Occlusion | 0%, 25%, 50%, 75% |
| Multi-object | 1, 2, 3 objects in scene |

**Key insight:** For a publication, minimum 50 samples per condition are needed  
(30 is CLT minimum but outliers skew results — 50 gives robust mean ± std and 95% CI)

*(Insert: screenshot of evaluation_plan.md showing the full metric breakdown)*

**Script for Slide 2:**
"Before collecting any data I researched what evaluation methods are appropriate for each part of this system — because the three pipelines need completely different metrics.

For the **ROI capture pipeline** — the part on Jetson — we measure IBVS centering performance: convergence time in seconds, final pixel error at convergence (target under 10 pixels), and convergence rate as a percentage of successful runs. On top of that, image quality metrics: SSIM and PSNR tell us spatial fidelity, VIF tells us perceived visual quality from a human perspective, and AlexNet cosine similarity compares the captured image to a reference to see if the captured content matches what was expected.

For the **gauge reading pipeline** on the server — we compute MAE and RMSE between the predicted reading and the true physical value, plus R-squared to show how well the model tracks across the full range of gauge values.

For the **VLM decision pipeline** — we use classification metrics: precision, recall, F1 for the PASS/FAIL decisions, BERTScore to evaluate how semantically close the VLM's text output is to a ground-truth caption, and LLM-as-judge where a second language model scores the output for correctness.

The test conditions are: 7 distances from 0.75m to 4m, 7 horizontal angles, 4 occlusion levels, and multi-object scenes. We are starting with reference baseline and distance evaluation first — because those do not require YOLO to work at angles, and YOLO currently has a detection quality issue at close range which we are fixing through retraining. Once retraining is done, all test conditions will be unlocked."

---

## Slide 3: Flexible Data Collection Script

**Problem with previous approach:** No structured way to vary angle, distance, occlusion — had to manually move things and remember to log everything.

**Solution built: `collect_dataset.py`**

```
Main menu:
  1. Angle evaluation    → enter exact angle + direction
  2. Distance evaluation → enter exact measured distance
  3. Occlusion testing   → enter exact % occlusion applied
  4. Reference baseline  → clean unoccluded captures
  5. VLM / Gauge special sessions

For each capture:
  1. User enters: angle/distance/occlusion value + how many images
  2. Script runs full pipeline automatically (camera → YOLO → IBVS → capture)
  3. Automatically detects new img_01.jpg
  4. Automatically parses IBVS convergence time and final error from terminal output
  5. Auto-copies image to correct evaluation folder
  6. Logs everything to capture_log.csv (timestamp, angle, distance, occlusion, IBVS stats)
```

**What's automated (no manual steps):**
- Running the pipeline
- Detecting new captured image
- Copying to correct folder
- Logging IBVS convergence time and error

*(Insert: screenshot of script running — menu + progress output)*

**Script for Slide 3:**
"The data collection script handles everything automatically. You enter the experimental condition — the angle you measured, the distance, the occlusion percentage — then press enter. The script runs the full pipeline, waits for it to complete, pulls out the IBVS stats from the output, copies the image to the right folder, and logs everything to a CSV. There's no manual file copying or data entry — which means less errors and faster collection."

---

## Slide 4: Blocker — Joystick Control Didn't Work

**Plan:** Use a USB joystick to control the pan-tilt mechanism during data collection (move servo to desired angle → capture)

**What happened:**
- Joystick detection was unreliable (SDL2 / pygame device enumeration issues on Jetson)
- Even when detected, analog stick values drifted — servos twitched instead of holding position
- Wasted time debugging hardware compatibility

**Solution:** Replaced joystick with **keyboard arrow key control** in the YOLO dataset collection script

```python
# In collect_yolo_dataset.py — cv2.waitKey() captures arrow keys:
cv2.waitKey(30) & 0xFF
# Left/Right arrow → pan servo  (+/- step degrees)
# Up/Down arrow   → tilt servo  (+/- step degrees)
# SPACE           → capture both cameras simultaneously
# +/-             → adjust step size (1°–15° per keypress)
# h               → home (90°, 90°)
# c               → change class
```

**Result:** More reliable than joystick — step size is controllable, no drift, works over SSH

**Script for Slide 4:**
"The original plan was to use a joystick for fine servo control during data collection. That didn't work — the joystick detection on Jetson was unreliable and even when it worked the stick drifted and caused servo twitching. I replaced it with keyboard control using OpenCV's waitKey — arrow keys move the servos in configurable steps, space bar captures both cameras simultaneously. It's actually more controllable than a joystick because you can set the step size and each press is exactly that many degrees."

---

## Slide 5: Dataset Collection — What We Got

**Evaluation dataset collected (`evaluation/`):**

| Folder | Images | Notes |
|--------|--------|-------|
| `reference/` | 2 images | Clean baseline — fire extinguisher, head-on, 1m |
| `distance_eval/0.75m/` | 2 images | Close range |
| `distance_eval/1m/` | 2 images | Standard range |
| `distance_eval/1.25m/` | 1 image | |
| `distance_eval/1.5m/` | 1 image | |

**Observation on captured images:**
- ROI images are **clear and well-centred** — IBVS working correctly at 1-2m
- Image quality sufficient for VLM evaluation at these distances
- Angle evaluation not yet completed (blocked by YOLO issue — see next slide)

**YOLO training dataset collected (`yolo_dataset/`):**
- Classes: `fire_extinguisher`, `door`, `gauge`
- Cameras: Insta360 + Logitech captured simultaneously per class
- Purpose: Re-train YOLO model to improve detection robustness

*(Insert: sample ROI capture showing clean centered fire extinguisher)*

**Script for Slide 5:**
"We started collecting the evaluation dataset and the images look good — the IBVS centering is working and the ROI captures are clear. However we hit a significant blocker with YOLO detection that prevented completing the angle evaluation. I'll explain that on the next slide. In parallel I also set up a separate dataset collection for YOLO retraining — capturing both cameras simultaneously for three classes."

---

## Slide 6: Blocker — YOLO Detection Degraded After ROS Conversion

**Symptom:** After converting to ROS2, YOLO detection on the Logitech camera was producing:
- Double bounding boxes (body + nozzle detected separately)
- False positive: red fire safety cabinet on wall detected as fire extinguisher
- Lower confidence scores than before ROS conversion
- Detection failing at close range / changing lighting

**Before ROS conversion (direct Python):** Clean single tight bounding box at all tested conditions

**Root cause found:**

```python
# Original ibvs_pipeline.py (line 603-605):
frame_logi_display = cv2.resize(frame_logi, (640, 360))   # ← RESIZED
results_logi = model(frame_logi_display, ...)              # ← YOLO on 640x360

# ROS2 ibvs_action_server.py (before fix):
frame = self._get_logi()     # 640x480 raw frame
results = self.model(frame)  # ← YOLO on 640x480  ← MISMATCH
```

**The TensorRT engine was compiled for 640×360 input.**  
Feeding it 640×480 caused letterboxing artifacts → degraded confidence scores → false positives pass through.

**Fix applied:**
```python
# _detect_raw() now resizes before YOLO, scales coords back after:
YOLO_INPUT_H = 360
if h_orig != self.YOLO_INPUT_H:
    yolo_frame = cv2.resize(frame, (w_orig, self.YOLO_INPUT_H))
    scale_y    = h_orig / self.YOLO_INPUT_H   # 480/360 = 1.333
results = self.model(yolo_frame, ...)
# Then scale y-coords back × 1.333 for correct IBVS pixel error
```

*(Insert: before/after comparison — multiple noisy boxes vs clean single box)*

**Script for Slide 6:**
"This was the most important blocker of the week. YOLO detection that was working perfectly before ROS conversion was producing garbage results after. It took some investigation to find the root cause — the TensorRT engine was compiled for 640 by 360 input, but the ROS camera node was publishing at 640 by 480, and nobody was resizing before inference. The fix is a single resize in the detection function — we resize down, run YOLO to get coordinates in 360 space, then scale the coordinates back up to 480 space for the IBVS pixel error calculation."

---

## Slide 7: Current Status + Next Week Plan

**What's working now:**
- ✅ YOLO detection fix applied (resize 640×480 → 640×360 before TRT inference)
- ✅ Reference dataset captured and ready for evaluation
- ✅ YOLO retraining dataset collection started (fire_ext, door, gauge)
- ✅ Evaluation folder structure in place, CSV logging working

**What we can do in parallel now:**

| Task | Status | Who |
|------|--------|-----|
| Run evaluation on reference dataset (IBVS + image quality metrics) | **Can start now** | Dinethra |
| Complete angle / distance / occlusion data collection | Next session | Dinethra |
| YOLO retraining (label collected data → fine-tune) | After more data | Dinethra |
| BT integration testing with Ravith | Continues | Ravith + Dinethra |

**Next week targets:**
- Complete distance evaluation (7 distances × 50 images each)
- Complete angle evaluation (7 angles × 50 images each)
- Run image quality metrics (SSIM, PSNR, VIF) on collected reference images
- Send labelled YOLO training data → begin fine-tuning run
- IBVS convergence stats: mean, std, 95% CI per condition

**Script for Slide 7:**
"The plan for next week is parallel — while more data collection continues, we can already start running the evaluation metrics on the reference data we have. The evaluation scripts are ready. For YOLO retraining, once we have enough images labeled (targeting 200+ per class), we'll run a fine-tuning run and benchmark improvement. The goal is to have all angle and distance evaluation data collected by end of next week so we can start statistical analysis."

---

## Technical Notes (for your reference)

**Why TRT engine input size matters:**
- TensorRT compiles a fixed computation graph optimized for one specific input shape
- Feeding a different shape: engine internally letterboxes/pads → different feature map alignment → lower detection quality
- Always export TRT engine at the same resolution used during inference

**Why 50 samples minimum per condition:**
- 30 samples satisfies the Central Limit Theorem (distribution of sample mean → normal)
- But with 30 samples, a single outlier shifts the mean by ~3%
- With 50 samples, one outlier = ~2% shift — reportable with ±std error bars in paper
- For a conference paper table: report N, mean ± std, min, max, 95% CI

**Evaluation file locations:**
```
Local machine:
  Evaluation_V_I_ws/eval_dataset/          ← pulled from Jetson evaluation/
  data/captures_from_jetson/               ← pulled from Jetson captures/
  data/yolo_dataset/                       ← pulled from Jetson yolo_dataset/

Jetson:
  ~/Documents/Visual_Inspection_ws/evaluation/
  ~/Documents/Visual_Inspection_ws/captures/
  ~/Documents/Visual_Inspection_ws/yolo_dataset/
```

---

*Report prepared by: Dinethra | 2026-03-17*
