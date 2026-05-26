# Visual Inspection System — Data Collection & Evaluation Master Guide

**System:** Go2 quadruped robot + Jetson Orin Nano + Insta360 (pole-mounted) + Logitech C920 (pan-tilt)  
**Pipeline:** Insta360 → YOLOv11n TensorRT → ByteTrack → Coarse (degree-4 polynomial) → IBVS PID → Logitech ROI → Server (Gauge 10-step or VLM/Gemini)  
**See also:** `evaluation_plan.md` — full metric formulas and evaluation scripts

---

## HOW DATA IS STORED — READ THIS FIRST

Every capture stores data in **two places**. You need both. Do not confuse them.

### 1. `capture_log.csv` — Runtime metadata per capture (Jetson)
**Location on Jetson:** `~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv`  
**Location on laptop:** `/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/capture_log.csv` (after scp)

This is written **automatically** by `collect_dataset.py` each time you press ENTER to capture.

**Current columns and what they mean:**

| Column | Example | Status | Used for |
|---|---|---|---|
| `timestamp` | `2026-04-18_19:20:24` | ✅ Working | Traceability |
| `folder` | `reference` | ✅ Working | Knowing which session |
| `filename` | `fire_ref_28.jpg` | ✅ Working | Linking image to metadata |
| `object_type` | `fire_extinguisher` | ✅ Working | All metrics |
| `distance_m` | `1` | ✅ Working | Distance robustness |
| `angle_deg` | `0` | ✅ Working | Angle robustness |
| `angle_direction` | `center` | ✅ Working | Angle robustness |
| `occlusion_pct` | `0` | ✅ Working | Occlusion robustness |
| `n_objects` | `1` | ✅ Working | Multi-object eval |
| `ibvs_time_s` | `0.92` | ✅ Real seconds | IBVS convergence time |
| `ibvs_fps` | `0.0` | ⚠️ Bug (ignore) | IBVS FPS |
| `final_error_px` | `9.85` | ✅ Real px | Final centering error |
| `converged` | `True` | ✅ Working | Convergence rate % |
| `coarse_time_s` | `0.0` | ⚠️ Not set | Coarse stage time |
| `pipeline_time_s` | `0.0` | ⚠️ Bug (ignore) | Total pipeline time |
| `detection_confidence` | `0.8975` | ✅ Working | YOLO confidence |
| `objects_inspected` | `1` | ✅ Working | Multi-object handling |
| `ground_truth_value` | `N/A` or `2.5` | ✅ You enter | Gauge MAE/RMSE, VLM PASS/FAIL |
| `notes` | caption text | ✅ You enter | BERTScore, LLM-judge |

> **During collection, verify after each capture:** open a second terminal and run  
> `tail -3 ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv`  
> You should see the new row with real `ibvs_time_s` (not 0.0) and `converged=True/False`.

### 2. Captured images — Logitech ROI images (Jetson)
**Location:** `~/Documents/Visual_Inspection_ws/captures/inspection/<session_ts>/<class>/instance_N/`

Each capture saves **4 Logitech images** + 1 `metadata.json`:
```
captures/inspection/20260418_192024/fire_extinguisher/instance_1/
    img_01_conf0.90.jpg
    img_02_conf0.90.jpg
    img_03_conf0.90.jpg
    img_04_conf0.90.jpg
    metadata.json    ← ibvs_time_s, ibvs_error_px, ibvs_converged, confidence all here
```

**These images are the input to all offline evaluation scripts.** SSIM, PSNR, VIF, AlexNet, gauge reading, VLM decisions — all computed by running scripts on these images AFTER collection.

> **During collection, verify:** `ls ~/Documents/Visual_Inspection_ws/captures/inspection/ | tail -5`  
> You should see new timestamped folders appearing after each capture.

---

## WHAT GETS COMPUTED WHEN

| Metric | Stored during capture? | Computed when? | Script |
|---|---|---|---|
| IBVS convergence time | ✅ CSV `ibvs_time_s` | During collection | automatic |
| Final pixel error | ✅ CSV `final_error_px` | During collection | automatic |
| Convergence rate % | ✅ CSV `converged` | After — `converged.mean()` | pandas on CSV |
| YOLO confidence | ✅ CSV `detection_confidence` | During collection | automatic |
| **SSIM** | ❌ Not in CSV | **Offline** on images | `evaluate_image_quality.py` |
| **PSNR** | ❌ Not in CSV | **Offline** on images | `evaluate_image_quality.py` |
| **VIF** | ❌ Not in CSV | **Offline** on images | `evaluate_image_quality.py` |
| **AlexNet cosine similarity** | ❌ Not in CSV | **Offline** on images | `evaluate_image_quality.py` |
| **Gauge predicted reading** | ❌ Not in CSV | **Offline** — send image to server | `evaluate_gauge.py` |
| **Gauge MAE / RMSE** | ❌ Not in CSV | **Offline** from predicted vs GT | `evaluate_gauge.py` |
| **VLM PASS/FAIL decision** | ❌ Not in CSV | **Offline** — send image to server | `evaluate_vlm.py` |
| **VLM precision/recall/F1** | ❌ Not in CSV | **Offline** from decisions | `evaluate_vlm.py` |
| **BERTScore** | ❌ Not in CSV | **Offline** from captions | `evaluate_vlm.py` |
| **LLM-as-judge** | ❌ Not in CSV | **Offline** | `evaluate_vlm_llm_judge.py` |

**Bottom line:** If images are saved and CSV has real `ibvs_time_s` values, you have everything needed. The CSV alone is not the final result — it is one of two inputs.

---

## COLLECTION PRIORITY ORDER

```
PRIORITY 1 — Reference Images (baseline for ALL image quality metrics)
PRIORITY 2 — Angle Evaluation (Professor specifically required this)
PRIORITY 3 — Distance Evaluation (1m, 2m, 3m, 4m)
PRIORITY 4 — Gauge Ground Truth (for MAE/RMSE)
PRIORITY 5 — VLM Labelled Images (PASS/FAIL for each object type)
PRIORITY 6 — Occlusion Evaluation
PRIORITY 7 — Multi-object scenes
```

---

## PRIORITY 1 — REFERENCE IMAGES

### What we evaluate with this dataset
These are the **gold-standard baseline images** that every other image is compared against.  
Used to compute: **SSIM, PSNR, VIF, AlexNet cosine similarity** for all other sessions.  
Without reference images, you cannot run any image quality metric.

### Parameters that must be stored

**In CSV (automatic):**
- `ibvs_time_s` — should be < 3s (best possible conditions)
- `final_error_px` — should be < 10px
- `converged` — must be True (discard if False)
- `detection_confidence` — should be > 0.7 for extinguisher, > 0.3 for gauge

**As image files (automatic):**
- 4 Logitech ROI images per capture in `captures/inspection/...`
- These are the reference images you will use in `evaluate_image_quality.py`

**You enter manually:**
- `distance_m` = `1.0` (exact measurement)
- `angle_deg` = `0`
- `notes` = any issues (lighting, blur, etc.)

### How to verify during collection
```bash
# Check CSV has real values (not all zeros):
tail -3 ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
# → ibvs_time_s should be 0.5–5.0, NOT 0.0
# → converged should be True

# Check images exist:
ls ~/Documents/Visual_Inspection_ws/captures/inspection/ | tail -3
```

### Setup
- Distance: **exactly 1.0 m** (tape measure)
- Angle: **0° head-on** — robot centred in front of object
- Lighting: even, no shadows, no glare on object
- Object: perfectly positioned, fully visible

### Minimum: 5 images per object type (gauge, fire_ext, door)

---

## PRIORITY 2 — ANGLE EVALUATION

### What we evaluate with this dataset
**IBVS Centering Performance vs angle** — does convergence time increase at larger angles?  
**Image quality vs angle** — does SSIM/PSNR drop at large angles?  
**Maximum usable angle** — at what angle does the system fail?

Fills in **Table 1** of the paper: Conv. Time, Final Error, Success % per angle.

### Parameters that must be stored

**In CSV (automatic):**
- `ibvs_time_s` — key metric: expect < 5s at 0°, longer at 45°
- `final_error_px` — key metric: expect < 10px when converged
- `converged` — key: success rate drops at large angles
- `detection_confidence` — does confidence drop at steep angles?
- `angle_deg` + `angle_direction` — **critical: enter correctly**
- `distance_m` — must be `2.0` for all angle tests

**As image files (automatic):**
- Logitech ROI images → used for SSIM/PSNR/VIF vs reference

**You enter manually:**
- `angle_deg` — the angle you set up (15, 30, 45)
- `angle_direction` — L or R

### How to verify during collection
```bash
# After each angle position, check:
grep "angle_eval" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv | tail -5
# → You should see different angle_deg values: 15, 30, 45
# → ibvs_time_s should INCREASE as angle increases
# → converged should go False at some point (that's the finding!)
```

### Angles to cover
| Angle | Direction | Images | Expected result |
|---|---|---|---|
| 0° | head-on | 10 | converge < 3s, err < 10px |
| 15° | L + R | 5 each | converge < 5s |
| 30° | L + R | 5 each | converge 5-8s |
| 45° | L + R | 5 each | may fail to converge |
| 60° | L + R | 3 each | likely fails |

Distance fixed at **2.0 m** for all.

---

## PRIORITY 3 — DISTANCE EVALUATION

### What we evaluate with this dataset
**How performance degrades with distance.**  
CSV gives: convergence time vs distance, final error vs distance, success rate vs distance.  
Images give: SSIM/PSNR/VIF vs distance — showing image quality degradation.  
At 4m: test if gauge OCR can still read numbers (visual check of images).

Fills in **Table 1** of the paper: SSIM column and Conv. Time column per distance.

### Parameters that must be stored

**In CSV (automatic):**
- `ibvs_time_s` — expect 2s at 1m, up to 8s at 4m
- `final_error_px` — larger at farther distances
- `converged` — may fail at 4m
- `detection_confidence` — drops with distance
- `distance_m` — **critical: enter exact measured value**

**As image files (automatic):**
- Logitech ROI images → compared to reference for SSIM/PSNR vs distance

**You enter:**
- `distance_m` — tape-measured, not estimated

### How to verify during collection
```bash
grep "distance_eval" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
# → distance_m column should have: 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0
# → ibvs_time_s should increase as distance increases
```

### Distances: 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0 m
- Object: **gauge** (best to show OCR degradation)
- Angle: always **0°**
- 10 images per distance

---

## PRIORITY 4 — GAUGE GROUND TRUTH

### What we evaluate with this dataset
**Gauge reading pipeline accuracy (MAE, RMSE, MAPE, Success Rate).**  
The pipeline (10-step CV+DL on server) predicts a reading. You compare to true value.  
`MAE = mean(|predicted - true|)` — needs `ground_truth_value` in CSV for every image.

### Parameters that must be stored

**In CSV (automatic):**
- `ibvs_time_s`, `final_error_px`, `converged` — IBVS quality
- `detection_confidence` — gauge confidence

**In CSV — YOU MUST ENTER:**
- `ground_truth_value` — the **actual gauge reading** (e.g. `2.5`)  
  **This is THE most critical field for this session. Without it, no MAE/RMSE is possible.**
- `notes` — describe pointer position precisely ("between 2 and 3 markings, closer to 2.5")

**As image files (automatic):**
- Logitech ROI images → sent to server by `evaluate_gauge.py` → get predicted readings

**Computed offline (NOT in CSV):**
- Predicted gauge reading from server
- MAE, RMSE, MAPE, bias — all from `evaluate_gauge.py`

### How to verify during collection
```bash
grep "gauge_accuracy" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
# → ground_truth_value column MUST have actual numbers (2.5, 4.0, 7.0...)
# → NOT "N/A" — if you see N/A for gauge_accuracy rows, that run is useless
```

### Readings to collect
Cover the full gauge range with at least 6 different readings, 3 images each minimum:
`0, 25%, 50%, 75%, 100%` of scale + a few intermediate points.

---

## PRIORITY 5 — VLM EVALUATION IMAGES

### What we evaluate with this dataset
**VLM (Gemini) PASS/FAIL decision accuracy.**  
Metrics: Accuracy, Precision, Recall, F1 per object type.  
Also: BERTScore (text quality) and LLM-as-judge (needs your written caption).

### Parameters that must be stored

**In CSV (automatic):**
- `ibvs_time_s`, `converged`, `detection_confidence` — capture quality

**In CSV — YOU MUST ENTER:**
- `ground_truth_value` — `PASS` or `FAIL` (not a number — write the word)
  **Without this, you cannot compute accuracy/recall/F1.**
- `notes` — a **one-sentence description of exactly what the camera sees**  
  Example: `"Red fire extinguisher on wall, large brown box placed directly in front blocking access"`  
  **This is the ground truth caption for BERTScore and LLM-as-judge.**

**As image files (automatic):**
- Logitech ROI images → sent to server by `evaluate_vlm.py` → get predicted decisions

**Computed offline (NOT in CSV):**
- VLM predicted decision (PASS/FAIL/UNKNOWN)
- VLM confidence, summary, findings
- Accuracy, Precision, Recall, F1 — from `evaluate_vlm.py`
- BERTScore — compares VLM summary to your `notes` caption
- LLM-as-judge score — from `evaluate_vlm_llm_judge.py`

### How to verify during collection
```bash
grep "vlm_eval" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
# → ground_truth_value column should say "PASS" or "FAIL" (not N/A)
# → notes column should have a human-readable description (not empty)
```

### Scenarios needed (per object type)
| Object | PASS setup | FAIL setup | Min images each |
|---|---|---|---|
| fire_extinguisher | on wall, nothing blocking | box/chair directly in front | 10 |
| emergency_exit | clear walkway | chairs stacked blocking door | 10 |
| door | normal open or closed | objects leaning against it | 10 |
| main_cylinder | dry floor | small water puddle near base | 10 |

---

## PRIORITY 6 — OCCLUSION EVALUATION

### What we evaluate with this dataset
**Maximum tolerable occlusion before detection/IBVS fails.**  
Produces: Detection rate vs occlusion curve, IBVS success rate vs occlusion.  
Also: SSIM/AlexNet drop vs occlusion (from images vs reference).

### Parameters that must be stored

**In CSV (automatic):**
- `converged` — key: at what % does success rate drop below 80%?
- `detection_confidence` — key: how does YOLO confidence drop?
- `final_error_px` — does error increase with occlusion?
- `occlusion_pct` — **critical: enter the exact level you applied**

**As image files (automatic):**
- Logitech ROI images → compared to reference for SSIM drop

**You enter:**
- `occlusion_pct` — must match what you physically applied

### How to verify during collection
```bash
grep "occlusion" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
# → occlusion_pct column should have: 0, 25, 50, 75 (not all zeros)
# → detection_confidence should DROP as occlusion_pct increases
# → converged should go False at some point (that's the finding!)
```

### Levels: 0%, 25%, 50%, 75%, 90%
- Object: fire_extinguisher (easiest to partially cover)
- Distance: 2.0m, angle: 0°
- Cover bottom-up with cardboard
- 10 images per level

---

## PRIORITY 7 — MULTI-OBJECT SCENES

### What we evaluate with this dataset
**ByteTrack multi-object tracking correctness.**  
Confirms system handles 2+ objects: correct class labels, correct inspection order, correct count.

### Parameters that must be stored

**In CSV (automatic):**
- `objects_inspected` — should equal number of objects in scene
- `n_objects` — **enter how many objects you placed**
- `converged` — did IBVS succeed for each object?

**You enter:**
- `n_objects` — how many objects actually in scene (2 or 3)
- `notes` — describe the scene ("2 fire extinguishers side by side, 0.5m apart")

### How to verify during collection
```bash
grep "multi_object" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv
# → n_objects should be 2 or 3
# → objects_inspected should match n_objects (if both detected and converged)
```

---

## QUICK VERIFICATION CHECKLIST — Run after every session

```bash
# 1. Is the CSV growing?
wc -l ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv

# 2. Are ibvs values real (not all 0.0)?
awk -F',' '{print $10,$12,$13}' ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv | tail -10
# → ibvs_time_s (col 10) should NOT be all 0.0
# → final_error_px (col 12) should NOT be all 0.0
# → converged (col 13) should be True or False

# 3. Are images being saved?
ls ~/Documents/Visual_Inspection_ws/captures/inspection/ | wc -l
# → Should increase after each capture session

# 4. For gauge session — ground truth filled in?
grep "gauge_accuracy" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv | awk -F',' '{print $18}'
# → Should show actual numbers (2.5, 4.0...) NOT "N/A"

# 5. For VLM session — PASS/FAIL and caption filled in?
grep "vlm_eval" ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv | awk -F',' '{print $18,$19}'
# → Should show "PASS" or "FAIL" and a text caption
```

---

## AFTER COLLECTION — Export everything

```bash
# From Jetson, copy to laptop:
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/

scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/captures/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/captures/
```

Then run offline evaluation scripts on laptop/server:
- `evaluate_image_quality.py` → SSIM, PSNR, VIF, AlexNet (needs reference + test images)
- `evaluate_gauge.py` → MAE, RMSE (needs images + ground_truth_value from CSV)
- `evaluate_vlm.py` → Accuracy, Recall, F1 (needs images + PASS/FAIL from CSV)
- `evaluate_vlm_llm_judge.py` → LLM judge (needs images + notes captions from CSV)

---

## WHAT THE CURRENT LOG ROW GIVES YOU

```
2026-04-18_19:20:24, reference, fire_ref_28.jpg, fire_extinguisher,
1, 0, center, 0, 1,
ibvs_time=1.12, ibvs_fps=0.0, final_error=9.85, converged=True,
coarse=0.0, pipeline=0.0, conf=0.8975, inspected=1, N/A,
```

| Field | Value | Usable? |
|---|---|---|
| ibvs_time_s | 1.12 s | ✅ Real — use for Table 1 |
| final_error_px | 9.85 px | ✅ Real — use for Table 1 |
| converged | True | ✅ Real — use for success rate |
| detection_confidence | 0.8975 | ✅ Real — use for Table 2 |
| ibvs_fps | 0.0 | ⚠️ Bug — skip this column |
| coarse_time_s | 0.0 | ⚠️ Not instrumented — skip |
| pipeline_time_s | 0.0 | ⚠️ Bug — skip this column |
| SSIM / PSNR / VIF | not in CSV | → compute from saved images offline |

**The reference session images that are saving alongside this log ARE sufficient.  
Continue collecting — you are not losing data.**

---

*Last updated: 2026-04-22 | Robot: Go2 + Jetson Orin Nano*