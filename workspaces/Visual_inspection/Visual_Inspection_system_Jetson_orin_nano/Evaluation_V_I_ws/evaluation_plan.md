# Visual Inspection System — Full Evaluation Plan

**Robot:** Go2 quadruped with Jetson Orin Nano, Insta360 (tall pole mount), Logitech C920 on 3D-printed pan-tilt  
**System:** Insta360 → YOLO → Coarse (polynomial) → IBVS → Capture → Server (Gauge / VLM)

---

## Why This Evaluation Plan

The system has **three distinct pipelines** to evaluate:
1. **ROI Capture Pipeline** (Jetson) — does it reliably point the camera and get a usable image?
2. **Gauge Reading Pipeline** (Server) — is the analog reading numerically accurate?
3. **VLM Decision Pipeline** (Server) — are the inspection decisions (PASS/FAIL) correct?

Each pipeline has different failure modes and needs different metrics. The metrics your friend suggested are **all valid** — we just need to map each one to the right pipeline. Some (SSIM, PSNR, VIF, AlexNet cosine) measure **image quality**. Others (BERTScore, LLM-as-judge) measure **language output correctness**. Several (occlusion, angle, distance) are **robustness test conditions** rather than metrics themselves.

---

## PART 1: ROI Capture Pipeline Evaluation

**Goal:** Prove that the Insta360 → YOLO → Coarse → IBVS → Logitech capture chain reliably delivers a high-quality, well-centered image of the target object to the downstream pipelines.

---

### 1.1 IBVS Centering Performance

**What it is:** How accurately and how quickly does the IBVS PID loop center the target object in the Logitech frame?

**Metrics:**

| Metric | Description | How to compute | Target |
|--------|-------------|----------------|--------|
| Convergence Time (s) | Time from coarse position → final error < 10px | Log `time.time()` at coarse done and IBVS done | < 5s |
| Final Pixel Error (px) | Distance of object center from frame center at convergence | `sqrt(ex² + ey²)` at the moment convergence is declared | < 10px |
| Convergence Rate (%) | Percentage of runs that successfully converge within 40s | `n_success / n_total * 100` | > 90% |
| IBVS FPS | Detection frames per second during IBVS loop | Count frames per second in the IBVS while loop | ~15-20 FPS |

**Test conditions:**

| Variable | Levels |
|----------|--------|
| Distance from target | 1 m, 2 m, 3 m |
| Object class | gauge, fire_extinguisher, door |
| Approach angle | 0° (head-on), 15°, 30°, 45° |
| Occlusion level | 0% (clear), 25%, 50% occluded |
| Scene | 1 object, 2 objects, 3 objects |

**How to collect data (add to `ibvs_action_server.py`):**

```python
import csv
from pathlib import Path

eval_log = Path('~/eval_results/capture_eval.csv').expanduser()
eval_log.parent.mkdir(exist_ok=True)

# After IBVS completes:
with open(eval_log, 'a', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        datetime.now().isoformat(),
        cls_name,          # object class
        distance_m,        # set manually before each run (1/2/3)
        approach_angle,    # set manually (0/15/30/45)
        occlusion_pct,     # set manually (0/25/50)
        n_objects,         # how many objects in scene
        ibvs_time_s,       # seconds to converge
        final_pixel_error, # px at convergence
        converged,         # True or False
        coarse_time_s,     # time for coarse stage
    ])
```

---

### 1.2 Image Quality Metrics

**Goal:** Prove that the captured Logitech image is sharp enough, bright enough, and similar enough to a reference to be useful as input to the gauge/VLM pipelines.

First, capture a **reference image** for each object: place the object at 1m, run IBVS, capture after full autofocus — this is your "gold standard" image. All subsequent captured images are compared to this reference.

---

#### 1.2.1 SSIM — Structural Similarity Index

**What it is:** Compares two images across three channels: luminance, contrast, and structural information. SSIM = 1 means identical images. It correlates well with human perception of image quality.

**Why relevant here:** If the IBVS centering is slightly off, or the autofocus didn't work, or there is motion blur, SSIM will drop. Tells us if the captured ROI is visually close to the ideal reference.

**Range:** -1 to 1 (higher is better). Typically > 0.7 is considered acceptable.

**How to compute:**
```python
from skimage.metrics import structural_similarity as ssim
import cv2

ref = cv2.imread('reference_gauge.jpg', cv2.IMREAD_GRAYSCALE)
cap = cv2.imread('captured_gauge.jpg', cv2.IMREAD_GRAYSCALE)
# Resize to same dimensions
cap_r = cv2.resize(cap, (ref.shape[1], ref.shape[0]))
score, _ = ssim(ref, cap_r, full=True)
print(f"SSIM: {score:.4f}")
```

**Expected target:** SSIM > 0.70 for all successful captures.

---

#### 1.2.2 PSNR — Peak Signal-to-Noise Ratio

**What it is:** Measures image quality in decibels (dB). Comes from signal processing — compares the max possible pixel value (the "peak signal") to the noise (pixel-level differences between captured and reference). Higher dB = less noise/distortion.

**Why relevant here:** Detects blurriness, noise, or exposure issues in the captured image. Directly affects whether OCR can read gauge numbers.

**Scale:** Typically:
- > 40 dB = excellent quality
- 30–40 dB = good
- 20–30 dB = acceptable  
- < 20 dB = poor

**How to compute:**
```python
import cv2

ref = cv2.imread('reference_gauge.jpg')
cap = cv2.imread('captured_gauge.jpg')
cap_r = cv2.resize(cap, (ref.shape[1], ref.shape[0]))
psnr_val = cv2.PSNR(ref, cap_r)
print(f"PSNR: {psnr_val:.2f} dB")
```

**Expected target:** PSNR > 30 dB for gauge images to be readable.

---

#### 1.2.3 YCbCr Y-Channel (Luminance) Comparison

**What it is:** Convert both images from BGR to YCbCr colour space. **Y = luminance (brightness/illuminance), Cb = blue-difference chrominance, Cr = red-difference chrominance.** By extracting only the Y channel, we isolate brightness differences from colour differences — this is important because our inspection objects are judged in terms of their features, not their colour.

**Why relevant here:** The Logitech camera may auto-expose differently at different distances or angles. Two structurally identical captures with different brightness will have low SSIM/PSNR even though the content is the same. Y-channel comparison strips out colour and focuses on lighting quality and sharpness.

**How to compute:**
```python
import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim

ref = cv2.imread('reference_gauge.jpg')
cap = cv2.imread('captured_gauge.jpg')
cap_r = cv2.resize(cap, (ref.shape[1], ref.shape[0]))

# Convert to YCbCr
ref_ycbcr = cv2.cvtColor(ref, cv2.COLOR_BGR2YCrCb)
cap_ycbcr = cv2.cvtColor(cap_r, cv2.COLOR_BGR2YCrCb)

# Extract Y (luminance) channel
ref_y = ref_ycbcr[:, :, 0]
cap_y = cap_ycbcr[:, :, 0]

# Compare luminance directly
y_ssim, _ = ssim(ref_y, cap_y, full=True)
y_mean_diff = abs(float(ref_y.mean()) - float(cap_y.mean()))

print(f"Y-channel SSIM: {y_ssim:.4f}")
print(f"Y-channel mean brightness: ref={ref_y.mean():.1f}, cap={cap_y.mean():.1f}, diff={y_mean_diff:.1f}")
```

**Reports:** Mean Y value (is image too dark / too bright?), Y-channel SSIM (structural similarity ignoring colour). Ideal Y range: 80–180 (avoids very dark < 80 or overexposed > 200).

---

#### 1.2.4 VIF — Visual Information Fidelity

**What it is:** A more advanced perceptual image quality metric based on **natural scene statistics** and the **human visual system model (HVS)**. Unlike PSNR (which counts pixel errors) or SSIM (which compares structural features), VIF models how much *visual information* from the reference image is preserved in the test image, accounting for how the human eye perceives that information. Range: 0 to 1+ (higher is better, 1 = reference itself).

**Why relevant here:** VIF is more sensitive to blur and distortion in fine details (like gauge numbers or scale markings) than SSIM or PSNR. A gauge image that looks "okay" by eye but has blurred numbers will score low on VIF — which correctly predicts OCR failure.

**How to compute:**
```python
# Install: pip install piq
import piq
import torch
import cv2

ref = cv2.imread('reference_gauge.jpg')
cap = cv2.imread('captured_gauge.jpg')
cap_r = cv2.resize(cap, (ref.shape[1], ref.shape[0]))

# Convert to torch tensors [B, C, H, W], float32, 0-1
def to_tensor(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype('float32') / 255.0
    return torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)

ref_t = to_tensor(ref)
cap_t = to_tensor(cap_r)

vif_score = piq.vif_p(ref_t, cap_t, data_range=1.0)
print(f"VIF: {vif_score.item():.4f}")
```

**Expected target:** VIF > 0.4 for images with readable gauge detail.

---

#### 1.2.5 AlexNet Cosine Similarity

**What it is:** Extract deep CNN features from both images using **AlexNet** (pretrained on ImageNet) and measure **cosine similarity** between the feature vectors. Cosine similarity = dot product of two unit vectors, ranging from -1 (opposite) to 1 (identical direction).  

Unlike pixel metrics (SSIM, PSNR), AlexNet features capture **semantic and perceptual similarity** — two images of the same gauge from slightly different angles will have high cosine similarity even though pixels differ. This validates that the camera is pointing at the **correct object**, not just capturing a similar-brightness image.

**Why relevant here:** Validates the full system end-to-end — after Insta360 detection, coarse pointing, and IBVS, is the captured image semantically about the same object as the reference? High cosine similarity confirms IBVS converged onto the right target.

**How to compute:**
```python
import torch
import torchvision.models as models
import torchvision.transforms as transforms
import torch.nn.functional as F
from PIL import Image

# Load AlexNet (pretrained), remove final classifier layers
alexnet = models.alexnet(pretrained=True)
feature_extractor = torch.nn.Sequential(*list(alexnet.features.children()))
feature_extractor.eval()

preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

def get_features(img_path):
    img = Image.open(img_path).convert('RGB')
    t = preprocess(img).unsqueeze(0)
    with torch.no_grad():
        feat = feature_extractor(t)
    return feat.flatten()

ref_feat = get_features('reference_gauge.jpg')
cap_feat = get_features('captured_gauge.jpg')

cos_sim = F.cosine_similarity(ref_feat.unsqueeze(0), cap_feat.unsqueeze(0))
print(f"AlexNet Cosine Similarity: {cos_sim.item():.4f}")
```

**Expected target:** Cosine similarity > 0.85 confirms camera is looking at the correct object with good centering.

---

### 1.3 Robustness Evaluation

#### 1.3.1 Occlusion Robustness

**What it is:** Systematically test how much occlusion the system can tolerate before detection fails, IBVS fails to converge, or image quality drops below threshold.

**Setup:** Use tape/cardboard/boxes to cover known fractions of the target object during testing.

| Occlusion Level | How to simulate | Measure |
|-----------------|----------------|---------|
| 0% (baseline) | Nothing blocking | All metrics as baseline |
| 25% | Cover bottom quarter of object | Does YOLO still detect? IBVS still converge? |
| 50% | Cover half of object | Same questions |
| 75% | Cover three-quarters | At what point does detection fail? |

**Metrics to record at each occlusion level:**
- YOLO detection confidence score
- IBVS convergence success rate (%)
- Final pixel error at convergence
- SSIM of captured image vs reference
- AlexNet cosine similarity

**Expected outcome:** Plot occlusion % vs detection confidence. Find the **critical occlusion threshold** where detection drops below 0.3 confidence or convergence rate drops below 80%.

---

#### 1.3.2 Angle Robustness

**What it is:** Test system performance when the robot approaches the target at different horizontal and vertical angles. The Insta360 + coarse mapping was calibrated roughly — offsets at large angles will be bigger, requiring more IBVS correction.

| Test | Angles |
|------|--------|
| Horizontal approach angle | 0°, 15°, 30°, 45° left/right |
| Vertical tilt (height difference) | Target at eye level, 30cm above, 30cm below |
| Combined | 30° horizontal + 20cm height offset |

**Metrics:**
- Coarse position offset (how far off is the first servo move?)
- IBVS convergence time at each angle (longer = more correction needed)
- Final pixel error at convergence
- Success rate per angle

**Expected outcome:** Convergence time increases with angle. Find the **maximum usable angle** where system still converges reliably.

---

#### 1.3.3 Distance Evaluation

| Distance | Expected FPS | Expected Conv. Time | Expected Image Quality |
|----------|-------------|---------------------|----------------------|
| 1 m | ~20 FPS | < 3s | High (SSIM > 0.80) |
| 2 m | ~20 FPS | 3-5s | Good (SSIM > 0.70) |
| 3 m | ~18 FPS | 4-7s | Acceptable (SSIM > 0.60) |
| 4 m | ~18 FPS | 5-9s | Poor (likely gauge OCR fails) |

---

## PART 2: Gauge Reading Pipeline Evaluation

**Goal:** Prove that the 10-step geometric+DL pipeline accurately reads analog gauge values.

### Pipeline Reminder (10 steps):
1. YOLO detects gauge face bounding box
2. Crop + square pad + resize to 448×448
3. Heatmap key point model detects notch/tick marks + start/end scale markers
4. Algebraic ellipse fitted to notch ring
5. Zero-point reference angle computed from start/end markers
6. PaddleOCR reads scale numbers (warp + rotation corrected)
7. Decimal point detector checks ROIs around each number, corrects (e.g. "25"→"2.5")
8. Segmentation model finds needle pixels, fits a line
9. OCR numbers + needle projected onto ellipse in polar coordinates (angle per number + needle angle)
10. RANSAC line fit maps (angle→value), needle angle → final reading

### 2.1 Accuracy Metrics

| Metric | Formula | Target |
|--------|---------|--------|
| MAE (Mean Absolute Error) | mean(|pred - true|) | < 5% of gauge full range |
| RMSE (Root Mean Squared Error) | sqrt(mean((pred-true)²)) | < 5% of gauge full range |
| MAPE (Mean Absolute % Error) | mean(|pred-true|/true × 100%) | < 5% |
| Success Rate | n_readings / n_total | > 80% |
| Over/Under Read Bias | mean(pred - true) | Near 0 (no systematic bias) |

### 2.2 Failure Mode Breakdown

Track which step fails using the error log files the gauge pipeline writes:

| Failure Mode | How detected | Expected frequency |
|-------------|-------------|-------------------|
| Gauge not detected | YOLO conf < threshold | < 5% |
| Ellipse fit failed ("not an ellipse" error) | Exception in step 4 | < 10% |
| OCR found no numbers | `OCR_NONE_DETECTED_KEY` in errors | < 10% |
| Segmentation failed (no needle) | `SEGMENTATION_FAILED_KEY` | < 5% |
| RANSAC insufficient points | Too few OCR numbers | < 10% |

### 2.3 Data Collection Script

```python
# evaluate_gauge.py — run on server with server running
import requests, json, time, csv, statistics
from pathlib import Path

SERVER = "http://localhost:8001"

# Build ground truth dict: {image_path: true_reading}
# You create this by setting gauge to known positions and photographing
ground_truth = {
    "eval_images/gauge_0.5.jpg": 0.5,
    "eval_images/gauge_1.0.jpg": 1.0,
    "eval_images/gauge_2.5.jpg": 2.5,
    "eval_images/gauge_5.0.jpg": 5.0,
    # ... add all your test images
}

results = []
for img_path, true_val in ground_truth.items():
    with open(img_path, 'rb') as f:
        r = requests.post(f"{SERVER}/api/v1/jobs",
                         files={"file": f},
                         data={"object_type": "gauge"})
    job_id = r.json()["job_id"]
    t0 = time.time()

    while True:
        job = requests.get(f"{SERVER}/api/v1/jobs/{job_id}").json()
        if job["status"] in ["DONE", "FAILED"]: break
        time.sleep(2)

    elapsed = round(time.time() - t0, 1)
    pred_val, unit, err = None, None, None

    if job["status"] == "DONE":
        data = json.loads(job["result_json"])
        pred_val = data.get("reading")
        unit = data.get("unit")
        if pred_val is not None:
            err = abs(pred_val - true_val)

    results.append({"image": img_path, "true": true_val, "predicted": pred_val,
                     "unit": unit, "error": err, "time_s": elapsed,
                     "status": job["status"]})
    print(f"  {'✓' if err and err < 0.5 else '✗'} {img_path}: "
          f"true={true_val}, pred={pred_val}, err={err}")

# Save CSV
with open("gauge_eval_results.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

# Summary
good = [r for r in results if r["error"] is not None]
errors = [r["error"] for r in good]
print(f"\n=== GAUGE EVALUATION ===")
print(f"Total: {len(results)} | Success: {len(good)} ({len(good)/len(results)*100:.0f}%)")
print(f"MAE:  {statistics.mean(errors):.4f}")
print(f"RMSE: {(sum(e**2 for e in errors)/len(errors))**0.5:.4f}")
print(f"Bias: {statistics.mean(r['predicted']-r['true'] for r in good):.4f}")
print(f"Avg time: {statistics.mean(r['time_s'] for r in results):.1f}s")
```

---

## PART 3: VLM Pipeline Evaluation

**Goal:** Prove that the Google Gemini 2.5 Flash based VLM correctly makes PASS/FAIL/UNKNOWN inspection decisions across all object types.

### Routing Tree Reminder:
- `gauge` → gauge pipeline (not VLM)
- `fire_extinguisher` → VLM with specific prompt (present? accessible?)
- `door` → VLM with specific prompt (open/closed state?)
- `emergency_exit` → VLM with specific prompt (exit blocked?)
- `main_cylinder` → VLM with specific prompt (oil leak?)
- `unknown` → VLM with smart auto-detect prompt → identifies object → applies rules → if gauge, recommends gauge pipeline

### 3.1 Standard Classification Metrics

| Metric | Formula | Target |
|--------|---------|--------|
| Overall Accuracy | % correct decisions | > 85% |
| Precision (FAIL class) | TP_fail / (TP_fail + FP_fail) | > 80% |
| **Recall (FAIL class)** | TP_fail / (TP_fail + FN_fail) | **> 90%** (safety critical — must not miss failures) |
| F1 Score (FAIL) | 2 × Prec × Rec / (Prec + Rec) | > 0.85 |
| Response Time | seconds from POST to DONE | < 15s |
| Consistency | same image × 3 → same decision? | > 95% |

> **Why recall matters more than precision for FAIL:** Missing a blocked fire exit (False Negative) is a safety risk. Incorrectly flagging a clear exit as blocked (False Positive) just means manual re-check. So **recall > precision** priority.

### 3.2 BERTScore — Text Quality Evaluation

**What it is:** A metric from NLP that measures how well two pieces of text match semantically, using **BERT embeddings**. Instead of counting matching words (like BLEU), BERTScore compares contextual meaning. Gives Precision, Recall, F1 between 0 and 1.

**Why relevant here:** The VLM returns not just a decision but a natural language `summary` (e.g. "Fire extinguisher is present and the path is clear") and `findings` list. We want to verify these text outputs are **accurate and complete** descriptions of the scene — not just that the binary PASS/FAIL is right.

**How to use it:**
1. Build a **captioned dataset**: for each test image, write a ground truth description (e.g. "Red fire extinguisher mounted on wall, clearly accessible, no objects blocking it")
2. Compare VLM's generated `summary` against your written caption using BERTScore

```python
# Install: pip install bert-score
from bert_score import score

# Ground truth descriptions (you write these for each image)
references = [
    "Fire extinguisher is mounted on wall and fully accessible with no obstructions",
    "Emergency exit door is blocked by a large cardboard box placed directly in front",
    "No oil leak visible, floor surface around main cylinder is dry and clean",
]

# VLM output summaries (from result_json["summary"])
candidates = [
    "Fire extinguisher appears present and accessible with clear access path",
    "Exit appears to be obstructed by objects placed in front of the door",
    "Area appears dry with no visible oil leaks or puddles near the cylinder",
]

P, R, F1 = score(candidates, references, lang="en", verbose=False)
for i, (p, r, f) in enumerate(zip(P, R, F1)):
    print(f"Image {i+1}: BERTScore P={p:.3f} R={r:.3f} F1={f:.3f}")
```

**Expected target:** BERTScore F1 > 0.80 means VLM descriptions semantically match ground truth.

---

### 3.3 LLM as a Judge

**What it is:** Instead of comparing VLM text to a fixed reference, you use a **stronger LLM (e.g. GPT-4o or Gemini 1.5 Pro)** as an evaluator. You give it: (1) the image, (2) a ground truth human caption describing the scene, (3) the VLM's full output. The judge LLM then rates whether the inspection report is CORRECT / PARTIALLY_CORRECT / INCORRECT and explains why.

**Why better than BERTScore:** BERTScore only compares text. LLM-as-judge can look at the actual image AND the VLM output and reason about whether the evidence, findings, and decision are all consistent with what is visible. More robust to paraphrasing and catches logical errors.

**How to implement:**

```python
import google.generativeai as genai
import json
from PIL import Image

genai.configure(api_key="YOUR_GEMINI_KEY")
judge_model = genai.GenerativeModel("gemini-1.5-pro")

def judge_vlm_output(image_path, ground_truth_caption, vlm_output_dict):
    img = Image.open(image_path)
    
    judge_prompt = f"""You are an expert evaluator for an industrial visual inspection AI system.

GROUND TRUTH (what a human expert says about this image):
"{ground_truth_caption}"

VLM INSPECTION REPORT (what the AI system produced):
Decision: {vlm_output_dict.get('decision')}
Confidence: {vlm_output_dict.get('confidence')}
Summary: {vlm_output_dict.get('summary')}
Findings: {vlm_output_dict.get('findings')}
Evidence: {vlm_output_dict.get('evidence')}

Your task:
1. Look at the image carefully
2. Compare the AI report against the ground truth
3. Rate the report as: CORRECT / PARTIALLY_CORRECT / INCORRECT
4. Explain specifically what is right and what is wrong

Respond in JSON: {{"rating": "CORRECT/PARTIALLY_CORRECT/INCORRECT", "explanation": "...", "decision_correct": true/false, "findings_accurate": true/false}}
"""
    
    response = judge_model.generate_content([judge_prompt, img])
    return json.loads(response.text)

# Example usage
result = judge_vlm_output(
    "eval_images/fire_ext_blocked.jpg",
    "Fire extinguisher is present on wall but blocked by a large blue bin placed in front of it",
    {"decision": "FAIL", "confidence": 0.82,
     "summary": "Fire extinguisher is present but access is blocked",
     "findings": ["Red extinguisher visible", "Blue container blocking access"],
     "evidence": {"present": True, "blocked": True}}
)
print(result)
# → {"rating": "CORRECT", "explanation": "Decision and findings match the scene", ...}
```

**Report:** % CORRECT, % PARTIALLY_CORRECT, % INCORRECT per object_type.

---

### 3.4 VLM Consistency Test

**What it is:** Send the same image 3 times and check if the decision, confidence, and key findings are consistent.

**Why it matters:** Gemini API is non-deterministic. If the system gives PASS one time and FAIL another on the same image, it is unreliable.

```python
# Send same image 3 times
decisions = []
for _ in range(3):
    # POST image → wait → get result
    decision = result_json["decision"]
    decisions.append(decision)
    time.sleep(2)

consistency = len(set(decisions)) == 1  # all same?
print(f"Consistency: {decisions} → {'✓ Consistent' if consistency else '✗ Inconsistent'}")
```

**Target:** > 95% of images should give the same decision across 3 runs.

---

### 3.5 VLM Data Collection Script

```python
# evaluate_vlm.py
import requests, json, time, csv
from pathlib import Path
from collections import defaultdict

SERVER = "http://localhost:8001"

# Ground truth: {image_path: (object_type, expected_decision, human_caption)}
ground_truth = {
    "vlm_eval/ext_clear.jpg":    ("fire_extinguisher", "PASS",    "Extinguisher accessible"),
    "vlm_eval/ext_blocked.jpg":  ("fire_extinguisher", "FAIL",    "Extinguisher blocked by box"),
    "vlm_eval/exit_clear.jpg":   ("emergency_exit",   "PASS",    "Exit path clear"),
    "vlm_eval/exit_blocked.jpg": ("emergency_exit",   "FAIL",    "Exit blocked by equipment"),
    "vlm_eval/oil_leak.jpg":     ("main_cylinder",    "FAIL",    "Oil puddle on floor"),
    "vlm_eval/no_leak.jpg":      ("main_cylinder",    "PASS",    "Floor dry and clean"),
    "vlm_eval/door_open.jpg":    ("door",             "PASS",    "Door open state visible"),
    "vlm_eval/door_closed.jpg":  ("door",             "PASS",    "Door closed state visible"),
    # Add at least 10 per class
}

results = []
for img_path, (obj_type, true_dec, caption) in ground_truth.items():
    with open(img_path, 'rb') as f:
        r = requests.post(f"{SERVER}/api/v1/jobs",
                         files={"file": f},
                         data={"object_type": obj_type})
    job_id = r.json()["job_id"]
    t0 = time.time()
    while True:
        job = requests.get(f"{SERVER}/api/v1/jobs/{job_id}").json()
        if job["status"] in ["DONE", "FAILED"]: break
        time.sleep(1)

    elapsed = round(time.time() - t0, 1)
    data = json.loads(job.get("result_json", "{}"))
    pred = data.get("decision", "UNKNOWN")
    conf = data.get("confidence", 0.0)
    correct = (pred == true_dec)

    results.append({"image": img_path, "type": obj_type, "true": true_dec,
                     "pred": pred, "conf": conf, "correct": correct,
                     "time_s": elapsed, "caption": caption,
                     "summary": data.get("summary", "")[:100]})
    print(f"  {'✓' if correct else '✗'} {obj_type}: {true_dec} → {pred} ({conf:.2f})")

# Summary per type
by_type = defaultdict(list)
for r in results:
    by_type[r["type"]].append(r)

print(f"\n=== VLM EVALUATION ===")
for t, rows in by_type.items():
    acc = sum(r["correct"] for r in rows) / len(rows) * 100
    avg_t = sum(r["time_s"] for r in rows) / len(rows)
    print(f"  {t}: {acc:.0f}% accuracy | avg {avg_t:.1f}s")

# FAIL recall/precision
tp = sum(1 for r in results if r["true"]=="FAIL" and r["pred"]=="FAIL")
fp = sum(1 for r in results if r["true"]!="FAIL" and r["pred"]=="FAIL")
fn = sum(1 for r in results if r["true"]=="FAIL" and r["pred"]!="FAIL")
prec = tp/(tp+fp) if tp+fp > 0 else 0
rec  = tp/(tp+fn) if tp+fn > 0 else 0
print(f"\nFAIL detection — Precision: {prec:.2f}  Recall: {rec:.2f}")
```

---

## PART 4: Full End-to-End System Tests

| Test Scenario | Steps | Pass Criteria |
|---------------|-------|---------------|
| Full gauge inspection | Physical gauge → Insta360 detects → servo moves → IBVS → capture → send to server → reading | Reading within 10% of true value |
| Front zone detection | Object in front (cy < 200px) | `success=True, object_in_back=False` |
| Back zone detection | Object behind robot | `success=False, object_in_back=True` → BT rotates |
| Multi-object (2 objects) | 2 objects in Insta360 view | Both inspected, 2 ROI sets captured |
| Object disappears during IBVS | Remove object after coarse | `failed_reason="ibvs_timeout"` |
| Empty scene | No objects in view | `failed_reason="no_detection"` |
| MQTT delivery | Full pipeline with internet | ThingsBoard shows telemetry with image |
| Unknown object → VLM routing | Send image with `object_type=unknown` | VLM correctly identifies and inspects |

---

## PART 5: Summary Results Tables

Fill these in after running evaluations:

### Table 1 — Capture Pipeline

| Object | Distance | Conv. Time | Final Error | Success % | SSIM | PSNR | AlexNet Sim |
|--------|----------|-----------|-------------|-----------|------|------|-------------|
| Gauge | 1m | __ s | __ px | _% | __ | __ dB | __ |
| Gauge | 2m | __ s | __ px | _% | __ | __ dB | __ |
| Gauge | 3m | __ s | __ px | _% | __ | __ dB | __ |
| Fire ext | 1m | __ s | __ px | _% | __ | __ dB | __ |
| Fire ext | 2m | __ s | __ px | _% | __ | __ dB | __ |

### Table 2 — Occlusion Robustness

| Occlusion | YOLO Conf | IBVS Success % | SSIM | AlexNet Sim |
|-----------|----------|----------------|------|-------------|
| 0% | __ | _% | __ | __ |
| 25% | __ | _% | __ | __ |
| 50% | __ | _% | __ | __ |
| 75% | __ | _% | __ | __ |

### Table 3 — Gauge Reading Accuracy

| Metric | Value |
|--------|-------|
| N images tested | __ |
| Success rate | _% |
| MAE | __ units |
| RMSE | __ units |
| MAPE | _% |
| Avg processing time | __ s |

### Table 4 — VLM Decision Accuracy

| Object Type | N | Accuracy | Precision | Recall | F1 | BERTScore F1 | LLM Judge CORRECT% |
|-------------|---|----------|-----------|--------|----|--------------|--------------------|
| fire_extinguisher | __ | _% | __ | __ | __ | __ | _% |
| emergency_exit | __ | _% | __ | __ | __ | __ | _% |
| main_cylinder | __ | _% | __ | __ | __ | __ | _% |
| door | __ | _% | __ | __ | __ | __ | _% |
| unknown | __ | _% | __ | __ | __ | __ | _% |
| **Overall** | __ | _% | __ | __ | __ | __ | _% |

---

## Dependencies to Install

```bash
# On server (vi_server venv)
pip install scikit-image   # SSIM, PSNR
pip install piq            # VIF
pip install torch torchvision  # AlexNet cosine similarity
pip install bert-score     # BERTScore
pip install google-generativeai  # LLM-as-judge (already installed)

# On Jetson (for capture eval logging)
# No extra installs — uses standard library (csv, time, datetime)
```

---

## Files to Create

| File | Location | Purpose |
|------|----------|---------|
| `evaluate_gauge.py` | `vi_server/` | Gauge accuracy evaluation script |
| `evaluate_vlm.py` | `vi_server/` | VLM decision accuracy evaluation script |
| `evaluate_image_quality.py` | `vi_server/` | SSIM, PSNR, VIF, YCBCR, AlexNet for captured images |
| `evaluate_vlm_llm_judge.py` | `vi_server/` | LLM-as-judge scoring for VLM outputs |
| `gauge_eval_results.csv` | `vi_server/` | Raw gauge results |
| `vlm_eval_results.csv` | `vi_server/` | Raw VLM results |
| `capture_eval.csv` | Jetson `~/eval_results/` | IBVS convergence metrics per run |

---

*Plan version: 2026-03-12 | Author: Dinethra | Robot: Go2 + Jetson Orin Nano*