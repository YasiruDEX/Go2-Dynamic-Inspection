#!/usr/bin/env python3
"""
analyse_image_quality.py
========================
ROI Capture Pipeline — Image Quality Metrics
Section 1.2 of evaluation_plan.md

Metrics computed:
  • SSIM  — Structural Similarity Index
  • PSNR  — Peak Signal-to-Noise Ratio (dB)
  • Y-channel SSIM & brightness difference (YCbCr luminance)

For each test image the best-matching reference image
(same object class, highest SSIM) is used as the gold standard.

Usage:
    python3 analyse_image_quality.py
"""

import csv, math, statistics
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim_fn

# ── Paths ─────────────────────────────────────────────────────────────────────
EVAL    = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset')
LOG_CSV = EVAL / 'capture_log.csv'
REF_DIR = EVAL / 'reference'
OUT_MD  = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/image_quality_report.md')

TARGET_SIZE = (640, 480)   # resize all to same dims before comparison

# ── Load + preprocess reference images per class ───────────────────────────────
print("Loading reference images...")
refs = defaultdict(list)   # {class: [bgr_img, ...]}
for p in sorted(REF_DIR.glob('*.jpg')):
    name = p.stem.lower()
    if   'fire' in name:   cls = 'fire_extinguisher'
    elif 'gauge' in name:  cls = 'gauge'
    elif 'door' in name:   cls = 'door'
    elif 'person' in name: cls = 'person'
    else:                  cls = 'unknown'
    img = cv2.imread(str(p))
    if img is not None:
        refs[cls].append(cv2.resize(img, TARGET_SIZE))

for cls, imgs in refs.items():
    print(f"  {cls}: {len(imgs)} reference images")

# ── Metric functions ──────────────────────────────────────────────────────────
def to_gray(bgr): return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
def to_ycbcr(bgr): return cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)

def compute_ssim(ref, test):
    r = to_gray(ref); t = to_gray(test)
    score, _ = ssim_fn(r, t, full=True)
    return float(score)

def compute_psnr(ref, test):
    val = cv2.PSNR(ref, test)
    return float(val)   # dB; inf means identical

def compute_y_metrics(ref, test):
    ry = to_ycbcr(ref)[:,:,0].astype(float)
    ty = to_ycbcr(test)[:,:,0].astype(float)
    y_ssim, _ = ssim_fn(ry.astype(np.uint8), ty.astype(np.uint8), full=True)
    brightness_diff = abs(ry.mean() - ty.mean())
    ref_brightness  = ry.mean()
    return float(y_ssim), float(brightness_diff), float(ref_brightness)

def best_ref_metrics(obj_class, test_img):
    """Compare test_img against all reference images of same class.
    Return the BEST (highest SSIM) match metrics."""
    class_refs = refs.get(obj_class, refs.get('fire_extinguisher', []))
    if not class_refs:
        return None

    best = {'ssim': -1}
    for ref_img in class_refs:
        s = compute_ssim(ref_img, test_img)
        if s > best['ssim']:
            p             = compute_psnr(ref_img, test_img)
            y_ssim, y_bd, y_ref = compute_y_metrics(ref_img, test_img)
            best = {
                'ssim':      s,
                'psnr':      p,
                'y_ssim':    y_ssim,
                'y_bdiff':   y_bd,
                'y_brightness': y_ref,
            }
    return best

# ── CSV row helpers (same mixed-format fix) ────────────────────────────────────
COLS20 = 20
def pad_row(cols):
    n = len(cols)
    if n == 20: return cols
    if n == 14: return cols[0:10]+['0.0','0.0',cols[10],cols[11],'0.0','0.0','0.0','0']+cols[12:]
    if n == 19: return cols[0:11]+['0.0']+cols[11:]
    if n == 16: return cols[0:10]+['0.0']+cols[10:12]+[cols[12]]+cols[13:15]+['0.0','0']+[cols[15]]+['']
    return (cols+['']*20)[:20]

# ── Load CSV ───────────────────────────────────────────────────────────────────
print("\nLoading CSV...")
raw_rows = []
with open(LOG_CSV, errors='replace') as fh:
    reader = csv.reader(fh)
    next(reader)
    for row in reader:
        if len(row) < 3: continue
        raw_rows.append(pad_row([c.strip().strip('\r') for c in row]))

# ── Process: only IBVS/Logitech images (not insta360, not reference) ──────────
print("Computing metrics (this may take a minute)...")
results = []
skipped = 0

for row in raw_rows:
    folder   = row[1]
    filename = row[2]
    obj_type = row[3]
    notes    = row[19] if len(row) > 19 else ''

    if folder.lower() == 'reference': continue
    if 'insta360' in filename.lower() or 'insta360' in notes.lower(): continue

    img_path = EVAL / folder / filename
    if not img_path.exists(): skipped += 1; continue

    img = cv2.imread(str(img_path))
    if img is None: skipped += 1; continue
    img = cv2.resize(img, TARGET_SIZE)

    m = best_ref_metrics(obj_type, img)
    if m is None: skipped += 1; continue

    def flt(idx, d=0.0):
        try: return float(row[idx])
        except: return d

    converged = str(row[13]).strip().lower() == 'true'

    results.append({
        'folder':       folder,
        'filename':     filename,
        'object_type':  obj_type,
        'distance_m':   flt(4),
        'occlusion_pct':flt(7),
        'angle_deg':    flt(5),
        'angle_dir':    row[6],
        'converged':    converged,
        'confidence':   flt(16),
        **m
    })

print(f"  Processed: {len(results)} images | Skipped: {skipped}")

# ── Helper stats ───────────────────────────────────────────────────────────────
def mean(vals):
    v = [x for x in vals if x is not None and not math.isnan(x) and not math.isinf(x)]
    return statistics.mean(v) if v else float('nan')

def fmt(v, d=3):
    try: return f"{v:.{d}f}"
    except: return 'N/A'

def table_row(*cells): return '| ' + ' | '.join(str(c) for c in cells) + ' |'

def md_table(headers, rows):
    sep   = '| ' + ' | '.join(['---']*len(headers)) + ' |'
    lines = [table_row(*headers), sep]
    for r in rows: lines.append(table_row(*r))
    return '\n'.join(lines)

# ── Overall metrics ────────────────────────────────────────────────────────────
avg_ssim = mean([r['ssim']   for r in results])
avg_psnr = mean([r['psnr']   for r in results])
avg_yssim= mean([r['y_ssim'] for r in results])
avg_ybr  = mean([r['y_brightness'] for r in results])

# ── By distance ───────────────────────────────────────────────────────────────
by_dist = defaultdict(list)
for r in results:
    by_dist[r['distance_m']].append(r)

dist_rows = []
for d in sorted(by_dist):
    grp = by_dist[d]
    ssim_status = '✅' if mean([r['ssim'] for r in grp]) >= 0.70 else ('⚠️' if mean([r['ssim'] for r in grp]) >= 0.50 else '❌')
    dist_rows.append([
        f"{d}m", len(grp),
        fmt(mean([r['ssim']    for r in grp])),
        fmt(mean([r['psnr']    for r in grp]), 1) + ' dB',
        fmt(mean([r['y_ssim']  for r in grp])),
        fmt(mean([r['y_bdiff'] for r in grp]), 1) + ' lum',
        ssim_status,
    ])

# ── By occlusion ──────────────────────────────────────────────────────────────
by_occ = defaultdict(list)
for r in results:
    by_occ[r['occlusion_pct']].append(r)

occ_rows = []
for o in sorted(by_occ):
    grp = by_occ[o]
    ssim_v = mean([r['ssim'] for r in grp])
    psnr_v = mean([r['psnr'] for r in grp])
    status = '✅' if ssim_v >= 0.70 else ('⚠️' if ssim_v >= 0.50 else '❌')
    occ_rows.append([
        f"{int(o)}%", len(grp),
        fmt(ssim_v),
        fmt(psnr_v, 1) + ' dB',
        fmt(mean([r['y_ssim'] for r in grp])),
        status,
    ])

# ── By angle ──────────────────────────────────────────────────────────────────
by_angle = defaultdict(list)
for r in results:
    key = f"{int(r['angle_deg'])}° {r['angle_dir']}" if r['angle_deg'] != 0 else "0° (head-on)"
    by_angle[key].append(r)

angle_rows = []
for a in sorted(by_angle):
    grp = by_angle[a]
    angle_rows.append([
        a, len(grp),
        fmt(mean([r['ssim']   for r in grp])),
        fmt(mean([r['psnr']   for r in grp]), 1) + ' dB',
        fmt(mean([r['y_ssim'] for r in grp])),
    ])

# ── Write report ───────────────────────────────────────────────────────────────
print("\nWriting report...")
lines = []
A = lines.append

A("# Visual Inspection System — Evaluation Report")
A("## Part 1: ROI Capture Pipeline")
A("### Section 1.2 — Image Quality Metrics (SSIM / PSNR / Y-channel)")
A("")
A("**Method:** Each captured Logitech image is compared against the best-matching reference")
A("image of the same object class using SSIM, PSNR, and YCbCr luminance comparison.")
A("Only Logitech captures (not Insta360 fallbacks) are included. Images must exist on disk.")
A("")
A("**Thresholds:**")
A("- SSIM > 0.70 = ✅ acceptable | 0.50–0.70 = ⚠️ degraded | <0.50 = ❌ poor")
A("- PSNR > 30 dB = good | 20–30 dB = acceptable | <20 dB = poor")
A("")
A("---")
A("")
A("## 1. Overall Image Quality")
A("")
A(md_table(
    ['Metric', 'Value', 'Threshold', 'Status'],
    [
        ['**Avg SSIM**',       fmt(avg_ssim),        '> 0.70',  '✅' if avg_ssim >= 0.70 else '⚠️'],
        ['**Avg PSNR**',       fmt(avg_psnr,1)+' dB','> 30 dB', '✅' if avg_psnr >= 30 else '⚠️'],
        ['**Avg Y-channel SSIM**', fmt(avg_yssim),   '> 0.70',  '✅' if avg_yssim >= 0.70 else '⚠️'],
        ['**Avg Reference Brightness (Y)**', fmt(avg_ybr,1), '80–180', '✅' if 80<=avg_ybr<=180 else '⚠️'],
        ['**N images evaluated**', len(results), '—', '—'],
    ]
))
A("")
A("---")
A("")
A("## 2. Image Quality by Distance")
A("")
A(md_table(
    ['Distance', 'N', 'SSIM', 'PSNR', 'Y-SSIM', 'Y-Brightness Δ', 'Status'],
    dist_rows
))
A("")
A("> As distance increases, SSIM is expected to drop slightly due to lower resolution")
A("> of the object in frame. PSNR may drop if the camera autofocus is less accurate.")
A("")
A("---")
A("")
A("## 3. Image Quality by Occlusion")
A("")
A(md_table(
    ['Occlusion', 'N', 'SSIM', 'PSNR', 'Y-SSIM', 'Status'],
    occ_rows
))
A("")
A("> At high occlusion levels, SSIM drops because the visible object area is smaller")
A("> and background pixels dominate the structural comparison.")
A("")
A("---")
A("")
A("## 4. Image Quality by Approach Angle")
A("")
A(md_table(
    ['Angle', 'N', 'SSIM', 'PSNR', 'Y-SSIM'],
    angle_rows
))
A("")
A("> Off-axis angles produce perspective distortion compared to the head-on reference,")
A("> which causes SSIM to drop even if the image is sharp and well-exposed.")
A("")
A("---")
A("")
# Summary
A("## 5. Key Findings")
A("")
A(f"| KPI | Measured | Target | Status |")
A(f"|-----|----------|--------|--------|")
A(f"| Overall SSIM | {fmt(avg_ssim)} | > 0.70 | {'✅' if avg_ssim>=0.70 else '⚠️'} |")
A(f"| Overall PSNR | {fmt(avg_psnr,1)} dB | > 30 dB | {'✅' if avg_psnr>=30 else '⚠️'} |")
A(f"| Y-channel SSIM | {fmt(avg_yssim)} | > 0.70 | {'✅' if avg_yssim>=0.70 else '⚠️'} |")
A(f"| Reference brightness (Y) | {fmt(avg_ybr,1)} | 80–180 | {'✅' if 80<=avg_ybr<=180 else '⚠️'} |")
A("")
A("---")
A("")
A("*Report generated by `analyse_image_quality.py` — Visual Inspection System Evaluation*")

OUT_MD.write_text('\n'.join(lines))
print(f"Report written → {OUT_MD}")
