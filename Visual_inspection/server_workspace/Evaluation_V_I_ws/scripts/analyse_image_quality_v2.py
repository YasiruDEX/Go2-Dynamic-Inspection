#!/usr/bin/env python3
"""
analyse_image_quality_v2.py
============================
Image Quality — Per-Class Analysis (SSIM / PSNR / Y-channel)

Each class is compared ONLY against its own reference images.
Classes:
  gauge            → 0.3 m – 0.6 m  (close-range OCR target)
  fire_extinguisher → 0.75 m – 4 m  (medium/long range safety object)
"""

import csv, math, statistics
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim_fn

# ── Paths ──────────────────────────────────────────────────────────────────────
EVAL    = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset')
LOG_CSV = EVAL / 'capture_log.csv'
REF_DIR = EVAL / 'reference'
OUT_MD  = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/image_quality_report.md')

TARGET_SIZE = (640, 480)

# ── Helpers ────────────────────────────────────────────────────────────────────
def mean(vals):
    v = [x for x in vals if x is not None and not math.isnan(x) and not math.isinf(x)]
    return statistics.mean(v) if v else float('nan')

def fmt(v, d=3):
    try: return f"{v:.{d}f}"
    except: return 'N/A'

def status_ssim(v):
    if v >= 0.75: return '✅ Good'
    if v >= 0.60: return '⚠️ Fair'
    return '❌ Poor'

def table_row(*cells): return '| ' + ' | '.join(str(c) for c in cells) + ' |'
def md_table(headers, rows):
    sep = '| ' + ' | '.join(['---']*len(headers)) + ' |'
    return '\n'.join([table_row(*headers), sep] + [table_row(*r) for r in rows])

# ── Load reference images per class ───────────────────────────────────────────
print("Loading reference images...")
CLASS_MAP = {
    'fire': 'fire_extinguisher',
    'gauge': 'gauge',
    'door': 'door',
    'person': 'person',
}
refs = defaultdict(list)
for p in sorted(REF_DIR.glob('*.jpg')):
    stem = p.stem.lower()
    for k, cls in CLASS_MAP.items():
        if stem.startswith(k):
            img = cv2.imread(str(p))
            if img is not None:
                refs[cls].append(cv2.resize(img, TARGET_SIZE))
            break
for cls, imgs in refs.items():
    print(f"  {cls}: {len(imgs)} reference images")

# ── Metric functions ───────────────────────────────────────────────────────────
def to_gray(b): return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)

def compute_ssim(ref, test):
    score, _ = ssim_fn(to_gray(ref), to_gray(test), full=True)
    return float(score)

def compute_psnr(ref, test):
    return float(cv2.PSNR(ref, test))

def compute_y(ref, test):
    ry = cv2.cvtColor(ref,  cv2.COLOR_BGR2YCrCb)[:,:,0].astype(float)
    ty = cv2.cvtColor(test, cv2.COLOR_BGR2YCrCb)[:,:,0].astype(float)
    y_ssim, _ = ssim_fn(ry.astype(np.uint8), ty.astype(np.uint8), full=True)
    return float(y_ssim), float(abs(ry.mean() - ty.mean())), float(ry.mean())

def best_metrics(cls, test_img):
    class_refs = refs.get(cls, [])
    if not class_refs: return None
    best = {'ssim': -1}
    for ref_img in class_refs:
        s = compute_ssim(ref_img, test_img)
        if s > best['ssim']:
            p = compute_psnr(ref_img, test_img)
            ys, yd, yb = compute_y(ref_img, test_img)
            best = dict(ssim=s, psnr=p, y_ssim=ys, y_bdiff=yd, y_brightness=yb)
    return best

# ── CSV loading ────────────────────────────────────────────────────────────────
def pad_row(cols):
    n = len(cols)
    if n == 20: return cols
    if n == 14: return cols[0:10]+['0.0','0.0',cols[10],cols[11],'0.0','0.0','0.0','0']+cols[12:]
    if n == 19: return cols[0:11]+['0.0']+cols[11:]
    return (cols+['']*20)[:20]

print("\nLoading CSV...")
raw_rows = []
with open(LOG_CSV, errors='replace') as fh:
    reader = csv.reader(fh)
    next(reader)
    for row in reader:
        if len(row) < 3: continue
        raw_rows.append(pad_row([c.strip().strip('\r') for c in row]))

# ── Process ────────────────────────────────────────────────────────────────────
print("Computing metrics per class...")
records = defaultdict(list)  # {class: [rec,...]}
skipped = 0

for row in raw_rows:
    folder   = row[1]; filename = row[2]; obj_type = row[3]
    notes    = row[19] if len(row)>19 else ''
    if folder.lower() == 'reference': continue
    if 'insta360' in filename.lower() or 'insta360' in notes.lower(): continue
    img_path = EVAL / folder / filename
    if not img_path.exists(): skipped += 1; continue
    img = cv2.imread(str(img_path))
    if img is None: skipped += 1; continue
    img = cv2.resize(img, TARGET_SIZE)
    m = best_metrics(obj_type, img)
    if m is None: skipped += 1; continue

    def flt(idx, d=0.0):
        try: return float(row[idx])
        except: return d

    records[obj_type].append({
        'folder': folder, 'filename': filename,
        'distance_m':    flt(4),
        'occlusion_pct': flt(7),
        'angle_deg':     flt(5),
        'angle_dir':     row[6],
        'converged':     str(row[13]).strip().lower()=='true',
        **m
    })

for cls, recs in records.items():
    print(f"  {cls}: {len(recs)} images")
print(f"  Skipped: {skipped}")

# ═══════════════════════════════════════════════════════════════════════════════
# Build report
# ═══════════════════════════════════════════════════════════════════════════════
lines = []
A = lines.append

A("# Visual Inspection System — Evaluation Report")
A("## Part 1: ROI Capture Pipeline")
A("### Section 1.2 — Image Quality per Class (SSIM / PSNR / Y-channel)")
A("")
A("> **Important:** Each class is compared **only against its own reference images**.")
A("> Gauge images → gauge references only. Fire extinguisher → fire extinguisher references only.")
A("> This ensures the metric reflects actual image quality, not class appearance difference.")
A("")
A("**Quality thresholds (SSIM):**")
A("- ✅ **Good** ≥ 0.75 — system can reliably use this image for detection/OCR")
A("- ⚠️ **Fair** 0.60–0.75 — usable with some degradation")
A("- ❌ **Poor** < 0.60 — object too distorted/blurred/scaled for reliable use")
A("")
A("---")
A("")

# ── Per-class sections ─────────────────────────────────────────────────────────
CLASS_LABELS = {
    'gauge':             '🔵 Gauge (OCR / Reading Accuracy Target)',
    'fire_extinguisher': '🔴 Fire Extinguisher (Safety Object)',
    'door':              '🟡 Door',
    'person':            '🟢 Person',
}

for cls in ['gauge', 'fire_extinguisher', 'door', 'person']:
    recs = records.get(cls, [])
    if not recs: continue

    label = CLASS_LABELS.get(cls, cls)
    A(f"## {label}")
    A("")

    # --- Overall for this class ---
    avg_ssim = mean([r['ssim'] for r in recs])
    avg_psnr = mean([r['psnr'] for r in recs])
    avg_ys   = mean([r['y_ssim'] for r in recs])
    A(f"**Total images:** {len(recs)} | **Avg SSIM:** {fmt(avg_ssim)} | "
      f"**Avg PSNR:** {fmt(avg_psnr,1)} dB | **Status:** {status_ssim(avg_ssim)}")
    A("")

    # --- By distance ---
    by_dist = defaultdict(list)
    for r in recs:
        by_dist[r['distance_m']].append(r)

    dist_rows = []
    best_dist_label = None; best_dist_ssim = -1
    for d in sorted(by_dist):
        grp  = by_dist[d]
        s    = mean([r['ssim']    for r in grp])
        p    = mean([r['psnr']    for r in grp])
        ys   = mean([r['y_ssim']  for r in grp])
        ybd  = mean([r['y_bdiff'] for r in grp])
        st   = status_ssim(s)
        dist_rows.append([f"{d}m", len(grp), fmt(s), fmt(p,1)+' dB', fmt(ys), fmt(ybd,1), st])
        if s > best_dist_ssim:
            best_dist_ssim = s; best_dist_label = f"{d}m"

    A("### Quality by Distance")
    A("")
    A(md_table(
        ['Distance','N','SSIM','PSNR','Y-SSIM','Brightness Δ','Quality'],
        dist_rows
    ))
    A("")

    # Determine operational range (where SSIM >= 0.60)
    good_dists  = [d for d in sorted(by_dist) if mean([r['ssim'] for r in by_dist[d]]) >= 0.75]
    fair_dists  = [d for d in sorted(by_dist) if 0.60 <= mean([r['ssim'] for r in by_dist[d]]) < 0.75]
    poor_dists  = [d for d in sorted(by_dist) if mean([r['ssim'] for r in by_dist[d]]) < 0.60]

    A(f"> 🎯 **Best distance:** `{best_dist_label}` (SSIM = {fmt(best_dist_ssim)})")
    if good_dists:
        A(f"> ✅ **Optimal range:** {', '.join(str(d)+'m' for d in good_dists)} — image quality is good")
    if fair_dists:
        A(f"> ⚠️ **Fair range:** {', '.join(str(d)+'m' for d in fair_dists)} — usable but degraded")
    if poor_dists:
        A(f"> ❌ **Avoid:** {', '.join(str(d)+'m' for d in poor_dists)} — quality too low for reliable use")
    A("")

    # --- By occlusion (if any) ---
    has_occ = [r for r in recs if r['occlusion_pct'] > 0]
    occ_data = defaultdict(list)
    for r in recs:
        occ_data[r['occlusion_pct']].append(r)

    if len(occ_data) > 1:
        A("### Quality by Occlusion")
        A("")
        occ_rows = []
        for o in sorted(occ_data):
            grp = occ_data[o]
            s   = mean([r['ssim'] for r in grp])
            p   = mean([r['psnr'] for r in grp])
            ys  = mean([r['y_ssim'] for r in grp])
            occ_rows.append([f"{int(o)}%", len(grp), fmt(s), fmt(p,1)+' dB', fmt(ys), status_ssim(s)])
        A(md_table(['Occlusion','N','SSIM','PSNR','Y-SSIM','Quality'], occ_rows))
        A("")

        # Find critical threshold
        for o in sorted(occ_data):
            s = mean([r['ssim'] for r in occ_data[o]])
            if s < 0.60:
                A(f"> ⚠️ **Occlusion limit:** Quality drops below acceptable at **{int(o)}% occlusion**")
                break
        else:
            A(f"> ✅ Quality remains acceptable across all tested occlusion levels")
        A("")

    # --- By angle (if any angles > 0) ---
    has_angles = [r for r in recs if r['angle_deg'] > 0]
    if has_angles:
        A("### Quality by Approach Angle")
        A("")
        by_angle = defaultdict(list)
        for r in recs:
            key = f"{int(r['angle_deg'])}° {r['angle_dir']}" if r['angle_deg']!=0 else "0° head-on"
            by_angle[key].append(r)
        ang_rows = []
        for a in sorted(by_angle):
            grp = by_angle[a]
            s   = mean([r['ssim'] for r in grp])
            p   = mean([r['psnr'] for r in grp])
            ang_rows.append([a, len(grp), fmt(s), fmt(p,1)+' dB', status_ssim(s)])
        A(md_table(['Angle','N','SSIM','PSNR','Quality'], ang_rows))
        A("")
        best_a = max(by_angle, key=lambda a: mean([r['ssim'] for r in by_angle[a]]))
        A(f"> 🎯 **Best angle:** `{best_a}` — closest to reference perspective gives highest quality")
        A("")

    A("---")
    A("")

# ── Cross-class summary ────────────────────────────────────────────────────────
A("## Summary: Recommended Operating Conditions")
A("")
A("| Class | Best Distance | Optimal Range | Occlusion Limit | Notes |")
A("| --- | --- | --- | --- | --- |")

for cls in ['gauge', 'fire_extinguisher']:
    recs = records.get(cls, [])
    if not recs: continue
    by_dist = defaultdict(list)
    for r in recs: by_dist[r['distance_m']].append(r)
    best_d  = max(by_dist, key=lambda d: mean([r['ssim'] for r in by_dist[d]]))
    good_ds = [d for d in sorted(by_dist) if mean([r['ssim'] for r in by_dist[d]]) >= 0.75]
    fair_ds = [d for d in sorted(by_dist) if 0.60 <= mean([r['ssim'] for r in by_dist[d]]) < 0.75]

    occ_data = defaultdict(list)
    for r in recs: occ_data[r['occlusion_pct']].append(r)
    occ_limit = 'N/A'
    for o in sorted(occ_data):
        if mean([r['ssim'] for r in occ_data[o]]) < 0.60:
            occ_limit = f"<{int(o)}%"; break

    if good_ds:
        range_str = f"{min(good_ds)}m – {max(good_ds)}m"
    elif fair_ds:
        range_str = f"{min(fair_ds)}m – {max(fair_ds)}m (fair)"
    else:
        range_str = 'limited data'

    notes_str = 'Close-range OCR' if cls=='gauge' else 'Safety inspection'
    A(f"| {cls.replace('_',' ').title()} | **{best_d}m** | {range_str} | {occ_limit} | {notes_str} |")

A("")
A("---")
A("")
A("*Report generated by `analyse_image_quality_v2.py`*")

OUT_MD.write_text('\n'.join(lines))
print(f"\nReport written → {OUT_MD}")
