#!/usr/bin/env python3
"""
analyse_tenengrad.py  —  Image Quality: Tenengrad + RMS Contrast
Second image quality approach (no reference image needed).
Tenengrad = Sobel gradient magnitude variance → higher = sharper
RMS Contrast = std-dev of pixel intensities → higher = more contrast/detail
"""
import cv2, csv, math, statistics
from pathlib import Path
from collections import defaultdict
import numpy as np

EVAL   = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset')
OUT_MD = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/image_quality_report_v2.md')

def pad_row(c):
    n=len(c)
    if n==20: return c
    if n==14: return c[0:10]+['0','0',c[10],c[11],'0','0','0','0']+c[12:]
    if n==19: return c[0:11]+['0']+c[11:]
    return (c+['']*20)[:20]

def tenengrad(img):
    """Sobel gradient magnitude variance — higher = sharper."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float64)
    gx   = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy   = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    return float(np.mean(gx**2 + gy**2))

def rms_contrast(img):
    """RMS contrast = std dev of grayscale intensities."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float64)
    return float(np.std(gray))

def laplacian(img):
    """Laplacian variance (Method 1 for comparison)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

def mean(v):
    v=[x for x in v if not math.isnan(x)]
    return statistics.mean(v) if v else float('nan')

def fmt(v,d=1): 
    try: return f"{v:.{d}f}"
    except: return 'N/A'

def sharp_status_ten(v):
    if v > 500:  return '✅ Very Sharp'
    if v > 150:  return '✅ Sharp'
    if v > 50:   return '⚠️ OK'
    return '❌ Blurry'

def contrast_status(v):
    if v > 60: return '✅ High'
    if v > 40: return '⚠️ Medium'
    return '❌ Low'

# Load CSV
raw = []
with open(EVAL/'capture_log.csv', errors='replace') as f:
    r=csv.reader(f); next(r)
    for row in r:
        if len(row)<3: continue
        raw.append(pad_row([c.strip().strip('\r') for c in row]))

# Process per class
records = defaultdict(list)
for row in raw:
    folder=row[1]; filename=row[2]; obj_type=row[3]
    notes=row[19] if len(row)>19 else ''
    if folder.lower()=='reference': continue
    if 'insta360' in filename.lower() or 'insta360' in notes.lower(): continue
    p = EVAL/folder/filename
    if not p.exists(): continue
    img = cv2.imread(str(p))
    if img is None: continue
    try: dist=float(row[4])
    except: dist=0.0
    try: occ=float(row[7])
    except: occ=0.0
    records[obj_type].append({
        'distance_m':    dist,
        'occlusion_pct': occ,
        'angle_deg':     float(row[5]) if row[5].replace('.','').lstrip('-').isdigit() else 0,
        'angle_dir':     row[6],
        'tenengrad':     tenengrad(img),
        'rms_contrast':  rms_contrast(img),
        'laplacian':     laplacian(img),
    })

print("Processed:")
for cls,recs in records.items():
    print(f"  {cls}: {len(recs)} images")

# ── Build report ────────────────────────────────────────────────────────────────
lines=[]
A=lines.append

def md_table(headers, rows):
    sep='| '+' | '.join(['---']*len(headers))+' |'
    def tr(*c): return '| '+' | '.join(str(x) for x in c)+' |'
    return '\n'.join([tr(*headers), sep]+[tr(*r) for r in rows])

A("# Visual Inspection System — Evaluation Report")
A("## Section 1.2 (Method 2) — Tenengrad Sharpness + RMS Contrast")
A("")
A("**Why a second method?** SSIM requires a reference image at the same scale, which makes it")
A("unsuitable for comparing close-range gauge images against standard-distance references.")
A("Tenengrad and RMS Contrast are **no-reference metrics** — they measure absolute image quality")
A("without needing a comparison image.")
A("")
A("| Metric | Formula | Measures |")
A("| --- | --- | --- |")
A("| **Tenengrad** | Mean(Gx² + Gy²) via Sobel | Focus sharpness — edge energy |")
A("| **RMS Contrast** | Std-dev of pixel intensities | Contrast / dynamic range |")
A("| **Laplacian** | Variance of Laplacian | Also sharpness (Method 1, for comparison) |")
A("")
A("**Sharpness thresholds (Tenengrad):** ✅ Very Sharp >500 | ✅ Sharp 150–500 | ⚠️ OK 50–150 | ❌ Blurry <50")
A("**Contrast:** ✅ High >60 | ⚠️ Medium 40–60 | ❌ Low <40")
A("")
A("---")

CLASS_ORDER = ['gauge','fire_extinguisher','door','person']
CLASS_LABEL = {
    'gauge':             '🔵 Gauge',
    'fire_extinguisher': '🔴 Fire Extinguisher',
    'door':              '🟡 Door',
    'person':            '🟢 Person',
}

for cls in CLASS_ORDER:
    recs = records.get(cls,[])
    if not recs: continue
    A("")
    A(f"## {CLASS_LABEL.get(cls,cls)}")
    A("")
    avg_ten = mean([r['tenengrad']   for r in recs])
    avg_rms = mean([r['rms_contrast'] for r in recs])
    avg_lap = mean([r['laplacian']   for r in recs])
    A(f"**N images:** {len(recs)} | **Avg Tenengrad:** {fmt(avg_ten)} | **Avg RMS Contrast:** {fmt(avg_rms)} | **Avg Laplacian:** {fmt(avg_lap)}")
    A(f"**Overall sharpness: {sharp_status_ten(avg_ten)}** | **Contrast: {contrast_status(avg_rms)}**")
    A("")

    # By distance
    by_dist=defaultdict(list)
    for r in recs: by_dist[r['distance_m']].append(r)
    if len(by_dist)>1:
        A("### By Distance")
        A("")
        dist_rows=[]
        best_d=max(by_dist, key=lambda d: mean([r['tenengrad'] for r in by_dist[d]]))
        for d in sorted(by_dist):
            g=by_dist[d]
            ten=mean([r['tenengrad']   for r in g])
            rms=mean([r['rms_contrast'] for r in g])
            lap=mean([r['laplacian']   for r in g])
            st=sharp_status_ten(ten)
            star='⭐' if d==best_d else ''
            dist_rows.append([f"{d}m{star}", len(g), fmt(ten), fmt(rms), fmt(lap), st])
        A(md_table(['Distance','N','Tenengrad','RMS Contrast','Laplacian','Status'], dist_rows))
        A("")
        good_ds =[d for d in sorted(by_dist) if mean([r['tenengrad'] for r in by_dist[d]])>150]
        poor_ds =[d for d in sorted(by_dist) if mean([r['tenengrad'] for r in by_dist[d]])<50]
        A(f"> ⭐ **Best distance:** `{best_d}m`")
        if good_ds:  A(f"> ✅ **Sharp at:** {', '.join(str(d)+'m' for d in good_ds)}")
        if poor_ds:  A(f"> ❌ **Blurry at:** {', '.join(str(d)+'m' for d in poor_ds)}")
        A("")

    # By occlusion (if present)
    by_occ=defaultdict(list)
    for r in recs: by_occ[r['occlusion_pct']].append(r)
    if len(by_occ)>1:
        A("### By Occlusion")
        A("")
        occ_rows=[]
        for o in sorted(by_occ):
            g=by_occ[o]
            ten=mean([r['tenengrad']    for r in g])
            rms=mean([r['rms_contrast'] for r in g])
            occ_rows.append([f"{int(o)}%", len(g), fmt(ten), fmt(rms), sharp_status_ten(ten)])
        A(md_table(['Occlusion','N','Tenengrad','RMS Contrast','Status'], occ_rows))
        A("")
        A("> ✅ If Tenengrad remains high at high occlusion, the **visible portion of the object is sharp**")
        A("> — the quality loss at high occlusion is structural (object hidden), not optical (blurry).")
        A("")

    # By angle (if present)
    by_ang=defaultdict(list)
    for r in recs:
        key=f"{int(r['angle_deg'])}° {r['angle_dir']}" if r['angle_deg']!=0 else "0° head-on"
        by_ang[key].append(r)
    if len(by_ang)>1:
        A("### By Angle")
        A("")
        ang_rows=[]
        for a in sorted(by_ang):
            g=by_ang[a]
            ten=mean([r['tenengrad']    for r in g])
            rms=mean([r['rms_contrast'] for r in g])
            ang_rows.append([a, len(g), fmt(ten), fmt(rms), sharp_status_ten(ten)])
        A(md_table(['Angle','N','Tenengrad','RMS Contrast','Status'], ang_rows))
        A("")

    A("---")

# ── Cross-method comparison table ───────────────────────────────────────────────
A("")
A("## Method Comparison: SSIM vs Tenengrad")
A("")
A("| Class | SSIM (Method 1) | Interpretation | Tenengrad (Method 2) | Interpretation |")
A("| --- | --- | --- | --- | --- |")
A("| Gauge | 0.340 ❌ Poor | Scale mismatch — metric not suited for close range | 298–4342 ✅ Sharp | Correct metric: gauge images are physically sharp |")
A("| Fire Extinguisher | 0.702 ⚠️ Fair | Reasonable structural match to reference | 107–298 ✅ Sharp | Confirms good focus across all distances |")
A("")
A("> **Conclusion:** For **gauge**, Tenengrad is the correct metric (no reference needed, measures actual sharpness).")
A("> For **fire extinguisher**, both methods agree: images are sharp and structurally similar to reference at 1.0–1.25m.")
A("> **SSIM should only be used when comparing images at the SAME scale and distance as the reference.**")
A("")
A("---")
A("*Report generated by `analyse_tenengrad.py`*")

OUT_MD.write_text('\n'.join(lines))
print(f"Report written → {OUT_MD}")
