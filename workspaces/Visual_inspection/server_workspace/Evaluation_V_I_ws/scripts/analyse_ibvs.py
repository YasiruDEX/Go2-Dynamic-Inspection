#!/usr/bin/env python3
"""
analyse_ibvs.py
===============
ROI Capture Pipeline — IBVS Centering Performance Analysis
Section 1.1 of evaluation_plan.md

Usage:
    python3 analyse_ibvs.py

Outputs:
    ibvs_report.md   — full markdown report with tables and findings
"""

import csv, math, statistics
from pathlib import Path
from collections import defaultdict

# ── Paths ─────────────────────────────────────────────────────────────────────
EVAL    = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset')
LOG_CSV = EVAL / 'capture_log.csv'
OUT_MD  = Path('/home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/ibvs_report.md')

# ── 20-column header (canonical) ──────────────────────────────────────────────
COLS = [
    'timestamp','folder','filename','object_type','distance_m',
    'angle_deg','angle_direction','occlusion_pct','n_objects',
    'ibvs_time_s','ibvs_fps','initial_error_px','final_error_px','converged',
    'coarse_time_s','pipeline_time_s','detection_confidence','objects_inspected',
    'ground_truth_value','notes'
]

def pad_row(cols):
    n = len(cols)
    if n == 20: return cols
    if n == 14:
        # old: [..., ibvs_time_s, final_error_px, converged, gtv, notes]
        return cols[0:10] + ['0.0','0.0', cols[10], cols[11], '0.0','0.0','0.0','0'] + cols[12:]
    if n == 19:
        # missing initial_error_px at position 11
        return cols[0:11] + ['0.0'] + cols[11:]
    if n == 16:
        return cols[0:10] + ['0.0'] + cols[10:12] + [cols[12]] + cols[13:15] + ['0.0','0'] + [cols[15]] + ['']
    return (cols + [''] * 20)[:20]

def f(v, digits=2):
    """Format float nicely."""
    try: return f"{float(v):.{digits}f}"
    except: return str(v)

# ── Load CSV ──────────────────────────────────────────────────────────────────
print("Loading CSV...")
raw_rows = []
with open(LOG_CSV, errors='replace') as fh:
    reader = csv.reader(fh)
    next(reader)   # skip header
    for row in reader:
        if len(row) < 3: continue
        raw_rows.append(pad_row([c.strip().strip('\r') for c in row]))

print(f"  Raw rows: {len(raw_rows)}")

# ── Parse and filter: only rows where image exists ────────────────────────────
records = []
skipped_no_img  = 0
skipped_ref     = 0

for row in raw_rows:
    folder   = row[1]
    filename = row[2]
    obj_type = row[3]

    # Skip reference images — not evaluation runs
    if folder.strip().lower() == 'reference':
        skipped_ref += 1
        continue

    img_path = EVAL / folder / filename
    if not img_path.exists():
        skipped_no_img += 1
        continue

    def flt(idx, default=0.0):
        try: return float(row[idx])
        except: return default

    converged = str(row[13]).strip().lower() == 'true'
    notes     = row[19] if len(row) > 19 else ''

    rec = {
        'timestamp':    row[0],
        'folder':       folder,
        'filename':     filename,
        'object_type':  obj_type,
        'distance_m':   flt(4),
        'angle_deg':    flt(5),
        'angle_dir':    row[6],
        'occlusion_pct':flt(7),
        'n_objects':    int(flt(8, 1)),
        'ibvs_time_s':  flt(9),
        'ibvs_fps':     flt(10),
        'initial_err_px': flt(11),
        'final_err_px': flt(12),
        'converged':    converged,
        'coarse_time_s':flt(14),
        'pipeline_time_s': flt(15),
        'confidence':   flt(16),
        'objects_inspected': int(flt(17, 0)),
        'gtv':          row[18],
        'notes':        notes,
        'is_insta360':  'insta360' in notes.lower(),
    }
    records.append(rec)

print(f"  After filters: {len(records)} valid records")
print(f"  Skipped (reference): {skipped_ref}")
print(f"  Skipped (no image):  {skipped_no_img}")

# ── Split: IBVS runs vs Insta360 fallback ─────────────────────────────────────
ibvs_records   = [r for r in records if not r['is_insta360']]
insta_records  = [r for r in records if r['is_insta360']]
conv_records   = [r for r in ibvs_records if r['converged']]

print(f"  IBVS runs: {len(ibvs_records)}  (converged: {len(conv_records)})")
print(f"  Insta360 fallback captures: {len(insta_records)}")

# ── Helper ─────────────────────────────────────────────────────────────────────
def mean(vals):
    vals = [v for v in vals if v is not None]
    return statistics.mean(vals) if vals else float('nan')

def stdev(vals):
    vals = [v for v in vals if v is not None]
    return statistics.stdev(vals) if len(vals) >= 2 else float('nan')

def pct(a, b): return (a/b*100) if b else 0.0

def table_row(*cells): return '| ' + ' | '.join(str(c) for c in cells) + ' |'

def md_table(headers, rows):
    sep  = '| ' + ' | '.join(['---']*len(headers)) + ' |'
    lines = [table_row(*headers), sep]
    for row in rows: lines.append(table_row(*row))
    return '\n'.join(lines)

# ═══════════════════════════════════════════════════════════════════════════════
# Section A: Overall IBVS metrics
# ═══════════════════════════════════════════════════════════════════════════════
print("\nComputing overall metrics...")

total_runs      = len(ibvs_records)
n_conv          = len(conv_records)
conv_rate       = pct(n_conv, total_runs)

avg_ibvs_time   = mean([r['ibvs_time_s']    for r in conv_records])
avg_coarse_time = mean([r['coarse_time_s']  for r in conv_records])
avg_pipe_time   = mean([r['pipeline_time_s'] for r in conv_records])
avg_fps         = mean([r['ibvs_fps']        for r in conv_records if r['ibvs_fps'] > 0])
avg_init_err    = mean([r['initial_err_px']  for r in conv_records])
avg_final_err   = mean([r['final_err_px']    for r in conv_records])
avg_conf        = mean([r['confidence']      for r in ibvs_records if r['confidence'] > 0])

err_reductions  = [(r['initial_err_px']-r['final_err_px'])/r['initial_err_px']
                   for r in conv_records if r['initial_err_px'] > 0]
avg_err_red     = mean(err_reductions)*100

# ═══════════════════════════════════════════════════════════════════════════════
# Section B: By distance
# ═══════════════════════════════════════════════════════════════════════════════
print("By distance...")
by_dist = defaultdict(list)
for r in ibvs_records:
    by_dist[r['distance_m']].append(r)

dist_table_rows = []
for dist in sorted(by_dist):
    grp   = by_dist[dist]
    conv  = [r for r in grp if r['converged']]
    n     = len(grp)
    nc    = len(conv)
    row = [
        f"{dist}m", n,
        f"{pct(nc,n):.0f}%",
        f"{mean([r['ibvs_time_s']   for r in conv]):.2f}s" if conv else 'N/A',
        f"{mean([r['coarse_time_s'] for r in conv]):.2f}s" if conv else 'N/A',
        f"{mean([r['pipeline_time_s'] for r in conv]):.2f}s" if conv else 'N/A',
        f"{mean([r['initial_err_px'] for r in conv]):.1f}px" if conv else 'N/A',
        f"{mean([r['final_err_px']   for r in conv]):.1f}px" if conv else 'N/A',
        f"{mean([r['ibvs_fps'] for r in conv if r['ibvs_fps']>0]):.1f}" if conv else 'N/A',
        f"{mean([r['confidence'] for r in grp if r['confidence']>0]):.3f}" if grp else 'N/A',
    ]
    dist_table_rows.append(row)

# ═══════════════════════════════════════════════════════════════════════════════
# Section C: By occlusion
# ═══════════════════════════════════════════════════════════════════════════════
print("By occlusion...")
by_occ = defaultdict(list)
for r in records:   # include insta360 for occlusion analysis
    by_occ[r['occlusion_pct']].append(r)

occ_table_rows = []
for occ in sorted(by_occ):
    grp   = by_occ[occ]
    ibvs  = [r for r in grp if not r['is_insta360']]
    conv  = [r for r in ibvs if r['converged']]
    insta = [r for r in grp if r['is_insta360']]
    n     = len(grp)
    row = [
        f"{int(occ)}%", n, len(ibvs), len(conv), len(insta),
        f"{pct(len(conv),len(ibvs)):.0f}%" if ibvs else 'N/A',
        f"{mean([r['final_err_px']  for r in conv]):.1f}px"  if conv  else 'N/A',
        f"{mean([r['confidence']    for r in ibvs if r['confidence']>0]):.3f}" if ibvs else 'N/A',
        f"{mean([r['ibvs_time_s']   for r in conv]):.2f}s"   if conv  else 'N/A',
    ]
    occ_table_rows.append(row)

# ═══════════════════════════════════════════════════════════════════════════════
# Section D: By angle
# ═══════════════════════════════════════════════════════════════════════════════
print("By angle...")
by_angle = defaultdict(list)
for r in ibvs_records:
    key = f"{int(r['angle_deg'])}° {r['angle_dir']}" if r['angle_deg'] != 0 else "0° (head-on)"
    by_angle[key].append(r)

angle_table_rows = []
for angle in sorted(by_angle):
    grp  = by_angle[angle]
    conv = [r for r in grp if r['converged']]
    n    = len(grp)
    nc   = len(conv)
    row = [
        angle, n,
        f"{pct(nc,n):.0f}%",
        f"{mean([r['ibvs_time_s'] for r in conv]):.2f}s" if conv else 'N/A',
        f"{mean([r['final_err_px'] for r in conv]):.1f}px" if conv else 'N/A',
        f"{mean([r['confidence'] for r in grp if r['confidence']>0]):.3f}" if grp else 'N/A',
    ]
    angle_table_rows.append(row)

# ═══════════════════════════════════════════════════════════════════════════════
# Section E: FPS stability
# ═══════════════════════════════════════════════════════════════════════════════
fps_vals = [r['ibvs_fps'] for r in conv_records if r['ibvs_fps'] > 0]
fps_min  = min(fps_vals) if fps_vals else 0
fps_max  = max(fps_vals) if fps_vals else 0
fps_std  = stdev(fps_vals)

# ═══════════════════════════════════════════════════════════════════════════════
# Write markdown report
# ═══════════════════════════════════════════════════════════════════════════════
print("\nWriting report...")

lines = []
A = lines.append

A("# Visual Inspection System — Evaluation Report")
A("## Part 1: ROI Capture Pipeline")
A("### Section 1.1 — IBVS Centering Performance")
A("")
A(f"**Dataset:** `{LOG_CSV}`  ")
A(f"**Images verified on disk:** Only rows with existing image files were used.  ")
A(f"**Generated:** from `analyse_ibvs.py`")
A("")
A("---")
A("")
A("## Dataset Overview")
A("")
A(md_table(
    ['Metric','Value'],
    [
        ['Total CSV rows (excl. reference)', total_runs + len(insta_records)],
        ['IBVS runs (Logitech captures)', total_runs],
        ['Insta360 fallback captures (IBVS failed / no detection)', len(insta_records)],
        ['Rows skipped (image missing from disk)', skipped_no_img],
        ['Reference images skipped', skipped_ref],
    ]
))
A("")
A("---")
A("")
A("## 1. Overall IBVS Performance")
A("")
A(md_table(
    ['Metric','Value','Notes'],
    [
        ['**Convergence Success Rate**', f"{conv_rate:.1f}% ({n_conv}/{total_runs})", 'Target: >90%'],
        ['**Avg IBVS Time** (converged runs)', f"{avg_ibvs_time:.3f} s", 'Time from coarse done → converged'],
        ['**Avg Coarse Time**', f"{avg_coarse_time:.3f} s", 'Initial servo move to detection point'],
        ['**Avg Total Pipeline Time**', f"{avg_pipe_time:.2f} s", 'Start to capture complete'],
        ['**Coarse / IBVS split**', f"{avg_coarse_time:.2f}s / {avg_ibvs_time:.2f}s", f"Coarse = {pct(avg_coarse_time, avg_coarse_time+avg_ibvs_time):.0f}% of active time"],
        ['**Avg Initial Error** (at detection)', f"{avg_init_err:.1f} px", 'How far off coarse pointing was'],
        ['**Avg Final Error** (at convergence)', f"{avg_final_err:.1f} px", 'Target: <10 px'],
        ['**Avg Error Reduction**', f"{avg_err_red:.1f}%", 'How much IBVS corrected the offset'],
        ['**Avg IBVS FPS**', f"{avg_fps:.1f} fps", 'Target: ~10–15 fps'],
        ['**FPS Range**', f"{fps_min:.1f} – {fps_max:.1f} fps", f"Std dev: {fps_std:.2f}"],
        ['**Avg Detection Confidence**', f"{avg_conf:.4f}", 'YOLO confidence score'],
    ]
))
A("")
A("---")
A("")
A("## 2. Performance by Distance")
A("")
A("Only converged runs used for timing/error metrics. Confidence uses all IBVS runs.")
A("")
A(md_table(
    ['Distance','N runs','Conv. Rate','IBVS Time','Coarse Time','Pipeline Time','Init Err','Final Err','IBVS FPS','Avg Conf'],
    dist_table_rows
))
A("")

# Distance analysis comments
A("> **Observation:**")
A("> - Convergence time is expected to increase at greater distances as the object appears smaller,")
A(">   requiring more servo corrections to center it.")
A("> - Initial error reflects the coarse-mapping accuracy at each distance.")
A("> - FPS should remain stable across distances (it reflects the detection loop speed, not geometry).")
A("")
A("---")
A("")
A("## 3. Occlusion Robustness")
A("")
A("Insta360 fallback images are counted separately — they indicate runs where IBVS could not proceed.")
A("")
A(md_table(
    ['Occlusion','Total','IBVS runs','Conv.','Insta360 fallback','Conv. Rate','Final Err','Conf','IBVS Time'],
    occ_table_rows
))
A("")
A("> **Observation:**")
A("> - At 0% occlusion the system should show maximum convergence rate.")
A("> - As occlusion increases, YOLO confidence drops and IBVS may fail to converge (→ Insta360 fallback).")
A("> - The **critical occlusion threshold** is where convergence rate drops below 80%.")
A("")
A("---")
A("")
A("## 4. Performance by Approach Angle")
A("")
A(md_table(
    ['Angle','N runs','Conv. Rate','IBVS Time','Final Err','Avg Conf'],
    angle_table_rows
))
A("")
A("> **Observation:**")
A("> - Head-on (0°) should give the lowest IBVS time and smallest final error.")
A("> - As angle increases, the coarse mapping error grows (more initial error), requiring more IBVS iterations.")
A("")
A("---")
A("")
A("## 5. IBVS FPS Stability")
A("")
A(md_table(
    ['Metric','Value'],
    [
        ['Mean FPS', f"{mean(fps_vals):.2f}"],
        ['Min FPS', f"{fps_min:.1f}"],
        ['Max FPS', f"{fps_max:.1f}"],
        ['Std Dev', f"{fps_std:.2f}"],
        ['N samples', str(len(fps_vals))],
    ]
))
A("")
A("> **Observation:**")
A("> - FPS reflects how fast the detection loop runs inside the IBVS controller.")
A("> - High std dev indicates inconsistent GPU scheduling, which can cause jerky servo motion.")
A("> - Target range: 10–15 FPS for smooth convergence without oscillation.")
A("")
A("---")
A("")
A("## 6. Insta360 Fallback Summary")
A("")
if insta_records:
    by_reason = defaultdict(int)
    for r in insta_records:
        n = r['notes']
        if 'ibvs_timeout' in n:    by_reason['ibvs_timeout'] += 1
        elif 'no_detection' in n:  by_reason['no_detection'] += 1
        else:                      by_reason['other']         += 1
    A(md_table(
        ['Reason','Count','Meaning'],
        [
            ['ibvs_timeout', by_reason['ibvs_timeout'], 'Object detected by YOLO but IBVS failed to focus'],
            ['no_detection', by_reason['no_detection'], 'Object not detected by YOLO at all'],
            ['other/unknown', by_reason['other'],       'Unclassified failure'],
        ]
    ))
    A("")
    occ_insta = sorted(set(r['occlusion_pct'] for r in insta_records))
    A(f"Insta360 fallbacks occurred at occlusion levels: **{', '.join(str(int(o))+'%' for o in occ_insta)}**")
else:
    A("No Insta360 fallback captures recorded in this dataset.")
A("")
A("---")
A("")
A("## 7. Key Findings & Targets")
A("")
A(f"| KPI | Measured | Target | Status |")
A(f"|-----|----------|--------|--------|")
conv_ok  = '✅' if conv_rate >= 90 else ('⚠️' if conv_rate >= 70 else '❌')
err_ok   = '✅' if avg_final_err < 10 else ('⚠️' if avg_final_err < 15 else '❌')
fps_ok   = '✅' if 10 <= avg_fps <= 20 else '⚠️'
A(f"| Convergence rate | {conv_rate:.1f}% | ≥90% | {conv_ok} |")
A(f"| Final pixel error | {avg_final_err:.1f} px | <10 px | {err_ok} |")
A(f"| IBVS FPS | {avg_fps:.1f} fps | 10–20 fps | {fps_ok} |")
A(f"| Avg pipeline time | {avg_pipe_time:.1f} s | <30 s | {'✅' if avg_pipe_time < 30 else '⚠️'} |")
A("")
A("---")
A("")
A("*Report generated by `analyse_ibvs.py` — Visual Inspection System Evaluation*")

OUT_MD.parent.mkdir(parents=True, exist_ok=True)
OUT_MD.write_text('\n'.join(lines))
print(f"\nReport written → {OUT_MD}")
