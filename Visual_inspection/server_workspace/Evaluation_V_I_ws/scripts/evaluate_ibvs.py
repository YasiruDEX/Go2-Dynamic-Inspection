#!/usr/bin/env python3
"""
evaluate_ibvs.py
Run on LAPTOP. Reads terminal output log OR reads capture_log.csv
to compute IBVS convergence time statistics.

Usage Option 1 — from capture_log.csv:
  python3 scripts/evaluate_ibvs.py \
      --log   eval_dataset/capture_log.csv \
      --out   results/ibvs_eval_results.csv

Usage Option 2 — if you saved terminal output to a file:
  python3 scripts/evaluate_ibvs.py \
      --terminal_log  terminal_output.txt \
      --out           results/ibvs_eval_results.csv
"""

import argparse, csv, re, sys, statistics
from pathlib import Path
from collections import defaultdict

def load_from_csv(log_path):
    """Load IBVS metrics from capture_log.csv."""
    rows = []
    with open(log_path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                ibvs_t = float(row.get('ibvs_time_s', '')) if row.get('ibvs_time_s','').strip() else None
                err_px = float(row.get('final_error_px','')) if row.get('final_error_px','').strip() else None
                conv   = row.get('converged','').strip().lower() in ('true','yes','1')
                rows.append({
                    'folder':    row.get('folder',''),
                    'filename':  row.get('filename',''),
                    'object':    row.get('object_type',''),
                    'distance':  row.get('distance_m',''),
                    'angle':     row.get('angle_deg',''),
                    'direction': row.get('angle_direction',''),
                    'occlusion': row.get('occlusion_pct',''),
                    'ibvs_s':    ibvs_t,
                    'err_px':    err_px,
                    'converged': conv
                })
            except Exception:
                continue
    return rows

def parse_terminal_log(log_path):
    """Parse ibvs_time and final error from saved terminal output."""
    rows = []
    text = Path(log_path).read_text()
    # Look for lines like: IBVS converged: err=8.1px at iter 42
    for m in re.finditer(r'IBVS converged: err=([\d.]+)px at iter (\d+)', text):
        rows.append({'err_px': float(m.group(1)), 'iters': int(m.group(2)),
                     'converged': True, 'ibvs_s': None})
    # Look for timeout lines
    for m in re.finditer(r'IBVS timeout', text):
        rows.append({'err_px': None, 'iters': None, 'converged': False, 'ibvs_s': None})
    return rows

def summarize(rows, group_by=None):
    if group_by:
        groups = defaultdict(list)
        for r in rows:
            groups[r.get(group_by,'unknown')].append(r)
    else:
        groups = {'all': rows}

    print(f'\n  {"Group":<20} {"N":>4} {"Conv%":>6} {"AvgTime":>8} {"AvgErr":>8} {"MaxErr":>8}')
    print(f'  {"-"*20} {"-"*4} {"-"*6} {"-"*8} {"-"*8} {"-"*8}')

    for key, grp in sorted(groups.items()):
        n       = len(grp)
        conv    = [r for r in grp if r['converged']]
        conv_r  = len(conv)/n*100 if n else 0
        times   = [r['ibvs_s'] for r in conv if r.get('ibvs_s')]
        errors  = [r['err_px'] for r in conv if r.get('err_px')]
        avg_t   = f'{statistics.mean(times):.1f}s' if times else 'N/A'
        avg_e   = f'{statistics.mean(errors):.1f}px' if errors else 'N/A'
        max_e   = f'{max(errors):.1f}px' if errors else 'N/A'
        print(f'  {str(key):<20} {n:>4} {conv_r:>5.0f}% {avg_t:>8} {avg_e:>8} {max_e:>8}')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log',            default='eval_dataset/capture_log.csv')
    ap.add_argument('--terminal_log',   default=None)
    ap.add_argument('--out',            default='results/ibvs_eval_results.csv')
    ap.add_argument('--group_by', choices=['distance','angle','occlusion','object'], default=None)
    args = ap.parse_args()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if args.terminal_log and Path(args.terminal_log).exists():
        rows = parse_terminal_log(args.terminal_log)
        print(f'Loaded {len(rows)} IBVS runs from terminal log')
    elif Path(args.log).exists():
        rows = load_from_csv(args.log)
        rows = [r for r in rows if r['ibvs_s'] or r['err_px']]
        print(f'Loaded {len(rows)} rows with IBVS data from capture_log.csv')
    else:
        print(f'No data source found. Provide --log or --terminal_log'); sys.exit(1)

    if not rows: print('No rows with IBVS data found.'); sys.exit(1)

    print(f'\n=== IBVS CONVERGENCE EVALUATION ===')

    # Overall stats
    conv  = [r for r in rows if r['converged']]
    rate  = len(conv)/len(rows)*100
    times = [r['ibvs_s'] for r in conv if r.get('ibvs_s')]
    errs  = [r['err_px'] for r in conv if r.get('err_px')]
    print(f'Total runs:       {len(rows)}')
    print(f'Converged:        {len(conv)} ({rate:.0f}%)  (target > 90%)')
    if times: print(f'Avg IBVS time:    {statistics.mean(times):.2f}s  (target < 5s)')
    if errs:  print(f'Avg final error:  {statistics.mean(errs):.2f}px (target < 10px)')
    if errs:  print(f'Max final error:  {max(errs):.2f}px')

    if args.group_by:
        print(f'\nBreakdown by {args.group_by}:')
        summarize(rows, group_by=args.group_by)

    # Save
    if rows and isinstance(rows[0], dict):
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        print(f'\nResults saved to: {out_path}')

if __name__ == '__main__':
    main()
