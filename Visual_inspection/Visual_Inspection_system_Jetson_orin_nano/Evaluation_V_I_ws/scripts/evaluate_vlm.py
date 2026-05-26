#!/usr/bin/env python3
"""
evaluate_vlm.py
Run on LAPTOP. Requires vi_server running.

Evaluates VLM PASS/FAIL decision accuracy for fire_extinguisher,
door, emergency_exit, main_cylinder, unknown object types.

Usage:
  python3 scripts/evaluate_vlm.py \
      --base  eval_dataset/vlm_eval/ \
      --log   eval_dataset/capture_log.csv \
      --server http://localhost:8001 \
      --out   results/vlm_eval_results.csv

Folder → object_type + expected decision mapping:
  fire_ext_pass/    → fire_extinguisher  PASS
  fire_ext_fail/    → fire_extinguisher  FAIL
  exit_pass/        → emergency_exit     PASS
  exit_fail/        → emergency_exit     FAIL
  door_pass/        → door               PASS
  door_fail/        → door               FAIL
  cylinder_pass/    → main_cylinder      PASS
  cylinder_fail/    → main_cylinder      FAIL
  unknown_various/  → unknown            None (just check routing)
"""

import argparse, csv, json, time, sys, statistics
from pathlib import Path
from collections import defaultdict
import requests

FOLDER_MAP = {
    'fire_ext_pass':  ('fire_extinguisher', 'PASS'),
    'fire_ext_fail':  ('fire_extinguisher', 'FAIL'),
    'exit_pass':      ('emergency_exit',    'PASS'),
    'exit_fail':      ('emergency_exit',    'FAIL'),
    'door_pass':      ('door',              'PASS'),
    'door_fail':      ('door',              'FAIL'),
    'cylinder_pass':  ('main_cylinder',     'PASS'),
    'cylinder_fail':  ('main_cylinder',     'FAIL'),
    'unknown_various':('unknown',           None),
}

def poll_job(server, job_id, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            r = requests.get(f'{server}/api/v1/jobs/{job_id}', timeout=10)
            job = r.json()
            if job.get('status') in ['DONE', 'FAILED']:
                return job
        except Exception:
            pass
        time.sleep(2)
    return {'status': 'TIMEOUT', 'result_json': '{}'}

def load_captions(log_csv):
    captions = {}
    if not Path(log_csv).exists():
        return captions
    with open(log_csv, newline='') as f:
        for row in csv.DictReader(f):
            fname = row.get('filename', '').strip()
            note  = row.get('notes', '').strip()
            true  = row.get('ground_truth_value', '').strip()
            if fname:
                captions[fname] = {'caption': note, 'true_decision': true}
    return captions

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base',   required=True, help='Base folder: eval_dataset/vlm_eval/')
    ap.add_argument('--log',    default='eval_dataset/capture_log.csv')
    ap.add_argument('--server', default='http://localhost:8001')
    ap.add_argument('--out',    default='results/vlm_eval_results.csv')
    ap.add_argument('--consistency', action='store_true',
                    help='Run each image 3x to test consistency')
    args = ap.parse_args()

    base     = Path(args.base)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    captions = load_captions(args.log)

    all_results = []

    for folder_name, (obj_type, expected) in FOLDER_MAP.items():
        folder = base / folder_name
        if not folder.exists():
            print(f'  [skip] {folder_name}/ not found')
            continue

        images = sorted(folder.glob('*.jpg')) + sorted(folder.glob('*.png'))
        if not images:
            continue

        print(f'\n--- {folder_name}/ ({obj_type}, expected={expected}) --- {len(images)} images')

        for img_path in images:
            fname = img_path.name
            cap_info = captions.get(fname, {})

            runs = 3 if args.consistency else 1
            decisions = []
            for run in range(runs):
                t0 = time.time()
                try:
                    with open(img_path, 'rb') as f:
                        r = requests.post(f'{args.server}/api/v1/jobs',
                                          files={'file': ('image.jpg', f, 'image/jpeg')},
                                          data={'object_type': obj_type}, timeout=30)
                    job_id  = r.json().get('job_id')
                    job     = poll_job(args.server, job_id)
                    elapsed = round(time.time() - t0, 1)

                    if job['status'] == 'DONE':
                        data     = json.loads(job.get('result_json', '{}'))
                        decision = data.get('decision', 'UNKNOWN')
                        conf     = data.get('confidence', 0.0)
                        summary  = data.get('summary', '')[:120]
                    else:
                        decision, conf, summary = 'ERROR', 0.0, ''

                    decisions.append(decision)

                    if run == 0:
                        correct = (expected is None) or (decision == expected)
                        print(f'  {"✓" if correct else "✗"} {fname}: '
                              f'{expected}→{decision} ({conf:.2f}) [{elapsed}s]')

                        all_results.append({
                            'folder': folder_name, 'filename': fname,
                            'object_type': obj_type, 'expected': expected or 'N/A',
                            'predicted': decision, 'confidence': conf,
                            'correct': correct, 'time_s': elapsed,
                            'summary': summary,
                            'caption': cap_info.get('caption', '')
                        })
                except Exception as e:
                    print(f'  ERROR {fname}: {e}')

            if args.consistency and len(decisions) == 3:
                consistent = len(set(decisions)) == 1
                all_results[-1]['consistency'] = consistent
                all_results[-1]['decisions_3x'] = str(decisions)
                print(f'    Consistency: {decisions} → {"✓" if consistent else "✗ INCONSISTENT"}')

    if not all_results:
        print('No results.'); sys.exit(1)

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=all_results[0].keys())
        writer.writeheader()
        writer.writerows(all_results)

    # --- Summary ---
    print(f'\n=== VLM EVALUATION SUMMARY ===')
    by_type = defaultdict(list)
    for r in all_results:
        by_type[r['object_type']].append(r)

    for t, rows in sorted(by_type.items()):
        labeled = [r for r in rows if r['expected'] != 'N/A']
        acc = sum(r['correct'] for r in labeled) / len(labeled) * 100 if labeled else 0
        avg_t = statistics.mean(r['time_s'] for r in rows)
        print(f'  {t:20s}: {acc:.0f}% accuracy | avg {avg_t:.1f}s/image | n={len(rows)}')

    labeled_all = [r for r in all_results if r['expected'] != 'N/A']
    if labeled_all:
        tp = sum(1 for r in labeled_all if r['expected']=='FAIL' and r['predicted']=='FAIL')
        fp = sum(1 for r in labeled_all if r['expected']!='FAIL' and r['predicted']=='FAIL')
        fn = sum(1 for r in labeled_all if r['expected']=='FAIL' and r['predicted']!='FAIL')
        prec = tp/(tp+fp) if (tp+fp) > 0 else 0
        rec  = tp/(tp+fn) if (tp+fn) > 0 else 0
        f1   = 2*prec*rec/(prec+rec) if (prec+rec) > 0 else 0
        overall = sum(r['correct'] for r in labeled_all)/len(labeled_all)*100
        print(f'\n  Overall accuracy: {overall:.0f}%')
        print(f'  FAIL class — Precision: {prec:.2f}  Recall: {rec:.2f}  F1: {f1:.2f}')
        print(f'  (Recall target > 0.90 — must not miss failures)')
    print(f'\n  Results saved to: {out_path}')

if __name__ == '__main__':
    main()
