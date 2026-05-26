#!/usr/bin/env python3
"""
evaluate_gauge.py
Run on LAPTOP. Requires vi_server running (locally or on server).

Sends each gauge evaluation image to the server and compares
predicted reading to the true reading in your capture_log.csv.

Usage:
  python3 scripts/evaluate_gauge.py \
      --images eval_dataset/gauge_accuracy/ \
      --log    eval_dataset/capture_log.csv \
      --server http://localhost:8001 \
      --out    results/gauge_eval_results.csv
"""

import argparse, csv, json, time, os, statistics, sys
from pathlib import Path
import requests

def poll_job(server, job_id, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            r = requests.get(f'{server}/api/v1/jobs/{job_id}', timeout=10)
            job = r.json()
            if job.get('status') in ['DONE', 'FAILED']:
                return job
        except Exception as e:
            print(f'    poll error: {e}')
        time.sleep(2)
    return {'status': 'TIMEOUT', 'result_json': '{}'}

def send_image(server, img_path, object_type='gauge'):
    with open(img_path, 'rb') as f:
        r = requests.post(f'{server}/api/v1/jobs',
                          files={'file': ('image.jpg', f, 'image/jpeg')},
                          data={'object_type': object_type},
                          timeout=30)
    return r.json().get('job_id')

def load_ground_truth(log_csv, images_dir):
    """Read ground_truth_value from capture_log.csv for each image filename."""
    gt = {}
    if not Path(log_csv).exists():
        print(f'WARNING: capture_log.csv not found at {log_csv}')
        print('  Will use filename to parse true value. Name images like: gauge_2.5bar_dist2m_n1.jpg')
        return gt
    with open(log_csv, newline='') as f:
        for row in csv.DictReader(f):
            fname = row.get('filename', '')
            true_val = row.get('ground_truth_value', '')
            if fname and true_val and true_val not in ('', 'N/A', 'PASS', 'FAIL'):
                try:
                    gt[fname] = float(true_val)
                except ValueError:
                    pass
    return gt

def parse_true_from_filename(fname):
    """Fallback: parse true value from filename like gauge_2.5bar_dist2m_n1.jpg"""
    import re
    m = re.search(r'gauge_([0-9.]+)', fname)
    if m:
        return float(m.group(1))
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', required=True)
    ap.add_argument('--log',    default='eval_dataset/capture_log.csv')
    ap.add_argument('--server', default='http://localhost:8001')
    ap.add_argument('--out',    default='results/gauge_eval_results.csv')
    args = ap.parse_args()

    img_dir  = Path(args.images)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    images = sorted(img_dir.glob('*.jpg')) + sorted(img_dir.glob('*.png'))
    if not images:
        print(f'No images found in {img_dir}'); sys.exit(1)

    gt = load_ground_truth(args.log, img_dir)
    print(f'\nGauge Evaluation: {len(images)} images | server={args.server}')

    results = []
    for img_path in images:
        fname = img_path.name
        true_val = gt.get(fname) or parse_true_from_filename(fname)
        if true_val is None:
            print(f'  SKIP {fname} — no ground truth value found')
            continue

        print(f'  Sending {fname} (true={true_val})...', end='', flush=True)
        t0 = time.time()
        try:
            job_id = send_image(args.server, img_path)
            job    = poll_job(args.server, job_id)
            elapsed = round(time.time() - t0, 1)

            if job['status'] == 'DONE':
                data = json.loads(job.get('result_json', '{}'))
                pred = data.get('reading')
                unit = data.get('unit', '')
                err  = abs(pred - true_val) if pred is not None else None
                ok   = err is not None and err < (true_val * 0.05 + 0.1)
                print(f' pred={pred} {unit}  err={err}  {"✓" if ok else "✗"}')
            else:
                pred, unit, err, ok = None, None, None, False
                print(f' {job["status"]}')

        except Exception as e:
            print(f' ERROR: {e}')
            pred, unit, err, ok, elapsed = None, None, None, False, 0

        results.append({'filename': fname, 'true_value': true_val,
                        'predicted': pred, 'unit': unit, 'abs_error': err,
                        'within_5pct': ok, 'time_s': elapsed})

    if not results:
        print('No results collected.'); sys.exit(1)

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    good   = [r for r in results if r['abs_error'] is not None]
    errors = [r['abs_error'] for r in good]
    bias   = [r['predicted'] - r['true_value'] for r in good if r['predicted']]

    print(f'\n=== GAUGE EVALUATION SUMMARY ===')
    print(f'Total images:    {len(results)}')
    print(f'Successful reads:{len(good)} ({len(good)/len(results)*100:.0f}%)')
    if errors:
        mae  = statistics.mean(errors)
        rmse = (sum(e**2 for e in errors)/len(errors))**0.5
        mape = statistics.mean(e/r['true_value']*100 for r,e in zip(good,errors) if r['true_value'])
        print(f'MAE:             {mae:.4f}  (target < 5% full range)')
        print(f'RMSE:            {rmse:.4f}')
        print(f'MAPE:            {mape:.2f}%')
        print(f'Bias:            {statistics.mean(bias):.4f}  (near 0 = no systematic error)')
        print(f'Avg time/image:  {statistics.mean(r["time_s"] for r in results):.1f}s')
    print(f'Results saved to:{out_path}')

if __name__ == '__main__':
    main()
