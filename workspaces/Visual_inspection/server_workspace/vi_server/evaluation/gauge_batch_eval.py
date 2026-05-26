"""
Gauge Pipeline Batch Evaluator
===============================
Sends all images in evaluation/gauge_images/ through the gauge reading
pipeline one by one. Saves results after EVERY image so nothing is lost
if interrupted.

Output (all in evaluation/gauge_results/):
  results_TIMESTAMP.json   full structured results
  results_TIMESTAMP.csv    flat CSV — open in Excel / pandas
  summary_TIMESTAMP.txt    human-readable report

Usage:
  1. Put gauge images in:  evaluation/gauge_images/
  2. Start server:         python -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
  3. Run:                  python evaluation/gauge_batch_eval.py
"""

import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import requests

# ── Config ───────────────────────────────────────────────────
SERVER_URL    = "http://localhost:8000"
INPUT_DIR     = Path(__file__).parent / "gauge_images"
RESULTS_DIR   = Path(__file__).parent / "gauge_results"

# Gauge pipeline runs a subprocess that itself has a 180s timeout.
# Add extra buffer for server overhead.
POLL_INTERVAL = 3.0    # seconds between status checks
TIMEOUT       = 240.0  # seconds max wait per image (4 min)

SUPPORTED_EXT = {".jpg", ".jpeg", ".png", ".bmp"}


# ── API helpers ───────────────────────────────────────────────
def upload_image(image_path: Path) -> str:
    """POST image to server gauge endpoint. Returns job_id."""
    with open(image_path, "rb") as f:
        resp = requests.post(
            f"{SERVER_URL}/api/v1/jobs",
            files={"file": (image_path.name, f, "image/jpeg")},
            data={"object_type": "gauge"},
            timeout=30,
        )
    resp.raise_for_status()
    return resp.json()["job_id"]


def poll_job(job_id: str) -> dict:
    """Poll job until DONE/FAILED/TIMEOUT. Returns final job dict."""
    start = time.time()
    while True:
        elapsed = time.time() - start
        if elapsed > TIMEOUT:
            return {"status": "TIMEOUT", "result_json": None,
                    "error_message": f"No result after {TIMEOUT:.0f}s"}
        try:
            resp = requests.get(f"{SERVER_URL}/api/v1/jobs/{job_id}", timeout=10)
            resp.raise_for_status()
            job = resp.json()
        except Exception as e:
            print(f"         Poll error: {e} — retrying...")
            time.sleep(POLL_INTERVAL)
            continue

        if job["status"] in ("DONE", "FAILED"):
            return job

        elapsed_str = f"{elapsed:.0f}s"
        print(f"         Waiting... [{elapsed_str}]", end="\r", flush=True)
        time.sleep(POLL_INTERVAL)


def extract_result(job: dict) -> dict:
    """
    Pull gauge reading fields from job result_json.

    gauge_pipeline.py wrapper returns:
        {"reading": float, "unit": str, "confidence": float, "method": str}
    On failure it returns:
        {"reading": None, "unit": None, "confidence": 0.0, "error": str, "method": str}
    """
    out = {
        "status":     job["status"],
        "reading":    None,
        "unit":       None,
        "confidence": None,
        "error":      job.get("error_message"),
        "raw":        None,
    }

    if job["status"] == "DONE" and job.get("result_json"):
        try:
            r = json.loads(job["result_json"])
            out["raw"]        = r
            out["reading"]    = r.get("reading")      # wrapper key = "reading"
            out["unit"]       = r.get("unit")
            out["confidence"] = r.get("confidence")
            # Surface inner pipeline error if reading is None
            if out["reading"] is None and r.get("error"):
                out["error"] = r["error"]
        except Exception as e:
            out["error"] = f"Could not parse result_json: {e}"

    return out


def save_results(all_records: list, run_ts: str):
    """Save JSON + CSV + summary. Called after every image."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    # ── JSON ──────────────────────────────────────────────────
    json_path = RESULTS_DIR / f"results_{run_ts}.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({
            "run_timestamp": run_ts,
            "server_url":    SERVER_URL,
            "total_images":  len(all_records),
            "results":       all_records,
        }, f, indent=2)

    # ── CSV ───────────────────────────────────────────────────
    csv_path = RESULTS_DIR / f"results_{run_ts}.csv"
    fields = ["index", "filename", "status", "reading", "unit", "confidence", "error"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(all_records)

    # ── Summary ───────────────────────────────────────────────
    done   = [r for r in all_records if r["status"] == "DONE" and r["reading"] is not None]
    failed = [r for r in all_records if r["reading"] is None]
    total  = len(all_records)
    ok_pct = len(done) / total * 100 if total else 0
    readings = [r["reading"] for r in done]
    avg = sum(readings) / len(readings) if readings else None

    summary_path = RESULTS_DIR / f"summary_{run_ts}.txt"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("Gauge Pipeline Evaluation Summary\n")
        f.write(f"Run timestamp : {run_ts}\n")
        f.write(f"{'='*52}\n\n")
        f.write(f"Processed  : {total}\n")
        f.write(f"Successful : {len(done)}  ({ok_pct:.1f}%)\n")
        f.write(f"Failed     : {len(failed)}\n")
        if avg is not None:
            f.write(f"Avg reading: {avg:.4f}\n")
        f.write(f"\nPer-image:\n{'-'*52}\n")
        for r in all_records:
            val = f"{r['reading']} {r['unit'] or ''}".strip() if r["reading"] is not None else "FAILED"
            err = f"  ← {r['error']}" if r.get("error") else ""
            f.write(f"  [{r['index']:02d}] {r['filename']:<35} {val}{err}\n")


# ── Main ──────────────────────────────────────────────────────
def main():
    # 1. Check server
    try:
        requests.get(f"{SERVER_URL}/api/v1/jobs", timeout=5)
    except Exception:
        print(f"[ERROR] Cannot reach server at {SERVER_URL}")
        print("        Run: python -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload")
        sys.exit(1)

    # 2. Collect images
    if not INPUT_DIR.exists():
        print(f"[ERROR] Folder not found: {INPUT_DIR}")
        print(f"        Create it and add your gauge images there.")
        sys.exit(1)

    images = sorted(p for p in INPUT_DIR.iterdir() if p.suffix.lower() in SUPPORTED_EXT)
    if not images:
        print(f"[ERROR] No images in {INPUT_DIR}")
        sys.exit(1)

    run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    print(f"{'='*60}")
    print(f"  Gauge Pipeline Batch Evaluation")
    print(f"{'='*60}")
    print(f"  Images  : {len(images)} found in gauge_images/")
    print(f"  Timeout : {TIMEOUT:.0f}s per image")
    print(f"  Results : evaluation/gauge_results/results_{run_ts}.csv")
    print(f"{'='*60}\n")

    all_records = []

    for idx, img_path in enumerate(images, start=1):
        print(f"[{idx:02d}/{len(images)}] {img_path.name}")

        record = {
            "index":      idx,
            "filename":   img_path.name,
            "status":     None,
            "reading":    None,
            "unit":       None,
            "confidence": None,
            "error":      None,
            "raw":        None,
        }

        try:
            job_id = upload_image(img_path)
            job    = poll_job(job_id)
            result = extract_result(job)
            record.update(result)

        except requests.exceptions.ConnectionError as e:
            record["status"] = "ERROR"
            record["error"]  = "Lost connection to server"
            print(f"         ERROR: {record['error']}")

        except Exception as e:
            record["status"] = "ERROR"
            record["error"]  = str(e)
            print(f"         ERROR: {e}")

        # Print result line
        print(" " * 40, end="\r")   # clear the "Waiting..." line
        if record["reading"] is not None:
            print(f"         ✅ Reading : {record['reading']} {record['unit'] or ''} "
                  f"(conf={record['confidence']})")
        else:
            print(f"         ❌ Failed  : {record['error']}")

        all_records.append(record)

        # ← SAVE AFTER EVERY IMAGE so nothing is lost on interrupt
        save_results(all_records, run_ts)
        print()

    # 3. Final summary to terminal
    done    = [r for r in all_records if r["reading"] is not None]
    failed  = [r for r in all_records if r["reading"] is None]
    ok_pct  = len(done) / len(all_records) * 100 if all_records else 0
    readings = [r["reading"] for r in done]
    avg = sum(readings) / len(readings) if readings else None

    print(f"{'='*60}")
    print(f"  DONE — {len(all_records)} images processed")
    print(f"  Successful readings : {len(done)}  ({ok_pct:.1f}%)")
    print(f"  Failed              : {len(failed)}")
    if avg is not None:
        print(f"  Average reading     : {avg:.4f}")
    print(f"\n  Saved to gauge_results/:")
    print(f"    results_{run_ts}.json")
    print(f"    results_{run_ts}.csv")
    print(f"    summary_{run_ts}.txt")
    print(f"{'='*60}")

    if failed:
        print(f"\n  Failed images:")
        for r in failed:
            print(f"    [{r['index']:02d}] {r['filename']} — {r['error']}")


if __name__ == "__main__":
    main()
