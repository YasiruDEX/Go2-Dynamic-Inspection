"""
upload_existing_results.py
==========================
One-shot script to upload ALL existing results from Local_database_results/
to the cloud PostgreSQL database.

Run from vi_server/ with the venv activated:
    python upload_existing_results.py

Use --dry-run to just print what would be uploaded without touching the DB.
    python upload_existing_results.py --dry-run
"""

import json
import sys
from pathlib import Path

# ── Load .env before importing settings ────────────────────────────────────
env_path = Path(__file__).parent / ".env"
if env_path.exists():
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, _, v = line.partition("=")
            import os
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))

from app.cloud_uploader import upload_result

RESULTS_DB = Path(__file__).parent.parent / "laptop_receiver_server" / "Local_database_results"
DRY_RUN    = "--dry-run" in sys.argv


def main():
    if not RESULTS_DB.exists():
        print(f"[ERROR] Results directory not found:\n  {RESULTS_DB.resolve()}")
        sys.exit(1)

    print(f"Scanning: {RESULTS_DB.resolve()}")
    print(f"Mode:     {'DRY RUN (no changes)' if DRY_RUN else 'LIVE UPLOAD'}\n")

    uploaded = 0
    skipped  = 0
    errors   = 0

    for event_dir in sorted(RESULTS_DB.iterdir()):
        if not event_dir.is_dir():
            continue
        for waypoint_dir in sorted(event_dir.iterdir()):
            if not waypoint_dir.is_dir():
                continue
            for class_dir in sorted(waypoint_dir.iterdir()):
                if not class_dir.is_dir():
                    continue
                for instance_dir in sorted(class_dir.iterdir()):
                    if not instance_dir.is_dir():
                        continue

                    result_file = instance_dir / "result.json"
                    image_file  = instance_dir / "inspection_image.jpg"

                    if not result_file.exists():
                        skipped += 1
                        continue

                    try:
                        result = json.loads(result_file.read_text(encoding="utf-8"))
                    except Exception as e:
                        print(f"  [SKIP] Bad JSON in {result_file}: {e}")
                        skipped += 1
                        continue

                    label = (
                        f"  {event_dir.name}/{waypoint_dir.name}/"
                        f"{class_dir.name}/{instance_dir.name}"
                        f"  →  {result.get('decision','?')}  "
                        f"(conf={result.get('confidence',0):.2f})"
                    )

                    if DRY_RUN:
                        print(f"[DRY] Would upload: {label}")
                        uploaded += 1
                        continue

                    try:
                        upload_result(
                            event_id     = event_dir.name,
                            waypoint_id  = waypoint_dir.name,
                            object_class = class_dir.name,
                            result       = result,
                            image_path   = str(image_file) if image_file.exists() else "",
                        )
                        print(f"[OK]  Uploaded: {label}")
                        uploaded += 1
                    except Exception as e:
                        print(f"[ERR] Failed:   {label}\n       {e}")
                        errors += 1

    print(f"\n{'─'*55}")
    print(f"  Uploaded : {uploaded}")
    print(f"  Skipped  : {skipped}")
    print(f"  Errors   : {errors}")
    print(f"{'─'*55}")


if __name__ == "__main__":
    main()
