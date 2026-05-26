"""
run_vlm_eval.py — Batch VLM evaluation script
===============================================
Place images in:
    vlm/eval_images/
    
Name images after object type:
    door.jpg, door_1.jpg, door_2.jpg
    emergency_exit.jpg, emergency_exit_1.jpg
    fire_extinguisher.jpg, fire_extinguisher_1.jpg
    main_cylinder.jpg
    person.jpg
    unknown.jpg

Supported object types:
    door, emergency_exit, fire_extinguisher, main_cylinder, person, unknown

Results save to:
    vlm/eval_results/
        door/door_result.json
        door/door_1_result.json
        fire_extinguisher/fire_extinguisher_result.json
        ...
        summary.json       ← overall summary of all results

Run from vi_server directory with venv activated:
    cd "E:\\...\\vi_server"
    .\\venv\\Scripts\\activate
    python app\\pipelines\\vlm\\run_vlm_eval.py
"""

import sys
import json
import time
import shutil
from pathlib import Path

# ── Allow running from vi_server root ────────────────────────────────────────
VLM_DIR = Path(__file__).parent
VI_SERVER_DIR = VLM_DIR.parent.parent.parent
sys.path.insert(0, str(VI_SERVER_DIR))

from app.pipelines.vlm.vlm_router import run_vlm_task

# ── Paths ─────────────────────────────────────────────────────────────────────
EVAL_IMAGES_DIR = VLM_DIR / "eval_images"
EVAL_RESULTS_DIR = VLM_DIR / "eval_results"

VALID_TYPES = {
    "door", "emergency_exit", "fire_extinguisher",
    "main_cylinder", "person", "unknown"
}


def get_object_type(filename: str) -> str | None:
    """
    Extract object type from filename.
    door.jpg → 'door'
    door_1.jpg → 'door'
    fire_extinguisher_2.jpg → 'fire_extinguisher'
    """
    stem = Path(filename).stem.lower()
    # Try full name first, then strip trailing _N
    if stem in VALID_TYPES:
        return stem
    # Remove trailing _digit(s)
    parts = stem.rsplit("_", 1)
    if len(parts) == 2 and parts[1].isdigit() and parts[0] in VALID_TYPES:
        return parts[0]
    return None


def run_eval():
    EVAL_IMAGES_DIR.mkdir(exist_ok=True)
    EVAL_RESULTS_DIR.mkdir(exist_ok=True)

    images = sorted([
        f for f in EVAL_IMAGES_DIR.iterdir()
        if f.suffix.lower() in {".jpg", ".jpeg", ".png"}
    ])

    if not images:
        print(f"No images found in: {EVAL_IMAGES_DIR}")
        print("Put your images there and name them like: door.jpg, door_1.jpg, fire_extinguisher.jpg ...")
        return

    print(f"Found {len(images)} images — starting evaluation...\n")

    summary = []
    failed  = []

    for img_path in images:
        obj_type = get_object_type(img_path.name)
        if obj_type is None:
            print(f"[SKIP] {img_path.name} — cannot determine object type from filename")
            continue

        print(f"[{obj_type.upper()}] Processing: {img_path.name} ...", end=" ", flush=True)
        t0 = time.time()

        try:
            result = run_vlm_task(obj_type, str(img_path), {})
            elapsed = round(time.time() - t0, 1)

            # ── Save individual result ────────────────────────────────────
            result_subdir = EVAL_RESULTS_DIR / obj_type
            result_subdir.mkdir(exist_ok=True)

            result_name = img_path.stem + "_result.json"
            result_file = result_subdir / result_name
            with open(result_file, "w") as f:
                json.dump(result, f, indent=2)

            # Copy image alongside result
            shutil.copy2(img_path, result_subdir / img_path.name)

            decision = result.get("decision", "?")
            confidence = result.get("confidence", "-")
            print(f"→ {decision}  (conf={confidence})  [{elapsed}s]")

            summary.append({
                "image":      img_path.name,
                "task":       obj_type,
                "decision":   decision,
                "confidence": confidence,
                "result_file": str(result_file.relative_to(VLM_DIR)),
                "elapsed_s":  elapsed,
            })

        except Exception as e:
            elapsed = round(time.time() - t0, 1)
            print(f"→ ERROR: {e}  [{elapsed}s]")
            failed.append({"image": img_path.name, "error": str(e)})

    # ── Save summary ──────────────────────────────────────────────────────────
    summary_data = {
        "total": len(summary) + len(failed),
        "processed": len(summary),
        "failed": len(failed),
        "results": summary,
        "errors": failed,
    }
    summary_file = EVAL_RESULTS_DIR / "summary.json"
    with open(summary_file, "w") as f:
        json.dump(summary_data, f, indent=2)

    # ── Print summary table ───────────────────────────────────────────────────
    print("\n" + "="*55)
    print(f"  DONE — {len(summary)} processed, {len(failed)} failed")
    print("="*55)
    by_task = {}
    for r in summary:
        by_task.setdefault(r["task"], {"PASS": 0, "FAIL": 0, "UNKNOWN": 0})
        by_task[r["task"]][r["decision"]] = by_task[r["task"]].get(r["decision"], 0) + 1

    for task, counts in sorted(by_task.items()):
        print(f"  {task:<22} {counts}")

    print(f"\n  Summary saved → {summary_file}")
    print(f"  Results saved → {EVAL_RESULTS_DIR}")


if __name__ == "__main__":
    run_eval()
