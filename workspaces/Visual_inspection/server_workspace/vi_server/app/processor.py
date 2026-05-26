"""
processor.py — Processes inspection jobs from the queue.

Routes:
  object_class == "gauge"  → gauge reading pipeline
  anything else            → VLM pipeline

Saves locally:
  instance_dir/result.json
  instance_dir/inspection_image.jpg   (copy of first image)

Then uploads to cloud (stub until DB schema provided).
"""

import asyncio
import json
import logging
import shutil
from pathlib import Path
from typing import Optional

# Results save here — same tree as Local_database but separate
_RESULTS_DB = Path(__file__).parent.parent.parent / \
              "laptop_receiver_server" / "Local_database_results"

from app.pipelines.gauge_pipeline import run_gauge_pipeline
from app.pipelines.vlm.vlm_router import run_vlm_task
from app.cloud_uploader import upload_result

logger = logging.getLogger(__name__)

_queue: asyncio.Queue = asyncio.Queue()
_worker_task: Optional[asyncio.Task] = None


def enqueue(item: dict):
    """Thread-safe enqueue (called from watcher thread)."""
    _queue.put_nowait(item)


async def start_worker():
    global _worker_task
    _worker_task = asyncio.create_task(_worker_loop())
    logger.info("Processor worker started")


async def stop_worker():
    if _worker_task:
        _worker_task.cancel()
        try:
            await _worker_task
        except asyncio.CancelledError:
            pass
    logger.info("Processor worker stopped")


async def _worker_loop():
    while True:
        try:
            item = await asyncio.wait_for(_queue.get(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        except asyncio.CancelledError:
            break

        try:
            await asyncio.get_event_loop().run_in_executor(None, _process, item)
        except Exception as e:
            logger.error(f"Processing error: {e}", exc_info=True)


def _process(item: dict):
    """Run in thread pool — blocking pipeline calls are fine here."""
    event_id    = item["event_id"]
    waypoint_id = item["waypoint_id"]
    object_class = item["object_class"]
    instance_dir = Path(item["instance_dir"])

    logger.info(f"Processing {event_id}/{waypoint_id}/{object_class}/{instance_dir.name}")

    # Find first image
    images = sorted(instance_dir.glob("img_*.jpg"))
    if not images:
        images = sorted(instance_dir.glob("*.jpg"))
    if not images:
        logger.warning(f"No images found in {instance_dir}")
        return

    first_image = str(images[0])

    # Load metadata if present
    meta_file = instance_dir / "metadata.json"
    metadata = {}
    if meta_file.exists():
        with open(meta_file) as f:
            metadata = json.load(f)

    # ── Route to pipeline ────────────────────────────────────────────────
    if object_class == "gauge":
        raw = run_gauge_pipeline(first_image)
        reading = raw.get("reading")
        result = {
            "task":     "gauge",
            "decision": "PASS" if reading is not None else "FAIL",
            "findings": [reading] if reading is not None else [],
        }
    else:
        result = run_vlm_task(object_class, first_image, metadata)

    # ── Save to Local_database_results (separate from raw input) ──────────
    result_dir = _RESULTS_DB / event_id / waypoint_id / object_class / instance_dir.name
    result_dir.mkdir(parents=True, exist_ok=True)

    result_path = result_dir / "result.json"
    with open(result_path, "w") as f:
        json.dump(result, f, indent=2)
    logger.info(f"Saved result.json → {result_path}")

    # Copy first image as inspection_image.jpg into results folder
    inspection_img = result_dir / "inspection_image.jpg"
    shutil.copy2(first_image, inspection_img)

    # Also mark source folder as processed (empty sentinel so watcher skips it)
    (instance_dir / "result.json").write_text('{"processed": true}')

    # ── Upload to cloud ──────────────────────────────────────────────────
    try:
        upload_result(
            event_id=event_id,
            waypoint_id=waypoint_id,
            object_class=object_class,
            result=result,
            image_path=str(inspection_img),  # path in Local_database_results
        )
    except Exception as e:
        logger.warning(f"Cloud upload failed (will retry later): {e}")
