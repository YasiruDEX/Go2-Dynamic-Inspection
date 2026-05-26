"""
watcher.py — Watches Local_database for unprocessed inspection folders.
Adds new folders to the processing queue.
"""

import os
import time
import threading
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

# Local_database is two levels up from vi_server
_LOCAL_DB = Path(__file__).parent.parent.parent / \
            "laptop_receiver_server" / "Local_database"

POLL_INTERVAL = 5   # seconds


def _scan(queue_put_fn):
    """Yield every instance folder that has no result.json yet."""
    if not _LOCAL_DB.exists():
        return
    # Structure: Local_database/<EventID>/<WaypointID>/<object_class>/<instance>/
    for event_dir in _LOCAL_DB.iterdir():
        if not event_dir.is_dir():
            continue
        for waypoint_dir in event_dir.iterdir():
            if not waypoint_dir.is_dir():
                continue
            for obj_dir in waypoint_dir.iterdir():
                if not obj_dir.is_dir():
                    continue
                for instance_dir in obj_dir.iterdir():
                    if not instance_dir.is_dir():
                        continue
                    if not (instance_dir / "result.json").exists():
                        queue_put_fn({
                            "event_id":    event_dir.name,
                            "waypoint_id": waypoint_dir.name,
                            "object_class": obj_dir.name,
                            "instance_dir": str(instance_dir),
                        })


def start_watcher(queue_put_fn):
    """Start background watcher thread."""
    def _loop():
        logger.info(f"Watcher started — polling {_LOCAL_DB} every {POLL_INTERVAL}s")
        seen = set()
        while True:
            try:
                for event_dir in (_LOCAL_DB.iterdir() if _LOCAL_DB.exists() else []):
                    if not event_dir.is_dir():
                        continue
                    for waypoint_dir in event_dir.iterdir():
                        if not waypoint_dir.is_dir():
                            continue
                        for obj_dir in waypoint_dir.iterdir():
                            if not obj_dir.is_dir():
                                continue
                            for instance_dir in obj_dir.iterdir():
                                if not instance_dir.is_dir():
                                    continue
                                key = str(instance_dir)
                                result_path = instance_dir / "result.json"
                                if key not in seen and not result_path.exists():
                                    seen.add(key)
                                    queue_put_fn({
                                        "event_id":    event_dir.name,
                                        "waypoint_id": waypoint_dir.name,
                                        "object_class": obj_dir.name,
                                        "instance_dir": key,
                                    })
                                    logger.info(f"Queued: {key}")
            except Exception as e:
                logger.error(f"Watcher error: {e}")
            time.sleep(POLL_INTERVAL)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return t
