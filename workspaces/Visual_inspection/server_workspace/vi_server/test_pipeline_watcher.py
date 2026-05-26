"""
test_pipeline_watcher.py — Tests the full watcher + processor pipeline.
Drops a fake folder into Local_database and waits for result.json.

Usage:
    python test_pipeline_watcher.py          # test gauge (default)
    python test_pipeline_watcher.py vlm      # test VLM (door)
"""

import os
import sys
import json
import shutil
import time
from pathlib import Path

MODE = sys.argv[1] if len(sys.argv) > 1 else "gauge"

LOCAL_DB     = Path(__file__).parent.parent / "laptop_receiver_server" / "Local_database"
RESULTS_DB   = Path(__file__).parent.parent / "laptop_receiver_server" / "Local_database_results"

if MODE == "vlm":
    EVENT_ID     = "ETest002"
    WAYPOINT_ID  = "WayTest002"
    OBJ_CLASS    = "door"
    INSTANCE     = "instance_test_vlm"
    # Find any jpg to use as a test image
    src_images = list((Path(__file__).parent / "sample_images").glob("*.jpg"))
    if not src_images:
        src_images = list(Path(__file__).parent.glob("**/*.jpg"))
else:
    EVENT_ID     = "ETest003"
    WAYPOINT_ID  = "WayTest003"
    OBJ_CLASS    = "gauge"
    INSTANCE     = "instance_test_gauge"
    GAUGE_IMGS   = Path(__file__).parent / "app" / "pipelines" / "gauge" / "test_images"
    src_images   = sorted(GAUGE_IMGS.glob("*.jpg")) if GAUGE_IMGS.exists() else []

# Create target folder
target = LOCAL_DB / EVENT_ID / WAYPOINT_ID / OBJ_CLASS / INSTANCE

# Clean up previous run so watcher re-processes
if target.exists():
    shutil.rmtree(target)
result_dir = RESULTS_DB / EVENT_ID / WAYPOINT_ID / OBJ_CLASS / INSTANCE
if result_dir.exists():
    shutil.rmtree(result_dir)

target.mkdir(parents=True, exist_ok=True)

# Copy image
if src_images:
    shutil.copy(src_images[0], target / "img_0.jpg")
    print(f"Copied: {src_images[0].name}")
else:
    print("WARNING: No image found — create img_0.jpg manually in:")
    print(f"  {target}")

# Write metadata
metadata = {
    "session_label": f"{EVENT_ID},{WAYPOINT_ID}",
    "object_class": OBJ_CLASS,
    "subfolder": INSTANCE,
}
(target / "metadata.json").write_text(json.dumps(metadata, indent=2))

print(f"\nTest folder: {target}")
print(f"\nWaiting for result (gauge ~3min, VLM ~20s)...")

result_path = result_dir / "result.json"
for i in range(60):
    time.sleep(10)
    elapsed = (i + 1) * 10
    if result_path.exists():
        print(f"\n✅ SUCCESS after {elapsed}s — result.json:")
        print(result_path.read_text())
        print(f"\nInspection image: {result_dir / 'inspection_image.jpg'}")
        break
    print(f"  ...{elapsed}s elapsed")
else:
    print("\n❌ Timed out — check vi_server terminal for errors")
