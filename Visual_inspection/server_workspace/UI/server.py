"""
UI Dashboard Server for Visual Inspection Results
===================================================
Reads Local_database_results/ and serves a live inspection dashboard.

Run:
    pip install flask
    python server.py

Access at: http://localhost:5050
"""

import json
import os
from pathlib import Path
from datetime import datetime
from flask import Flask, jsonify, send_file, abort, render_template_string

# ── Config ─────────────────────────────────────────────────────────────────
PORT        = 5050
RESULTS_DIR = Path(__file__).parent.parent / "laptop_receiver_server" / "Local_database_results"
# ───────────────────────────────────────────────────────────────────────────

app = Flask(__name__)


def _scan_results():
    """
    Scans Local_database_results/ and returns a structured list.
    Returns list of dicts sorted by newest first.
    """
    inspections = []

    if not RESULTS_DIR.exists():
        return inspections

    for event_dir in sorted(RESULTS_DIR.iterdir(), reverse=True):
        if not event_dir.is_dir():
            continue
        event_id = event_dir.name

        for waypoint_dir in sorted(event_dir.iterdir(), reverse=True):
            if not waypoint_dir.is_dir():
                continue
            waypoint_id = waypoint_dir.name

            for class_dir in sorted(waypoint_dir.iterdir()):
                if not class_dir.is_dir():
                    continue
                object_class = class_dir.name

                for instance_dir in sorted(class_dir.iterdir()):
                    if not instance_dir.is_dir():
                        continue
                    instance_id = instance_dir.name

                    result_file  = instance_dir / "result.json"
                    image_file   = instance_dir / "inspection_image.jpg"

                    if not result_file.exists():
                        continue

                    try:
                        result = json.loads(result_file.read_text(encoding="utf-8"))
                    except Exception:
                        result = {}

                    # Use mtime of result.json as timestamp
                    mtime = result_file.stat().st_mtime
                    ts    = datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M:%S")

                    inspections.append({
                        "event_id":     event_id,
                        "waypoint_id":  waypoint_id,
                        "object_class": object_class,
                        "instance_id":  instance_id,
                        "timestamp":    ts,
                        "mtime":        mtime,
                        "has_image":    image_file.exists(),
                        "result":       result,
                        "image_url":    f"/api/image/{event_id}/{waypoint_id}/{object_class}/{instance_id}",
                        "id":           f"{event_id}/{waypoint_id}/{object_class}/{instance_id}",
                    })

    # Sort newest first by mtime
    inspections.sort(key=lambda x: x["mtime"], reverse=True)
    return inspections


# ── API Endpoints ───────────────────────────────────────────────────────────

@app.route("/api/summary")
def api_summary():
    return jsonify(_scan_results())


@app.route("/api/image/<event_id>/<waypoint_id>/<object_class>/<instance_id>")
def api_image(event_id, waypoint_id, object_class, instance_id):
    image_path = RESULTS_DIR / event_id / waypoint_id / object_class / instance_id / "inspection_image.jpg"
    if not image_path.exists():
        abort(404)
    return send_file(str(image_path), mimetype="image/jpeg")

@app.route("/api/result/<event_id>/<waypoint_id>/<object_class>/<instance_id>", methods=["PUT"])
def api_save_result(event_id, waypoint_id, object_class, instance_id):
    from flask import request
    result_path = RESULTS_DIR / event_id / waypoint_id / object_class / instance_id / "result.json"
    if not result_path.exists():
        abort(404)
    try:
        updated = request.get_json(force=True)
        if not isinstance(updated, dict):
            return jsonify({"error": "Invalid JSON body"}), 400
        result_path.write_text(json.dumps(updated, indent=2), encoding="utf-8")
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/")
def index():
    with open(Path(__file__).parent / "index.html", encoding="utf-8") as f:
        return f.read()


if __name__ == "__main__":
    print(f"  Visual Inspection Dashboard")
    print(f"  Serving results from: {RESULTS_DIR.resolve()}")
    print(f"  Open: http://localhost:{PORT}")
    app.run(host="0.0.0.0", port=PORT, debug=False)
