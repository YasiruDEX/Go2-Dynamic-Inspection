"""
laptop_receiver.py — HTTP server running on the laptop.
Receives images + metadata.json from the Jetson ROS2 image_uploader node.

Jetson sends (multipart/form-data POST to /upload):
    Form fields:  session_label, subfolder, object_class
    Files (key='files'): img_*.jpg + metadata.json

Folder structure saved to Local_database/:
    Local_database/<session_label>/<object_class>/<subfolder>/
        img_01.jpg, img_02.jpg, metadata.json

Run:
    pip install flask
    python laptop_receiver.py
"""

import json
import pathlib
from flask import Flask, request, jsonify

PORT      = 8888
SAVE_ROOT = pathlib.Path(__file__).parent / 'Local_database'

app = Flask(__name__)


@app.route('/upload', methods=['POST'])
def upload():
    session_label = request.form.get('session_label', 'unknown_session')
    subfolder     = request.form.get('subfolder', 'instance_1')
    object_class  = request.form.get('object_class', 'unknown')

    if not request.files:
        return jsonify({'success': False, 'info': 'No files in request'}), 400

    # ── Collect all files in memory ───────────────────────────────────────
    collected = {}
    for file in request.files.getlist('files'):
        if file.filename:
            collected[file.filename] = file.read()

    # ── Read location_label from metadata.json if present ─────────────────
    # Jetson may send location_label inside metadata.json for EventID,WaypointID
    location_label = session_label   # default: use session_label as top folder
    if 'metadata.json' in collected:
        try:
            meta = json.loads(collected['metadata.json'].decode('utf-8'))
            # Try to get a richer label from metadata
            loc = (meta.get('location_label')
                   or meta.get('section_label')
                   or meta.get('label')
                   or session_label)
            if loc:
                location_label = loc
            # Object class can also come from metadata
            object_class = meta.get('class', object_class) or object_class
        except Exception as e:
            print(f'[!] Could not parse metadata.json: {e}')

    # ── Parse EventID / WaypointID if label is "EventID,WaypointID" ───────
    # If label is just a session string (e.g. "210503H"), use it as EventID
    parts = [p.strip() for p in location_label.split(',')]
    event_id    = parts[0]
    waypoint_id = parts[1] if len(parts) >= 2 else subfolder

    # ── Build save path ────────────────────────────────────────────────────
    # Local_database/<EventID>/<WaypointID>/<object_class>/<subfolder>/
    dest = SAVE_ROOT / event_id / waypoint_id / object_class / subfolder
    dest.mkdir(parents=True, exist_ok=True)

    # ── Save all files ─────────────────────────────────────────────────────
    saved = []
    for filename, data in collected.items():
        (dest / filename).write_bytes(data)
        saved.append(filename)

    print(f'[OK] Saved {len(saved)} files → {dest}')

    return jsonify({
        'success':     True,
        'saved_to':    str(dest),
        'event_id':    event_id,
        'waypoint_id': waypoint_id,
        'files':       saved,
    }), 200


@app.route('/health', methods=['GET'])
def health():
    return jsonify({'status': 'running', 'port': PORT}), 200


if __name__ == '__main__':
    SAVE_ROOT.mkdir(parents=True, exist_ok=True)
    print(f'Receiver listening on 0.0.0.0:{PORT}')
    print(f'Saving to: {SAVE_ROOT.resolve()}')
    app.run(host='0.0.0.0', port=PORT, debug=False)
