"""
laptop_receiver.py — HTTP server that runs on YOUR LAPTOP
=========================================================
Receives images + metadata.json from the Jetson via HTTP POST
and saves them in an organized folder structure.

HOW TO RUN (on your laptop, connected to YasiruDEX network):
    pip install flask
    python3 laptop_receiver.py

HOW TO FIND YOUR LAPTOP IP (on YasiruDEX network):
    ip route get 8.8.8.8 | awk '{print $7; exit}'
    # → something like 192.168.1.XXX — use this in the Jetson's laptop_url param

SAVED FOLDER STRUCTURE (on laptop):
    received_captures/
    ├── engine_room_A/          ← session_label from BT
    │   ├── gauge/
    │   │   └── instance_1/
    │   │       ├── img_01_conf0.85.jpg
    │   │       ├── img_02_conf0.85.jpg
    │   │       ├── img_03_conf0.85.jpg
    │   │       └── metadata.json
    │   └── extinguisher/
    │       └── instance_1/
    │           ├── img_01_conf0.90.jpg
    │           └── metadata.json
"""

import os
import json
import pathlib
from datetime import datetime
from flask import Flask, request, jsonify

# ── Config ────────────────────────────────────────────────────────────────────
PORT       = 8888
SAVE_ROOT  = pathlib.Path(__file__).parent / 'received_captures'
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)


@app.route('/upload', methods=['POST'])
def upload():
    """
    Receives files from the Jetson image_uploader node.
    Form fields:  session_label, subfolder, object_class
    Files:        images + metadata.json

    Folder structure on laptop:
        received_captures/<location_label>/<object_class>/<subfolder>/

    location_label is read directly from the metadata.json content.
    Falls back to session_label form field if metadata.json not present.
    """
    session_label = request.form.get('session_label', 'unknown_session')
    subfolder     = request.form.get('subfolder',     'instance_1')
    object_class  = request.form.get('object_class',  'unknown')

    if not request.files:
        return jsonify({'success': False, 'info': 'No files in request'}), 400

    # ── Collect all files in memory first so we can read metadata.json ────────
    collected = {}
    for _, file in request.files.items(multi=True):
        if file.filename == '':
            continue
        collected[file.filename] = file.read()

    # ── Extract location_label from metadata.json (if present) ────────────────
    location_label = session_label   # default fallback
    if 'metadata.json' in collected:
        try:
            meta = json.loads(collected['metadata.json'].decode('utf-8'))
            location_label = meta.get('location_label', session_label) or session_label
            object_class   = meta.get('class', object_class)
        except Exception as e:
            print(f'[!] Could not parse metadata.json: {e}')

    # ── Build save path ───────────────────────────────────────────────────────
    # received_captures/<location_label>/<object_class>/<instance_N>/
    dest = SAVE_ROOT / location_label / object_class / subfolder
    dest.mkdir(parents=True, exist_ok=True)

    # ── Save all files ────────────────────────────────────────────────────────
    saved_files = []
    for filename, data in collected.items():
        save_path = dest / filename
        save_path.write_bytes(data)
        saved_files.append(filename)

    print(f'[✓] Saved {len(saved_files)} files → {dest}')
    print(f'    Location: {location_label}  Class: {object_class}')
    print(f'    Files: {saved_files}')

    return jsonify({
        'success':        True,
        'saved_count':    len(saved_files),
        'saved_to':       str(dest),
        'location_label': location_label,
        'files':          saved_files
    }), 200



@app.route('/status', methods=['GET'])
def status():
    """Health-check endpoint — Jetson can ping this to verify server is up."""
    folders = list(SAVE_ROOT.rglob('instance_*')) if SAVE_ROOT.exists() else []
    return jsonify({
        'status':     'running',
        'save_root':  str(SAVE_ROOT),
        'instances':  len(folders)
    }), 200


if __name__ == '__main__':
    SAVE_ROOT.mkdir(parents=True, exist_ok=True)
    print('=' * 60)
    print('  Visual Inspection — Laptop Receiver')
    print('=' * 60)
    print(f'  Saves to : {SAVE_ROOT.resolve()}')
    print(f'  Port     : {PORT}')
    print()
    print('  Find your IP (run in another terminal):')
    print('    ip route get 8.8.8.8 | awk \'{print $7; exit}\'')
    print()
    print('  Then on Jetson, run image_uploader with:')
    print('    ros2 run visual_inspection_ros image_uploader \\')
    print('        --ros-args -p laptop_url:=http://<YOUR_IP>:8888/upload')
    print('=' * 60)
    app.run(host='0.0.0.0', port=PORT, debug=False)
