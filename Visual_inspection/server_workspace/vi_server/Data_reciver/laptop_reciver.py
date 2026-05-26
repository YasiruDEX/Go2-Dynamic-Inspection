"""
Laptop/Server Data Receiver for Visual Inspection System
=========================================================
Receives ROI images sent from the Jetson Orin Nano via HTTP POST.

Jetson sends:
  POST /upload
  JSON payload:
    {
      "session":     "20260429_193000",
      "object_id":   1,
      "class_name":  "fire_extinguisher",
      "image_idx":   1,          (1-4 = closeup, 5 = overview)
      "total":       5,
      "timestamp":   1745893200,
      "location":    "gauge_room_A",
      "image_b64":   "<base64-encoded JPEG>"
    }

Saved folder structure:
  data/
  └── <session>/
      └── <class_name>/
          └── instance_<object_id>/
              ├── img_01.jpg
              ├── img_02.jpg
              ├── img_03.jpg
              ├── img_04.jpg
              └── overview_<location>_01.jpg

Run:
  python laptop_reciver.py [--port 9000] [--host 0.0.0.0]
"""

import argparse
import base64
import json
import logging
import os
import sys
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

# ── Configuration ────────────────────────────────────────────
DATA_DIR = Path(__file__).parent / "data"   # save path
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9000

# ── Logging ──────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger("DataReceiver")

# ── Stats counter ────────────────────────────────────────────
stats = {
    "total_received": 0,
    "total_saved": 0,
    "total_errors": 0,
    "sessions": set(),
}


def save_image(payload: dict) -> Path:
    """
    Save decoded image to structured folder.

    Returns:
        Path where image was saved.

    Raises:
        KeyError: If required fields are missing from payload.
        ValueError: If base64 decoding fails.
    """
    session    = payload["session"]
    object_id  = int(payload.get("object_id", 1))
    class_name = payload["class_name"].strip().replace(" ", "_")
    image_idx  = int(payload.get("image_idx", 1))
    location   = payload.get("location", "unknown")
    image_b64  = payload["image_b64"]

    # Decode image
    try:
        image_bytes = base64.b64decode(image_b64)
    except Exception as e:
        raise ValueError(f"Base64 decode failed: {e}")

    # Build folder: data/<session>/<class_name>/instance_<object_id>/
    instance_dir = DATA_DIR / session / class_name / f"instance_{object_id}"
    instance_dir.mkdir(parents=True, exist_ok=True)

    # Filename logic: idx 1-4 = img_0N.jpg, idx 5+ = overview
    if image_idx <= 4:
        filename = f"img_{image_idx:02d}.jpg"
    else:
        # overview image — include location in name
        overview_count = image_idx - 4
        filename = f"overview_{location}_{overview_count:02d}.jpg"

    save_path = instance_dir / filename

    with open(save_path, "wb") as f:
        f.write(image_bytes)

    return save_path


class ReceiverHandler(BaseHTTPRequestHandler):
    """HTTP request handler for image upload endpoint."""

    def log_message(self, format, *args):
        """Suppress default HTTP logs — we use our own."""
        pass

    def _send_json(self, status: int, body: dict):
        """Send JSON response."""
        data = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        """Health check endpoint."""
        if self.path == "/health" or self.path == "/":
            self._send_json(200, {
                "status": "ok",
                "total_received": stats["total_received"],
                "total_saved": stats["total_saved"],
                "total_errors": stats["total_errors"],
                "sessions": list(stats["sessions"]),
                "data_dir": str(DATA_DIR.absolute()),
            })
        else:
            self._send_json(404, {"error": "Not found"})

    def do_POST(self):
        """Receive image upload from Jetson."""
        if self.path != "/upload":
            self._send_json(404, {"error": "Unknown endpoint. Use POST /upload"})
            return

        # Read body
        content_length = int(self.headers.get("Content-Length", 0))
        if content_length == 0:
            self._send_json(400, {"error": "Empty request body"})
            return

        raw_body = self.rfile.read(content_length)

        # Parse JSON
        try:
            payload = json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError as e:
            logger.error(f"JSON parse error: {e}")
            self._send_json(400, {"error": f"Invalid JSON: {e}"})
            stats["total_errors"] += 1
            return

        stats["total_received"] += 1

        # Validate required fields
        required = ["session", "class_name", "image_b64"]
        missing = [f for f in required if f not in payload]
        if missing:
            logger.error(f"Missing fields: {missing}")
            self._send_json(400, {"error": f"Missing fields: {missing}"})
            stats["total_errors"] += 1
            return

        # Save image
        try:
            save_path = save_image(payload)
            stats["total_saved"] += 1
            stats["sessions"].add(payload["session"])

            logger.info(
                f"✅ Saved [{payload['class_name']}] "
                f"session={payload['session']} "
                f"obj={payload.get('object_id',1)} "
                f"idx={payload.get('image_idx',1)}/{payload.get('total','?')} "
                f"→ {save_path.relative_to(DATA_DIR)}"
            )

            self._send_json(200, {
                "status": "ok",
                "saved_to": str(save_path),
                "session": payload["session"],
                "class_name": payload["class_name"],
                "image_idx": payload.get("image_idx", 1),
            })

        except KeyError as e:
            logger.error(f"Missing field in payload: {e}")
            self._send_json(400, {"error": f"Missing field: {e}"})
            stats["total_errors"] += 1

        except ValueError as e:
            logger.error(f"Image decode error: {e}")
            self._send_json(422, {"error": str(e)})
            stats["total_errors"] += 1

        except Exception as e:
            logger.error(f"Unexpected error saving image: {e}", exc_info=True)
            self._send_json(500, {"error": f"Server error: {e}"})
            stats["total_errors"] += 1


def run_server(host: str, port: int):
    """Start the HTTP receiver server."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    server = HTTPServer((host, port), ReceiverHandler)

    logger.info("=" * 55)
    logger.info("  Visual Inspection — Data Receiver")
    logger.info("=" * 55)
    logger.info(f"  Listening : http://{host}:{port}")
    logger.info(f"  Upload at : POST http://<this-pc-ip>:{port}/upload")
    logger.info(f"  Health    : GET  http://<this-pc-ip>:{port}/health")
    logger.info(f"  Data dir  : {DATA_DIR.absolute()}")
    logger.info("=" * 55)
    logger.info("  Waiting for images from Jetson...")
    logger.info("")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("")
        logger.info("=" * 55)
        logger.info(f"  Stopped. Summary:")
        logger.info(f"    Received : {stats['total_received']}")
        logger.info(f"    Saved    : {stats['total_saved']}")
        logger.info(f"    Errors   : {stats['total_errors']}")
        logger.info(f"    Sessions : {list(stats['sessions'])}")
        logger.info("=" * 55)
        server.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Receive ROI images from Jetson Orin Nano"
    )
    parser.add_argument(
        "--host", default=DEFAULT_HOST,
        help=f"Bind host (default: {DEFAULT_HOST})"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"Bind port (default: {DEFAULT_PORT})"
    )
    args = parser.parse_args()

    run_server(args.host, args.port)
