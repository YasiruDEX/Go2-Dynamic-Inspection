#!/usr/bin/env python3
"""
Test Script 2: Test Camera Feeds (Browser Streaming)
======================================================
Streams both cameras as MJPEG over HTTP.
No display needed on Jetson - just open in your laptop browser!

Usage:
    python3 test/02_test_camera_feeds.py

Then on your LAPTOP open browser at:
    http://<JETSON_IP>:8080

Uses only Python built-in libraries + OpenCV (already installed).
No extra packages needed!
"""

import cv2
import sys
import os
import glob
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── Global frames ────────────────────────────────────────────────
frame_insta = None
frame_logi  = None
frame_lock  = threading.Lock()
running     = True

# ── Camera detection ─────────────────────────────────────────────
def find_camera(name_pattern):
    paths = sorted(glob.glob('/sys/class/video4linux/video*'))
    candidates = []
    for path in paths:
        try:
            name_path = os.path.join(path, 'name')
            if not os.path.exists(name_path):
                continue
            with open(name_path, 'r') as f:
                name = f.read().strip()
            if name_pattern.lower() in name.lower():
                idx = int(path.split('video')[-1])
                candidates.append(idx)
        except:
            pass

    for idx in candidates:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return idx
    return -1

# ── Camera capture thread ────────────────────────────────────────
def capture_loop(insta_id, logi_id):
    global frame_insta, frame_logi, running

    cap_insta = cv2.VideoCapture(insta_id)
    cap_logi  = cv2.VideoCapture(logi_id)

    cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH,   640)
    cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT,  480)

    frame_count = 0
    while running:
        ret_i, fi = cap_insta.read()
        ret_l, fl = cap_logi.read()

        if not ret_i or not ret_l:
            print("⚠️  Lost camera frame, retrying...")
            time.sleep(0.1)
            continue

        frame_count += 1

        # Add labels
        cv2.putText(fi, f"Insta360 | Frame {frame_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(fl, f"Logitech | Frame {frame_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        with frame_lock:
            frame_insta = fi.copy()
            frame_logi  = fl.copy()

        time.sleep(0.033)  # ~30 FPS

    cap_insta.release()
    cap_logi.release()
    print("📷 Cameras released")

# ── JPEG encode helper ───────────────────────────────────────────
def encode_jpeg(frame):
    if frame is None:
        # Return a blank grey frame if not ready yet
        blank = 255 * __import__('numpy').ones((240, 320, 3), dtype='uint8') * 100
        cv2.putText(blank, "Waiting for camera...",
                    (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
        frame = blank
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes()

# ── HTTP Handler ─────────────────────────────────────────────────
class CameraHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass  # Suppress request logs

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
<!DOCTYPE html>
<html>
<head>
  <title>Jetson Camera Test</title>
  <style>
    body { background: #1a1a2e; color: #eee; font-family: Arial, sans-serif;
           display: flex; flex-direction: column; align-items: center; padding: 20px; }
    h1   { color: #00d4ff; }
    .cams { display: flex; gap: 20px; flex-wrap: wrap; justify-content: center; }
    .cam  { background: #16213e; border-radius: 10px; padding: 15px; text-align: center; }
    .cam h2 { color: #00d4ff; margin: 0 0 10px 0; }
    img  { border-radius: 8px; width: 640px; max-width: 90vw; }
    .status { margin-top: 20px; color: #0f3460; background:#00d4ff;
              padding: 8px 20px; border-radius: 20px; font-weight: bold; }
  </style>
</head>
<body>
  <h1>&#127909; Jetson Orin Nano - Camera Test</h1>
  <div class="cams">
    <div class="cam">
      <h2>&#127760; Insta360 X3 (Wide)</h2>
      <img src="/insta360" onerror="this.src=this.src">
    </div>
    <div class="cam">
      <h2>&#128247; Logitech C920 (Narrow)</h2>
      <img src="/logitech" onerror="this.src=this.src">
    </div>
  </div>
  <div class="status">&#9679; Live Stream - Both Cameras Active</div>
  <script>
    // Refresh images every 100ms
    function refreshCams() {
      var ts = new Date().getTime();
      document.querySelectorAll('img').forEach(function(img) {
        var base = img.src.split('?')[0];
        img.src = base + '?' + ts;
      });
    }
    setInterval(refreshCams, 100);
  </script>
</body>
</html>"""
            self.wfile.write(html.encode())

        elif self.path.startswith('/insta360'):
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with frame_lock:
                data = encode_jpeg(frame_insta)
            self.wfile.write(data)

        elif self.path.startswith('/logitech'):
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with frame_lock:
                data = encode_jpeg(frame_logi)
            self.wfile.write(data)

        else:
            self.send_response(404)
            self.end_headers()


# ── Main ─────────────────────────────────────────────────────────
def main():
    global running

    print("=" * 60)
    print("CAMERA FEED TEST — Browser Streaming Mode")
    print("=" * 60)
    print("\nNo display needed! Works over SSH.\n")

    # Find cameras
    print("🔍 Searching for cameras...")
    insta_id = find_camera("Insta360")
    logi_id  = find_camera("HD Pro Webcam")

    if insta_id == -1:
        print("❌ Insta360 not found!")
        print("   Run python3 test/01_check_cameras.py first")
        sys.exit(1)
    if logi_id == -1:
        print("❌ Logitech not found!")
        sys.exit(1)

    print(f"✅ Insta360  → /dev/video{insta_id}")
    print(f"✅ Logitech  → /dev/video{logi_id}")

    # Start camera capture thread
    cam_thread = threading.Thread(target=capture_loop, args=(insta_id, logi_id), daemon=True)
    cam_thread.start()
    print("\n📷 Camera capture started")

    # Get Jetson real network IP (skip loopback 127.x.x.x)
    try:
        import socket
        # Connect to external address (doesn't send data, just finds the right interface)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        jetson_ip = s.getsockname()[0]
        s.close()
    except:
        jetson_ip = "<JETSON_IP>"

    # Start HTTP server
    PORT = 8080
    server = HTTPServer(('0.0.0.0', PORT), CameraHandler)

    print(f"\n{'='*60}")
    print(f"🌐 Camera stream running!")
    print(f"{'='*60}")
    print(f"\n   Open on your LAPTOP browser:")
    print(f"\n   ➜  http://{jetson_ip}:{PORT}")
    print(f"\n   (If IP wrong, check with: hostname -I)")
    print(f"\n   Press Ctrl+C to stop\n")
    print("=" * 60)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n⚠️  Stopped by Ctrl+C")
        running = False
        server.shutdown()
        print("✅ Done")

if __name__ == "__main__":
    main()
