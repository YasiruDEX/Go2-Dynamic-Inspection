
# ------------------------------------------------------------------
# LIVE CALIBRATION VALIDATION (DUAL CAMERA)
# ------------------------------------------------------------------
import cv2
import serial
import time
import numpy as np
import glob
import os
import threading
import socket
import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
from ultralytics import YOLO

# ── Headless stream ────────────────────────────────────────────────────
_sf, _sl, _run = None, threading.Lock(), True
def _upd(f):
    global _sf
    with _sl: _sf = f.copy()
def _enc(f):
    if f is None:
        f = np.zeros((360,1280,3),dtype=np.uint8)
        cv2.putText(f,'Waiting...',(500,180),cv2.FONT_HERSHEY_SIMPLEX,1,(0,200,0),2)
    _, b = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return b.tobytes()
class _H(BaseHTTPRequestHandler):
    def log_message(self,*a): pass
    def do_GET(self):
        if self.path=='/':
            self.send_response(200); self.send_header('Content-type','text/html'); self.end_headers()
            self.wfile.write(b"""<!DOCTYPE html><html><head><title>Calibration Test</title>
<style>body{background:#111;color:#eee;font-family:Arial;text-align:center;padding:20px}h1{color:#00d4ff}img{max-width:98vw;border-radius:8px}</style></head>
<body><h1>&#127919; Calibration Validation &#8212; Jetson Orin Nano</h1>
<img src='/f' id='i'><script>setInterval(()=>{var t=Date.now();document.getElementById('i').src='/f?'+t},120);</script></body></html>""")
        elif self.path.startswith('/f'):
            self.send_response(200); self.send_header('Content-type','image/jpeg'); self.send_header('Cache-Control','no-cache'); self.end_headers()
            with _sl: self.wfile.write(_enc(_sf))
        else: self.send_response(404); self.end_headers()
def _srv(port=8081):
    s=HTTPServer(('0.0.0.0',port),_H)
    threading.Thread(target=s.serve_forever,daemon=True).start()
    try:
        sk=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sk.connect(("8.8.8.8",80)); ip=sk.getsockname()[0]; sk.close()
    except: ip="<JETSON_IP>"
    print(f"\n{'='*50}\n🌐 Calibration stream: http://{ip}:{port}\n{'='*50}\n")

# ==================================================================
# 1. CALIBRATED MATH (Degree 3 Polynomials)
# ==================================================================
# ==================================================================
# 1. LOAD CALIBRATED MATH
# ==================================================================
import sys
import os

# Ensure we can find the config file in the 'tools' directory
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    from calibration_config import calculate_pan, calculate_tilt
    print("✅ Successfully loaded calibration formulas.")
except ImportError as e:
    print(f"⚠️ Import Error: {e}")
    print("Fallback: Using default values to prevent crash (BUT CALIBRATION WILL BE WRONG)")
    def calculate_pan(x, y): return 90
    def calculate_tilt(x, y): return 90

# ==================================================================
# 2. AUTO-DETECTION UTILS
# ==================================================================
def get_camera_indices():
    """Returns (insta_idx, logi_idx) by scanning /sys/class/video4linux and testing"""
    def find_working_camera(name_pattern):
        paths = sorted(glob.glob('/sys/class/video4linux/video*'))
        candidates = []
        
        for path in paths:
            try:
                name_path = os.path.join(path, 'name')
                if not os.path.exists(name_path):
                    continue
                with open(name_path, 'r') as f:
                    name = f.read().strip()
                if name_pattern in name:
                    idx = int(path.split('video')[-1])
                    candidates.append(idx)
            except:
                pass
        
        # Test each candidate
        for idx in candidates:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    return idx
        return -1
    
    insta_idx = find_working_camera("Insta360")
    logi_idx = find_working_camera("HD Pro Webcam")  # Logitech C920
    
    print(f"Detected: Insta360 at {insta_idx}, Logitech at {logi_idx}")
    return insta_idx, logi_idx

# ==================================================================
# 3. MAIN LOOP
# ==================================================================
def main():
    parser = argparse.ArgumentParser(description='Calibration Validation')
    parser.add_argument('--headless', action='store_true',
                        help='Headless mode: browser stream on port 8081')
    args = parser.parse_args()
    HEADLESS = args.headless

    # A. Connect Arduino
    try:
        arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1.0)
        time.sleep(2)
        print("Arduino Connected.")
    except Exception as e:
        print(f"ARDUINO NOT FOUND! Error: {e}")
        print("Running in simulation mode.")
        arduino = None

    # B. Connect Cameras
    insta_id, logi_id = get_camera_indices()

    if insta_id == -1:
        print("CRITICAL ERROR: Insta360 not found! Check connection.")
        return

    print(f"Opening Insta360 (ID {insta_id})...")
    cap_insta = cv2.VideoCapture(insta_id)
    cap_insta.set(3, 640); cap_insta.set(4, 360)

    cap_logi = None
    if logi_id != -1:
        print(f"Opening Logitech (ID {logi_id})...")
        cap_logi = cv2.VideoCapture(logi_id)
        cap_logi.set(3, 640); cap_logi.set(4, 480)
    else:
        print("WARNING: Logitech not found. Showing Insta only.")

    # C. Load YOLO
    print("Loading YOLO...")
    model = YOLO("weights/yolo11n.pt")

    # D. Start headless stream server (port 8081)
    if HEADLESS:
        print("\n--- CALIBRATION VALIDATION (headless) ---")
        print("No visualization. Pure processing + servo control.")
        print("FPS printed every second to terminal.")
    else:
        _srv(port=8081)
        print("\n--- CALIBRATION VALIDATION (headed) ---")
        print("Open http://<JETSON_IP>:8081 in your browser")
    print("Press Ctrl+C to stop at any time")

    # FPS tracking
    fps_count, fps_t0, cur_fps = 0, time.time(), 0.0

    try:
     while _run:
        # Read Frames
        ret1, frame_insta = cap_insta.read()
        if not ret1:
            print("Lost Insta360 frame!"); break
            
        if cap_logi:
            ret2, frame_logi = cap_logi.read()
            if not ret2: frame_logi = np.zeros((360, 640, 3), dtype=np.uint8)
        else:
            frame_logi = np.zeros((360, 640, 3), dtype=np.uint8)

        # Ensure sizes match for concatenation
        frame_logi_resized = cv2.resize(frame_logi, (640, 360))

        # Run YOLO on Insta Only
        results = model(frame_insta, verbose=False, conf=0.5)
        
        target_found = False
        
        if results and len(results[0].boxes) > 0:
            # Pick largest object
            box = results[0].boxes[0] 
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            
            # --- USE CALIBRATED MATH ---
            pred_pan = calculate_pan(cx, cy)
            pred_tilt = calculate_tilt(cx, cy)
            
            # Clamp
            pred_pan = max(0, min(180, int(pred_pan)))
            pred_tilt = max(20, min(160, int(pred_tilt)))
            
            target_found = True

            # Draw
            cv2.rectangle(frame_insta, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame_insta, f"P={pred_pan} T={pred_tilt}", (int(x1), int(y1)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Move Servo!
            if arduino:
                # INVERT TILT for reversed servo mounting
                inverted_tilt = 180 - pred_tilt
                cmd = f"{inverted_tilt},{pred_pan}\n"
                arduino.write(cmd.encode())

        # Combine Views (Side by Side)
        h_concat = np.hstack((frame_insta, frame_logi_resized))

        # Overlay Labels
        cv2.putText(h_concat, "INSTA360 (Sensor)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(h_concat, "LOGITECH (Tracking)", (640+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        # FPS
        fps_count += 1
        el = time.time() - fps_t0
        if el >= 1.0:
            cur_fps = fps_count / el
            fps_count = 0; fps_t0 = time.time()
            print(f"[FPS] {cur_fps:.1f} fps")

        # Visualization (headed only - skip entirely in headless for max speed)
        if not HEADLESS:
            h_concat = np.hstack((frame_insta, frame_logi_resized))
            cv2.putText(h_concat, "INSTA360 (Sensor)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.putText(h_concat, "LOGITECH (Tracking)", (640+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.putText(h_concat, f"FPS: {cur_fps:.1f}", (10, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
            _upd(h_concat)

     # end while
    except KeyboardInterrupt:
        print("\n⚠️  Stopped by Ctrl+C")

    cap_insta.release()
    if cap_logi: cap_logi.release()
    if arduino: arduino.close()
    print("\n✅ Done")

if __name__ == "__main__":
    main()
