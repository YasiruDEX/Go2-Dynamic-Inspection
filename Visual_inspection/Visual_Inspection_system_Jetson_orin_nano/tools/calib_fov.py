# Calibration Tool with YOLO Assistance
# -------------------------------------

import cv2
import numpy as np
import time
import serial
from ultralytics import YOLO
import os

# Auto-detect cameras
def get_camera_indices():
    insta_idx = -1
    logi_idx = -1
    import glob
    import os
    
    paths = sorted(glob.glob('/sys/class/video4linux/video*'))
    for path in paths:
        try:
            name_path = os.path.join(path, 'name')
            if not os.path.exists(name_path): continue
            with open(name_path, 'r') as f:
                name = f.read().strip()
            idx = int(path.split('video')[-1])
            
            if "Insta360" in name:
                if insta_idx == -1 or idx < insta_idx: insta_idx = idx
            elif "C920" in name or "Logitech" in name:
                if logi_idx == -1 or idx < logi_idx: logi_idx = idx
        except: pass
    return insta_idx, logi_idx

# Configuration
INSTA360_ID, LOGITECH_ID = get_camera_indices()
if INSTA360_ID == -1: INSTA360_ID = 2 # Fallback
if LOGITECH_ID == -1: LOGITECH_ID = 0 # Fallback

print(f"Detected Cameras -> Insta360: {INSTA360_ID}, Logitech: {LOGITECH_ID}")

ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
MODEL_PATH = "weights/yolo11n.pt"

# State
current_pan = 90
current_tilt = 90
data_points = [] # List of (inst_y, tilt_angle)

def main():
    global current_pan, current_tilt
    
    # 1. Connect to Arduino
    try:
        print(f"Connecting to Arduino on {ARDUINO_PORT}...")
        arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1.0)
        time.sleep(3) # Wait for Arduino reset
        print("Sending initial HOME command...")
        arduino.write(b"90,90\n")
        arduino.flush()
        time.sleep(1)
        print("Arduino READY.")
    except Exception as e:
        print(f"Arduino NOT connected: {e}")
        arduino = None

    # 2. Manage Data File (Auto-Archive)
    data_path = "/home/dinethra/Jetson_orin_nano/data/calibration_points.csv"
    if os.path.exists(data_path):
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = f"/home/dinethra/Jetson_orin_nano/data/calibration_points_{timestamp}.csv"
        try:
            os.rename(data_path, backup_path)
            print(f"\n[INFO] Old calibration file detected.")
            print(f"       -> Archived to: {os.path.basename(backup_path)}")
            print(f"       -> Starting FRESH: {os.path.basename(data_path)}")
        except OSError as e:
             print(f"[WARN] Could not archive file: {e}")
    else:
        print(f"\n[INFO] Starting FRESH calibration file.")

    # 3. Load YOLO
    print("Loading YOLO model...")
    model = YOLO(MODEL_PATH)

    # 3. Open Cameras
    cap_insta = cv2.VideoCapture(INSTA360_ID)
    cap_logi = cv2.VideoCapture(LOGITECH_ID)
    
    # Set resolutions
    cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    cv2.namedWindow("Automatic Calibration")

    print("\nControls:")
    print("  W/S : Tilt Up/Down")
    print("  A/D : Pan Left/Right")
    print("  SPACE: Record data point (when object is centered in Logitech)")
    print("  C   : Calculate equation")
    print("  Q   : Quit")
    
    # Track last known position
    insta_x = -1
    insta_y = -1
    
    while True:
        ret1, frame_insta = cap_insta.read()
        ret2, frame_logi = cap_logi.read()
        
        if not ret1 or not ret2: break
        
        # Run YOLO on BOTH frames
        results_insta = model(frame_insta, verbose=False, conf=0.5)
        results_logi = model(frame_logi, verbose=False, conf=0.5)
        
        # Draw on Insta360
        # Reset temporarily for this frame drawing, but keep last valid for Space
        current_insta_x = -1 
        current_insta_y = -1
        
        if results_insta and len(results_insta[0].boxes) > 0:
            box = results_insta[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            current_insta_x = (x1 + x2) / 2
            current_insta_y = (y1 + y2) / 2
            
            # Update global tracking vars only if found
            insta_x = current_insta_x
            insta_y = current_insta_y
            
            cv2.rectangle(frame_insta, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame_insta, (int(insta_x), int(insta_y)), 5, (0, 0, 255), -1)
            
        # Draw on Logitech
        centered = False
        if results_logi and len(results_logi[0].boxes) > 0:
            box = results_logi[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = (x1+x2)/2, (y1+y2)/2
            cv2.rectangle(frame_logi, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            
            # Check if centered (within 50px of center)
            h, w = frame_logi.shape[:2]
            if abs(cx - w/2) < 50 and abs(cy - h/2) < 50:
                centered = True
                cv2.putText(frame_logi, "ALIGNED!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show alignment help
        cv2.line(frame_logi, (320, 0), (320, 480), (100, 100, 100), 1)
        cv2.line(frame_logi, (0, 240), (640, 240), (100, 100, 100), 1)

        # Combine
        h1, w1 = frame_insta.shape[:2]
        h2, w2 = frame_logi.shape[:2]
        canvas = np.zeros((h1 + h2, max(w1, w2), 3), dtype=np.uint8)
        canvas[:h1, :w1] = frame_insta
        canvas[h1:h1+h2, :w2] = frame_logi
        
        cv2.putText(canvas, f"Points: {len(data_points)}", (10, h1+30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Automatic Calibration", canvas)
        
        # Keys
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'): break
        elif key == ord('w'): 
            current_tilt = max(20, current_tilt - 2)
            cmd = f"{current_tilt},{current_pan}\n"
            print(f"Sending: {cmd.strip()}")
            if arduino: 
                arduino.write(cmd.encode())
                arduino.flush()
        elif key == ord('s'): 
            current_tilt = min(160, current_tilt + 2)
            cmd = f"{current_tilt},{current_pan}\n"
            print(f"Sending: {cmd.strip()}")
            if arduino: 
                arduino.write(cmd.encode())
                arduino.flush()
        elif key == ord('a'): 
            current_pan = max(0, current_pan - 2)
            cmd = f"{current_tilt},{current_pan}\n"
            print(f"Sending: {cmd.strip()}")
            if arduino: 
                arduino.write(cmd.encode())
                arduino.flush()
        elif key == ord('d'): 
            current_pan = min(180, current_pan + 2)
            cmd = f"{current_tilt},{current_pan}\n"
            print(f"Sending: {cmd.strip()}")
            if arduino: 
                arduino.write(cmd.encode())
                arduino.flush()
            
        elif key == ord(' ') and insta_y != -1:
            # Record Point (Full X, Y, Pan, Tilt)
            data_points.append((insta_x, insta_y, current_pan, current_tilt))
            print(f"Captured: Insta=({insta_x:.1f},{insta_y:.1f}) -> Servo=({current_pan},{current_tilt})")
            
            # Save to file immediately with flush/fsync
            abs_path = "/home/dinethra/Jetson_orin_nano/data/calibration_points.csv"
            with open(abs_path, "a") as f:
                f.write(f"{insta_x},{insta_y},{current_pan},{current_tilt}\n")
                f.flush()
                os.fsync(f.fileno())
            
            print(f"  -> Saved to {abs_path}")
            
            # Flash
            cv2.circle(canvas, (int(insta_x), int(insta_y)), 10, (255, 0, 0), -1)
            cv2.imshow("Automatic Calibration", canvas)
            cv2.waitKey(200)

        elif key == ord('c'):
            calculate_mapping(data_points, h1, w1)

    cap_insta.release()
    cap_logi.release()
    cv2.destroyAllWindows()

def calculate_mapping(points, height, width):
    if len(points) < 2:
        print("Need at least 2 points!")
        return
        
    pts = np.array(points)
    # Col 0: Insta X, Col 1: Insta Y, Col 2: Pan, Col 3: Tilt
    x = pts[:, 0] 
    y = pts[:, 1]
    pan = pts[:, 2]
    tilt = pts[:, 3]
    
    # 1. Pan Calibration: Pan = m_x * Pixel_X + c_x
    A_x = np.vstack([x, np.ones(len(x))]).T
    m_x, c_x = np.linalg.lstsq(A_x, pan, rcond=None)[0]
    
    # 2. Tilt Calibration: Tilt = m_y * Pixel_Y + c_y
    A_y = np.vstack([y, np.ones(len(y))]).T
    m_y, c_y = np.linalg.lstsq(A_y, tilt, rcond=None)[0]
    
    print("\n" + "="*40)
    print("FULL CALIBRATION RESULTS")
    print("="*40)
    print(f"Data Points: {len(points)}")
    print("-" * 40)
    print(f"PAN Equation:  Pan  = ({m_x:.4f}) * Pixel_X + ({c_x:.4f})")
    print(f"TILT Equation: Tilt = ({m_y:.4f}) * Pixel_Y + ({c_y:.4f})")
    print("-" * 40)
    print("Test Values:")
    print(f"  Pixel X=0   (Left)   -> Pan  = {c_x:.1f}")
    print(f"  Pixel X={width} (Right)  -> Pan  = {m_x*width + c_x:.1f}")
    print(f"  Pixel Y=0   (Top)    -> Tilt = {c_y:.1f}")
    print(f"  Pixel Y={height} (Bottom) -> Tilt = {m_y*height + c_y:.1f}")
    print("="*40 + "\n")

if __name__ == "__main__":
    main()
