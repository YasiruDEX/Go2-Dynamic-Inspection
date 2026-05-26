#!/usr/bin/env python3
"""
IBVS (Image-Based Visual Servoing) Pipeline
============================================
Two-stage object tracking and centering system:

Stage 1: Coarse Direction (Insta360)
  - YOLO detects object in panoramic view
  - Calibrated formulas → servo angles
  - Pan-tilt moves to point Logitech at object

Stage 2: Precise Centering (Logitech + IBVS)
  - YOLO detects object in Logitech view
  - ByteTrack tracks object across frames
  - IBVS control loop centers object using camera intrinsics
  - Distance-independent (closed-loop control)

Usage:
  python3 tools/ibvs_pipeline.py
"""

import cv2
import serial
import time
import numpy as np
import yaml
import os
import sys
import glob
from ultralytics import YOLO
from collections import deque

# Add tools directory to path for calibration config
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Import calibrated formulas for Insta360 → Servo
try:
    from calibration_config import calculate_pan, calculate_tilt
    print("✅ Loaded Insta360 calibration formulas")
except ImportError:
    print("⚠️ WARNING: Using default formulas (calibration_config.py not found)")
    def calculate_pan(x, y): return 90
    def calculate_tilt(x, y): return 90

# ============================================================================
# CONFIGURATION
# ============================================================================

class Config:
    # Arduino
    ARDUINO_PORT = '/dev/ttyACM0'
    ARDUINO_BAUD = 9600
    
    # Servo limits
    PAN_MIN, PAN_MAX = 0, 180
    TILT_MIN, TILT_MAX = 20, 160
    
    # YOLO
    YOLO_MODEL = "weights/yolo11n.pt"
    YOLO_CONF = 0.5
    
    # IBVS Control (PID Tuning)
    IBVS_ENABLED = True
    
    # PID Gains for Pan (tune these to reduce overshooting!)
    IBVS_KP_PAN = 0.12   # Proportional - increased for faster response
    IBVS_KI_PAN = 0.002  # Integral - helps eliminate steady-state error
    IBVS_KD_PAN = 0.02   # Derivative - dampens oscillations
    
    # PID Gains for Tilt
    IBVS_KP_TILT = 0.12
    IBVS_KI_TILT = 0.002
    IBVS_KD_TILT = 0.02
    
    # Control limits
    IBVS_DEADZONE = 10  # pixels - don't move if error < this
    IBVS_MAX_ITER = 200  # Max iterations per centering
    IBVS_CONVERGENCE_THRESHOLD = 10  # pixels - consider centered
    IBVS_SERVO_DELAY = 0.01  # seconds - 10ms for fast response
    
    # Tracking
    TRACK_HISTORY_LEN = 30  # frames
    TRACK_LOST_THRESHOLD = 10  # frames - switch back to Insta if lost
    
    # Display
    SHOW_DEBUG = True

# ============================================================================
# CAMERA UTILITIES
# ============================================================================

def find_camera(name_pattern):
    """Find camera by name pattern and verify it can open"""
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
    
    # Test each candidate to see if it can actually open
    for idx in candidates:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return idx
    
    return -1

def load_logitech_intrinsics():
    """Load Logitech camera intrinsics from Kalibr calibration"""
    yaml_path = 'config/logitech_intrinsics.yaml'
    if not os.path.exists(yaml_path):
        print(f"⚠️ WARNING: {yaml_path} not found! Using default intrinsics.")
        return {
            'fx': 640.0,
            'fy': 640.0,
            'cx': 320.0,
            'cy': 240.0
        }
    
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    
    print(f"✅ Loaded Logitech intrinsics from Kalibr:")
    print(f"   fx={data['fx']:.2f}, fy={data['fy']:.2f}")
    print(f"   cx={data['cx']:.2f}, cy={data['cy']:.2f}")
    
    return data

# ============================================================================
# SIMPLE TRACKER (Centroid-based)
# ============================================================================

class SimpleTracker:
    """Simple centroid-based tracker for single object"""
    def __init__(self, max_disappeared=10):
        self.max_disappeared = max_disappeared
        self.disappeared = 0
        self.centroid = None
        self.bbox = None
        self.is_tracking = False
    
    def update(self, detection):
        """
        Update tracker with new detection
        detection: (x1, y1, x2, y2, conf, cls) or None
        """
        if detection is None:
            self.disappeared += 1
            if self.disappeared > self.max_disappeared:
                self.is_tracking = False
                self.centroid = None
                self.bbox = None
            return self.centroid, self.bbox
        
        # Reset disappeared counter
        self.disappeared = 0
        self.is_tracking = True
        
        x1, y1, x2, y2 = detection[:4]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        self.centroid = (cx, cy)
        self.bbox = (x1, y1, x2, y2)
        
        return self.centroid, self.bbox

# ============================================================================
# IBVS CONTROLLER
# ============================================================================

class IBVSController:
    """Image-Based Visual Servoing Controller with PID"""
    def __init__(self, intrinsics, config):
        self.fx = intrinsics['fx']
        self.fy = intrinsics['fy']
        self.cx = intrinsics['cx']
        self.cy = intrinsics['cy']
        self.config = config
        
        # PID gains (tune these!)
        self.kp_pan = config.IBVS_KP_PAN      # Proportional
        self.ki_pan = config.IBVS_KI_PAN      # Integral
        self.kd_pan = config.IBVS_KD_PAN      # Derivative
        
        self.kp_tilt = config.IBVS_KP_TILT
        self.ki_tilt = config.IBVS_KI_TILT
        self.kd_tilt = config.IBVS_KD_TILT
        
        # PID state
        self.integral_pan = 0
        self.integral_tilt = 0
        self.prev_error_pan = 0
        self.prev_error_tilt = 0
        
        # Anti-windup limits
        self.integral_limit = 50.0  # degrees
        
        # History for smoothing
        self.error_history = deque(maxlen=10)
    
    def reset(self):
        """Reset PID state"""
        self.integral_pan = 0
        self.integral_tilt = 0
        self.prev_error_pan = 0
        self.prev_error_tilt = 0
    
    def compute_error(self, detected_point):
        """
        Compute pixel error from image center
        Returns: (e_x, e_y) in pixels
        """
        if detected_point is None:
            return None, None
        
        cx_detected, cy_detected = detected_point
        e_x = cx_detected - self.cx
        e_y = cy_detected - self.cy
        
        return e_x, e_y
    
    def pixel_to_angular_error(self, e_x, e_y):
        """
        Convert pixel error to angular error (radians)
        Using small angle approximation: θ ≈ tan(θ) = pixel_error / focal_length
        """
        if e_x is None or e_y is None:
            return None, None
        
        theta_x = np.arctan(e_x / self.fx)
        theta_y = np.arctan(e_y / self.fy)
        
        return theta_x, theta_y
    
    def compute_servo_correction(self, e_x, e_y, dt=0.033):
        """
        Compute servo angle corrections using PID control with improvements:
        1. Sequential control: Center pan first, then tilt
        2. Velocity limiting: Slow down when close to target
        3. Deadband: Don't move if error is very small
        
        dt: time step in seconds (default ~30fps)
        Returns: (delta_pan, delta_tilt) in degrees
        """
        # Convert to angular error
        theta_x, theta_y = self.pixel_to_angular_error(e_x, e_y)
        
        if theta_x is None:
            return 0, 0
        
        # Convert radians to degrees
        error_pan = np.degrees(theta_x)
        error_tilt = np.degrees(theta_y)
        
        # Deadband - don't move if error is tiny
        DEADBAND = 0.5  # degrees
        if abs(error_pan) < DEADBAND:
            error_pan = 0
        if abs(error_tilt) < DEADBAND:
            error_tilt = 0
        
        # === SEQUENTIAL CONTROL ===
        # Only control tilt if pan is already centered
        PAN_THRESHOLD = 2.0  # degrees - consider pan "centered" if error < this
        
        if abs(error_pan) > PAN_THRESHOLD:
            # Pan is not centered - only move pan, freeze tilt
            error_tilt = 0
            self.integral_tilt = 0  # Reset tilt integral
        
        # === PAN PID with Velocity Limiting ===
        # Proportional
        p_pan = self.kp_pan * error_pan
        
        # Integral (with anti-windup)
        self.integral_pan += error_pan * dt
        self.integral_pan = np.clip(self.integral_pan, -self.integral_limit, self.integral_limit)
        i_pan = self.ki_pan * self.integral_pan
        
        # Derivative
        d_pan = self.kd_pan * (error_pan - self.prev_error_pan) / dt
        self.prev_error_pan = error_pan
        
        delta_pan = -(p_pan + i_pan + d_pan)
        
        # Velocity limiting for pan (slow down when close)
        MAX_SPEED_PAN = 3.0  # degrees per step - balanced speed
        SLOW_ZONE_PAN = 15.0  # degrees - start slowing down earlier
        
        if abs(error_pan) < SLOW_ZONE_PAN:
            # Scale speed based on distance
            speed_scale = abs(error_pan) / SLOW_ZONE_PAN
            max_speed = MAX_SPEED_PAN * speed_scale
        else:
            max_speed = MAX_SPEED_PAN
        
        delta_pan = np.clip(delta_pan, -max_speed, max_speed)
        
        # === TILT PID with Velocity Limiting ===
        # Proportional
        p_tilt = self.kp_tilt * error_tilt
        
        # Integral (with anti-windup)
        self.integral_tilt += error_tilt * dt
        self.integral_tilt = np.clip(self.integral_tilt, -self.integral_limit, self.integral_limit)
        i_tilt = self.ki_tilt * self.integral_tilt
        
        # Derivative
        d_tilt = self.kd_tilt * (error_tilt - self.prev_error_tilt) / dt
        self.prev_error_tilt = error_tilt
        
        delta_tilt = -(p_tilt + i_tilt + d_tilt)
        
        # Velocity limiting for tilt
        MAX_SPEED_TILT = 3.0  # balanced speed
        SLOW_ZONE_TILT = 15.0
        
        if abs(error_tilt) < SLOW_ZONE_TILT:
            speed_scale = abs(error_tilt) / SLOW_ZONE_TILT
            max_speed = MAX_SPEED_TILT * speed_scale
        else:
            max_speed = MAX_SPEED_TILT
        
        delta_tilt = np.clip(delta_tilt, -max_speed, max_speed)
        
        return delta_pan, delta_tilt
    
    def is_centered(self, e_x, e_y):
        """Check if object is centered within threshold"""
        if e_x is None or e_y is None:
            return False
        
        error_magnitude = np.sqrt(e_x**2 + e_y**2)
        return error_magnitude < self.config.IBVS_CONVERGENCE_THRESHOLD

# ============================================================================
# MAIN PIPELINE
# ============================================================================

def main():
    config = Config()
    
    # Load Logitech intrinsics
    intrinsics = load_logitech_intrinsics()
    
    # Initialize IBVS controller
    ibvs = IBVSController(intrinsics, config)
    
    # Connect Arduino
    try:
        arduino = serial.Serial(config.ARDUINO_PORT, config.ARDUINO_BAUD, timeout=1.0)
        time.sleep(2)
        print("✅ Arduino connected")
    except Exception as e:
        print(f"⚠️ Arduino not found: {e}")
        print("Running in simulation mode (no servo control)")
        arduino = None
    
    # Find cameras
    insta_id = find_camera("Insta360")
    logi_id = find_camera("HD Pro Webcam")  # Logitech C920 identifies as "HD Pro Webcam C920"
    
    if insta_id == -1:
        print("❌ ERROR: Insta360 not found!")
        return
    if logi_id == -1:
        print("❌ ERROR: Logitech not found!")
        return
    
    print(f"📷 Insta360: /dev/video{insta_id}")
    print(f"📷 Logitech: /dev/video{logi_id}")
    
    # Open cameras
    cap_insta = cv2.VideoCapture(insta_id)
    cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    
    cap_logi = cv2.VideoCapture(logi_id)
    cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Load YOLO
    print(f"🔍 Loading YOLO model...")
    model = YOLO(config.YOLO_MODEL)
    
    # Initialize tracker
    tracker = SimpleTracker(max_disappeared=config.TRACK_LOST_THRESHOLD)
    
    # State machine
    state = "COARSE"  # COARSE (using Insta) or FINE (using Logitech IBVS)
    current_pan = 90
    current_tilt = 90
    ibvs_iterations = 0
    
    print("\n" + "="*60)
    print("🎯 IBVS PIPELINE STARTED")
    print("="*60)
    print("Stage 1: COARSE - Insta360 detects and points Logitech")
    print("Stage 2: FINE - Logitech IBVS centers object precisely")
    print("Press 'q' to quit")
    print("="*60 + "\n")
    
    while True:
        # Read frames
        ret_insta, frame_insta = cap_insta.read()
        ret_logi, frame_logi = cap_logi.read()
        
        if not ret_insta or not ret_logi:
            print("❌ Lost camera feed!")
            break
        
        # Resize Logitech to match Insta height for display
        frame_logi_display = cv2.resize(frame_logi, (640, 360))
        
        # ====================================================================
        # STAGE 1: COARSE DIRECTION (Insta360)
        # ====================================================================
        if state == "COARSE":
            results_insta = model(frame_insta, verbose=False, conf=config.YOLO_CONF)
            
            if results_insta and len(results_insta[0].boxes) > 0:
                # Get largest detection
                box = results_insta[0].boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                
                # Calculate servo angles using calibrated formulas
                pred_pan = calculate_pan(cx, cy)
                pred_tilt = calculate_tilt(cx, cy)
                
                # Clamp
                pred_pan = max(config.PAN_MIN, min(config.PAN_MAX, int(pred_pan)))
                pred_tilt = max(config.TILT_MIN, min(config.TILT_MAX, int(pred_tilt)))
                
                # Move servos
                if arduino:
                    inverted_tilt = 180 - pred_tilt  # Invert tilt
                    cmd = f"{inverted_tilt},{pred_pan}\n"
                    arduino.write(cmd.encode())
                
                current_pan = pred_pan
                current_tilt = pred_tilt
                
                # Draw on Insta frame
                cv2.rectangle(frame_insta, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame_insta, f"COARSE: P={pred_pan} T={pred_tilt}", 
                           (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Switch to FINE mode after a short delay
                time.sleep(0.5)  # Let servos move
                state = "FINE"
                ibvs_iterations = 0
                ibvs.reset()  # Reset PID state
                print(f"→ Switched to FINE mode (IBVS)")
        
        # ====================================================================
        # STAGE 2: FINE CENTERING (Logitech IBVS)
        # ====================================================================
        elif state == "FINE":
            results_logi = model(frame_logi, verbose=False, conf=config.YOLO_CONF)
            
            detection = None
            if results_logi and len(results_logi[0].boxes) > 0:
                box = results_logi[0].boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detection = (x1, y1, x2, y2)
            
            # Update tracker
            centroid, bbox = tracker.update(detection)
            
            if not tracker.is_tracking:
                # Lost object - switch back to COARSE
                print(f"⚠️ Object lost in Logitech view - switching to COARSE")
                state = "COARSE"
                continue
            
            # Compute pixel error
            e_x, e_y = ibvs.compute_error(centroid)
            
            # Check if centered
            if ibvs.is_centered(e_x, e_y):
                # Object is centered!
                cv2.putText(frame_logi_display, "✓ CENTERED", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Print success message once
                if ibvs_iterations > 0:
                    error_mag = np.sqrt(e_x**2 + e_y**2)
                    print(f"✓ CENTERED after {ibvs_iterations} iterations (error={error_mag:.1f}px)")
                    ibvs_iterations = 0  # Reset to prevent repeated messages
            else:
                # Compute servo corrections
                delta_pan, delta_tilt = ibvs.compute_servo_correction(e_x, e_y)
                
                # Apply corrections
                new_pan = current_pan + delta_pan
                new_tilt = current_tilt + delta_tilt
                
                # Clamp
                new_pan = max(config.PAN_MIN, min(config.PAN_MAX, int(new_pan)))
                new_tilt = max(config.TILT_MIN, min(config.TILT_MAX, int(new_tilt)))
                
                # Move servos
                if arduino:
                    inverted_tilt = 180 - new_tilt
                    cmd = f"{inverted_tilt},{new_pan}\n"
                    arduino.write(cmd.encode())
                    time.sleep(config.IBVS_SERVO_DELAY)  # Wait for servo to settle
                
                current_pan = new_pan
                current_tilt = new_tilt
                ibvs_iterations += 1
                
                # Draw error vector
                center_x, center_y = int(intrinsics['cx']), int(intrinsics['cy'])
                cv2.circle(frame_logi_display, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.arrowedLine(frame_logi_display, (center_x, center_y), 
                               (int(centroid[0]), int(centroid[1])), (255, 0, 0), 2)
                
                cv2.putText(frame_logi_display, f"IBVS: err=({e_x:.1f},{e_y:.1f}) iter={ibvs_iterations}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Console progress every 10 iterations
                if ibvs_iterations % 10 == 0:
                    error_mag = np.sqrt(e_x**2 + e_y**2)
                    print(f"  IBVS iter {ibvs_iterations}: error={error_mag:.1f}px")
            
            # Draw bounding box
            if bbox:
                x1, y1, x2, y2 = bbox
                cv2.rectangle(frame_logi_display, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            
            # Max iterations check
            if ibvs_iterations > config.IBVS_MAX_ITER:
                print(f"⚠️ IBVS max iterations reached - resetting to COARSE")
                state = "COARSE"
                ibvs_iterations = 0
        
        # ====================================================================
        # DISPLAY
        # ====================================================================
        # Combine views
        combined = np.hstack((frame_insta, frame_logi_display))
        
        # Add labels
        cv2.putText(combined, "INSTA360 (Coarse)", (10, 330), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(combined, "LOGITECH (IBVS)", (650, 330), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(combined, f"Mode: {state}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        cv2.imshow("IBVS Pipeline", combined)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Cleanup
    cap_insta.release()
    cap_logi.release()
    if arduino:
        arduino.close()
    cv2.destroyAllWindows()
    print("\n✅ Pipeline stopped")

if __name__ == "__main__":
    main()
