#!/usr/bin/env python3
"""
Integrated Visual Inspection Pipeline
======================================
Combines all components: 
- Insta360 detection with noise filtering
- Hemisphere mapping to pan/tilt angles
- Arduino servo control
- Logitech ROI capture

Usage:
    python3 integrated_pipeline.py --target fire_extinguisher
"""

import cv2
import numpy as np
import argparse
import time
import yaml
import serial
from pathlib import Path
from collections import deque
from ultralytics import YOLO


# Import detection confirmation class
import sys
sys.path.append(str(Path(__file__).parent))
from test_scripts.detection_confirmation import DetectionConfirmation


def load_config():
    """Load camera and Arduino configuration."""
    config_path = Path(__file__).parent / "config" / "camera_calibration.yaml"
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def pixel_to_ptz(detect_x, detect_y, frame_width, frame_height):
    """Convert Insta360 pixel coordinates to pan/tilt angles."""
    hemisphere_height = frame_height / 2
    is_front_hemisphere = (detect_y < hemisphere_height)
    
    # Normalize coordinates
    u_norm = detect_x / frame_width
    
    # Get local y coordinate within hemisphere
    if is_front_hemisphere:
        y_local = detect_y
    else:
        y_local = detect_y - hemisphere_height
    
    v_norm = y_local / hemisphere_height
    
    # Map to angles
    yaw_in_hemisphere = u_norm * 180 - 90
    pitch = 90 - v_norm * 180
    
    # Convert to global yaw
    if is_front_hemisphere:
        yaw = yaw_in_hemisphere
    else:
        yaw = yaw_in_hemisphere + 180
    
    # Normalize to -180 to +180
    if yaw > 180:
        yaw = yaw - 360
    
    # Clamp pitch
    pitch = np.clip(pitch, -90, 90)
    
    return yaw, pitch


class ArduinoController:
    """Arduino servo controller."""
    
    def __init__(self, port, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        
    def connect(self):
        """Connect to Arduino."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(2.0)  # Wait for Arduino reset
            print(f"[+] Arduino connected on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Arduino connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
    
    def move_to(self, tilt, pan):
        """Move both servos. Format: tilt,pan"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            cmd = f"{int(tilt)},{int(pan)}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            time.sleep(0.05)
            
            # Read response if available
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                print(f"[Arduino] {response}")
            
            return True
        except Exception as e:
            print(f"[ERROR] Servo move failed: {e}")
            return False
    
    def home(self):
        """Move to home position."""
        return self.move_to(90, 90)


class VisualInspectionPipeline:
    """Integrated visual inspection pipeline."""
    
    def __init__(self, config):
        self.config = config
        
        # Initialize cameras
        self.insta_cap = None
        self.logitech_cap = None
        
        # Initialize YOLO
        weight_path = Path(__file__).parent / "weights" / "yolo11n.pt"
        self.model = YOLO(str(weight_path))
        self.model.conf = 0.5
        
        # Detection confirmation tracker
        self.tracker = DetectionConfirmation(confirmation_frames=5, iou_threshold=0.5)
        
        # Arduino controller
        self.arduino = ArduinoController(config['arduino']['port'])
        
    def initialize(self):
        """Initialize all components."""
        print("\n" + "="*60)
        print("Visual Inspection System - Initialization")
        print("="*60)
        
        # Open Insta360
        insta_idx = self.config['insta360']['device_index']
        self.insta_cap = cv2.VideoCapture(insta_idx)
        if not self.insta_cap.isOpened():
            print("[ERROR] Failed to open Insta360 camera")
            return False
        print(f"[+] Insta360 opened: /dev/video{insta_idx}")
        
        # Open Logitech
        logitech_idx = self.config['logitech_c920']['device_index']
        self.logitech_cap = cv2.VideoCapture(logitech_idx)
        if not self.logitech_cap.isOpened():
            print("[ERROR] Failed to open Logitech camera")
            return False
        print(f"[+] Logitech opened: /dev/video{logitech_idx}")
        
        # Connect Arduino
        if not self.arduino.connect():
            return False
        
        # Move to home position
        self.arduino.home()
        time.sleep(1)
        
        print("="*60)
        print("[+] All systems ready!")
        print("="*60 + "\n")
        
        return True
    
    def detect_and_track_insta360(self, target_class):
        """
        Monitor Insta360 for target object with noise filtering.
        Returns confirmed detection or None.
        """
        print(f"\n[1] Scanning with Insta360 for '{target_class}'...")
        print("    Looking for stable detection across multiple frames...")
        
        frame_count = 0
        max_frames = 300  # 10 seconds at 30 FPS
        
        while frame_count < max_frames:
            ret, frame = self.insta_cap.read()
            if not ret:
                continue
            
            frame_count += 1
            
            # Run YOLO
            try:
                results = self.model(frame, verbose=False)
                if results is None or len(results) == 0:
                    continue
            except:
                continue
            
            # Extract detections
            detections = []
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = results[0].names[cls]
                
                if class_name == target_class:
                    detections.append((class_name, (x1, y1, x2, y2), conf))
            
            # Update tracker
            confirmed = self.tracker.update(detections)
            
            # Check for confirmed detection of target
            for det in confirmed:
                if det['class'] == target_class:
                    print(f"    [+] Target confirmed! Confidence: {det['confidence']:.2f}")
                    print(f"        Stability: {det['stability']*100:.0f}%")
                    return det
        
        print("    [!] No stable detection found")
        return None
    
    def move_to_target(self, detection):
        """Calculate angles and move servos to point at target."""
        print(f"\n[2] Moving servos to target...")
        
        # Get frame dimensions
        frame_width = self.config['insta360']['resolution']['width']
        frame_height = self.config['insta360']['resolution']['height']
        
        # Get detection center
        bbox = detection['bbox']
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2
        
        # Convert to pan/tilt
        yaw, pitch = pixel_to_ptz(center_x, center_y, frame_width, frame_height)
        
        print(f"    Detection at pixel ({int(center_x)}, {int(center_y)})")
        print(f"    Calculated angles: Pan={yaw:.1f}°, Tilt={pitch:.1f}°")
        
        # Convert to servo angles (mapping depends on mounting)
        # Assuming: yaw maps to pan, pitch maps to tilt
        pan_angle = 90 + yaw  # Center at 90, ±90 range
        tilt_angle = 90 - pitch  # Inverted tilt
        
        # Enforce limits
        pan_angle = np.clip(pan_angle, 0, 180)
        tilt_angle = np.clip(tilt_angle, 20, 160)
        
        print(f"    Servo command: Pan={pan_angle:.0f}° Tilt={tilt_angle:.0f}°")
        
        # Move servos
        self.arduino.move_to(tilt_angle, pan_angle)
        time.sleep(2)  # Wait for servos to reach position
        
        print(f"    [+] Servos moved!")
        
    def capture_roi(self):
        """Capture ROI with Logitech camera."""
        print(f"\n[3] Capturing ROI with Logitech...")
        
        # Capture frame
        ret, frame = self.logitech_cap.read()
        if not ret:
            print("    [ERROR] Failed to capture frame")
            return None
        
        # Save image
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"roi_capture_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        
        print(f"    [+] ROI saved: {filename}")
        
        return frame
    
    def inspect_object(self, target_class):
        """
        Full inspection workflow.
        1. Detect in Insta360
        2. Move servos
        3. Capture Logitech ROI
        """
        print("\n" + "="*60)
        print(f"INSPECTION: {target_class}")
        print("="*60)
        
        # Step 1: Detect with Insta360
        detection = self.detect_and_track_insta360(target_class)
        if detection is None:
            print("\n[RESULT] Target not found")
            return False
        
        # Step 2: Move servos
        self.move_to_target(detection)
        
        # Step 3: Capture ROI
        roi_frame = self.capture_roi()
        
        # Return to home
        print(f"\n[4] Returning to home position...")
        self.arduino.home()
        time.sleep(1)
        
        print("\n[RESULT] Inspection complete!")
        print("="*60 + "\n")
        
        return True
    
    def cleanup(self):
        """Clean up resources."""
        if self.insta_cap:
            self.insta_cap.release()
        if self.logitech_cap:
            self.logitech_cap.release()
        if self.arduino:
            self.arduino.disconnect()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='Visual inspection pipeline')
    parser.add_argument('--target', type=str, default='fire_extinguisher',
                        help='Target object class to inspect')
    args = parser.parse_args()
    
    # Load config
    config = load_config()
    
    # Create pipeline
    pipeline = VisualInspectionPipeline(config)
    
    try:
        # Initialize
        if not pipeline.initialize():
            print("[ERROR] Initialization failed")
            return
        
        # Run inspection
        pipeline.inspect_object(args.target)
        
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    
    finally:
        pipeline.cleanup()
        print("[+] System shutdown complete")


if __name__ == "__main__":
    main()
