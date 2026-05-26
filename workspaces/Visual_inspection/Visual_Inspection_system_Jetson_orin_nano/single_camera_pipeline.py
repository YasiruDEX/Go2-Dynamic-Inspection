#!/usr/bin/env python3
"""
Single Camera Visual Pipeline - LOGITECH ONLY
==============================================
Simplified version using only Logitech C920 for testing.

Usage:
    python3 single_camera_pipeline.py --target fire_extinguisher
"""

import cv2
import numpy as np
import argparse
import time
import yaml
import serial
from pathlib import Path
from ultralytics import YOLO
from collections import deque


class DetectionTracker:
    """Simple multi-frame detection tracker."""
    def __init__(self, frames=5):
        self.frames = frames
        self.history = deque(maxlen=frames)
    
    def update(self, detected):
        self.history.append(detected)
        if len(self.history) < 3:
            return False
        return sum(self.history) >= 3
    
    def reset(self):
        self.history.clear()


def load_config():
    config_path = Path(__file__).parent / "config" / "camera_calibration.yaml"
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class SingleCameraPipeline:
    """Visual inspection with Logitech camera only."""
    
    def __init__(self, config, target_class):
        self.config = config
        self.target_class = target_class
        
        # Camera (Logitech only)
        self.camera = None
        
        # YOLO
        weight_path = Path(__file__).parent / "weights" / "yolo11n.pt"
        self.model = YOLO(str(weight_path))
        self.model.conf = 0.5
        
        # Tracker
        self.tracker = DetectionTracker(frames=5)
        
        # Arduino (optional)
        self.arduino = None
        try:
            port = config['arduino']['port']
            self.arduino = serial.Serial(port, 9600, timeout=1.0)
            time.sleep(2)
            print(f"[+] Arduino connected: {port}")
        except:
            print("[WARNING] Arduino not connected - servo control disabled")
        
        self.state = "SCANNING"
        
    def initialize(self):
        """Open Logitech camera."""
        # Try /dev/video0 first
        self.camera = cv2.VideoCapture(0)
        
        if not self.camera.isOpened():
            print("[ERROR] Failed to open camera at /dev/video0")
            return False
        
        print(f"[+] Logitech camera opened at /dev/video0")
        return True
    
    def run(self):
        """Main loop with single camera display."""
        if not self.initialize():
            return
        
        print(f"\n[SCANNING] Looking for {self.target_class}...")
        print("─" * 70)
        print("Single camera mode - Logitech C920 only")
        print("Press 'q' to quit, SPACE to capture")
        print("─" * 70)
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break
            
            display_frame = frame.copy()
            
            # Run YOLO
            try:
                results = self.model(frame, verbose=False)
                if results is None or len(results) == 0 or results[0].boxes is None:
                    cv2.imshow("Single Camera Detection", display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
            except Exception as e:
                print(f"[WARNING] YOLO error: {e}")
                continue
            
            # Process detections
            target_found = False
            target_info = None
            
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = results[0].names[cls]
                
                # Draw all detections
                color = (128, 128, 128)
                cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(display_frame, f"{class_name} {conf:.2f}", 
                           (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
                # Check for target
                if class_name == self.target_class:
                    target_found = True
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Highlight target
                    cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                                 (0, 255, 0), 3)
                    cv2.circle(display_frame, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)
                    
                    target_info = {
                        'bbox': (x1, y1, x2, y2),
                        'center': (center_x, center_y),
                        'conf': conf
                    }
                    
                    print(f"  [{self.target_class}] Detected at ({int(center_x)}, {int(center_y)}) | Conf={conf:.2f}")
            
            # Update tracker
            confirmed = self.tracker.update(target_found)
            
            # Add status
            status_color = (0, 255, 0) if confirmed else (0, 165, 255)
            status_text = f"TARGET: {'CONFIRMED' if confirmed else 'SEARCHING'}"
            cv2.putText(display_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            
            tracker_text = f"Tracker: {sum(self.tracker.history)}/5 frames"
            cv2.putText(display_frame, tracker_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # If confirmed
            if confirmed and target_info:
                cv2.putText(display_frame, "Press SPACE to capture", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if self.state == "SCANNING":
                    print(f"\n✓ TARGET LOCKED: {self.target_class.upper()}")
                    print(f"  Confidence: {target_info['conf']:.2f}")
                    print(f"  Press SPACE to capture")
                    self.state = "READY"
            
            cv2.imshow("Single Camera Detection", display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' ') and self.state == "READY":
                # Capture
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"capture_{self.target_class}_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"\n✓ IMAGE CAPTURED: {filename}")
                print(f"\n[SCANNING] Ready for next detection...")
                print("─" * 70)
                
                self.state = "SCANNING"
                self.tracker.reset()
        
        # Cleanup
        self.camera.release()
        if self.arduino:
            self.arduino.close()
        cv2.destroyAllWindows()
        
        print("\n[+] Pipeline stopped")


def main():
    parser = argparse.ArgumentParser(description='Single camera detection (Logitech only)')
    parser.add_argument('--target', type=str, default='fire_extinguisher',
                        help='Target object class')
    args = parser.parse_args()
    
    config = load_config()
    pipeline = SingleCameraPipeline(config, args.target)
    
    print("\n" + "="*70)
    print("SINGLE CAMERA DETECTION - LOGITECH C920 ONLY")
    print("="*70)
    print(f"Target: {args.target}")
    print("NOTE: This is simplified mode without Insta360")
    print("="*70)
    
    try:
        pipeline.run()
    except KeyboardInterrupt:
        print("\n[!] Stopped by user")


if __name__ == "__main__":
    main()
