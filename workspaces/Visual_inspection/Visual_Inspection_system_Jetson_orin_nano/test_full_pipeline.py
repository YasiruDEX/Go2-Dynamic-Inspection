#!/usr/bin/env python3
"""
Full Pipeline Test - Insta360 + Arduino Pan/Tilt
================================================
Tests the complete workflow with available hardware:
- Insta360 for detection
- Arduino pan/tilt control
- Manual ROI capture (no Logitech yet)

Usage:
    python3 test_full_pipeline.py --target fire_extinguisher
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
    """Multi-frame detection tracker."""
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


def pixel_to_ptz(detect_x, detect_y, frame_width, frame_height):
    """Convert pixel to pan/tilt angles."""
    # For now, simple proportional mapping
    # Center of frame = (90, 90)
    # This is simplified - you'll calibrate later
    
    center_x = frame_width / 2
    center_y = frame_height / 2
    
    # Calculate offset from center
    offset_x = (detect_x - center_x) / center_x  # -1 to 1
    offset_y = (detect_y - center_y) / center_y  # -1 to 1
    
    # Map to servo angles
    pan = 90 + (offset_x * 90)  # 0-180
    tilt = 90 - (offset_y * 70)  # 20-160
    
    # Clamp
    pan = np.clip(pan, 0, 180)
    tilt = np.clip(tilt, 20, 160)
    
    return pan, tilt


class FullPipelineTest:
    """Full pipeline with Insta360 + Arduino."""
    
    def __init__(self, config, target_class):
        self.config = config
        self.target_class = target_class
        
        # Camera
        self.camera = None
        
        # YOLO
        weight_path = Path(__file__).parent / "weights" / "yolo11n.pt"
        self.model = YOLO(str(weight_path))
        self.model.conf = 0.5
        
        # Tracker
        self.tracker = DetectionTracker(frames=5)
        
        # Arduino
        self.arduino = None
        try:
            port = config['arduino']['port']
            self.arduino = serial.Serial(port, 9600, timeout=1.0)
            time.sleep(2)
            print(f"[+] Arduino connected: {port}")
        except Exception as e:
            print(f"[WARNING] Arduino not connected: {e}")
            print("[WARNING] Servo control disabled")
        
        self.state = "SCANNING"
        
    def initialize(self):
        """Open Insta360 camera."""
        insta_idx = self.config['insta360']['device_index']
        self.camera = cv2.VideoCapture(insta_idx)
        
        if not self.camera.isOpened():
            print(f"[ERROR] Failed to open Insta360 at /dev/video{insta_idx}")
            return False
        
        print(f"[+] Insta360 opened at /dev/video{insta_idx}")
        return True
    
    def move_servos(self, tilt, pan):
        """Send command to Arduino."""
        if not self.arduino:
            print(f"  [DEMO] Would move to: Tilt={tilt:.0f}° Pan={pan:.0f}°")
            return
        
        try:
            cmd = f"{int(tilt)},{int(pan)}\n"
            self.arduino.write(cmd.encode('utf-8'))
            time.sleep(0.1)
            # Read response
            response = self.arduino.readline().decode('utf-8').strip()
            if response:
                print(f"  [Arduino] {response}")
        except Exception as e:
            print(f"  [ERROR] Servo command failed: {e}")
    
    def run(self):
        """Main loop."""
        if not self.initialize():
            return
        
        print("\n" + "="*70)
        print("FULL PIPELINE TEST")
        print("="*70)
        print(f"Target: {self.target_class}")
        print("Hardware: Insta360 + Arduino")
        print("\nControls:")
        print("  'q' = Quit")
        print("  SPACE = Capture image when target locked")
        print("  'h' = Return servos to home (90, 90)")
        print("="*70)
        
        frame_count = 0
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("[ERROR] Failed to read frame")
                break
            
            frame_count += 1
            display = frame.copy()
            h, w = frame.shape[:2]
            
            # Run YOLO
            target_found = False
            target_info = None
            
            try:
                results = self.model(frame, verbose=False)
                if results and len(results) > 0 and results[0].boxes is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        class_name = results[0].names[cls]
                        
                        # Draw all detections
                        color = (128, 128, 128)
                        cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                        cv2.putText(display, f"{class_name} {conf:.2f}", 
                                   (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                        
                        # Check for target
                        if class_name == self.target_class:
                            target_found = True
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            
                            # Highlight
                            cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), 
                                         (0, 255, 0), 3)
                            cv2.circle(display, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)
                            
                            # Calculate servo angles
                            pan, tilt = pixel_to_ptz(center_x, center_y, w, h)
                            
                            target_info = {
                                'center': (center_x, center_y),
                                'conf': conf,
                                'pan': pan,
                                'tilt': tilt
                            }
                            
                            # Print to terminal
                            print(f"  [{self.target_class}] Pixel=({int(center_x)},{int(center_y)}) | "
                                  f"Servo: Pan={pan:.0f}° Tilt={tilt:.0f}° | Conf={conf:.2f}")
                            
                            # Draw servo info
                            info_text = f"Pan={pan:.0f} Tilt={tilt:.0f}"
                            cv2.putText(display, info_text, (int(x1), int(y2)+20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            except Exception as e:
                print(f"[WARNING] YOLO error: {e}")
            
            # Update tracker
            confirmed = self.tracker.update(target_found)
            
            # Add status overlay
            status_color = (0, 255, 0) if confirmed else (0, 165, 255)
            status_text = f"TARGET: {'CONFIRMED' if confirmed else 'SEARCHING'}"
            cv2.putText(display, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            
            tracker_text = f"Tracker: {sum(self.tracker.history)}/5 frames"
            cv2.putText(display, tracker_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Draw crosshair at center
            cv2.line(display, (w//2-20, h//2), (w//2+20, h//2), (255, 0, 0), 2)
            cv2.line(display, (w//2, h//2-20), (w//2, h//2+20), (255, 0, 0), 2)
            
            # If confirmed, prepare to move
            if confirmed and target_info and self.state == "SCANNING":
                self.state = "LOCKED"
                print("\n" + "─" * 70)
                print(f"✓ TARGET LOCKED: {self.target_class.upper()}")
                print(f"  Confidence: {target_info['conf']:.2f}")
                print(f"  Position: ({int(target_info['center'][0])}, {int(target_info['center'][1])})")
                print(f"  → MOVING SERVOS: Pan={target_info['pan']:.0f}° Tilt={target_info['tilt']:.0f}°")
                print("─" * 70)
                self.move_servos(target_info['tilt'], target_info['pan'])
                print("\n[READY] Press SPACE to capture, 'h' for home, or let it scan again")
            
            if self.state == "LOCKED":
                cv2.putText(display, "Press SPACE to capture, 'h' for home", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow("Full Pipeline Test - Insta360", display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' ') and self.state == "LOCKED":
                # Capture
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"capture_{self.target_class}_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"\n✓ IMAGE CAPTURED: {filename}")
                print(f"  → Returning to home position...")
                self.move_servos(90, 90)
                time.sleep(1)
                self.state = "SCANNING"
                self.tracker.reset()
                print(f"\n[SCANNING] Ready for next detection...")
                print("─" * 70)
            elif key == ord('h'):
                # Home
                print(f"\n[HOME] Returning to home position (90, 90)...")
                self.move_servos(90, 90)
                self.state = "SCANNING"
                self.tracker.reset()
        
        # Cleanup
        self.camera.release()
        if self.arduino:
            print("\n[+] Returning to home position...")
            self.move_servos(90, 90)
            self.arduino.close()
        cv2.destroyAllWindows()
        
        print("\n[+] Pipeline test complete")


def main():
    parser = argparse.ArgumentParser(description='Full pipeline test')
    parser.add_argument('--target', type=str, default='fire_extinguisher',
                        help='Target object class')
    args = parser.parse_args()
    
    config = load_config()
    pipeline = FullPipelineTest(config, args.target)
    
    try:
        pipeline.run()
    except KeyboardInterrupt:
        print("\n[!] Stopped by user")


if __name__ == "__main__":
    main()
