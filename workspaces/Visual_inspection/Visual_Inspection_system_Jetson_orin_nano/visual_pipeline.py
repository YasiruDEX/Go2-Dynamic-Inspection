#!/usr/bin/env python3
"""
Visual Inspection Pipeline - WITH LIVE VIDEO DISPLAY
====================================================
Shows both camera feeds so you can see what's happening.

Usage:
    python3 visual_pipeline.py --target fire_extinguisher
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
        """Update with True/False if target detected."""
        self.history.append(detected)
        
        # Require detection in at least 3 out of last 5 frames
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
    """
    Convert pixel to pan/tilt angles.
    Front Hemisphere (Top Half) -> Pan 0-180
    Back Hemisphere (Bottom Half) -> Pan 180-360
    """
    hemisphere_height = frame_height / 2
    is_front = (detect_y < hemisphere_height)
    
    # Normalized coordinates (0 to 1)
    u_norm = detect_x / frame_width
    
    # Calculate Pan
    if is_front:
        # Front: 0 to 180
        pan_angle = u_norm * 180
    else:
        # Back: 180 to 360
        pan_angle = 180 + (u_norm * 180)
        
    # Calculate Tilt (Pitch)
    # Map Y within hemisphere to 0-90 (approx)
    # This needs calibration, but for now simple linear map
    y_local = detect_y if is_front else detect_y - hemisphere_height
    v_norm = y_local / hemisphere_height
    
    # Simple mapping: Top of hemisphere is 90 deg up, Bottom is 90 deg down?
    # Or matches servo 20-160?
    # Let's keep it abstract -90 to 90 for now, pipeline handles clamping
    # Assuming detection Y=0 is top (90 deg), Y=h/2 is horizon (0 deg)
    tilt_angle = 90 - (v_norm * 180)
    
    return pan_angle, tilt_angle


class VisualPipeline:
    """Visual inspection pipeline with live display."""
    
    def __init__(self, config, target_class):
        self.config = config
        self.target_class = target_class
        
        # Cameras
        self.insta_cap = None
        self.logitech_cap = None
        
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
        except:
            print("[WARNING] Arduino not connected - servo control disabled")
        
        # State
        self.state = "SCANNING"  # SCANNING, MOVING, CAPTURING
        self.target_detection = None
        
    def initialize(self):
        """Open cameras."""
        insta_idx = self.config['insta360']['device_index']
        logitech_idx = self.config['logitech_c920']['device_index']
        
        self.insta_cap = cv2.VideoCapture(insta_idx)
        self.logitech_cap = cv2.VideoCapture(logitech_idx)
        
        if not self.insta_cap.isOpened() or not self.logitech_cap.isOpened():
            print("[ERROR] Failed to open cameras")
            return False
        
        print(f"[+] Cameras opened")
        return True
    
    def move_servos(self, tilt, pan):
        """Send command to Arduino."""
        if not self.arduino:
            print(f"[DEMO] Would move to: Tilt={tilt:.0f}° Pan={pan:.0f}°")
            return
        
        try:
            cmd = f"{int(tilt)},{int(pan)}\n"
            self.arduino.write(cmd.encode('utf-8'))
            time.sleep(0.05)
        except Exception as e:
            print(f"[ERROR] Servo command failed: {e}")
    
    def process_insta360_frame(self, frame):
        """
        Process Insta360 frame: detect target, draw annotations.
        Returns: (annotated_frame, target_detected, detection_info)
        """
        h, w = frame.shape[:2]
        
        # Run YOLO - exact same as test_scripts/07
        try:
            results = self.model.predict(frame, verbose=False, conf=0.5)
        except Exception as e:
            print(f"[WARNING] YOLO error: {e}")
            return frame, False, None
        
        # Check if we got valid results
        if results is None or len(results) == 0:
            return frame, False, None
        
        # Extract boxes - check if they exist
        try:
            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                return frame, False, None
        except:
            return frame, False, None
        
        # Find target
        target_found = False
        target_info = None
        
        # Process each detection
        for box in boxes:
            try:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                class_name = results[0].names[cls]
            except:
                continue
            
            # Draw all detections in gray
            color = (128, 128, 128)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", 
                       (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Check if any of our 3 classes
            if class_name in ['fire_extinguisher', 'door', 'gauge']:
                target_found = True
                self.target_class = class_name  # Update to detected class
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Highlight target in green
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                             (0, 255, 0), 3)
                cv2.circle(frame, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)
                
                # Calculate angles
                # pixel_to_ptz now returns absolute pan (0-360) and pitch (-90 to 90)
                pan_abs, pitch = pixel_to_ptz(center_x, center_y, w, h)
                
                # Pan is already 0-360. 
                # If current servo is only 0-180, we might need to clamp or warn
                # User said: "from now we have 0-180 use that for front... I will replace servo for back"
                pan_angle = pan_abs # 0-360
                
                # Tilt calculation: 90 is horizon? 
                # If pitch is 90 (up) -> Servo 0?
                # If pitch is -90 (down) -> Servo 180?
                tilt_angle = 90 - pitch
                
                # Clamp for safety (current hardware limits)
                # Pan 0-180 (Front) or 180-360 (Back)
                # NOTE: Current physical servo only does 0-180. 
                # If back detected, we can't move there yet physically.
                pan_angle = np.clip(pan_angle, 0, 360) 
                tilt_angle = np.clip(tilt_angle, 20, 160)
                
                # Print to terminal
                hemisphere = "FRONT" if center_y < h/2 else "BACK"
                print(f"  [{class_name}] Pixel=({int(center_x)},{int(center_y)}) {hemisphere} | "
                      f"Servo: Pan={pan_angle:.0f}° Tilt={tilt_angle:.0f}° | Conf={conf:.2f}")
                
                target_info = {
                    'bbox': (x1, y1, x2, y2),
                    'center': (center_x, center_y),
                    'conf': conf,
                    'pan': pan_angle,
                    'tilt': tilt_angle
                }
                
                # Draw angle info
                info_text = f"Pan={pan_angle:.0f} Tilt={tilt_angle:.0f}"
                cv2.putText(frame, info_text, (int(x1), int(y2)+20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw hemisphere separator
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 255), 2)
        cv2.putText(frame, "FRONT 180°", (10, h//2 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, "BACK 180°", (10, h//2 + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return frame, target_found, target_info
    
    def run(self):
        """Main loop with visual display."""
        if not self.initialize():
            return
        
        print(f"\n[SCANNING] Detecting: fire_extinguisher, door, gauge...")
        print("─" * 70)
        
        frame_count = 0
        last_print_time = time.time()
        
        while True:
            # Read frames
            ret_insta, frame_insta = self.insta_cap.read()
            ret_logitech, frame_logitech = self.logitech_cap.read()
            
            if not ret_insta or not ret_logitech:
                break
            
            frame_count += 1
            
            # Process Insta360 (detection + tracking)
            if self.state == "SCANNING":
                annotated_insta, target_found, target_info = self.process_insta360_frame(frame_insta.copy())
                
                # Update tracker
                confirmed = self.tracker.update(target_found)
                
                # Add status overlay
                status_color = (0, 255, 0) if confirmed else (0, 165, 255)
                status_text = f"TARGET: {'CONFIRMED' if confirmed else 'SEARCHING'}"
                cv2.putText(annotated_insta, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
                
                tracker_text = f"Tracker: {sum(self.tracker.history)}/5 frames"
                cv2.putText(annotated_insta, tracker_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # If confirmed, move servos
                if confirmed and target_info:
                    self.state = "MOVING"
                    self.target_detection = target_info
                    print("\n" + "─" * 70)
                    print(f"✓ TARGET LOCKED: {self.target_class.upper()}")
                    print(f"  Confidence: {target_info['conf']:.2f}")
                    print(f"  Position: ({int(target_info['center'][0])}, {int(target_info['center'][1])})")
                    print(f"  → MOVING SERVOS: Pan={target_info['pan']:.0f}° Tilt={target_info['tilt']:.0f}°")
                    print("─" * 70)
                    self.move_servos(target_info['tilt'], target_info['pan'])
                    time.sleep(2)  # Wait for servos
                    self.state = "CAPTURING"
                    print("\n[READY] Press SPACE to capture ROI...")

                
            elif self.state == "MOVING":
                # Just show status
                annotated_insta = frame_insta.copy()
                cv2.putText(annotated_insta, "MOVING SERVOS...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
            elif self.state == "CAPTURING":
                # Run YOLO on Logitech to verify object in view
                annotated_insta = frame_insta.copy()
                
                try:
                    # Detect on Logitech camera
                    logitech_results = self.model.predict(frame_logitech, verbose=False, conf=0.5)
                    
                    if logitech_results and len(logitech_results) > 0:
                        logitech_boxes = logitech_results[0].boxes
                        
                        if logitech_boxes is not None and len(logitech_boxes) > 0:
                            # Check if target class is present
                            target_in_view = False
                            for box in logitech_boxes:
                                try:
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    cls = int(box.cls[0].cpu().numpy())
                                    conf = float(box.conf[0].cpu().numpy())
                                    class_name = logitech_results[0].names[cls]
                                    
                                    # Draw detection on Logitech
                                    if class_name == self.target_class:
                                        target_in_view = True
                                        cv2.rectangle(frame_logitech, (int(x1), int(y1)), (int(x2), int(y2)), 
                                                     (0, 255, 0), 3)
                                        cv2.putText(frame_logitech, f"{class_name} {conf:.2f}", 
                                                   (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                except:
                                    continue
                            
                            if target_in_view:
                                cv2.putText(annotated_insta, f"VERIFIED: {self.target_class.upper()} in Logitech view", (10, 30),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            else:
                                cv2.putText(annotated_insta, f"Searching for {self.target_class} in Logitech...", (10, 30),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                except Exception as e:
                    print(f"[WARNING] Logitech YOLO error: {e}")
                
                # Show capture instructions
                cv2.putText(annotated_insta, "ROI READY - Press SPACE to capture", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Highlight Logitech view
                cv2.rectangle(frame_logitech, (5, 5), 
                             (frame_logitech.shape[1]-5, frame_logitech.shape[0]-5),
                             (0, 255, 0), 3)
                cv2.putText(frame_logitech, "PRESS SPACE TO CAPTURE", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                annotated_insta = frame_insta
            
            # Resize for display
            h_insta = annotated_insta.shape[0]
            w_insta = annotated_insta.shape[1]
            target_h = 400
            scale = target_h / h_insta
            
            insta_display = cv2.resize(annotated_insta, (int(w_insta*scale), target_h))
            logitech_display = cv2.resize(frame_logitech, 
                                         (int(frame_logitech.shape[1]*target_h/frame_logitech.shape[0]), target_h))
            
            # Combine side by side
            combined = cv2.hconcat([insta_display, logitech_display])
            
            # Add labels
            cv2.putText(combined, "Insta360 (Scanning)", (10, target_h-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(combined, "Logitech (ROI)", (insta_display.shape[1]+10, target_h-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("Visual Inspection Pipeline", combined)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' ') and self.state == "CAPTURING":
                # Save ROI to data/captured_rois/
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_dir = Path("data/captured_rois")
                save_dir.mkdir(parents=True, exist_ok=True)
                
                filename = save_dir / f"roi_{self.target_class}_{timestamp}.jpg"
                cv2.imwrite(str(filename), frame_logitech)
                print(f"\n✓ ROI CAPTURED: {filename}")
                
                # Return home and reset
                print(f"  → Returning to home position (90°, 90°)...")
                self.move_servos(90, 90)
                time.sleep(1)
                
                self.state = "SCANNING"
                self.tracker.reset()
                print(f"\n[SCANNING] Ready for next detection...")
                print("─" * 70)

        
        # Cleanup
        self.insta_cap.release()
        self.logitech_cap.release()
        if self.arduino:
            # Return home
            self.move_servos(90, 90)
            self.arduino.close()
        cv2.destroyAllWindows()
        
        print("\n[+] Pipeline stopped")


def main():
    parser = argparse.ArgumentParser(description='Visual inspection with live display')
    parser.add_argument('--target', type=str, default='any',
                        help='Target object class (default: fire_extinguisher)')
    args = parser.parse_args()
    
    config = load_config()
    pipeline = VisualPipeline(config, args.target)
    
    print("\n" + "="*70)
    print("VISUAL INSPECTION PIPELINE")
    print("="*70)
    print(f"Target: {args.target}")
    print("Controls: 'q'=quit, SPACE=capture ROI")
    print("="*70)
    
    try:
        pipeline.run()
    except KeyboardInterrupt:
        print("\n[!] Stopped by user")


if __name__ == "__main__":
    main()
