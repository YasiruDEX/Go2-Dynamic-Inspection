#!/usr/bin/env python3
"""
Multi-Frame Detection Confirmation
===================================
Filters noisy YOLO detections by requiring consistent detections
across multiple consecutive frames before accepting.

This prevents jittery pan-tilt movement from false positives.
"""

import cv2
import numpy as np
from collections import deque


class DetectionConfirmation:
    """
    Confirms detections across multiple frames.
    Only accepts detections that appear consistently.
    """
    
    def __init__(self, confirmation_frames=5, iou_threshold=0.5):
        """
        Args:
            confirmation_frames: Number of consecutive frames needed
            iou_threshold: Minimum IoU to consider same object
        """
        self.confirmation_frames = confirmation_frames
        self.iou_threshold = iou_threshold
        
        # Track detections per class
        self.detection_history = {}  # class_name -> deque of bboxes
        
    def calculate_iou(self, box1, box2):
        """Calculate Intersection over Union between two boxes."""
        x1_min, y1_min, x1_max, y1_max = box1
        x2_min, y2_min, x2_max, y2_max = box2
        
        # Intersection area
        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)
        
        if inter_x_max < inter_x_min or inter_y_max < inter_y_min:
            return 0.0
        
        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
        
        # Union area
        box1_area = (x1_max - x1_min) * (y1_max - y1_min)
        box2_area = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = box1_area + box2_area - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0
    
    def calculate_centroid_distance(self, box1, box2):
        """Calculate distance between box centroids (normalized)."""
        x1_min, y1_min, x1_max, y1_max = box1
        x2_min, y2_min, x2_max, y2_max = box2
        
        # Calculate centroids
        c1_x = (x1_min + x1_max) / 2
        c1_y = (y1_min + y1_max) / 2
        c2_x = (x2_min + x2_max) / 2
        c2_y = (y2_min + y2_max) / 2
        
        # Euclidean distance
        distance = np.sqrt((c1_x - c2_x)**2 + (c1_y - c2_y)**2)
        
        # Normalize by box size
        box1_size = max((x1_max - x1_min), (y1_max - y1_min))
        
        return distance / box1_size if box1_size > 0 else float('inf')
    
    def boxes_match(self, box1, box2):
        """
        Check if two boxes represent the same object.
        Uses BOTH IoU and centroid distance for robustness.
        """
        iou = self.calculate_iou(box1, box2)
        centroid_dist = self.calculate_centroid_distance(box1, box2)
        
        # Match if either:
        # 1. Good IoU overlap (object mostly same position)
        # 2. Close centroids (object moved slightly but nearby)
        return iou >= self.iou_threshold or centroid_dist < 0.5
    
    def update(self, detections):
        """
        Update detection history with new frame detections.
        
        Args:
            detections: List of (class_name, bbox, confidence)
                       where bbox = (x1, y1, x2, y2)
        
        Returns:
            List of confirmed detections after multi-frame check
        """
        # Initialize history for new classes
        detected_classes = set([d[0] for d in detections])
        for cls in detected_classes:
            if cls not in self.detection_history:
                self.detection_history[cls] = deque(maxlen=self.confirmation_frames)
        
        # Update history with current detections
        current_frame_detections = {}
        for class_name, bbox, conf in detections:
            if class_name not in current_frame_detections:
                current_frame_detections[class_name] = []
            current_frame_detections[class_name].append((bbox, conf))
        
        # Add current frame to history
        for cls in detected_classes:
            if cls in current_frame_detections:
                self.detection_history[cls].append(current_frame_detections[cls])
            else:
                # No detection for this class in current frame
                self.detection_history[cls].append([])
        
        # Check for confirmed detections
        confirmed = []
        
        for class_name, history in self.detection_history.items():
            # Need full history buffer
            if len(history) < self.confirmation_frames:
                continue
            
            # Check if detection is consistent across frames
            if not history[0]:
                continue
            
            for first_bbox, first_conf in history[0]:
                # Track this detection across frames
                consistent_count = 1
                matching_bboxes = [first_bbox]
                confidences = [first_conf]
                
                # Check subsequent frames
                for frame_dets in list(history)[1:]:
                    # Find best matching detection in this frame
                    best_match_score = 0.0
                    best_match = None
                    best_conf = 0.0
                    
                    for bbox, conf in frame_dets:
                        # Use combined matching criteria
                        if self.boxes_match(first_bbox, bbox):
                            iou = self.calculate_iou(first_bbox, bbox)
                            if iou > best_match_score:
                                best_match_score = iou
                                best_match = bbox
                                best_conf = conf
                    
                    # If we found a match
                    if best_match is not None:
                        consistent_count += 1
                        matching_bboxes.append(best_match)
                        confidences.append(best_conf)
                
                # Require detection in at least 3 out of 5 frames (more flexible)
                min_required = max(3, self.confirmation_frames - 2)
                
                if consistent_count >= min_required:
                    # Average the bounding boxes and confidence
                    avg_bbox = np.mean(matching_bboxes, axis=0)
                    avg_conf = np.mean(confidences)
                    
                    confirmed.append({
                        'class': class_name,
                        'bbox': avg_bbox,
                        'confidence': avg_conf,
                        'stability': consistent_count / self.confirmation_frames,
                        'frames_detected': consistent_count
                    })
        
        return confirmed
    
    def reset(self):
        """Reset detection history."""
        self.detection_history.clear()


# Example usage
if __name__ == "__main__":
    import yaml
    from pathlib import Path
    from ultralytics import YOLO
    
    # Load config
    config_path = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Load model
    weight_path = Path(__file__).parent.parent / "weights" / "yolo11n.pt"
    model = YOLO(str(weight_path))
    model.conf = 0.5
    
    # Create confirmation tracker
    tracker = DetectionConfirmation(
        confirmation_frames=5,  # Require 5 consecutive frames
        iou_threshold=0.5
    )
    
    # Open camera (Insta360)
    insta_idx = config['insta360']['device_index']
    cap = cv2.VideoCapture(insta_idx)
    
    print("\n" + "="*60)
    print("Multi-Frame Detection Confirmation Test")
    print("="*60)
    print("Confirmation frames required: 5")
    print("Only stable detections will be highlighted")
    print("\nPress 'q' to quit")
    print("="*60 + "\n")
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame from camera")
            break
        
        frame_count += 1
        
        # Run YOLO
        try:
            results = model(frame, verbose=False)
            
            # Check if results are valid
            if results is None or len(results) == 0 or results[0].boxes is None:
                print(f"[WARNING] Frame {frame_count}: No valid YOLO results")
                continue
                
        except Exception as e:
            print(f"[ERROR] YOLO inference failed: {e}")
            continue
        
        # Extract detections
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = results[0].names[cls]
            
            detections.append((class_name, (x1, y1, x2, y2), conf))
        
        # Update tracker and get confirmed detections
        confirmed = tracker.update(detections)
        
        # Draw all detections in gray (unconfirmed)
        for class_name, bbox, conf in detections:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                         (128, 128, 128), 1)
            cv2.putText(frame, f"{class_name} {conf:.2f}", 
                       (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.4, (128, 128, 128), 1)
        
        # Draw confirmed detections in green (stable)
        for det in confirmed:
            x1, y1, x2, y2 = det['bbox']
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                         (0, 255, 0), 3)
            label = f"{det['class']} {det['confidence']:.2f} [CONFIRMED]"
            cv2.putText(frame, label, (int(x1), int(y1)-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Show stats
        info_text = f"Frame: {frame_count} | Raw: {len(detections)} | Confirmed: {len(confirmed)}"
        cv2.putText(frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        cv2.imshow("Multi-Frame Confirmation (Gray=Raw, Green=Confirmed)", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n[+] Test complete")
