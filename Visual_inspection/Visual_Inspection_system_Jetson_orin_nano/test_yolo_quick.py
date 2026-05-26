#!/usr/bin/env python3
"""
Quick YOLO Test - Verify Detection
===================================
Tests if YOLO can detect fire extinguisher in current camera view.
"""

import cv2
from ultralytics import YOLO

print("="*70)
print("YOLO DETECTION TEST")
print("="*70)

# Load model
print("\n[1] Loading YOLO model...")
model = YOLO('weights/yolo11n.pt')
model.conf = 0.5

print(f"  Model: {model.model_name}")
print(f"  Classes: {len(model.names)}")
print(f"  Confidence threshold: 0.5")

# Open Insta360
print("\n[2] Opening Insta360 camera...")
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("[ERROR] Cannot open camera!")
    exit(1)

print("  Camera opened successfully")

# Run detection on 10 frames
print("\n[3] Running detection on 10 frames...")
print("─"*70)

detections_found = False

for i in range(10):
    ret, frame = cap.read()
    if not ret:
        continue
    
    results = model(frame, verbose=False)
    
    if results and len(results) > 0 and results[0].boxes is not None:
        boxes = results[0].boxes
        
        if len(boxes) > 0:
            detections_found = True
            print(f"\nFrame {i+1}:")
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = results[0].names[cls]
                print(f"  - {class_name}: {conf:.2f}")
        else:
            print(f"Frame {i+1}: No detections")
    else:
        print(f"Frame {i+1}: YOLO returned None/empty")

cap.release()

print("\n" + "="*70)
if detections_found:
    print("✓ YOLO IS WORKING - Objects detected!")
else:
    print("✗ NO DETECTIONS - Check camera view or object visibility")
print("="*70)
