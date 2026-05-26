#!/usr/bin/env python3
"""
Simple Camera Test - Quick Identification
==========================================
Shows all cameras at once so you can quickly identify them.
"""

import cv2
import time

print("\n" + "="*70)
print("QUICK CAMERA TEST - All Cameras at Once")
print("="*70)

cameras = {}

# Try to open all possible video devices
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cameras[i] = {
                'cap': cap,
                'width': width,
                'height': height
            }
            print(f"\n✓ /dev/video{i} - {width}x{height}")
        else:
            cap.release()
    else:
        cap.release()

if not cameras:
    print("\n[ERROR] No cameras found!")
    exit(1)

print(f"\nFound {len(cameras)} working camera(s)")
print("\nShowing all cameras simultaneously...")
print("Press 'q' to quit\n")

time.sleep(1)

try:
    while True:
        frames_to_show = []
        
        for idx, cam_info in cameras.items():
            ret, frame = cam_info['cap'].read()
            if ret:
                # Resize to manageable size
                display_frame = cv2.resize(frame, (640, 480))
                
                # Add label
                label = f"/dev/video{idx} ({cam_info['width']}x{cam_info['height']})"
                cv2.putText(display_frame, label, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show in separate window
                cv2.imshow(f"Camera {idx}", display_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

# Cleanup
for cam_info in cameras.values():
    cam_info['cap'].release()
cv2.destroyAllWindows()

print("\n" + "="*70)
print("Camera Identification")
print("="*70)

print("\nBased on what you saw:")
print("- Which /dev/video is Insta360 X3? (wide/fisheye view)")
print("- Which /dev/video is Logitech C920? (normal webcam)")

print("\nManually update config/camera_calibration.yaml with correct indices.")
print("="*70)
