#!/usr/bin/env python3
"""
Auto Camera Detection and Configuration
========================================
Automatically detects all video devices and helps you identify them.
"""

import cv2
import glob
import time

print("\n" + "="*70)
print("AUTOMATIC CAMERA DETECTION")
print("="*70)

# Get all video devices using glob
video_devices = sorted(glob.glob('/dev/video*'))

if not video_devices:
    print("\n[ERROR] No video devices found!")
    print("Make sure cameras are connected.")
    exit(1)

print(f"\nFound {len(video_devices)} video device(s):")
for dev in video_devices:
    print(f"  - {dev}")

print("\n" + "="*70)
print("TESTING EACH CAMERA...")
print("A window will pop up for each camera.")
print("Press 's' to save a test image, 'q' to move to next camera")
print("="*70)

camera_info = {}

for device in video_devices:
    device_num = int(device.split('video')[-1])
    
    print(f"\n[Testing {device}]")
    
    cap = cv2.VideoCapture(device_num)
    
    if not cap.isOpened():
        print(f"  ✗ Could not open {device}")
        continue
    
    # Get properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {fps}")
    print(f"  Opening preview window...")
    
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"  ✗ Could not read frame")
            break
        
        frame_count += 1
        
        # Add info overlay
        cv2.putText(frame, f"{device} - {width}x{height}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "Press 's' to save, 'q' for next camera", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        cv2.imshow(f"Camera Test - {device}", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"camera_test_{device.replace('/', '_')}.jpg"
            cv2.imwrite(filename, frame)
            print(f"  ✓ Saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Ask user to identify
    print(f"\n  What is this camera?")
    print(f"  1) Insta360 X3")
    print(f"  2) Logitech C920")
    print(f"  3) Built-in webcam")
    print(f"  4) Other/Skip")
    
    choice = input(f"  Enter choice (1-4): ").strip()
    
    if choice == '1':
        camera_info[device_num] = {
            'name': 'insta360',
            'device': device,
            'width': width,
            'height': height
        }
        print(f"  ✓ Marked as Insta360 X3")
    elif choice == '2':
        camera_info[device_num] = {
            'name': 'logitech_c920',
            'device': device,
            'width': width,
            'height': height
        }
        print(f"  ✓ Marked as Logitech C920")
    elif choice == '3':
        camera_info[device_num] = {
            'name': 'builtin',
            'device': device,
            'width': width,
            'height': height
        }
        print(f"  ✓ Marked as Built-in webcam")
    else:
        print(f"  Skipped")

print("\n" + "="*70)
print("DETECTION SUMMARY")
print("="*70)

insta_idx = None
logitech_idx = None

for idx, info in camera_info.items():
    print(f"\n{info['device']}:")
    print(f"  Type: {info['name']}")
    print(f"  Resolution: {info['width']}x{info['height']}")
    
    if info['name'] == 'insta360':
        insta_idx = idx
    elif info['name'] == 'logitech_c920':
        logitech_idx = idx

print("\n" + "="*70)

if insta_idx is not None and logitech_idx is not None:
    print("✓ BOTH CAMERAS DETECTED!")
    print(f"\nInsta360: /dev/video{insta_idx}")
    print(f"Logitech: /dev/video{logitech_idx}")
    
    # Update config
    print(f"\nUpdating config/camera_calibration.yaml...")
    
    config_path = "config/camera_calibration.yaml"
    with open(config_path, 'r') as f:
        lines = f.readlines()
    
    # Update device indices
    with open(config_path, 'w') as f:
        for line in lines:
            if 'device_index:' in line and 'insta360' in ''.join(lines[max(0, lines.index(line)-5):lines.index(line)]):
                f.write(f"  device_index: {insta_idx}  # /dev/video{insta_idx} - Insta360 X3\n")
            elif 'device_index:' in line and 'logitech' in ''.join(lines[max(0, lines.index(line)-5):lines.index(line)]):
                f.write(f"  device_index: {logitech_idx}  # /dev/video{logitech_idx} - Logitech C920\n")
            else:
                f.write(line)
    
    print("✓ Configuration updated!")
    print(f"\nYou can now run:")
    print(f"  python3 visual_pipeline.py")
    
elif logitech_idx is not None:
    print("⚠ Only Logitech detected")
    print(f"\nLogitech: /dev/video{logitech_idx}")
    print(f"\nFor full system, connect Insta360 X3")
    print(f"For testing: python3 single_camera_pipeline.py")
    
else:
    print("⚠ No cameras properly identified")
    print("Please re-run and identify cameras")

print("="*70)
