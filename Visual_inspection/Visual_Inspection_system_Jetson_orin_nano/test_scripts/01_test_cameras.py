#!/usr/bin/env python3
"""
Step 1: Test Camera Opening
============================
Simple script to open both cameras and display live feeds.
Shows resolution, FPS, and allows you to verify camera setup.

Usage:
    python3 01_test_cameras.py --insta 0 --logitech 1

Controls:
    - Press 'q' to quit
    - Press 's' to save a screenshot
"""

import cv2
import argparse
import time
from datetime import datetime


def test_camera(device_index, camera_name):
    """Test opening a single camera and display its properties."""
    print(f"\n{'='*60}")
    print(f"Testing {camera_name} at /dev/video{device_index}")
    print(f"{'='*60}")
    
    cap = cv2.VideoCapture(device_index)
    
    if not cap.isOpened():
        print(f"❌ ERROR: Could not open camera at /dev/video{device_index}")
        print(f"   Make sure the camera is connected.")
        return False
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"✓ Camera opened successfully!")
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {fps}")
    print(f"\nPress 'q' to close this camera, 's' to save screenshot")
    
    frame_count = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("❌ ERROR: Failed to read frame")
            break
        
        frame_count += 1
        elapsed = time.time() - start_time
        
        # Calculate actual FPS
        if elapsed > 0:
            actual_fps = frame_count / elapsed
        else:
            actual_fps = 0
        
        # Add info overlay
        info_text = f"{camera_name} | {width}x{height} | FPS: {actual_fps:.1f}"
        cv2.putText(frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add guide for dual-hemisphere view (for Insta360)
        if "insta" in camera_name.lower():
            h = frame.shape[0]
            # Draw line separating front/back hemispheres
            cv2.line(frame, (0, h//2), (width, h//2), (0, 255, 255), 2)
            cv2.putText(frame, "FRONT 180deg", (10, h//2 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, "BACK 180deg", (10, h//2 + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.imshow(f"{camera_name} - /dev/video{device_index}", frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print(f"\n✓ Closing {camera_name}")
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"screenshot_{camera_name}_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"  Screenshot saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"{camera_name} Summary:")
    print(f"  Total frames captured: {frame_count}")
    print(f"  Average FPS: {actual_fps:.2f}")
    print(f"{'='*60}\n")
    
    return True


def test_both_cameras(insta_idx, logitech_idx):
    """Open both cameras simultaneously and display side by side."""
    print(f"\n{'='*60}")
    print(f"Opening BOTH Cameras Simultaneously")
    print(f"{'='*60}")
    
    cap_insta = cv2.VideoCapture(insta_idx)
    cap_logitech = cv2.VideoCapture(logitech_idx)
    
    if not cap_insta.isOpened():
        print(f"❌ ERROR: Could not open Insta360 at /dev/video{insta_idx}")
        return False
    
    if not cap_logitech.isOpened():
        print(f"❌ ERROR: Could not open Logitech at /dev/video{logitech_idx}")
        cap_insta.release()
        return False
    
    print(f"✓ Both cameras opened successfully!")
    print(f"  Insta360: /dev/video{insta_idx}")
    print(f"  Logitech: /dev/video{logitech_idx}")
    print(f"\nPress 'q' to quit")
    
    frame_count = 0
    start_time = time.time()
    
    while True:
        ret_insta, frame_insta = cap_insta.read()
        ret_logitech, frame_logitech = cap_logitech.read()
        
        if not ret_insta or not ret_logitech:
            print("❌ ERROR: Failed to read frames")
            break
        
        frame_count += 1
        elapsed = time.time() - start_time
        actual_fps = frame_count / elapsed if elapsed > 0 else 0
        
        # Resize frames to fit side by side (scale down if needed)
        h_insta, w_insta = frame_insta.shape[:2]
        h_logitech, w_logitech = frame_logitech.shape[:2]
        
        # Make them same height for display
        target_height = 480
        scale_insta = target_height / h_insta
        scale_logitech = target_height / h_logitech
        
        frame_insta_resized = cv2.resize(frame_insta, 
                                         (int(w_insta * scale_insta), target_height))
        frame_logitech_resized = cv2.resize(frame_logitech, 
                                            (int(w_logitech * scale_logitech), target_height))
        
        # Add labels
        cv2.putText(frame_insta_resized, f"Insta360 | FPS: {actual_fps:.1f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_logitech_resized, f"Logitech C920 | FPS: {actual_fps:.1f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add hemisphere separator for Insta360
        h = frame_insta_resized.shape[0]
        cv2.line(frame_insta_resized, (0, h//2), (frame_insta_resized.shape[1], h//2), 
                 (0, 255, 255), 1)
        
        # Concatenate side by side
        combined = cv2.hconcat([frame_insta_resized, frame_logitech_resized])
        
        cv2.imshow("Both Cameras - Press 'q' to quit, 's' to save", combined)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"screenshot_insta360_{timestamp}.jpg", frame_insta)
            cv2.imwrite(f"screenshot_logitech_{timestamp}.jpg", frame_logitech)
            print(f"  Screenshots saved")
    
    cap_insta.release()
    cap_logitech.release()
    cv2.destroyAllWindows()
    
    print(f"\n✓ Both cameras closed successfully")
    print(f"  Average FPS: {actual_fps:.2f}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Test camera opening and feeds')
    parser.add_argument('--insta', type=int, default=0, 
                        help='Insta360 device index (default: 0)')
    parser.add_argument('--logitech', type=int, default=1, 
                        help='Logitech device index (default: 1)')
    parser.add_argument('--mode', type=str, default='both', 
                        choices=['insta', 'logitech', 'both'],
                        help='Which camera(s) to test (default: both)')
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("STEP 1: Camera Opening Test")
    print("="*60)
    print("\nThis script will:")
    print("  1. Open camera(s)")
    print("  2. Display live feed(s)")
    print("  3. Show resolution and FPS")
    print("  4. Allow you to verify camera setup")
    
    if args.mode == 'insta':
        test_camera(args.insta, "Insta360")
    elif args.mode == 'logitech':
        test_camera(args.logitech, "Logitech C920")
    else:
        test_both_cameras(args.insta, args.logitech)
    
    print("\n" + "="*60)
    print("✓ Camera test complete!")
    print("="*60)
    print("\nNext steps:")
    print("  1. Note the actual resolutions you saw")
    print("  2. Update config/camera_calibration.yaml")
    print("  3. Move to Step 2: YOLO detection")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
