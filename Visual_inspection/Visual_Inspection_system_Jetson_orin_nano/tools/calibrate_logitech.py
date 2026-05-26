#!/usr/bin/env python3
"""
Logitech Camera Calibration Script
===================================
This script calibrates the Logitech C920 camera to get intrinsic parameters
needed for IBVS (Image-Based Visual Servoing).

Requirements:
1. Print a checkerboard pattern (9x6 squares, each 25mm)
   OR display this on a screen: https://markhedleyjones.com/storage/checkerboards/Checkerboard-A4-25mm-8x6.pdf
2. Show the checkerboard to the camera from different angles
3. Press SPACE to capture (need ~15-20 good images)
4. Press 'q' when done

Output: Saves calibration to config/logitech_intrinsics.yaml
"""

import cv2
import numpy as np
import yaml
import os
import glob

# Checkerboard dimensions (internal corners)
CHECKERBOARD = (8, 6)  # 9x7 squares = 8x6 internal corners
SQUARE_SIZE = 25.0  # mm

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (8,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Arrays to store object points and image points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

def find_logitech():
    """Auto-detect Logitech camera index"""
    paths = sorted(glob.glob('/sys/class/video4linux/video*'))
    for path in paths:
        try:
            name_path = os.path.join(path, 'name')
            if not os.path.exists(name_path):
                continue
            with open(name_path, 'r') as f:
                name = f.read().strip()
            if "C920" in name or "Logitech" in name:
                idx = int(path.split('video')[-1])
                return idx
        except:
            pass
    return 0  # Fallback

def main():
    # Find Logitech camera
    cam_id = find_logitech()
    print(f"Opening Logitech camera at index {cam_id}")
    
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap.isOpened():
        print("ERROR: Cannot open Logitech camera!")
        return
    
    print("\n=== Logitech Camera Calibration ===")
    print("1. Show checkerboard (9x7 squares, 25mm each) to camera")
    print("2. Move it around (different angles, distances)")
    print("3. Press SPACE when corners detected (green overlay)")
    print("4. Capture 15-20 images")
    print("5. Press 'q' when done\n")
    
    captured_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        
        display_frame = frame.copy()
        
        if ret_corners:
            # Refine corners
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw corners
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners_refined, ret_corners)
            cv2.putText(display_frame, "READY - Press SPACE to capture", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "No checkerboard detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.putText(display_frame, f"Captured: {captured_count}/15", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        cv2.imshow('Logitech Calibration', display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' ') and ret_corners:
            # Capture this frame
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            captured_count += 1
            print(f"✓ Captured image {captured_count}")
            
        elif key == ord('q'):
            if captured_count < 10:
                print(f"\nWARNING: Only {captured_count} images captured. Need at least 10!")
                response = input("Continue anyway? (y/n): ")
                if response.lower() != 'y':
                    continue
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    if captured_count < 10:
        print("\nERROR: Not enough images for calibration!")
        return
    
    print(f"\n=== Running Calibration with {captured_count} images ===")
    
    # Get frame size
    h, w = gray.shape[:2]
    
    # Calibrate camera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None
    )
    
    if not ret:
        print("ERROR: Calibration failed!")
        return
    
    # Calculate reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    mean_error /= len(objpoints)
    
    print(f"\n✅ Calibration Successful!")
    print(f"Reprojection Error: {mean_error:.3f} pixels")
    print(f"\nCamera Matrix (Intrinsics):")
    print(camera_matrix)
    print(f"\nDistortion Coefficients:")
    print(dist_coeffs)
    
    # Extract intrinsic parameters
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    print(f"\n📊 Key Parameters:")
    print(f"  f_x: {fx:.2f}")
    print(f"  f_y: {fy:.2f}")
    print(f"  c_x: {cx:.2f}")
    print(f"  c_y: {cy:.2f}")
    
    # Save to YAML
    calibration_data = {
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': dist_coeffs.tolist(),
        'image_width': w,
        'image_height': h,
        'reprojection_error': float(mean_error),
        'fx': float(fx),
        'fy': float(fy),
        'cx': float(cx),
        'cy': float(cy)
    }
    
    output_path = 'config/logitech_intrinsics.yaml'
    os.makedirs('config', exist_ok=True)
    
    with open(output_path, 'w') as f:
        yaml.dump(calibration_data, f, default_flow_style=False)
    
    print(f"\n💾 Saved calibration to: {output_path}")
    print("\nYou can now use this for IBVS control!")

if __name__ == "__main__":
    main()
