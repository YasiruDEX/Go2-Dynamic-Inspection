#!/usr/bin/env python3
"""
Step 5: Hemisphere Mapping - Insta360 to Pan/Tilt Angles
==========================================================
Converts pixel detections from Insta360 dual-hemisphere view to pan/tilt angles.

Usage:
    python3 05_hemisphere_mapping.py
"""

import cv2
import numpy as np
import yaml
from pathlib import Path
from ultralytics import YOLO


def load_config():
    """Load camera configuration."""
    config_path = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def pixel_to_ptz(detect_x, detect_y, frame_width, frame_height):
    """
    Convert Insta360 pixel coordinates to pan/tilt angles.
    
    Insta360 output format:
    - Top half (y < height/2): Front 180-degree hemisphere
    - Bottom half (y >= height/2): Back 180-degree hemisphere
    
    Args:
        detect_x: Detection center X coordinate (pixels)
        detect_y: Detection center Y coordinate (pixels)
        frame_width: Frame width (pixels)
        frame_height: Frame height (pixels)
    
    Returns:
        (pan, tilt): Pan and tilt angles in degrees
    """
    hemisphere_height = frame_height / 2
    
    # Determine which hemisphere
    is_front_hemisphere = (detect_y < hemisphere_height)
    
    # Normalize coordinates
    u_norm = detect_x / frame_width  # 0.0 to 1.0
    
    # Get local y coordinate within hemisphere
    if is_front_hemisphere:
        y_local = detect_y
    else:
        y_local = detect_y - hemisphere_height
    
    v_norm = y_local / hemisphere_height  # 0.0 to 1.0
    
    # Map to angles
    # Horizontal: 0 to 1 maps to -90 to +90 degrees within hemisphere
    yaw_in_hemisphere = u_norm * 180 - 90  # -90 to +90
    
    # Vertical: 0 to 1 maps to +90 to -90 degrees (top to bottom)
    pitch = 90 - v_norm * 180  # +90 at top, -90 at bottom
    
    # Convert to global yaw (0 to 360 degrees)
    if is_front_hemisphere:
        # Front hemisphere: yaw 0 is forward center
        yaw = yaw_in_hemisphere  # -90 to +90
    else:
        # Back hemisphere: add 180 degrees
        yaw = yaw_in_hemisphere + 180  # 90 to 270
    
    # Normalize to -180 to +180 range
    if yaw > 180:
        yaw = yaw - 360
    
    # Clamp pitch to reasonable values
    pitch = np.clip(pitch, -90, 90)
    
    return yaw, pitch


def apply_calibration_offsets(yaw, pitch, config):
    """
    Apply calibration offsets to convert from camera frame to servo frame.
    
    Calibration procedure:
    1. Place object at known servo position (e.g., pan=90, tilt=90)
    2. Detect object in Insta360 view
    3. Calculate offsets: offset = known_angle - detected_angle
    4. Store in config file
    """
    ptz_cfg = config.get('ptz_mapping', {})
    
    yaw_offset = ptz_cfg.get('yaw_offset', 0.0)
    pitch_offset = ptz_cfg.get('pitch_offset', 0.0)
    
    # Apply offsets
    pan_target = yaw + yaw_offset
    tilt_target = pitch + pitch_offset
    
    # Get servo limits
    servo_cfg = config.get('servo_limits', {})
    pan_min = servo_cfg.get('pan', {}).get('min', 0)
    pan_max = servo_cfg.get('pan', {}).get('max', 180)
    tilt_min = servo_cfg.get('tilt', {}).get('min', 20)
    tilt_max = servo_cfg.get('tilt', {}).get('max', 160)
    
    # Clamp to servo limits
    pan_target = np.clip(pan_target, pan_min, pan_max)
    tilt_target = np.clip(tilt_target, tilt_min, tilt_max)
    
    return pan_target, tilt_target


def test_hemisphere_mapping():
    """Test hemisphere mapping with live Insta360 feed."""
    config = load_config()
    
    insta_idx = config['insta360']['device_index']
    frame_width = config['insta360']['resolution']['width']
    frame_height = config['insta360']['resolution']['height']
    
    # Load YOLO model
    weight_path = Path(__file__).parent.parent / "weights" / "yolo11n.pt"
    model = YOLO(str(weight_path))
    model.conf = 0.5
    
    # Open camera
    cap = cv2.VideoCapture(insta_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    
    print("\n" + "="*60)
    print("Hemisphere Mapping Test")
    print("="*60)
    print(f"Frame size: {frame_width}x{frame_height}")
    print(f"Hemisphere height: {frame_height/2}")
    print("\nPress 'q' to quit")
    print("="*60 + "\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Run detection
        results = model(frame, verbose=False)
        
        # Process each detection
        for box in results[0].boxes:
            # Get bounding box
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            # Calculate center
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Get class info
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = results[0].names[cls]
            
            # Convert to pan/tilt
            yaw, pitch = pixel_to_ptz(center_x, center_y, frame_width, frame_height)
            
            # Apply calibration
            pan_target, tilt_target = apply_calibration_offsets(yaw, pitch, config)
            
            # Determine hemisphere
            is_front = (center_y < frame_height / 2)
            hemisphere = "FRONT" if is_front else "BACK"
            
            # Draw bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            
            # Display info
            info_lines = [
                f"{class_name} {conf:.2f}",
                f"{hemisphere} hemisphere",
                f"Pixel: ({int(center_x)}, {int(center_y)})",
                f"Raw: Pan={yaw:.1f} Tilt={pitch:.1f}",
                f"Target: Pan={pan_target:.1f} Tilt={tilt_target:.1f}"
            ]
            
            y_offset = int(y1) - 10
            for i, line in enumerate(info_lines):
                y_pos = y_offset - (len(info_lines) - i) * 20
                cv2.putText(frame, line, (int(x1), y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw hemisphere separator
        h_mid = frame_height // 2
        cv2.line(frame, (0, h_mid), (frame_width, h_mid), (0, 255, 255), 2)
        cv2.putText(frame, "FRONT 180deg", (10, h_mid - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, "BACK 180deg", (10, h_mid + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("Hemisphere Mapping Test", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n[+] Test complete")


if __name__ == "__main__":
    test_hemisphere_mapping()
