#!/usr/bin/env python3
"""
Step 4: YOLO Detection (TensorRT Mode)
=======================================
Run YOLOv11n detection on camera feeds using TensorRT acceleration.
Compare performance with CPU mode from Step 2.

Prerequisites:
    - TensorRT engine exported (run 03_export_to_tensorrt.py first)
    - Engine file should be: weights/yolo11n.engine

Usage:
    python3 04_yolo_detection_tensorrt.py --camera insta
    python3 04_yolo_detection_tensorrt.py --camera logitech
    python3 04_yolo_detection_tensorrt.py --camera both

Controls:
    - Press 'q' to quit
    - Press 's' to save detection screenshot
"""

import cv2
import argparse
import time
import yaml
from pathlib import Path
from datetime import datetime

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: ultralytics not installed")
    print("   Install with: pip install ultralytics")
    exit(1)


def load_config():
    """Load camera configuration from YAML file."""
    config_path = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"
    
    if not config_path.exists():
        print(f"❌ ERROR: Config file not found: {config_path}")
        return None
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def check_engine_exists():
    """Check if TensorRT engine file exists."""
    engine_path = Path(__file__).parent.parent / "weights" / "yolo11n.engine"
    
    if not engine_path.exists():
        print(f"❌ ERROR: TensorRT engine not found: {engine_path}")
        print("\n   Please run Step 3 first to export the model:")
        print("   python3 test_scripts/03_export_to_tensorrt.py")
        return None
    
    return engine_path


def run_detection_single_camera(camera_name, device_idx, width, height, model):
    """Run YOLO detection on a single camera feed with TensorRT."""
    print(f"\n{'='*60}")
    print(f"Starting {camera_name} Detection (TensorRT Mode)")
    print(f"{'='*60}")
    
    # Open camera
    cap = cv2.VideoCapture(device_idx)
    
    if not cap.isOpened():
        print(f"❌ ERROR: Could not open camera at /dev/video{device_idx}")
        return False
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"✓ Camera opened: {actual_width}x{actual_height}")
    print(f"  Running YOLOv11n with TensorRT acceleration...")
    print(f"\nPress 'q' to quit, 's' to save screenshot")
    
    frame_count = 0
    detection_count = 0
    start_time = time.time()
    total_inference_time = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("❌ ERROR: Failed to read frame")
            break
        
        # Run YOLO detection with TensorRT
        inference_start = time.time()
        results = model(frame, verbose=False)
        inference_time = time.time() - inference_start
        total_inference_time += inference_time
        
        # Get annotated frame with bounding boxes
        annotated_frame = results[0].plot()
        
        # Count detections
        num_detections = len(results[0].boxes)
        if num_detections > 0:
            detection_count += 1
        
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        avg_inference = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
        
        # Add info overlay
        info_text = f"{camera_name} | TensorRT FPS: {fps:.1f} | Inf: {inference_time*1000:.1f}ms"
        cv2.putText(annotated_frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        detection_text = f"Detections: {num_detections} | Avg Inf: {avg_inference:.1f}ms"
        cv2.putText(annotated_frame, detection_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Add hemisphere separator for Insta360
        if "insta" in camera_name.lower():
            h = annotated_frame.shape[0]
            cv2.line(annotated_frame, (0, h//2), (actual_width, h//2), 
                     (255, 255, 0), 2)
            cv2.putText(annotated_frame, "FRONT 180deg", (10, h//2 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(annotated_frame, "BACK 180deg", (10, h//2 + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Display detection details
        if num_detections > 0:
            y_offset = 90
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = results[0].names[cls]
                
                det_text = f"  {class_name}: {conf:.2f}"
                cv2.putText(annotated_frame, det_text, (10, y_offset), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y_offset += 25
        
        cv2.imshow(f"YOLO Detection - {camera_name} (TensorRT)", annotated_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print(f"\n✓ Closing {camera_name}")
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"detection_trt_{camera_name}_{timestamp}.jpg"
            cv2.imwrite(filename, annotated_frame)
            print(f"  Screenshot saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Print summary
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    avg_inf = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
    
    print(f"\n{'='*60}")
    print(f"{camera_name} TensorRT Detection Summary:")
    print(f"  Total frames: {frame_count}")
    print(f"  Frames with detections: {detection_count}")
    print(f"  Average FPS: {avg_fps:.2f}")
    print(f"  Average inference time: {avg_inf:.1f}ms")
    print(f"{'='*60}\n")
    
    return True


def run_detection_both_cameras(config, model):
    """Run YOLO detection on both cameras simultaneously with TensorRT."""
    print(f"\n{'='*60}")
    print(f"Starting Detection on BOTH Cameras (TensorRT Mode)")
    print(f"{'='*60}")
    
    insta_idx = config['insta360']['device_index']
    logitech_idx = config['logitech_c920']['device_index']
    
    cap_insta = cv2.VideoCapture(insta_idx)
    cap_logitech = cv2.VideoCapture(logitech_idx)
    
    if not cap_insta.isOpened() or not cap_logitech.isOpened():
        print("❌ ERROR: Could not open one or both cameras")
        return False
    
    print(f"✓ Both cameras opened")
    print(f"  Running YOLOv11n with TensorRT for both feeds...")
    print(f"\nPress 'q' to quit, 's' to save screenshots")
    
    frame_count = 0
    start_time = time.time()
    total_inference_time = 0
    
    while True:
        ret_insta, frame_insta = cap_insta.read()
        ret_logitech, frame_logitech = cap_logitech.read()
        
        if not ret_insta or not ret_logitech:
            print("❌ ERROR: Failed to read frames")
            break
        
        # Run YOLO on both frames
        inference_start = time.time()
        results_insta = model(frame_insta, verbose=False)
        results_logitech = model(frame_logitech, verbose=False)
        inference_time = time.time() - inference_start
        total_inference_time += inference_time
        
        # Get annotated frames
        annotated_insta = results_insta[0].plot()
        annotated_logitech = results_logitech[0].plot()
        
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        avg_inf = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
        
        # Resize for display
        target_height = 480
        h_insta, w_insta = annotated_insta.shape[:2]
        h_logitech, w_logitech = annotated_logitech.shape[:2]
        
        scale_insta = target_height / h_insta
        scale_logitech = target_height / h_logitech
        
        frame_insta_resized = cv2.resize(annotated_insta, 
                                         (int(w_insta * scale_insta), target_height))
        frame_logitech_resized = cv2.resize(annotated_logitech, 
                                            (int(w_logitech * scale_logitech), target_height))
        
        # Add labels
        info_text = f"Insta360 | TRT FPS: {fps:.1f} | {len(results_insta[0].boxes)} det"
        cv2.putText(frame_insta_resized, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        info_text = f"Logitech | TRT FPS: {fps:.1f} | {len(results_logitech[0].boxes)} det"
        cv2.putText(frame_logitech_resized, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        perf_text = f"Avg Inference: {avg_inf:.1f}ms"
        cv2.putText(frame_insta_resized, perf_text, (10, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Concatenate side by side
        combined = cv2.hconcat([frame_insta_resized, frame_logitech_resized])
        
        cv2.imshow("YOLO Detection - Both Cameras (TensorRT)", combined)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"detection_trt_insta360_{timestamp}.jpg", annotated_insta)
            cv2.imwrite(f"detection_trt_logitech_{timestamp}.jpg", annotated_logitech)
            print(f"  Screenshots saved")
    
    cap_insta.release()
    cap_logitech.release()
    cv2.destroyAllWindows()
    
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    avg_inf = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
    
    print(f"\n✓ Detection complete")
    print(f"  Average FPS: {avg_fps:.2f}")
    print(f"  Average inference: {avg_inf:.1f}ms")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='YOLO detection with TensorRT')
    parser.add_argument('--camera', type=str, default='both', 
                        choices=['insta', 'logitech', 'both'],
                        help='Which camera to use (default: both)')
    parser.add_argument('--conf', type=float, default=0.5,
                        help='Confidence threshold (default: 0.5)')
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("STEP 4: YOLO Detection (TensorRT Mode)")
    print("="*60)
    
    # Load config
    config = load_config()
    if config is None:
        return
    
    # Check if engine exists
    engine_path = check_engine_exists()
    if engine_path is None:
        return
    
    # Load YOLO model with TensorRT engine
    print(f"\nLoading TensorRT engine from: {engine_path}")
    
    try:
        model = YOLO(str(engine_path))
        model.conf = args.conf
        print(f"✓ TensorRT engine loaded successfully")
        print(f"  Confidence threshold: {args.conf}")
        print(f"  Detected classes: {list(model.names.values())}")
        print(f"\n[+] TensorRT should provide 3-5x speedup vs CPU")
    except Exception as e:
        print(f"❌ ERROR loading TensorRT engine: {e}")
        return
    
    # Run detection
    if args.camera == 'insta':
        insta_cfg = config['insta360']
        run_detection_single_camera(
            "Insta360", 
            insta_cfg['device_index'],
            insta_cfg['resolution']['width'],
            insta_cfg['resolution']['height'],
            model
        )
    elif args.camera == 'logitech':
        logitech_cfg = config['logitech_c920']
        run_detection_single_camera(
            "Logitech C920",
            logitech_cfg['device_index'],
            logitech_cfg['resolution']['width'],
            logitech_cfg['resolution']['height'],
            model
        )
    else:
        run_detection_both_cameras(config, model)
    
    print("\n" + "="*60)
    print("✓ TensorRT detection test complete!")
    print("="*60)
    print("\nPerformance Comparison:")
    print("  Run Step 2 (CPU) and Step 4 (TensorRT) to compare:")
    print("  - FPS improvement")
    print("  - Inference time reduction")
    print("\nNext steps:")
    print("  1. Move to Step 5: Hemisphere mapping")
    print("  2. Then: Arduino control integration")
    print("  3. Finally: IBVS refinement loop")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
