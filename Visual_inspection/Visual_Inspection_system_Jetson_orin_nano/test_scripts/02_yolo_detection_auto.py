#!/usr/bin/env python3
"""
Auto YOLO Detection - TensorRT with CPU Fallback
==================================================
Automatically uses TensorRT acceleration if GPU is available,
falls back to CPU mode if not. Works on both laptop and Jetson.

Usage:
    python3 02_yolo_detection_auto.py --camera both
"""

import cv2
import argparse
import time
import yaml
import os
from pathlib import Path
from datetime import datetime

try:
    from ultralytics import YOLO
    import torch
except ImportError:
    print("[ERROR] ultralytics or torch not installed")
    print("  Install with: pip install ultralytics torch")
    exit(1)


def check_cuda_available():
    """Check if CUDA is available for TensorRT acceleration."""
    if not torch.cuda.is_available():
        return False, "No CUDA"
    
    gpu_name = torch.cuda.get_device_name(0)
    cuda_version = torch.version.cuda
    
    return True, f"{gpu_name} (CUDA {cuda_version})"


def load_config():
    """Load camera configuration from YAML file."""
    config_path = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"
    
    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}")
        return None
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config


def check_model_exists():
    """Check if YOLO weight file exists."""
    weight_path = Path(__file__).parent.parent / "weights" / "yolo11n.pt"
    
    if not weight_path.exists():
        print(f"[ERROR] Weight file not found: {weight_path}")
        print("\n  Please place your yolo11n.pt file in the weights/ directory")
        return None
    
    return weight_path


def export_to_tensorrt(weight_path, force=False):
    """
    Export model to TensorRT if GPU available.
    Returns path to engine file or None if export fails.
    """
    engine_path = weight_path.parent / "yolo11n.engine"
    
    # Check if engine already exists
    if engine_path.exists() and not force:
        print(f"[+] TensorRT engine already exists: {engine_path}")
        return engine_path
    
    # Check CUDA availability
    cuda_available, gpu_info = check_cuda_available()
    if not cuda_available:
        print(f"[INFO] {gpu_info} - skipping TensorRT export")
        return None
    
    print(f"[+] GPU detected: {gpu_info}")
    print(f"[+] Exporting to TensorRT (this may take a few minutes)...")
    
    try:
        model = YOLO(str(weight_path))
        
        # Export with FP16 precision for better performance
        export_start = time.time()
        engine_path = model.export(
            format='engine',
            imgsz=640,
            half=True,
            device=0,
            verbose=False
        )
        export_time = time.time() - export_start
        
        print(f"[+] TensorRT export complete in {export_time:.1f}s")
        print(f"  Engine saved: {engine_path}")
        
        return Path(engine_path)
        
    except Exception as e:
        print(f"[WARNING] TensorRT export failed: {e}")
        print("  Falling back to CPU mode")
        return None


def load_model(weight_path, use_tensorrt=True):
    """
    Load YOLO model with automatic TensorRT detection.
    Returns (model, mode) where mode is 'TensorRT' or 'CPU'.
    """
    engine_path = weight_path.parent / "yolo11n.engine"
    
    # Try TensorRT first if requested
    if use_tensorrt:
        cuda_available, gpu_info = check_cuda_available()
        
        if cuda_available:
            # Try to load existing engine
            if engine_path.exists():
                try:
                    print(f"[+] Loading TensorRT engine: {engine_path}")
                    model = YOLO(str(engine_path))
                    print(f"[+] TensorRT mode active on {gpu_info}")
                    return model, 'TensorRT'
                except Exception as e:
                    print(f"[WARNING] Failed to load TensorRT engine: {e}")
            
            # Try to export if no engine exists
            else:
                print(f"[+] No TensorRT engine found, attempting export...")
                exported_path = export_to_tensorrt(weight_path)
                if exported_path and exported_path.exists():
                    try:
                        model = YOLO(str(exported_path))
                        print(f"[+] TensorRT mode active on {gpu_info}")
                        return model, 'TensorRT'
                    except Exception as e:
                        print(f"[WARNING] Failed to load exported engine: {e}")
    
    # Fall back to CPU mode
    print(f"[+] Loading model in CPU mode: {weight_path}")
    model = YOLO(str(weight_path))
    print(f"[+] CPU mode active")
    return model, 'CPU'


def run_detection_single_camera(camera_name, device_idx, width, height, model, mode):
    """Run YOLO detection on a single camera feed."""
    print(f"\n{'='*60}")
    print(f"Starting {camera_name} Detection ({mode} Mode)")
    print(f"{'='*60}")
    
    cap = cv2.VideoCapture(device_idx)
    
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera at /dev/video{device_idx}")
        return False
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"[+] Camera opened: {actual_width}x{actual_height}")
    print(f"[+] Running YOLOv11n with {mode} acceleration")
    print(f"\nPress 'q' to quit, 's' to save screenshot")
    
    frame_count = 0
    detection_count = 0
    start_time = time.time()
    total_inference_time = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("[ERROR] Failed to read frame")
            break
        
        # Run YOLO detection
        inference_start = time.time()
        results = model(frame, verbose=False)
        inference_time = time.time() - inference_start
        total_inference_time += inference_time
        
        annotated_frame = results[0].plot()
        
        num_detections = len(results[0].boxes)
        if num_detections > 0:
            detection_count += 1
        
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        avg_inference = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
        
        # Add info overlay
        info_text = f"{camera_name} | {mode} FPS: {fps:.1f} | Inf: {inference_time*1000:.1f}ms"
        cv2.putText(annotated_frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        detection_text = f"Detections: {num_detections} | Avg: {avg_inference:.1f}ms"
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
        
        cv2.imshow(f"YOLO Detection - {camera_name} ({mode})", annotated_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print(f"\n[+] Closing {camera_name}")
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"detection_{mode.lower()}_{camera_name}_{timestamp}.jpg"
            cv2.imwrite(filename, annotated_frame)
            print(f"  Screenshot saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Print summary
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    avg_inf = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
    
    print(f"\n{'='*60}")
    print(f"{camera_name} Detection Summary:")
    print(f"  Mode: {mode}")
    print(f"  Total frames: {frame_count}")
    print(f"  Frames with detections: {detection_count}")
    print(f"  Average FPS: {avg_fps:.2f}")
    print(f"  Average inference time: {avg_inf:.1f}ms")
    print(f"{'='*60}\n")
    
    return True


def run_detection_both_cameras(config, model, mode):
    """Run YOLO detection on both cameras simultaneously."""
    print(f"\n{'='*60}")
    print(f"Starting Detection on BOTH Cameras ({mode} Mode)")
    print(f"{'='*60}")
    
    insta_idx = config['insta360']['device_index']
    logitech_idx = config['logitech_c920']['device_index']
    
    cap_insta = cv2.VideoCapture(insta_idx)
    cap_logitech = cv2.VideoCapture(logitech_idx)
    
    if not cap_insta.isOpened() or not cap_logitech.isOpened():
        print("[ERROR] Could not open one or both cameras")
        return False
    
    print(f"[+] Both cameras opened")
    print(f"[+] Running YOLOv11n with {mode} for both feeds")
    print(f"\nPress 'q' to quit, 's' to save screenshots")
    
    frame_count = 0
    start_time = time.time()
    total_inference_time = 0
    
    while True:
        ret_insta, frame_insta = cap_insta.read()
        ret_logitech, frame_logitech = cap_logitech.read()
        
        if not ret_insta or not ret_logitech:
            print("[ERROR] Failed to read frames")
            break
        
        # Run YOLO on both frames
        inference_start = time.time()
        results_insta = model(frame_insta, verbose=False)
        results_logitech = model(frame_logitech, verbose=False)
        inference_time = time.time() - inference_start
        total_inference_time += inference_time
        
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
        info_text = f"Insta360 | {mode} FPS: {fps:.1f} | {len(results_insta[0].boxes)} det"
        cv2.putText(frame_insta_resized, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        info_text = f"Logitech | {mode} FPS: {fps:.1f} | {len(results_logitech[0].boxes)} det"
        cv2.putText(frame_logitech_resized, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        perf_text = f"Avg Inf: {avg_inf:.1f}ms"
        cv2.putText(frame_insta_resized, perf_text, (10, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        combined = cv2.hconcat([frame_insta_resized, frame_logitech_resized])
        
        cv2.imshow(f"YOLO Detection - Both Cameras ({mode})", combined)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"detection_{mode.lower()}_insta360_{timestamp}.jpg", annotated_insta)
            cv2.imwrite(f"detection_{mode.lower()}_logitech_{timestamp}.jpg", annotated_logitech)
            print(f"  Screenshots saved")
    
    cap_insta.release()
    cap_logitech.release()
    cv2.destroyAllWindows()
    
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    avg_inf = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
    
    print(f"\n[+] Detection complete")
    print(f"  Mode: {mode}")
    print(f"  Average FPS: {avg_fps:.2f}")
    print(f"  Average inference: {avg_inf:.1f}ms")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Auto YOLO detection (TensorRT or CPU)')
    parser.add_argument('--camera', type=str, default='both', 
                        choices=['insta', 'logitech', 'both'],
                        help='Which camera to use (default: both)')
    parser.add_argument('--conf', type=float, default=0.5,
                        help='Confidence threshold (default: 0.5)')
    parser.add_argument('--cpu', action='store_true',
                        help='Force CPU mode (skip TensorRT)')
    parser.add_argument('--export', action='store_true',
                        help='Force re-export to TensorRT')
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Auto YOLO Detection (TensorRT with CPU Fallback)")
    print("="*60)
    
    # Check CUDA status
    cuda_available, gpu_info = check_cuda_available()
    print(f"\nGPU Status: {gpu_info}")
    
    # Load config
    config = load_config()
    if config is None:
        return
    
    # Check weight file
    weight_path = check_model_exists()
    if weight_path is None:
        return
    
    # Force export if requested
    if args.export:
        print("\n[+] Forcing TensorRT export...")
        export_to_tensorrt(weight_path, force=True)
    
    # Load model (auto-detect TensorRT or CPU)
    use_tensorrt = not args.cpu
    model, mode = load_model(weight_path, use_tensorrt=use_tensorrt)
    model.conf = args.conf
    
    print(f"\nDetected classes: {list(model.names.values())}")
    print(f"Confidence threshold: {args.conf}")
    
    # Run detection
    if args.camera == 'insta':
        insta_cfg = config['insta360']
        run_detection_single_camera(
            "Insta360", 
            insta_cfg['device_index'],
            insta_cfg['resolution']['width'],
            insta_cfg['resolution']['height'],
            model,
            mode
        )
    elif args.camera == 'logitech':
        logitech_cfg = config['logitech_c920']
        run_detection_single_camera(
            "Logitech C920",
            logitech_cfg['device_index'],
            logitech_cfg['resolution']['width'],
            logitech_cfg['resolution']['height'],
            model,
            mode
        )
    else:
        run_detection_both_cameras(config, model, mode)
    
    print("\n" + "="*60)
    print(f"Detection test complete ({mode} mode)")
    print("="*60)
    if mode == 'CPU' and cuda_available:
        print("\nNote: GPU detected but TensorRT not used.")
        print("  Run with --export to create TensorRT engine")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
