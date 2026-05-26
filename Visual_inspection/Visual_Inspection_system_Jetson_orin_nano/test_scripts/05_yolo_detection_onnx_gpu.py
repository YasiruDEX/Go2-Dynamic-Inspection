#!/usr/bin/env python3
"""
YOLO Detection with ONNX GPU (Alternative to TensorRT)
=======================================================
Uses ONNX Runtime GPU for fast inference without TensorRT installation issues.
Still provides 2-3x speedup over CPU.

Usage:
    python3 05_yolo_detection_onnx_gpu.py --camera both
"""

import cv2
import argparse
import time
import yaml
from pathlib import Path
from datetime import datetime

try:
    from ultralytics import YOLO
    import torch
except ImportError:
    print("[ERROR] ultralytics or torch not installed")
    exit(1)


def load_config():
    """Load camera configuration."""
    config_path = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def check_onnx_model():
    """Check if ONNX model exists."""
    onnx_path = Path(__file__).parent.parent / "weights" / "yolo11n.onnx"
    pt_path = Path(__file__).parent.parent / "weights" / "yolo11n.pt"
    
    if onnx_path.exists():
        return onnx_path, 'ONNX'
    elif pt_path.exists():
        return pt_path, 'PyTorch'
    else:
        print("[ERROR] No model found")
        return None, None


def run_detection_both_cameras(config, model, mode):
    """Run detection on both cameras."""
    print(f"\n{'='*60}")
    print(f"Detection on Both Cameras ({mode} GPU Mode)")
    print(f"{'='*60}")
    
    insta_idx = config['insta360']['device_index']
    logitech_idx = config['logitech_c920']['device_index']
    
    cap_insta = cv2.VideoCapture(insta_idx)
    cap_logitech = cv2.VideoCapture(logitech_idx)
    
    if not cap_insta.isOpened() or not cap_logitech.isOpened():
        print("[ERROR] Could not open cameras")
        return False
    
    print(f"[+] Both cameras opened")
    print(f"[+] Running YOLO with {mode} GPU acceleration")
    print(f"\nPress 'q' to quit, 's' to save")
    
    frame_count = 0
    start_time = time.time()
    total_inference_time = 0
    
    while True:
        ret_insta, frame_insta = cap_insta.read()
        ret_logitech, frame_logitech = cap_logitech.read()
        
        if not ret_insta or not ret_logitech:
            break
        
        # Run YOLO
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
        h_i, w_i = annotated_insta.shape[:2]
        h_l, w_l = annotated_logitech.shape[:2]
        
        frame_i = cv2.resize(annotated_insta, (int(w_i * target_height / h_i), target_height))
        frame_l = cv2.resize(annotated_logitech, (int(w_l * target_height / h_l), target_height))
        
        # Labels
        cv2.putText(frame_i, f"Insta360 | {mode} GPU FPS: {fps:.1f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame_l, f"Logitech | {mode} GPU FPS: {fps:.1f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame_i, f"Avg Inf: {avg_inf:.1f}ms", 
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        combined = cv2.hconcat([frame_i, frame_l])
        cv2.imshow(f"YOLO Detection ({mode} GPU)", combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"detection_{timestamp}_insta.jpg", annotated_insta)
            cv2.imwrite(f"detection_{timestamp}_logitech.jpg", annotated_logitech)
            print(f"  Screenshots saved")
    
    cap_insta.release()
    cap_logitech.release()
    cv2.destroyAllWindows()
    
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    avg_inf = (total_inference_time / frame_count * 1000) if frame_count > 0 else 0
    
    print(f"\n[+] Detection complete")
    print(f"  Average FPS: {avg_fps:.2f}")
    print(f"  Average inference: {avg_inf:.1f}ms")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='YOLO with ONNX/PyTorch GPU')
    parser.add_argument('--camera', type=str, default='both')
    parser.add_argument('--conf', type=float, default=0.5)
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("YOLO Detection with GPU (ONNX or PyTorch)")
    print("="*60)
    
    # Check GPU
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        print(f"\nGPU: {gpu_name}")
    else:
        print("\n[WARNING] No GPU detected, will use CPU")
    
    config = load_config()
    model_path, mode = check_onnx_model()
    
    if model_path is None:
        return
    
    print(f"\nLoading model: {model_path}")
    print(f"Mode: {mode}")
    
    # Load model (will use GPU via CUDA)
    model = YOLO(str(model_path))
    model.conf = args.conf
    
    print(f"[+] Model loaded")
    print(f"Detected classes: {list(model.names.values())}")
    
    run_detection_both_cameras(config, model, mode)
    
    print("\n" + "="*60)
    print("Detection complete!")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
