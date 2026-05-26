#!/usr/bin/env python3
"""
Step 3: Export YOLO Model to TensorRT
======================================
Export YOLOv11n model to TensorRT engine for accelerated inference on Jetson.

This script:
1. Loads the .pt model
2. Exports to TensorRT .engine format
3. Validates the exported engine works

Prerequisites:
    - CUDA and TensorRT installed on Jetson
    - ultralytics installed: pip install ultralytics

Usage:
    python3 03_export_to_tensorrt.py
    python3 03_export_to_tensorrt.py --imgsz 640  # custom input size
"""

import argparse
from pathlib import Path
import time

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: ultralytics not installed")
    print("   Install with: pip install ultralytics")
    exit(1)


def check_model_exists():
    """Check if YOLO weight file exists."""
    weight_path = Path(__file__).parent.parent / "weights" / "yolo11n.pt"
    
    if not weight_path.exists():
        print(f"❌ ERROR: Weight file not found: {weight_path}")
        print("\n   Please place your yolo11n.pt file in the weights/ directory")
        return None
    
    return weight_path


def export_to_tensorrt(model_path, imgsz=640, half=True):
    """
    Export YOLO model to TensorRT format.
    
    Args:
        model_path: Path to .pt model file
        imgsz: Input image size (model will be optimized for this size)
        half: Use FP16 precision (recommended for Jetson)
    """
    print("\n" + "="*60)
    print("STEP 3: Exporting YOLOv11n to TensorRT")
    print("="*60)
    
    print(f"\nModel: {model_path}")
    print(f"Input size: {imgsz}x{imgsz}")
    print(f"Precision: {'FP16 (half)' if half else 'FP32 (full)'}")
    print("\n⚠️  NOTE: Export may take 5-10 minutes on Jetson")
    print("  This is a one-time operation. Be patient!\n")
    
    try:
        # Load model
        print("[+] Loading YOLO model...")
        model = YOLO(str(model_path))
        print("✓ Model loaded")
        
        # Export to TensorRT
        print("\n[+] Exporting to TensorRT engine...")
        print("  (This will take a while - optimizing for GPU...)")
        
        start_time = time.time()
        
        # Export with engine format
        # device=0 means use GPU 0
        engine_path = model.export(
            format='engine',
            imgsz=imgsz,
            half=half,
            device=0,
            verbose=True
        )
        
        export_time = time.time() - start_time
        
        print(f"\n✓ Export complete in {export_time:.1f} seconds")
        print(f"  TensorRT engine saved: {engine_path}")
        
        # Validate exported model
        print("\n[+] Validating exported engine...")
        
        try:
            # Load the exported engine
            trt_model = YOLO(engine_path)
            print("✓ TensorRT engine loaded successfully")
            
            # Test inference (dummy image)
            import numpy as np
            dummy_img = np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8)
            
            print("  Running test inference...")
            inference_start = time.time()
            results = trt_model(dummy_img, verbose=False)
            inference_time = (time.time() - inference_start) * 1000
            
            print(f"✓ Test inference successful")
            print(f"  Inference time: {inference_time:.1f}ms")
            
        except Exception as e:
            print(f"⚠️  Warning: Could not validate engine: {e}")
            print("   The engine file was created, but validation failed.")
            print("   Try running Step 4 to test with actual camera feed.")
        
        # Summary
        print("\n" + "="*60)
        print("✓ TensorRT Export Complete!")
        print("="*60)
        print(f"\nExported files:")
        print(f"  Engine: {engine_path}")
        print(f"  Export time: {export_time:.1f}s")
        print(f"\nNext steps:")
        print(f"  1. Run Step 4 to test TensorRT inference on camera")
        print(f"  2. Compare FPS: TensorRT vs CPU")
        print("="*60 + "\n")
        
        return engine_path
        
    except Exception as e:
        print(f"\n❌ ERROR during export: {e}")
        print("\nPossible issues:")
        print("  1. CUDA/TensorRT not installed properly")
        print("  2. Insufficient GPU memory")
        print("  3. Incompatible TensorRT version")
        print("\nFor Jetson, ensure you're using Ultralytics JetPack container:")
        print("  https://docs.ultralytics.com/guides/nvidia-jetson/")
        return None


def main():
    parser = argparse.ArgumentParser(description='Export YOLO model to TensorRT')
    parser.add_argument('--imgsz', type=int, default=640,
                        help='Input image size (default: 640)')
    parser.add_argument('--half', action='store_true', default=True,
                        help='Use FP16 precision (default: True)')
    parser.add_argument('--full', action='store_true',
                        help='Use FP32 precision instead of FP16')
    
    args = parser.parse_args()
    
    # Handle precision flag
    if args.full:
        half = False
    else:
        half = args.half
    
    # Check if model exists
    model_path = check_model_exists()
    if model_path is None:
        return
    
    # Export
    engine_path = export_to_tensorrt(model_path, imgsz=args.imgsz, half=half)
    
    if engine_path:
        print("Export successful! You can now use TensorRT acceleration.")
    else:
        print("Export failed. Check error messages above.")


if __name__ == "__main__":
    main()
