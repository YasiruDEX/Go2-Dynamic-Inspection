"""Quick test script for gauge pipeline integration."""

import sys
from pathlib import Path

# Add gauge directory to path
gauge_dir = Path("app/pipelines/gauge")
sys.path.insert(0, str(gauge_dir))

print("Testing gauge pipeline imports...")

try:
    print("1. Testing plots import...")
    from plots import Plotter
    print("   [OK] plots")
except Exception as e:
    print(f"   [FAIL] plots: {e}")

try:
    print("2. Testing gauge_detection import...")
    from gauge_detection.detection_inference import detection_gauge_face
    print("   [OK] gauge_detection")
except Exception as e:
    print(f"   [FAIL] gauge_detection: {e}")

try:
    print("3. Testing key_point_detection import...")
    from key_point_detection.key_point_inference import KeyPointInference
    print("   [OK] key_point_detection")
except Exception as e:
    print(f"   [FAIL] key_point_detection: {e}")

try:
    print("4. Testing segmentation import...")
    from segmentation.segmenation_inference import segment_gauge_needle
    print("   [OK] segmentation")
except Exception as e:
    print(f"   [FAIL] segmentation: {e}")

try:
    print("5. Testing OCR import...")
    from ocr.ocr_inference import ocr
    print("   [OK] OCR")
except Exception as e:
    print(f"   [FAIL] OCR: {e}")
    print(f"   Note: OCR requires mmcv/mmdet/mmocr which need C++ build tools")

try:
    print("6. Testing decimal_point_detection import...")
    from decimal_point_detection import NumberROIExtractor, DecimalPointDetector
    print("   [OK] decimal_point_detection")
except Exception as e:
    print(f"   [FAIL] decimal_point_detection: {e}")

print("\nTest complete!")
