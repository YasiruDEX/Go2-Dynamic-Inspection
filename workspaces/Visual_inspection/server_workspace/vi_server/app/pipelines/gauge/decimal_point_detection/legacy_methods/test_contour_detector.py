"""
Test contour-based decimal detection
"""

import cv2
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))

from decimal_point_detection.contour_based_detector import ContourBasedDecimalDetector


if __name__ == "__main__":
    # Load image
    image_path = "runs/run_20251212212910/gauge1.jpg/image_cropped.jpg"
    image = cv2.imread(image_path)
    
    if image is None:
        print(f"Error: Could not load {image_path}")
        exit(1)
    
    # OCR results (use your real ones if available)
    ocr_results = [
        {'text': '05', 'bbox': [[220, 300], [280, 300], [280, 340], [220, 340]]},
        {'text': '15', 'bbox': [[360, 160], [420, 160], [420, 200], [360, 200]]},
        {'text': '35', 'bbox': [[420, 330], [480, 330], [480, 370], [420, 370]]}
    ]
    
    # Create detector
    detector = ContourBasedDecimalDetector(
        min_dot_area=3,
        max_dot_area=30,
        confidence_threshold=0.5
    )
    
    # Detect decimals
    has_decimals, confidence, debug_info = detector.detect_decimals(
        image, ocr_results, debug=True
    )
    
    # Print results
    print(f"\n{'='*60}")
    print(f"Decimal detection using contour analysis")
    print(f"Decimals detected: {has_decimals}")
    print(f"Confidence: {confidence:.1%}")
    print(f"Numbers checked: {debug_info['total_checked']}")
    print(f"With decimals: {debug_info['decimal_count']}")
    
    for det in debug_info['detections']:
        print(f"  {det['text']}: {'✓' if det['has_decimal'] else '✗'} "
              f"(dots found: {det.get('dot_count', 0)})")
    
    print(f"\nOriginal reading: 19.866")
    corrected = detector.apply_correction(19.866, has_decimals)
    print(f"Corrected reading: {corrected}")
    print(f"{'='*60}\n")
    
    # Show visualization
    vis = detector.visualize_detections(image, debug_info)
    cv2.imshow("Contour-Based Decimal Detection", vis)
    
    # Show individual processed regions
    for i, det in enumerate(debug_info['detections']):
        if det.get('processed_region') is not None:
            cv2.imshow(f"Region {det['text']}", det['processed_region'])
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()