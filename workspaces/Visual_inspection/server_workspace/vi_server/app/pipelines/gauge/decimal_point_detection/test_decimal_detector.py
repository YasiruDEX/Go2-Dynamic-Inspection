"""
Test Decimal Point Detector with Real ROI Images
This script loads the extracted ROIs and detects decimal points in them.
"""

import cv2
import numpy as np
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from decimal_point_detection.decimal_detector import (
    DecimalPointDetector,
    apply_decimal_correction,
    save_detection_results
)


def load_rois_from_directory(roi_dir: str):
    """Load ROI images from directory."""
    rois = []
    roi_info = []
    
    # Get all PNG files
    files = sorted([f for f in os.listdir(roi_dir) if f.endswith('.png')])
    
    for idx, filename in enumerate(files):
        filepath = os.path.join(roi_dir, filename)
        roi_image = cv2.imread(filepath)
        
        if roi_image is not None:
            # Extract number from filename (e.g., "individual_000_35.png" -> "35")
            parts = filename.replace('.png', '').split('_')
            number = parts[-1] if len(parts) > 0 else str(idx)
            
            rois.append(roi_image)
            roi_info.append({
                'roi_id': idx,
                'numbers': [number],
                'filename': filename
            })
    
    return rois, roi_info


def test_decimal_detection():
    """Test decimal point detection on extracted ROIs."""
    print("=" * 70)
    print("Testing Decimal Point Detection")
    print("=" * 70)
    
    # Path to ROI directory
    roi_dir = os.path.join(
        os.path.dirname(__file__),
        "test_output_real_data",
        "number_rois",
        "individual"
    )
    
    if not os.path.exists(roi_dir):
        print(f"Error: ROI directory not found: {roi_dir}")
        print("Please run test_with_real_data.py first to extract ROIs.")
        return
    
    # Load ROIs
    print(f"\nLoading ROIs from: {roi_dir}")
    rois, roi_info = load_rois_from_directory(roi_dir)
    print(f"Loaded {len(rois)} ROI images")
    
    if len(rois) == 0:
        print("No ROI images found!")
        return
    
    # Initialize detector
    print("\nInitializing Decimal Point Detector...")
    detector = DecimalPointDetector(
        min_dot_area=3,
        max_dot_area=100,
        circularity_threshold=0.4,
        position_threshold=0.3
    )
    
    # Analyze ROIs
    print("\nAnalyzing ROIs for decimal points...")
    results = detector.analyze_roi_batch(rois, roi_info, debug=True)
    
    # Print results
    print("\n" + "=" * 70)
    print("Detection Results")
    print("=" * 70)
    
    decimal_count = 0
    
    for result in results:
        print(f"\nROI {result['roi_id']}: {', '.join(result['numbers'])}")
        print(f"  Filename: {roi_info[result['roi_id']]['filename']}")
        print(f"  Has Decimal: {result['has_decimal']}")
        print(f"  Confidence: {result['confidence']:.2f}")
        print(f"  Candidates Found: {result['num_candidates']}")
        
        if result['has_decimal']:
            decimal_count += 1
            # Show corrected value
            for num in result['numbers']:
                corrected = apply_decimal_correction(num, True)
                if corrected is not None:
                    print(f"  [DECIMAL] Corrected: {num} -> {corrected}")
        else:
            # No decimal detected, use original value
            for num in result['numbers']:
                corrected = apply_decimal_correction(num, False)
                if corrected is not None:
                    print(f"  Original: {num} -> {corrected}")
    
    # Summary
    print("\n" + "=" * 70)
    print("Summary")
    print("=" * 70)
    print(f"Total ROIs analyzed: {len(results)}")
    print(f"Decimal points detected: {decimal_count}")
    print(f"No decimal detected: {len(results) - decimal_count}")
    
    # Save results
    output_dir = os.path.join(
        os.path.dirname(__file__),
        "test_output_real_data",
        "decimal_detection"
    )
    
    print(f"\nSaving results to: {output_dir}")
    save_detection_results(results, output_dir, save_debug_images=True)
    
    print("\n" + "=" * 70)
    print("Decimal Detection Complete!")
    print("=" * 70)
    print("\nCheck the following files:")
    print(f"  - {os.path.join(output_dir, 'decimal_detection_summary.txt')}")
    print(f"  - {os.path.join(output_dir, 'debug_images/')} (debug visualizations)")
    
    return results


if __name__ == "__main__":
    try:
        results = test_decimal_detection()
    except Exception as e:
        print(f"\nError during testing: {e}")
        import traceback
        traceback.print_exc()
