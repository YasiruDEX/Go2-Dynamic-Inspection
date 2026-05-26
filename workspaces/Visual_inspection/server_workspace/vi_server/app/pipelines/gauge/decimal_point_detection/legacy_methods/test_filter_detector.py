"""
Test filter-based decimal detection with REAL OCR data
"""

import cv2
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))

from decimal_point_detection.filter_based_detector import FilterBasedDecimalDetector


def load_real_ocr_results(run_folder: str):
    """Load actual OCR results from pipeline output"""
    run_path = Path(run_folder)
    
    # Check for OCR result files
    possible_files = [
        run_path / "ocr_results.json",
        run_path / "ocr_result.json", 
        run_path / "result_full.json"
    ]
    
    for file_path in possible_files:
        if file_path.exists():
            print(f"Loading OCR from: {file_path}")
            with open(file_path, 'r') as f:
                return json.load(f)
    
    return None


if __name__ == "__main__":
    # Paths
    run_folder = "runs/run_20251212212910/gauge1.jpg"
    image_path = f"{run_folder}/image_cropped.jpg"
    
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load {image_path}")
        exit(1)
    
    # Load REAL OCR data
    ocr_data = load_real_ocr_results(run_folder)
    
    if ocr_data is None:
        print("ERROR: No OCR results found!")
        print(f"Checked in: {run_folder}")
        print("Please provide the correct OCR JSON file path")
        exit(1)
    
    # Print OCR structure to see format
    print(f"\nOCR Data Preview:")
    print(json.dumps(ocr_data, indent=2)[:800])
    print("\n" + "="*60)
    
    # Parse OCR results (format depends on your pipeline)
    ocr_results = []
    if isinstance(ocr_data, list):
        ocr_results = ocr_data
    elif isinstance(ocr_data, dict):
        if 'ocr_numbers' in ocr_data:
            ocr_results = ocr_data['ocr_numbers']
        elif 'results' in ocr_data:
            ocr_results = ocr_data['results']
    
    if not ocr_results:
        print("ERROR: Could not parse OCR format!")
        print("Please show me the OCR JSON structure")
        exit(1)
    
    print(f"Found {len(ocr_results)} OCR detections\n")
    
    # Run detector
    detector = FilterBasedDecimalDetector(
        min_dot_size=1,
        max_dot_size=5,
        sensitivity=0.7
    )
    
    has_decimals, confidence, debug_image = detector.detect_decimal_points_smart(
        image, ocr_results, debug=True
    )
    
    # Print results
    print(f"\n{'='*60}")
    print(f"Decimal points detected: {has_decimals}")
    print(f"Confidence: {confidence:.2%}")
    print(f"\nOriginal reading: 19.866")
    print(f"Corrected reading: {19.866 * 0.1 if has_decimals else 19.866}")
    print(f"{'='*60}\n")
    
    # Show visualization
    if debug_image is not None:
        cv2.imshow("Smart Decimal Detection", debug_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()