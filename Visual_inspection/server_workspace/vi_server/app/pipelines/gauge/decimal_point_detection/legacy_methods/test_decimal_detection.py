"""
Test script for decimal point detection.
"""

import cv2
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from decimal_point_detection import DecimalPointDetector


def test_on_image(image_path: str, ocr_results: list):
    """
    Test decimal detection on an image.
    
    Args:
        image_path: Path to gauge image
        ocr_results: List of OCR results
    """
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image {image_path}")
        return
    
    # Create detector
    detector = DecimalPointDetector(
        min_dot_radius=1,
        max_dot_radius=5,
        confidence_threshold=0.6
    )
    
    # Detect decimal scale
    scale_factor, debug_info = detector.detect_decimal_scale(
        image, ocr_results, debug=True
    )
    
    # Print results
    print(f"\n{'='*50}")
    print(f"Image: {image_path}")
    print(f"Scale Factor: {scale_factor}")
    print(f"Confidence: {debug_info.get('confidence', 0):.2%}")
    print(f"Decimal Points Found: {debug_info.get('decimal_count', 0)}/{debug_info.get('total_checked', 0)}")
    print(f"{'='*50}\n")
    
    # Visualize
    vis = detector.visualize_detection(image, debug_info, ocr_results)
    
    # Show
    cv2.imshow("Decimal Detection", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Example test
    example_ocr_results = [
        {
            'text': '05',
            'bbox': [[240, 310], [270, 310], [270, 330], [240, 330]]
        },
        {
            'text': '15',
            'bbox': [[380, 175], [410, 175], [410, 195], [380, 195]]
        },
        {
            'text': '35',
            'bbox': [[440, 340], [470, 340], [470, 360], [440, 360]]
        }
    ]
    
    test_on_image("test_images/gauge1.jpg", example_ocr_results)