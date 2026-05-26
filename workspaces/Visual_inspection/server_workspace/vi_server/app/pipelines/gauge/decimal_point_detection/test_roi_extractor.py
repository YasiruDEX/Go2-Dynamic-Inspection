"""
Test script for NumberROIExtractor
Demonstrates ROI extraction from gauge images with OCR results
"""

import cv2
import numpy as np
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from decimal_point_detection.number_roi_extractor import NumberROIExtractor
from ocr.ocr_reading import OCRReading


def create_mock_ocr_results():
    """
    Create mock OCR results for testing.
    In real usage, these would come from the OCR pipeline.
    """
    # Mock OCR results with polygons
    mock_results = []
    
    # Number "0" at position (50, 100)
    mock_results.append(OCRReading(
        polygon=np.array([[50, 100], [70, 100], [70, 130], [50, 130]], dtype=np.float32),
        reading="0",
        confidence=0.95
    ))
    
    # Number "5" at position (150, 100)
    mock_results.append(OCRReading(
        polygon=np.array([[150, 100], [170, 100], [170, 130], [150, 130]], dtype=np.float32),
        reading="5",
        confidence=0.92
    ))
    
    # Number "10" at position (250, 100) - two digits close together
    mock_results.append(OCRReading(
        polygon=np.array([[250, 100], [280, 100], [280, 130], [250, 130]], dtype=np.float32),
        reading="10",
        confidence=0.88
    ))
    
    # Number "15" at position (350, 100)
    mock_results.append(OCRReading(
        polygon=np.array([[350, 100], [380, 100], [380, 130], [350, 130]], dtype=np.float32),
        reading="15",
        confidence=0.91
    ))
    
    # Unit "bar" at position (200, 200) - should be filtered out
    mock_results.append(OCRReading(
        polygon=np.array([[200, 200], [240, 200], [240, 220], [200, 220]], dtype=np.float32),
        reading="bar",
        confidence=0.85
    ))
    
    return mock_results


def test_with_mock_data():
    """Test ROI extraction with mock data."""
    print("=" * 60)
    print("Testing NumberROIExtractor with Mock Data")
    print("=" * 60)
    
    # Create a blank test image
    test_image = np.ones((400, 500, 3), dtype=np.uint8) * 255
    
    # Draw some mock numbers on the image
    cv2.putText(test_image, "0", (50, 125), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(test_image, "5", (150, 125), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(test_image, "10", (250, 125), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(test_image, "15", (350, 125), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(test_image, "bar", (200, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Create mock OCR results
    ocr_results = create_mock_ocr_results()
    
    # Initialize extractor
    extractor = NumberROIExtractor(
        proximity_threshold=1.5,
        padding_ratio=0.3,
        min_roi_size=10
    )
    
    # Test 1: Extract ROIs with grouping
    print("\nTest 1: Extracting ROIs with adjacent number grouping")
    print("-" * 60)
    result_grouped = extractor.extract_number_rois(
        image=test_image,
        ocr_results=ocr_results,
        group_adjacent=True,
        debug=True
    )
    
    print(f"Total ROIs extracted (grouped): {len(result_grouped['rois'])}")
    for i, info in enumerate(result_grouped['roi_info']):
        print(f"\nROI {i}:")
        print(f"  Numbers: {info['numbers']}")
        print(f"  Group size: {info['group_size']}")
        print(f"  ROI shape: {info['roi_shape']}")
        print(f"  Bounding box: {info['bbox']}")
    
    # Test 2: Extract ROIs without grouping
    print("\n" + "=" * 60)
    print("Test 2: Extracting ROIs without grouping")
    print("-" * 60)
    result_individual = extractor.extract_number_rois(
        image=test_image,
        ocr_results=ocr_results,
        group_adjacent=False,
        debug=True
    )
    
    print(f"Total ROIs extracted (individual): {len(result_individual['rois'])}")
    for i, info in enumerate(result_individual['roi_info']):
        print(f"\nROI {i}:")
        print(f"  Numbers: {info['numbers']}")
        print(f"  ROI shape: {info['roi_shape']}")
    
    # Save results
    output_dir = os.path.join(os.path.dirname(__file__), "test_output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Save visualization
    if result_grouped['visualization'] is not None:
        vis_path = os.path.join(output_dir, "roi_visualization_grouped.png")
        cv2.imwrite(vis_path, result_grouped['visualization'])
        print(f"\nVisualization saved to: {vis_path}")
    
    if result_individual['visualization'] is not None:
        vis_path = os.path.join(output_dir, "roi_visualization_individual.png")
        cv2.imwrite(vis_path, result_individual['visualization'])
        print(f"Visualization saved to: {vis_path}")
    
    # Save individual ROIs
    print("\nSaving individual ROI crops...")
    saved_paths_grouped = extractor.save_rois(
        rois=result_grouped['rois'],
        roi_info=result_grouped['roi_info'],
        output_dir=os.path.join(output_dir, "grouped"),
        prefix="grouped_roi"
    )
    
    saved_paths_individual = extractor.save_rois(
        rois=result_individual['rois'],
        roi_info=result_individual['roi_info'],
        output_dir=os.path.join(output_dir, "individual"),
        prefix="individual_roi"
    )
    
    print(f"Saved {len(saved_paths_grouped)} grouped ROIs")
    print(f"Saved {len(saved_paths_individual)} individual ROIs")
    print(f"\nAll outputs saved to: {output_dir}")
    
    return result_grouped, result_individual


def test_with_real_image(image_path):
    """
    Test with a real gauge image.
    Note: This requires running the full OCR pipeline first.
    """
    print("\n" + "=" * 60)
    print(f"Testing with real image: {image_path}")
    print("=" * 60)
    
    if not os.path.exists(image_path):
        print(f"Error: Image not found at {image_path}")
        return
    
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image from {image_path}")
        return
    
    print(f"Image loaded: {image.shape}")
    print("\nNote: To test with real OCR results, you need to:")
    print("1. Run the OCR pipeline on this image")
    print("2. Pass the OCR results to the extractor")
    print("\nFor now, this test only loads the image.")
    
    return image


if __name__ == "__main__":
    # Test with mock data
    result_grouped, result_individual = test_with_mock_data()
    
    # Optionally test with real image
    test_images_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), 
        "test_images"
    )
    
    if os.path.exists(test_images_dir):
        test_images = [f for f in os.listdir(test_images_dir) if f.endswith(('.png', '.jpg', '.jpeg'))]
        if test_images:
            test_image_path = os.path.join(test_images_dir, test_images[0])
            test_with_real_image(test_image_path)
    
    print("\n" + "=" * 60)
    print("Testing complete!")
    print("=" * 60)
