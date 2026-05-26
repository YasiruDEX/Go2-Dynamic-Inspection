"""
Test ROI Extractor with Real Gauge Data
This script demonstrates how to use the ROI extractor with real gauge images
by running a simplified version of the pipeline.
"""

import cv2
import numpy as np
import os
import sys
from PIL import Image

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from decimal_point_detection.number_roi_extractor import NumberROIExtractor
from decimal_point_detection.pipeline_integration import ROIPipelineIntegration
from ocr.ocr_inference import ocr
from gauge_detection.detection_inference import detection_gauge_face


def crop_image(img, box):
    """Crop image to bounding box."""
    cropped_img = img[box[1]:box[3], box[0]:box[2], :]
    height = int(box[3] - box[1])
    width = int(box[2] - box[0])
    
    # Pad to square
    if height > width:
        delta = height - width
        left, right = delta // 2, delta - (delta // 2)
        top = bottom = 0
    else:
        delta = width - height
        top, bottom = delta // 2, delta - (delta // 2)
        left = right = 0
    
    pad_color = [0, 0, 0]
    new_img = cv2.copyMakeBorder(cropped_img, top, bottom, left, right,
                                 cv2.BORDER_CONSTANT, value=pad_color)
    return new_img


def test_roi_extraction_with_real_image(image_path, detection_model_path=None):
    """
    Test ROI extraction with a real gauge image.
    
    Args:
        image_path: Path to gauge image
        detection_model_path: Path to gauge detection model (optional)
    """
    print("=" * 70)
    print("Testing ROI Extraction with Real Gauge Image")
    print("=" * 70)
    print(f"\nImage: {image_path}")
    
    # Check if image exists
    if not os.path.exists(image_path):
        print(f"Error: Image not found at {image_path}")
        return
    
    # Load image
    image = Image.open(image_path).convert("RGB")
    image = np.asarray(image)
    print(f"Image loaded: {image.shape}")
    
    # Create output directory
    output_dir = os.path.join(
        os.path.dirname(__file__), 
        "test_output_real_data"
    )
    os.makedirs(output_dir, exist_ok=True)
    
    # Step 1: Detect gauge (if model available)
    if detection_model_path and os.path.exists(detection_model_path):
        print("\nStep 1: Detecting gauge face...")
        try:
            box, all_boxes = detection_gauge_face(image, detection_model_path)
            cropped_img = crop_image(image, box)
            print(f"Gauge detected and cropped to: {cropped_img.shape}")
        except Exception as e:
            print(f"Gauge detection failed: {e}")
            print("Using full image instead...")
            cropped_img = image
    else:
        print("\nStep 1: Skipping gauge detection (no model provided)")
        print("Using full image...")
        cropped_img = image
    
    # Save cropped image
    cropped_path = os.path.join(output_dir, "01_cropped_gauge.png")
    cv2.imwrite(cropped_path, cv2.cvtColor(cropped_img, cv2.COLOR_RGB2BGR))
    print(f"Cropped image saved to: {cropped_path}")
    
    # Step 2: Run OCR
    print("\nStep 2: Running OCR...")
    try:
        ocr_readings, ocr_visualization = ocr(cropped_img, visualize=True)
        print(f"OCR detected {len(ocr_readings)} items")
        
        # Save OCR visualization
        ocr_vis_path = os.path.join(output_dir, "02_ocr_visualization.png")
        cv2.imwrite(ocr_vis_path, ocr_visualization)
        print(f"OCR visualization saved to: {ocr_vis_path}")
        
        # Print OCR results
        print("\nOCR Results:")
        for i, reading in enumerate(ocr_readings):
            print(f"  {i}: '{reading.reading}' (confidence: {reading.confidence:.2f})")
        
    except Exception as e:
        print(f"OCR failed: {e}")
        print("Cannot proceed without OCR results.")
        return
    
    # Step 3: Extract ROIs
    print("\nStep 3: Extracting number ROIs...")
    
    integrator = ROIPipelineIntegration(
        save_rois=True,
        output_dir=output_dir,
        proximity_threshold=1.5,
        padding_ratio=0.3
    )
    
    roi_results = integrator.extract_rois_from_pipeline(
        image=cropped_img,
        ocr_results=ocr_readings,
        run_path=output_dir,
        debug=True
    )
    
    # Print results
    individual_count = len(roi_results['individual']['rois'])
    grouped_count = len(roi_results['grouped']['rois'])
    
    print(f"\nExtracted {individual_count} individual ROIs (one per number label)")
    print(f"Extracted {grouped_count} grouped ROIs (optional, for advanced analysis)")
    
    print("\nIndividual ROI Details:")
    for i, info in enumerate(roi_results['individual']['roi_info']):
        print(f"\n  ROI {i}:")
        print(f"    Numbers: {', '.join(info['numbers'])}")
        print(f"    Group size: {info['group_size']}")
        print(f"    ROI shape: {info['roi_shape']}")
        print(f"    Confidences: {[f'{c:.2f}' for c in info['confidences']]}")
    
    # Step 4: Prepare for decimal detection
    print("\nStep 4: Preparing ROIs for decimal detection...")
    prepared_rois = integrator.prepare_rois_for_decimal_analysis(roi_results)
    
    print(f"Prepared {len(prepared_rois)} ROIs for decimal analysis")
    print("\nThese ROIs can now be used for:")
    print("  - Decimal point detection")
    print("  - Advanced OCR refinement")
    print("  - Number segmentation analysis")
    
    # Summary
    print("\n" + "=" * 70)
    print("Summary")
    print("=" * 70)
    print(f"Total OCR detections: {len(ocr_readings)}")
    print(f"Numeric detections: {individual_count}")
    print(f"Individual ROIs extracted: {individual_count}")
    print(f"Grouped ROIs (optional): {grouped_count}")
    print(f"\nAll outputs saved to: {output_dir}")
    print("  - 01_cropped_gauge.png: Cropped gauge image")
    print("  - 02_ocr_visualization.png: OCR results")
    print("  - number_rois_visualization.png: ROI extraction visualization")
    print("  - number_rois/individual/: Individual number ROIs (one per label)")
    print("  - number_rois/grouped/: Grouped number ROIs (optional)")
    
    return roi_results


if __name__ == "__main__":
    # Find test images
    test_images_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), 
        "test_images"
    )
    
    if not os.path.exists(test_images_dir):
        print(f"Error: Test images directory not found: {test_images_dir}")
        sys.exit(1)
    
    # Get first test image
    test_images = [f for f in os.listdir(test_images_dir) 
                   if f.endswith(('.png', '.jpg', '.jpeg'))]
    
    if not test_images:
        print(f"Error: No test images found in {test_images_dir}")
        sys.exit(1)
    
    test_image_path = os.path.join(test_images_dir, test_images[0])
    
    # Check for detection model
    detection_model_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        "models",
        "gauge_detection_model.pt"
    )
    
    if not os.path.exists(detection_model_path):
        print(f"Warning: Detection model not found at {detection_model_path}")
        detection_model_path = None
    
    # Run test
    try:
        roi_results = test_roi_extraction_with_real_image(
            image_path=test_image_path,
            detection_model_path=detection_model_path
        )
        
        print("\n" + "=" * 70)
        print("Test completed successfully!")
        print("=" * 70)
        
    except Exception as e:
        print(f"\nError during testing: {e}")
        import traceback
        traceback.print_exc()