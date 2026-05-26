"""
Complete Pipeline Test with Decimal Point Detection
Tests the full gauge reading pipeline with integrated decimal point detection.
"""

import os
import sys
import json
import time

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pipeline import process_image


def test_complete_pipeline():
    """Test the complete pipeline with decimal detection on test images."""
    print("=" * 80)
    print("COMPLETE GAUGE READING PIPELINE TEST")
    print("With Integrated Decimal Point Detection")
    print("=" * 80)
    
    # Paths
    test_images_dir = os.path.join(os.path.dirname(__file__), "test_images")
    models_dir = os.path.join(os.path.dirname(__file__), "models")
    
    detection_model = os.path.join(models_dir, "gauge_detection_model.pt")
    key_point_model = os.path.join(models_dir, "key_point_model.pt")
    segmentation_model = os.path.join(models_dir, "segmentation_model.pt")
    
    # Check if test images exist
    if not os.path.exists(test_images_dir):
        print(f"Error: Test images directory not found: {test_images_dir}")
        return
    
    # Get test images
    test_images = [f for f in os.listdir(test_images_dir) 
                   if f.endswith(('.png', '.jpg', '.jpeg'))]
    
    if not test_images:
        print(f"Error: No test images found in {test_images_dir}")
        return
    
    print(f"\nFound {len(test_images)} test image(s)")
    print(f"Test images directory: {test_images_dir}")
    
    # Create output directory
    time_str = time.strftime("%Y%m%d%H%M%S")
    output_base = os.path.join(os.path.dirname(__file__), "runs")
    output_dir = os.path.join(output_base, f"pipeline_test_{time_str}")
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Output directory: {output_dir}")
    print("\n" + "=" * 80)
    
    # Process each test image
    results_summary = []
    
    for idx, image_name in enumerate(test_images):
        print(f"\n[{idx + 1}/{len(test_images)}] Processing: {image_name}")
        print("-" * 80)
        
        image_path = os.path.join(test_images_dir, image_name)
        run_path = os.path.join(output_dir, image_name)
        
        try:
            # Run the complete pipeline
            process_image(
                image=image_path,
                detection_model_path=detection_model,
                key_point_model_path=key_point_model,
                segmentation_model_path=segmentation_model,
                run_path=run_path,
                debug=True,  # Enable debug mode for visualizations
                eval_mode=False,
                image_is_raw=False
            )
            
            # Read result from result.json file
            result_file = os.path.join(run_path, "result.json")
            if os.path.exists(result_file):
                with open(result_file, 'r') as f:
                    result_data = json.load(f)
                    
                if result_data and len(result_data) > 0:
                    reading_value = result_data[0].get('reading', 'N/A')
                    reading_unit = result_data[0].get('unit', 'N/A')
                else:
                    reading_value = 'N/A'
                    reading_unit = 'N/A'
            else:
                reading_value = 'N/A'
                reading_unit = 'N/A'
            
            print(f"\n[SUCCESS]")
            print(f"  Final Reading: {reading_value} {reading_unit}")
            
            results_summary.append({
                'image': image_name,
                'status': 'SUCCESS',
                'reading': reading_value,
                'unit': reading_unit,
                'output_path': run_path
            })
            
        except Exception as e:
            print(f"\n[FAILED]")
            print(f"  Error: {str(e)}")
            
            results_summary.append({
                'image': image_name,
                'status': 'FAILED',
                'error': str(e),
                'output_path': run_path
            })
    
    # Print summary
    print("\n" + "=" * 80)
    print("PIPELINE TEST SUMMARY")
    print("=" * 80)
    
    success_count = sum(1 for r in results_summary if r['status'] == 'SUCCESS')
    fail_count = len(results_summary) - success_count
    
    print(f"\nTotal Images: {len(results_summary)}")
    print(f"Successful: {success_count}")
    print(f"Failed: {fail_count}")
    
    print("\nDetailed Results:")
    print("-" * 80)
    
    for result in results_summary:
        print(f"\n{result['image']}:")
        print(f"  Status: {result['status']}")
        
        if result['status'] == 'SUCCESS':
            print(f"  Reading: {result['reading']} {result['unit']}")
        else:
            print(f"  Error: {result.get('error', 'Unknown error')}")
        
        print(f"  Output: {result['output_path']}")
    
    # Save summary to JSON
    summary_path = os.path.join(output_dir, "test_summary.json")
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(results_summary, f, indent=4)
    
    print("\n" + "=" * 80)
    print(f"Summary saved to: {summary_path}")
    print("=" * 80)
    
    # Print instructions
    print("\nNext Steps:")
    print("1. Check the output directory for detailed results and visualizations")
    print("2. Review the 'decimal_rois_visualization.png' in each run folder")
    print("3. Compare the final readings with expected values")
    print("4. Check 'result.json' in each run folder for the final reading")
    
    return results_summary


if __name__ == "__main__":
    try:
        results = test_complete_pipeline()
        print("\n[SUCCESS] Pipeline test completed successfully!")
    except Exception as e:
        print(f"\n[FAILED] Pipeline test failed: {e}")
        import traceback
        traceback.print_exc()
