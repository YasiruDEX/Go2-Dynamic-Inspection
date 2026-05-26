"""
Run inference on a single specific image
Usage:
    python run_single_image.py --image_id <id> --model <model_name>
    python run_single_image.py --image_name <filename> --model <model_name>
"""

import json
import argparse
import os
from pathlib import Path
from method1_inference import Method1SimpleVLM

def find_image_in_dataset(dataset_path, image_id=None, image_name=None):
    """Find specific image in dataset and return the full item"""
    with open(dataset_path, 'r') as f:
        dataset = json.load(f)
    
    if image_id:
        # Search by question_id
        for item in dataset:
            if item['question_id'] == image_id:
                return item
    elif image_name:
        # Search by image filename
        for item in dataset:
            if image_name.lower() in item['image_path'].lower():
                return item
    
    return None

def main():
    parser = argparse.ArgumentParser(description='Run inference on a single image')
    parser.add_argument('--dataset', type=str, 
                       default='../data/measurebench/real_world/real_world.json',
                       help='Path to dataset JSON file')
    parser.add_argument('--image_id', type=str, help='Question ID of the image (e.g., real_world_0363)')
    parser.add_argument('--image_name', type=str, help='Image filename to search for (e.g., real_world_0363.png)')
    parser.add_argument('--model', type=str, default='gemini-2.5-flash',
                       choices=[
                           'gpt-4o', 'gpt-4o-mini',
                           'gemini-3-pro-preview', 'gemini-3-pro',
                           'gemini-2.5-pro', 'gemini-2.5-flash', 'gemini-2.5-flash-lite',
                           'gemini-2.0-flash-exp', 'gemini-2.0-pro-exp-02-05',
                           'qwen2-vl-7b'
                       ],
                       help='VLM model to use')
    parser.add_argument('--output_dir', type=str, default='results',
                       help='Output directory for results')
    parser.add_argument('--colab_url', type=str,
                       help='Colab ngrok URL for Qwen (e.g., https://abc123.ngrok-free.app)')
    
    args = parser.parse_args()
    
    # Validate input
    if not args.image_id and not args.image_name:
        print("Error: Please provide either --image_id or --image_name")
        print("\nExamples:")
        print("  python run_single_image.py --image_id real_world_0363 --model gemini-2.5-flash")
        print("  python run_single_image.py --image_name real_world_0363.png --model gpt-4o")
        return
    
    # Find the image in dataset
    print(f"Searching for image in dataset: {args.dataset}")
    item = find_image_in_dataset(args.dataset, args.image_id, args.image_name)
    
    if not item:
        print(f"Error: Image not found!")
        print(f"  Searched for: image_id='{args.image_id}', image_name='{args.image_name}'")
        print("\nTip: Check the dataset file for valid question_ids")
        return
    
    # Display found image info
    print(f"\n{'='*70}")
    print(f"Found Image in Dataset:")
    print(f"{'='*70}")
    print(f"Question ID: {item['question_id']}")
    print(f"Image Path: {item['image_path']}")
    print(f"Image Type: {item.get('image_type', 'Unknown')}")
    print(f"Question: {item['question']}")
    
    # Check for ground truth
    if 'ground_truth' in item:
        gt_data = item['ground_truth']
        print(f"Ground Truth: {gt_data.get('interval', 'N/A')} {gt_data.get('unit', 'N/A')}")
    else:
        print(f"Ground Truth: Not found in dataset")
        gt_data = None
    
    print(f"{'='*70}\n")
    
    # Initialize evaluator
    print(f"Initializing {args.model} model...")
    if args.model == "qwen2-vl-7b":
        if not args.colab_url:
            print("Error: --colab_url required for Qwen model")
            print("Example: --colab_url https://abc123.ngrok-free.app")
            return
        os.environ['COLAB_QWEN_URL'] = args.colab_url
    
    evaluator = Method1SimpleVLM(model_name=args.model)
    
    # Process single image directly using process_single method
    print(f"Processing image using Method 1...")
    image_path = item['image_path']
    question = item['question']
    instrument_type = item.get('image_type', 'Unknown')
    
    result = evaluator.process_single(image_path, question, instrument_type)
    
    # Check if result is valid
    if not result or 'question_id' not in result:
        print(f"\n{'='*70}")
        print(f"ERROR: Failed to process image")
        print(f"{'='*70}")
        print(f"The VLM query failed. This could be due to:")
        print(f"  1. API rate limit exceeded (wait and retry)")
        print(f"  2. API key issues")
        print(f"  3. Network problems")
        print(f"\nTry using a different model:")
        print(f"  python run_single_image.py --image_id {item['question_id']} --model gemini-2.5-flash")
        print(f"{'='*70}\n")
        return
    
    # Add ground truth from dataset
    if 'ground_truth' in item:
        result['ground_truth'] = item['ground_truth']
    else:
        result['ground_truth'] = {'interval': [None, None], 'unit': None}
    
    # Display result
    print(f"\n{'='*70}")
    print(f"RESULT:")
    print(f"{'='*70}")
    print(f"Question ID: {result['question_id']}")
    print(f"Image Type: {result.get('image_type', 'Unknown')}")
    print(f"\n--- PREDICTION ---")
    vlm_answer = result.get('prediction', 'No answer')
    if vlm_answer:
        print(f"VLM Answer: {str(vlm_answer)[:200]}{'...' if len(str(vlm_answer)) > 200 else ''}")
    else:
        print(f"VLM Answer: No answer")
    
    print(f"\nExtracted Value: {result.get('predicted_value')}")
    print(f"Extracted Unit: {result.get('predicted_unit')}")
    
    # Display ground truth
    print(f"\n--- GROUND TRUTH ---")
    gt = result.get('ground_truth', {})
    gt_interval = gt.get('interval', [None, None])
    gt_unit = gt.get('unit', 'N/A')
    print(f"Interval: {gt_interval}")
    print(f"Unit: {gt_unit}")
    
    # Check correctness
    pred_value = result.get('predicted_value')
    pred_unit = result.get('predicted_unit')
    
    # --- EVALUATION ---
    print("\n--- EVALUATION ---")
    
    if pred_value is not None and gt_interval and gt_interval[0] is not None:
        try:
            # CONVERT ground truth to float for comparison
            gt_min = float(gt_interval[0])
            gt_max = float(gt_interval[1])
            
            value_correct = gt_min <= pred_value <= gt_max
            
            if value_correct:
                print("Value: CORRECT")
            else:
                print("Value: INCORRECT")
                print(f"  Expected: [{gt_min}, {gt_max}]")
                print(f"  Got: {pred_value}")
        except (ValueError, TypeError) as e:
            print(f"Value: Cannot compare (conversion error: {e})")
            value_correct = False
        
        # Unit check
        if pred_unit and gt_unit:
            unit_correct = pred_unit.lower() == gt_unit.lower()
            if unit_correct:
                print("\nUnit: CORRECT")
            else:
                print("\nUnit: INCORRECT")
                print(f"  Expected: {gt_unit}")
                print(f"  Got: {pred_unit}")
        else:
            unit_correct = (gt_unit is None)  # If no ground truth unit, consider correct
            if not gt_unit:
                print("\nUnit: N/A (no ground truth unit)")
        
        # Overall
        overall_correct = value_correct and unit_correct
        print(f"\nOverall: {'PASS' if overall_correct else 'FAIL'}")
    else:
        print("Cannot evaluate: Missing prediction or ground truth")
        value_correct = False
        unit_correct = False
        overall_correct = False
    
    print(f"{'='*70}\n")
    
    # Save result
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Save individual result
    output_file = output_dir / f"single_{args.model}_{result['question_id']}.json"
    with open(output_file, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"Individual result saved to: {output_file}")
    
    # Update or append to main predictions file
    predictions_file = output_dir / f"{args.model}_predictions.json"
    if predictions_file.exists():
        with open(predictions_file, 'r') as f:
            all_predictions = json.load(f)
        
        # Check if this image already exists, update it
        existing_idx = None
        for idx, pred in enumerate(all_predictions):
            if pred['question_id'] == result['question_id']:
                existing_idx = idx
                break
        
        if existing_idx is not None:
            all_predictions[existing_idx] = result
            print(f"Updated existing prediction in {predictions_file}")
        else:
            all_predictions.append(result)
            print(f"Added new prediction to {predictions_file}")
    else:
        all_predictions = [result]
        print(f"Created new predictions file: {predictions_file}")
    
    with open(predictions_file, 'w') as f:
        json.dump(all_predictions, f, indent=2)
    
    print(f"\n{'='*70}")
    print(f"Done! You can now visualize this result in the notebook:")
    print(f"{'='*70}")
    print(f"In visualize_results.ipynb, run:")
    print(f"  check_specific_image('{result['question_id']}', search_by='id', model_name='{args.model}')")
    print(f"{'='*70}\n")

if __name__ == "__main__":
    main()