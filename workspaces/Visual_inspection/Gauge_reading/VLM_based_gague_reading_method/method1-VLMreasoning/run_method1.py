"""
Run Method 1 on MeasureBench Dataset

Usage:
    python run_method1.py --dataset ../data/measurebench/real_world/real_world.json --model gpt-4o
"""

import argparse
import os
from method1_inference import Method1SimpleVLM


def main():
    parser = argparse.ArgumentParser(
        description="Run Method 1: Simple VLM ROI-Based Gauge Reading"
    )
    
    parser.add_argument(
        "--dataset",
        type=str,
        required=True,
        help="Path to dataset JSON file (e.g., ../data/measurebench/real_world/real_world.json)"
    )
    
    parser.add_argument(
        "--model",
        type=str,
        default="gpt-4o",
        choices=[
            "gpt-4o", "gpt-5", 
            "gemini-2.5-pro", "gemini-2.0-flash-exp",
            "qwen2-vl-2b", "qwen2-vl-7b", "qwen2-vl-72b",
            "llama-3.2-11b-vision", "llama-3.2-90b-vision"
        ],
        help="VLM model to use"
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default="results",
        help="Output directory for results (default: results/)"
    )
    
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Limit number of samples to process (for testing)"
    )
    
    args = parser.parse_args()
    
    # Validate dataset path
    if not os.path.exists(args.dataset):
        print(f"❌ Error: Dataset file not found: {args.dataset}")
        print("\nDid you run dataset_download.py first?")
        return
    
    print("="*60)
    print("METHOD 1: Simple VLM ROI-Based Gauge Reading")
    print("="*60)
    print(f"Model: {args.model}")
    print(f"Dataset: {args.dataset}")
    print(f"Output: {args.output_dir}")
    print("="*60)
    
    # Initialize Method 1
    method1 = Method1SimpleVLM(model_name=args.model)
    
    # If limit is set, modify the dataset temporarily
    if args.limit:
        import json
        with open(args.dataset, 'r') as f:
            full_dataset = json.load(f)
        
        limited_dataset = full_dataset[:args.limit]
        
        # Save temporary limited dataset
        temp_path = args.dataset.replace('.json', f'_limit{args.limit}.json')
        with open(temp_path, 'w') as f:
            json.dump(limited_dataset, f, indent=2)
        
        print(f"\n⚠️  Limited to {args.limit} samples (saved to {temp_path})")
        dataset_path = temp_path
    else:
        dataset_path = args.dataset
    
    # Run evaluation
    try:
        results = method1.evaluate_dataset(
            dataset_json_path=dataset_path,
            output_dir=args.output_dir
        )
        
        print(f"\n✅ Method 1 completed successfully!")
        print(f"📊 Processed {len(results)} samples")
        print(f"📁 Results saved in {args.output_dir}/")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during evaluation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()