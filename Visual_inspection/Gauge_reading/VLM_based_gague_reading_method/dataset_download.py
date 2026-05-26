"""
Download MeasureBench dataset from HuggingFace
"""

from datasets import load_dataset
import json
import os
from PIL import Image

def download_measurebench(output_dir="data/measurebench"):
    """
    Download MeasureBench dataset and save locally
    """
    os.makedirs(output_dir, exist_ok=True)
    
    print("Downloading MeasureBench from HuggingFace...")
    dataset = load_dataset("FlagEval/MeasureBench")
    
    # Get absolute path to main folder
    base_dir = os.path.abspath(os.path.dirname(__file__))
    
    # The dataset has 2 splits: real_world and synthetic
    for split_name in dataset.keys():
        print(f"\nProcessing split: {split_name}")
        split_data = dataset[split_name]
        
        split_dir = os.path.join(output_dir, split_name)
        os.makedirs(split_dir, exist_ok=True)
        os.makedirs(os.path.join(split_dir, "images"), exist_ok=True)
        
        dataset_json = []
        
        for idx, item in enumerate(split_data):
            # Save image
            image_filename = f"{split_name}_{idx:04d}.png"
            image_path = os.path.join(split_dir, "images", image_filename)
            item['image'].save(image_path)
            
            # Parse evaluator_kwargs to get ground truth
            eval_kwargs = json.loads(item['evaluator_kwargs'])
            
            # Use absolute path for images
            absolute_image_path = os.path.abspath(image_path)
            
            # Create dataset entry
            entry = {
                "question_id": f"{split_name}_{idx:04d}",
                "question": item['question'],
                "image_path": absolute_image_path,  # Changed to absolute path
                "image_type": item['image_type'],
                "design": item['design'],
                "evaluator": item['evaluator'],
                "ground_truth": {
                    "interval": eval_kwargs.get('interval'),
                    "unit": eval_kwargs.get('unit')
                },
                "meta_info": item['meta_info']
            }
            
            dataset_json.append(entry)
        
        # Save JSON
        json_path = os.path.join(split_dir, f"{split_name}.json")
        with open(json_path, 'w') as f:
            json.dump(dataset_json, f, indent=2)
            
        print(f"Saved {len(dataset_json)} samples to {json_path}")
    
    print("\n✅ Dataset download complete!")

if __name__ == "__main__":
    download_measurebench()