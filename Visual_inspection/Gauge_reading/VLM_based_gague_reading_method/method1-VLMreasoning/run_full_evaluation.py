import json
import time
import subprocess
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load the .env file FIRST
env_path = Path(__file__).resolve().parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

# Force UTF-8 just in case
os.environ["PYTHONIOENCODING"] = "utf-8"

def run_evaluation():
    # Configuration
    MODEL = "gemini-2.5-flash"
    DATASET_PATH = "../data/measurebench/real_world/real_world.json"
    OUTPUT_FILE = f"results/{MODEL}_predictions.json"
    
    # CHANGE FROM 10 TO 5 SECONDS
    DELAY_SECONDS = 2
    
    # Verify API key
    api_key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    if not api_key:
        print(f"[ERROR] API key not found!")
        return
    
    print(f"[OK] API Key loaded (first 10 chars: {api_key[:10]}...)")
    
    # Load dataset
    if not os.path.exists(DATASET_PATH):
        print(f"[ERROR] Dataset not found")
        return

    with open(DATASET_PATH, 'r') as f:
        dataset = json.load(f)
    
    print(f"[START] Evaluating {len(dataset)} images with {MODEL}")
    print(f"[CONFIG] Delay: {DELAY_SECONDS}s between requests")
    
    # Load existing predictions to know what's already done
    processed_ids = set()
    
    if os.path.exists(OUTPUT_FILE):
        try:
            with open(OUTPUT_FILE, 'r') as f:
                existing_data = json.load(f)
                processed_ids = {p['question_id'] for p in existing_data}
            print(f"[INFO] Resuming... {len(processed_ids)} already done")
        except:
            print("[WARNING] Starting fresh")

    # Process each image
    for i, item in enumerate(dataset):
        image_id = item['question_id']
        
        # Skip if already done
        if image_id in processed_ids:
            continue
            
        print(f"\n{'='*70}")
        print(f"[{i+1}/{len(dataset)}] Processing {image_id}...")
        print('='*70)
        
        # Run prediction - run_single_image.py handles all saving
        cmd = f"python run_single_image.py --image_id {image_id} --model {MODEL}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, env=os.environ.copy())
        
        if result.returncode != 0:
            print(f"[FAILURE] Error on {image_id}")
            print(result.stderr[:500])
            print("[STOP] Stopping due to error")
            sys.exit(1)
        
        # Now read the result that run_single_image.py just saved
        try:
            # Reload the predictions file to get the latest entry
            with open(OUTPUT_FILE, 'r') as f:
                all_preds = json.load(f)
            
            # Find the prediction for this image
            prediction = None
            for p in all_preds:
                if p['question_id'] == image_id:
                    prediction = p
                    break
            
            if prediction:
                pred_value = prediction.get('predicted_value')
                pred_unit = prediction.get('predicted_unit', '')
                gt = prediction.get('ground_truth', {})
                gt_interval = gt.get('interval', [None, None])
                gt_unit = gt.get('unit', '')
                
                # Display result
                print(f"[RESULT] Predicted: {pred_value} {pred_unit}")
                print(f"[RESULT] Ground Truth: [{gt_interval[0]}, {gt_interval[1]}] {gt_unit}")
                
                if pred_value is not None and gt_interval[0] is not None:
                    value_ok = gt_interval[0] <= pred_value <= gt_interval[1]
                    unit_ok = (pred_unit and gt_unit and pred_unit.lower() == gt_unit.lower())
                    status = "[CORRECT]" if (value_ok and unit_ok) else "[WRONG]"
                    print(f"[RESULT] Status: {status}")
                else:
                    print(f"[RESULT] Status: [NO PREDICTION]")
                
                print(f"[SAVED] Total in file: {len(all_preds)}")
            else:
                print(f"[WARNING] Prediction not found in file")
            
            print(f"[SUCCESS] {image_id} completed")
            
        except Exception as e:
            print(f"[WARNING] Could not read result: {e}")
            print(f"[SUCCESS] {image_id} completed")
        
        print(f"[WAIT] Sleeping {DELAY_SECONDS}s...")
        print('='*70)
        time.sleep(DELAY_SECONDS)

    print("\n" + "="*80)
    
    # Final count
    try:
        with open(OUTPUT_FILE, 'r') as f:
            final_data = json.load(f)
        print(f"[DONE] Complete! Total predictions: {len(final_data)}")
    except:
        print(f"[DONE] Complete!")
    
    print("="*80)

if __name__ == "__main__":
    run_evaluation()