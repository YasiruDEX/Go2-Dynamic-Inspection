"""
Method 1: Simple VLM ROI-Based Gauge Reading

Zero-shot visual reasoning using Vision-Language Models
"""

import json
import os
from typing import Dict, List
from PIL import Image
from tqdm import tqdm
from pathlib import Path

from vlm_client import VLMClient
from answer_parser import AnswerParser


class Method1SimpleVLM:
    """Simple VLM-based gauge reading (Method 1)"""
    
    def __init__(self, model_name: str = "gpt-4o"):
        """
        Initialize Method 1
        
        Args:
            model_name: VLM model to use (gpt-4o, gpt-5, gemini-2.5-pro, qwen2-vl-7b, llama-3.2-11b-vision)
        """
        self.model_name = model_name
        colab_url = os.getenv('COLAB_QWEN_URL')  # Get from environment
        self.vlm_client = VLMClient(model_name=model_name, colab_url=colab_url)
        self.answer_parser = AnswerParser()
        
        print(f"🚀 Method 1 initialized with {model_name}")
        
    def create_prompt(self, question: str, instrument_type: str = None) -> str:
        """
        Create structured prompt for gauge reading
        
        Args:
            question: The question (e.g., "What is the reading of the ammeter?")
            instrument_type: Optional instrument type hint (e.g., "ammeter")
            
        Returns:
            Formatted prompt string
        """
        
        prompt = f"""You are an expert at reading measuring instruments with high precision.

**Question:** {question}

**Instructions:**
Please carefully examine the image and determine the exact numerical reading by following these steps:

**Step 1 - Identify the Scale:**
- What is the full scale range? (e.g., 0-100, 0-300)
- What are the MAJOR tick mark intervals? (e.g., every 10 units, every 50 units)
- What are the MINOR tick mark intervals? (e.g., every 1 unit, every 2 units)
- How many subdivisions are between major marks?

**Step 2 - Locate the Indicator:**
- Find the pointer, needle, meniscus, or other indicator
- Identify which major tick marks it is between
- Count how many minor ticks past the lower major mark

**Step 3 - Calculate the Reading:**
- Start from the lower major tick value
- Add the contribution from minor ticks
- If the indicator is between minor ticks, interpolate

**Step 4 - Identify the Unit:**
- What unit is displayed on the scale? (A, V, ml, °C, psi, mm, cm, etc.)
- Make sure to include the correct unit symbol

**Step 5 - Provide Final Answer:**
After showing your reasoning, provide your final answer in this EXACT format:
Answer: <number> <unit>

Example: Answer: 4.4 A
Example: Answer: 66 ml
Example: Answer: 285 psi
"""

        if instrument_type:
            prompt = f"**Instrument Type:** {instrument_type}\n\n" + prompt
            
        return prompt
    
    def process_single(self, image_path: str, question: str, 
                      instrument_type: str = None) -> Dict:
        """Process a single image"""
        try:
            # Load image
            image = Image.open(image_path)
            
            # Create prompt
            prompt = self.create_prompt(question, instrument_type)
            
            # Query VLM
            vlm_response = self.vlm_client.query(image, prompt)
            
            # ADDED: Print the actual response or error
            if vlm_response is None:
                print("ERROR: VLM returned None - check vlm_client.py for actual error")
                return {
                    "question_id": Path(image_path).stem,
                    "prediction": None,
                    "predicted_value": None,
                    "predicted_unit": None,
                    "image_type": instrument_type
                }
            
            # Parse answer
            parsed = self.answer_parser.parse_answer(vlm_response)
            
            return {
                "question_id": Path(image_path).stem,
                "question": question,
                "prediction": vlm_response,
                "predicted_value": parsed.get("value"),
                "predicted_unit": parsed.get("unit"),
                "image_type": instrument_type
            }
            
        except Exception as e:
            print(f"ERROR in process_single: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def evaluate_dataset(self, dataset_json_path: str, output_dir: str = "results") -> List[Dict]:
        """
        Run Method 1 on entire dataset
        
        Args:
            dataset_json_path: Path to dataset JSON file
            output_dir: Where to save results (default: results/)
            
        Returns:
            List of result dictionaries
        """
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Load dataset
        print(f"📂 Loading dataset from: {dataset_json_path}")
        with open(dataset_json_path, 'r') as f:
            dataset = json.load(f)
        
        print(f"📊 Dataset size: {len(dataset)} samples")
        
        results = []
        
        print(f"\n🔄 Processing samples with {self.model_name}...\n")
        
        for item in tqdm(dataset, desc="Processing"):
            question_id = item['question_id']
            question = item['question']
            image_path = item['image_path']
            instrument_type = item.get('image_type')
            ground_truth = item.get('ground_truth')
            
            # Process
            result = self.process_single(image_path, question, instrument_type)
            
            # Format for evaluation
            results.append({
                "question_id": question_id,
                "question": question,
                "image_type": instrument_type,
                "prediction": result["prediction"],
                "predicted_value": result["predicted_value"],
                "predicted_unit": result["predicted_unit"],
                "ground_truth": ground_truth,
                "vlm_response": result["vlm_response"]
            })
        
        # Save results
        output_filename = f"{self.model_name.replace('/', '_')}_predictions.json"
        output_path = os.path.join(output_dir, output_filename)
        
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n✅ Results saved to: {output_path}")
        print(f"📊 Total samples processed: {len(results)}")
        
        return results