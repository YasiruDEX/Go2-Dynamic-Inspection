"""
Evaluate Method 1 predictions against ground truth
"""

import json
import argparse
from typing import Dict, List


class Method1Evaluator:
    """Evaluate gauge reading predictions"""
    
    @staticmethod
    def is_value_correct(predicted: float, ground_truth_interval: List[float]) -> bool:
        """
        Check if predicted value falls within ground truth interval
        
        Args:
            predicted: Predicted numerical value
            ground_truth_interval: [min, max] acceptable range
            
        Returns:
            True if within range, False otherwise
        """
        if predicted is None or ground_truth_interval is None:
            return False
        
        min_val, max_val = ground_truth_interval
        return min_val <= predicted <= max_val
    
    @staticmethod
    def is_unit_correct(predicted: str, ground_truth: str) -> bool:
        """Check if predicted unit matches ground truth"""
        if predicted is None or ground_truth is None:
            return False
        
        # Normalize units (handle case variations)
        pred_unit = predicted.lower().strip()
        gt_unit = ground_truth.lower().strip()
        
        return pred_unit == gt_unit
    
    def evaluate(self, predictions_path: str) -> Dict:
        """
        Evaluate predictions from JSON file
        
        Args:
            predictions_path: Path to predictions JSON
            
        Returns:
            Dict with evaluation metrics
        """
        
        # Load predictions
        with open(predictions_path, 'r') as f:
            predictions = json.load(f)
        
        total = len(predictions)
        value_correct = 0
        unit_correct = 0
        both_correct = 0
        
        detailed_results = []
        
        for pred in predictions:
            question_id = pred['question_id']
            predicted_value = pred.get('predicted_value')
            predicted_unit = pred.get('predicted_unit')
            gt = pred.get('ground_truth', {})
            gt_interval = gt.get('interval')
            gt_unit = gt.get('unit')
            
            # Check value
            value_ok = self.is_value_correct(predicted_value, gt_interval)
            # Check unit
            unit_ok = self.is_unit_correct(predicted_unit, gt_unit)
            
            if value_ok:
                value_correct += 1
            if unit_ok:
                unit_correct += 1
            if value_ok and unit_ok:
                both_correct += 1
            
            detailed_results.append({
                "question_id": question_id,
                "value_correct": value_ok,
                "unit_correct": unit_ok,
                "both_correct": value_ok and unit_ok,
                "predicted_value": predicted_value,
                "predicted_unit": predicted_unit,
                "ground_truth_interval": gt_interval,
                "ground_truth_unit": gt_unit
            })
        
        metrics = {
            "total_samples": total,
            "value_accuracy": value_correct / total if total > 0 else 0,
            "unit_accuracy": unit_correct / total if total > 0 else 0,
            "overall_accuracy": both_correct / total if total > 0 else 0,
            "value_correct_count": value_correct,
            "unit_correct_count": unit_correct,
            "both_correct_count": both_correct
        }
        
        return metrics, detailed_results


def main():
    parser = argparse.ArgumentParser(description="Evaluate Method 1 predictions")
    parser.add_argument("--predictions", type=str, required=True,
                       help="Path to predictions JSON file")
    parser.add_argument("--output", type=str, default=None,
                       help="Path to save detailed evaluation results (optional)")
    
    args = parser.parse_args()
    
    evaluator = Method1Evaluator()
    metrics, detailed = evaluator.evaluate(args.predictions)
    
    print("\n" + "="*60)
    print("METHOD 1 EVALUATION RESULTS")
    print("="*60)
    print(f"Total Samples: {metrics['total_samples']}")
    print(f"\nValue Accuracy: {metrics['value_accuracy']:.2%} ({metrics['value_correct_count']}/{metrics['total_samples']})")
    print(f"Unit Accuracy: {metrics['unit_accuracy']:.2%} ({metrics['unit_correct_count']}/{metrics['total_samples']})")
    print(f"Overall Accuracy: {metrics['overall_accuracy']:.2%} ({metrics['both_correct_count']}/{metrics['total_samples']})")
    print("="*60)
    
    if args.output:
        output_data = {
            "metrics": metrics,
            "detailed_results": detailed
        }
        with open(args.output, 'w') as f:
            json.dump(output_data, f, indent=2)
        print(f"\n✅ Detailed results saved to: {args.output}")


if __name__ == "__main__":
    main()