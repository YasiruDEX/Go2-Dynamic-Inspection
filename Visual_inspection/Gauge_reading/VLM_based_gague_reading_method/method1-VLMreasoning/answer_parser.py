"""
Parse VLM outputs to extract numerical answers
"""

import re
from typing import Dict, Optional


class AnswerParser:
    """Extract structured answer from VLM text output"""
    
    @staticmethod
    def parse_answer(text: str) -> Dict[str, any]:
        """
        Extract numerical value and unit from VLM response
        
        Args:
            text: VLM output text
            
        Returns:
            {
                "value": float or None,
                "unit": str or None,
                "raw_answer": str or None
            }
        """
        
        # Pattern 1: "Answer: 4.4 A" or "Answer: 66 ml"
        pattern1 = r'Answer:\s*([0-9.]+)\s*([a-zA-Z°/%]+)'
        match = re.search(pattern1, text, re.IGNORECASE)
        if match:
            return {
                "value": float(match.group(1)),
                "unit": match.group(2),
                "raw_answer": f"{match.group(1)} {match.group(2)}"
            }
        
        # Pattern 2: "Final reading: 65 ml" or "Reading: 4.4 A"
        pattern2 = r'(?:Final\s+)?Reading:\s*([0-9.]+)\s*([a-zA-Z°/%]+)'
        match = re.search(pattern2, text, re.IGNORECASE)
        if match:
            return {
                "value": float(match.group(1)),
                "unit": match.group(2),
                "raw_answer": f"{match.group(1)} {match.group(2)}"
            }
        
        # Pattern 3: "approximately X unit" or "around X unit"
        pattern3 = r'(?:approximately|around|about)\s+([0-9.]+)\s*([a-zA-Z°/%]+)'
        match = re.search(pattern3, text, re.IGNORECASE)
        if match:
            return {
                "value": float(match.group(1)),
                "unit": match.group(2),
                "raw_answer": f"{match.group(1)} {match.group(2)}"
            }
        
        # Pattern 4: Just find last number + unit combination
        pattern4 = r'([0-9.]+)\s*([a-zA-Z°/%]+)'
        matches = re.findall(pattern4, text)
        if matches:
            # Take the last occurrence (usually the final answer)
            last = matches[-1]
            return {
                "value": float(last[0]),
                "unit": last[1],
                "raw_answer": f"{last[0]} {last[1]}"
            }
        
        # No match found
        return {
            "value": None,
            "unit": None,
            "raw_answer": None
        }
    
    @staticmethod
    def format_answer(value: float, unit: str) -> str:
        """Format value and unit as string"""
        if value is None or unit is None:
            return "No answer found"
        return f"{value} {unit}"