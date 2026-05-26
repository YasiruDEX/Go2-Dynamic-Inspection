"""
Decimal Point Detector
Analyzes number ROIs to detect if they contain decimal points.
Uses computer vision techniques to identify small dots/points in the image.
"""

import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional
import os


class DecimalPointDetector:
    """
    Detects decimal points in number ROI images using computer vision.
    """
    
    def __init__(self,
                 min_dot_area: int = 3,
                 max_dot_area: int = 100,
                 circularity_threshold: float = 0.4,
                 position_threshold: float = 0.3):
        """
        Args:
            min_dot_area: Minimum area (pixels) for a decimal point
            max_dot_area: Maximum area (pixels) for a decimal point
            circularity_threshold: Minimum circularity (0-1) for decimal point
            position_threshold: Vertical position threshold (0-1, where 0.5 is middle)
        """
        self.min_dot_area = min_dot_area
        self.max_dot_area = max_dot_area
        self.circularity_threshold = circularity_threshold
        self.position_threshold = position_threshold
    
    def detect_decimal_point(self, roi_image: np.ndarray, debug: bool = False) -> Dict:
        """
        Detect if a decimal point exists in the ROI image.
        
        Args:
            roi_image: ROI image containing a number
            debug: If True, return debug visualizations
        
        Returns:
            Dictionary with:
                - 'has_decimal': Boolean indicating if decimal point detected
                - 'confidence': Confidence score (0-1)
                - 'decimal_position': Position of decimal point if detected
                - 'debug_image': Debug visualization (if debug=True)
        """
        # Convert to grayscale if needed
        if len(roi_image.shape) == 3:
            gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi_image.copy()
        
        height, width = gray.shape
        
        # Apply binary thresholding
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Analyze contours for decimal point candidates
        decimal_candidates = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if area < self.min_dot_area or area > self.max_dot_area:
                continue
            
            # Calculate circularity
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # Filter by circularity
            if circularity < self.circularity_threshold:
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate center position
            center_x = x + w / 2
            center_y = y + h / 2
            
            # Normalize position (0-1)
            norm_x = center_x / width
            norm_y = center_y / height
            
            # Decimal points are typically in the lower-middle portion
            # and should be relatively small compared to digits
            if norm_y > self.position_threshold:  # Lower half of image
                decimal_candidates.append({
                    'contour': contour,
                    'area': area,
                    'circularity': circularity,
                    'position': (center_x, center_y),
                    'norm_position': (norm_x, norm_y),
                    'bbox': (x, y, w, h)
                })
        
        # Determine if decimal point exists
        has_decimal = len(decimal_candidates) > 0
        confidence = 0.0
        decimal_position = None
        
        if has_decimal:
            # Use the most circular, smallest candidate
            best_candidate = max(decimal_candidates, 
                                key=lambda c: c['circularity'] * (1.0 / (c['area'] + 1)))
            
            confidence = min(best_candidate['circularity'], 1.0)
            decimal_position = best_candidate['position']
        
        result = {
            'has_decimal': has_decimal,
            'confidence': confidence,
            'decimal_position': decimal_position,
            'num_candidates': len(decimal_candidates)
        }
        
        # Create debug visualization if requested
        if debug:
            debug_img = self._create_debug_visualization(
                roi_image, binary, decimal_candidates, result
            )
            result['debug_image'] = debug_img
        
        return result
    
    def analyze_roi_batch(self, 
                         rois: List[np.ndarray],
                         roi_info: List[Dict],
                         debug: bool = False) -> List[Dict]:
        """
        Analyze a batch of ROIs for decimal points.
        
        Args:
            rois: List of ROI images
            roi_info: List of ROI metadata
            debug: If True, include debug visualizations
        
        Returns:
            List of detection results for each ROI
        """
        results = []
        
        for idx, (roi, info) in enumerate(zip(rois, roi_info)):
            detection = self.detect_decimal_point(roi, debug=debug)
            
            # Combine with ROI info
            result = {
                'roi_id': info['roi_id'],
                'numbers': info['numbers'],
                'has_decimal': detection['has_decimal'],
                'confidence': detection['confidence'],
                'decimal_position': detection['decimal_position'],
                'num_candidates': detection['num_candidates']
            }
            
            if debug and 'debug_image' in detection:
                result['debug_image'] = detection['debug_image']
            
            results.append(result)
        
        return results
    
    def _create_debug_visualization(self,
                                   original: np.ndarray,
                                   binary: np.ndarray,
                                   candidates: List[Dict],
                                   result: Dict) -> np.ndarray:
        """Create debug visualization showing detection process."""
        # Convert images to BGR for visualization
        if len(original.shape) == 2:
            vis_original = cv2.cvtColor(original, cv2.COLOR_GRAY2BGR)
        else:
            vis_original = original.copy()
        
        vis_binary = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        
        # Draw candidates on original
        for candidate in candidates:
            x, y, w, h = candidate['bbox']
            cv2.rectangle(vis_original, (x, y), (x + w, y + h), (0, 255, 0), 1)
            
            # Draw circle at center
            center = (int(candidate['position'][0]), int(candidate['position'][1]))
            cv2.circle(vis_original, center, 2, (0, 0, 255), -1)
        
        # Add text
        status = "DECIMAL DETECTED" if result['has_decimal'] else "NO DECIMAL"
        color = (0, 255, 0) if result['has_decimal'] else (0, 0, 255)
        
        cv2.putText(vis_original, status, (5, 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        if result['has_decimal']:
            conf_text = f"Conf: {result['confidence']:.2f}"
            cv2.putText(vis_original, conf_text, (5, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Stack images horizontally
        debug_img = np.hstack([vis_original, vis_binary])
        
        return debug_img


def apply_decimal_correction(ocr_value: str, has_decimal: bool) -> float:
    """
    Apply decimal correction to OCR value.
    
    Args:
        ocr_value: OCR detected value (e.g., "35", "05", "15")
        has_decimal: Whether decimal point was detected
    
    Returns:
        Corrected numerical value
    """
    try:
        value = float(ocr_value)
        
        # If decimal detected, divide by 10
        if has_decimal:
            value = value / 10.0
        
        return value
    except ValueError:
        return None


def save_detection_results(results: List[Dict],
                          output_dir: str,
                          save_debug_images: bool = True):
    """
    Save decimal detection results to disk.
    
    Args:
        results: List of detection results
        output_dir: Directory to save results
        save_debug_images: Whether to save debug visualizations
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Save summary text file
    summary_path = os.path.join(output_dir, "decimal_detection_summary.txt")
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write("Decimal Point Detection Results\n")
        f.write("=" * 50 + "\n\n")
        
        for result in results:
            f.write(f"ROI {result['roi_id']}: {', '.join(result['numbers'])}\n")
            f.write(f"  Has Decimal: {result['has_decimal']}\n")
            f.write(f"  Confidence: {result['confidence']:.2f}\n")
            f.write(f"  Candidates Found: {result['num_candidates']}\n")
            
            if result['has_decimal']:
                # Apply correction
                for num in result['numbers']:
                    corrected = apply_decimal_correction(num, True)
                    if corrected is not None:
                        f.write(f"  Corrected Value: {num} -> {corrected}\n")
            
            f.write("\n")
    
    # Save debug images if available
    if save_debug_images:
        debug_dir = os.path.join(output_dir, "debug_images")
        os.makedirs(debug_dir, exist_ok=True)
        
        for result in results:
            if 'debug_image' in result:
                filename = f"debug_{result['roi_id']:03d}_{''.join(result['numbers'])}.png"
                filepath = os.path.join(debug_dir, filename)
                cv2.imwrite(filepath, result['debug_image'])
    
    print(f"Detection results saved to: {output_dir}")
    print(f"Summary: {summary_path}")
