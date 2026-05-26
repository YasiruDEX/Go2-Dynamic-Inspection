"""
Smart filter-based decimal point detector
Only searches between detected numbers
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional


class FilterBasedDecimalDetector:
    """
    Detect decimal points ONLY in regions between numbers
    """
    
    def __init__(self, 
                 min_dot_size: int = 1,
                 max_dot_size: int = 5,
                 sensitivity: float = 0.7):
        self.min_dot_size = min_dot_size
        self.max_dot_size = max_dot_size
        self.sensitivity = sensitivity
    
    def detect_decimal_points_smart(self, 
                                    image: np.ndarray,
                                    ocr_results: List[dict],
                                    debug: bool = False) -> Tuple[bool, float, np.ndarray]:
        """
        Smart detection: Only search between number pairs
        
        Args:
            image: Gauge image (BGR)
            ocr_results: [{'text': '05', 'bbox': [[x,y], ...]}, ...]
            debug: Return debug visualization
        
        Returns:
            has_decimals: True if decimals found
            confidence: Percentage of number pairs with decimal points
            debug_image: Visualization
        """
        
        if len(ocr_results) < 2:
            return False, 0.0, None
        
        # Filter to only numeric OCR results
        numeric_results = [r for r in ocr_results if self._is_numeric(r.get('text', ''))]
        
        if len(numeric_results) < 2:
            return False, 0.0, None
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Prepare image for dot detection
        processed = self._preprocess_for_dots(gray)
        
        # Check each number for decimal point to its LEFT
        decimal_found_count = 0
        total_checked = 0
        detection_results = []
        
        for ocr_item in numeric_results:
            text = ocr_item.get('text', '')
            bbox = ocr_item.get('bbox', [])
            
            if not bbox:
                continue
            
            total_checked += 1
            
            # Define search region (LEFT side of number where decimal should be)
            search_region = self._get_decimal_search_region(bbox, image.shape)
            
            if search_region is None:
                detection_results.append({
                    'text': text,
                    'has_decimal': False,
                    'reason': 'no_search_region'
                })
                continue
            
            # Extract and search this region
            x1, y1, x2, y2 = search_region
            roi_gray = gray[y1:y2, x1:x2]
            roi_processed = processed[y1:y2, x1:x2]
            
            # Find dots in this specific region
            has_dot, dot_pos = self._find_dot_in_region(roi_processed)
            
            if has_dot:
                decimal_found_count += 1
                detection_results.append({
                    'text': text,
                    'has_decimal': True,
                    'search_region': search_region,
                    'dot_position': (x1 + dot_pos[0], y1 + dot_pos[1]) if dot_pos else None
                })
            else:
                detection_results.append({
                    'text': text,
                    'has_decimal': False,
                    'search_region': search_region
                })
        
        # Calculate confidence
        confidence = decimal_found_count / total_checked if total_checked > 0 else 0.0
        has_decimals = confidence >= 0.5  # At least 50% of numbers have decimals
        
        # Create debug visualization
        debug_image = None
        if debug:
            debug_image = self._create_smart_debug_viz(image, detection_results)
        
        return has_decimals, confidence, debug_image
    
    def _is_numeric(self, text: str) -> bool:
        """Check if text is numeric"""
        try:
            float(text.replace('O', '0').replace('o', '0'))
            return True
        except:
            return False
    
    def _preprocess_for_dots(self, gray: np.ndarray) -> np.ndarray:
        """
        Preprocess image to enhance small dots
        """
        # Apply morphological top-hat to extract small bright features
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)
        
        # Threshold to get small bright spots
        _, binary = cv2.threshold(tophat, 20, 255, cv2.THRESH_BINARY)
        
        # Remove very small noise
        kernel_clean = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_clean)
        
        return cleaned
    
    def _get_decimal_search_region(self, 
                                   bbox: List[List[float]], 
                                   image_shape: Tuple) -> Optional[Tuple[int, int, int, int]]:
        """
        Get region to LEFT of number where decimal point should be
        
        Returns:
            (x1, y1, x2, y2) or None
        """
        if len(bbox) < 2:
            return None
        
        # Get bounding box
        xs = [p[0] for p in bbox]
        ys = [p[1] for p in bbox]
        
        min_x, max_x = int(min(xs)), int(max(xs))
        min_y, max_y = int(min(ys)), int(max(ys))
        
        num_width = max_x - min_x
        num_height = max_y - min_y
        
        # Search region: LEFT of number
        # Decimal point is typically 20-50% of digit width to the left
        search_width = int(num_width * 0.6)
        search_x1 = max(0, min_x - search_width)
        search_x2 = min_x + int(num_width * 0.1)  # Small overlap
        
        # Vertical: bottom 60% of digit height (where decimal sits)
        search_y1 = min_y + int(num_height * 0.4)
        search_y2 = max_y
        
        # Clamp to image bounds
        h, w = image_shape[:2]
        search_x1 = max(0, min(search_x1, w))
        search_x2 = max(0, min(search_x2, w))
        search_y1 = max(0, min(search_y1, h))
        search_y2 = max(0, min(search_y2, h))
        
        if search_x2 <= search_x1 or search_y2 <= search_y1:
            return None
        
        return (search_x1, search_y1, search_x2, search_y2)
    
    def _find_dot_in_region(self, binary_roi: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int]]]:
        """
        Find a single small dot in the region
        
        Returns:
            (has_dot, position)
        """
        if binary_roi.size == 0 or binary_roi.shape[0] < 3 or binary_roi.shape[1] < 3:
            return False, None
        
        # Find contours
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_dot = None
        best_score = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 0.5:
                continue
            
            # Get enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            
            # Check size
            if not (self.min_dot_size <= radius <= self.max_dot_size):
                continue
            
            # Check circularity
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter ** 2)
                
                # Score this candidate (prefer more circular, smaller dots)
                score = circularity * (1.0 / (radius + 1))
                
                if circularity > self.sensitivity and score > best_score:
                    best_score = score
                    best_dot = (int(x), int(y))
        
        return (best_dot is not None), best_dot
    
    def _create_smart_debug_viz(self, image: np.ndarray, detection_results: List[dict]) -> np.ndarray:
        """
        Create debug visualization showing search regions and results
        """
        vis = image.copy()
        
        for result in detection_results:
            text = result['text']
            has_decimal = result['has_decimal']
            search_region = result.get('search_region')
            
            if search_region:
                x1, y1, x2, y2 = search_region
                
                # Color: Green if decimal found, Red if not
                color = (0, 255, 0) if has_decimal else (0, 0, 255)
                
                # Draw search box
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                
                # Label
                label = f"{text}: {'✓' if has_decimal else '✗'}"
                cv2.putText(vis, label, (x1, y1-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Draw dot position if found
                if has_decimal and result.get('dot_position'):
                    dx, dy = result['dot_position']
                    cv2.circle(vis, (dx, dy), 6, (255, 0, 255), 2)
                    cv2.circle(vis, (dx, dy), 2, (255, 0, 255), -1)
        
        return vis
    
    def apply_decimal_correction(self, reading: float, has_decimals: bool) -> float:
        """Apply correction"""
        if has_decimals:
            return reading * 0.1
        return reading