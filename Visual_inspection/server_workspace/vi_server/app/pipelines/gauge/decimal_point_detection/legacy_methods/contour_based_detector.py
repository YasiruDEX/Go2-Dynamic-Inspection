"""
Contour-based decimal point detector
Crops number regions, applies perspective transform, finds decimal dots
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
import imutils
from imutils.perspective import four_point_transform


class ContourBasedDecimalDetector:
    """
    Detect decimal points using contour analysis on cropped number regions
    """
    
    def __init__(self,
                 min_dot_area: float = 3,
                 max_dot_area: float = 30,
                 confidence_threshold: float = 0.5):
        """
        Args:
            min_dot_area: Minimum pixel area for decimal point
            max_dot_area: Maximum pixel area for decimal point
            confidence_threshold: Minimum ratio of numbers with decimals
        """
        self.min_dot_area = min_dot_area
        self.max_dot_area = max_dot_area
        self.confidence_threshold = confidence_threshold
    
    def detect_decimals(self, 
                       image: np.ndarray,
                       ocr_results: List[dict],
                       debug: bool = False) -> Tuple[bool, float, dict]:
        """
        Detect decimal points by analyzing cropped number regions
        
        Args:
            image: Full gauge image (BGR)
            ocr_results: [{'text': '15', 'bbox': [[x,y], ...]}, ...]
            debug: Return debug info
        
        Returns:
            has_decimals: True if decimals detected
            confidence: Percentage of numbers with decimal points
            debug_info: Detection details
        """
        
        if len(ocr_results) < 2:
            return False, 0.0, {}
        
        # Filter numeric results
        numeric_results = [r for r in ocr_results if self._is_numeric(r.get('text', ''))]
        
        if len(numeric_results) < 2:
            return False, 0.0, {}
        
        decimal_found_count = 0
        total_checked = 0
        detection_results = []
        
        # Check each number region
        for ocr_item in numeric_results:
            text = ocr_item.get('text', '')
            bbox = ocr_item.get('bbox', [])
            
            if not bbox or len(bbox) < 4:
                continue
            
            total_checked += 1
            
            # Expand bbox to include decimal point area (to the left)
            expanded_bbox = self._expand_bbox_for_decimal(bbox, image.shape)
            
            # Crop and transform the region
            region = self._extract_region(image, expanded_bbox)
            
            if region is None or region.size == 0:
                detection_results.append({
                    'text': text,
                    'has_decimal': False,
                    'reason': 'invalid_region'
                })
                continue
            
            # Detect decimal point in this region
            has_dot, dot_count, processed_region = self._find_decimal_in_region(region)
            
            if has_dot:
                decimal_found_count += 1
            
            detection_results.append({
                'text': text,
                'has_decimal': has_dot,
                'dot_count': dot_count,
                'bbox': expanded_bbox,
                'processed_region': processed_region if debug else None
            })
        
        # Calculate confidence
        confidence = decimal_found_count / total_checked if total_checked > 0 else 0.0
        has_decimals = confidence >= self.confidence_threshold
        
        debug_info = {
            'decimal_count': decimal_found_count,
            'total_checked': total_checked,
            'confidence': confidence,
            'detections': detection_results
        }
        
        return has_decimals, confidence, debug_info
    
    def _is_numeric(self, text: str) -> bool:
        """Check if text is numeric"""
        try:
            float(text.replace('O', '0').replace('o', '0'))
            return True
        except:
            return False
    
    def _expand_bbox_for_decimal(self, 
                                 bbox: List[List[float]], 
                                 image_shape: Tuple) -> List[List[float]]:
        """
        Expand bounding box to the left to include decimal point area
        
        Args:
            bbox: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
            image_shape: (height, width, channels)
        
        Returns:
            Expanded bbox
        """
        # Convert to numpy array
        pts = np.array(bbox, dtype=np.float32)
        
        # Get width of number
        xs = pts[:, 0]
        width = np.max(xs) - np.min(xs)
        
        # Expand left side by 50% of width to catch decimal point
        expansion = width * 0.5
        
        # Shift left edge points
        pts[:, 0] = pts[:, 0] - expansion
        
        # Clamp to image bounds
        h, w = image_shape[:2]
        pts[:, 0] = np.clip(pts[:, 0], 0, w)
        pts[:, 1] = np.clip(pts[:, 1], 0, h)
        
        return pts.tolist()
    
    def _extract_region(self, 
                       image: np.ndarray, 
                       bbox: List[List[float]]) -> Optional[np.ndarray]:
        """
        Extract and transform the region defined by bbox
        
        Args:
            image: Full image
            bbox: Bounding box points
        
        Returns:
            Cropped and transformed region (grayscale)
        """
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Get the four corner points
            pts = np.array(bbox, dtype=np.float32)
            
            # Apply perspective transform to get clean rectangular view
            warped = four_point_transform(gray, pts)
            
            if warped is None or warped.size == 0:
                return None
            
            return warped
        
        except Exception as e:
            print(f"Warning: Failed to extract region: {e}")
            return None
    
    def _find_decimal_in_region(self, region: np.ndarray) -> Tuple[bool, int, np.ndarray]:
        """
        Find decimal point in the extracted region using contour detection
        
        Args:
            region: Grayscale cropped region
        
        Returns:
            (has_decimal, dot_count, processed_image)
        """
        # Blur to reduce noise
        blurred = cv2.GaussianBlur(region, (5, 5), 0)
        
        # Threshold to binary
        thresh = cv2.threshold(blurred, 0, 255, 
                              cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        
        # Find contours
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, 
                               cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        # Filter contours by area (decimal points are small)
        dot_count = 0
        contour_image = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        
        for c in cnts:
            area = cv2.contourArea(c)
            
            # Check if area matches decimal point size
            if self.min_dot_area <= area <= self.max_dot_area:
                # Additional check: should be roughly circular
                perimeter = cv2.arcLength(c, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter ** 2)
                    
                    if circularity > 0.4:  # Reasonably circular
                        dot_count += 1
                        # Draw on debug image
                        cv2.drawContours(contour_image, [c], 0, (0, 255, 0), 2)
        
        has_decimal = dot_count >= 1
        
        return has_decimal, dot_count, contour_image
    
    def apply_correction(self, reading: float, has_decimals: bool) -> float:
        """Apply decimal correction to reading"""
        if has_decimals:
            return reading * 0.1
        return reading
    
    def visualize_detections(self, 
                           image: np.ndarray,
                           debug_info: dict) -> np.ndarray:
        """Create visualization of detection results"""
        vis = image.copy()
        
        for detection in debug_info.get('detections', []):
            text = detection['text']
            has_decimal = detection['has_decimal']
            bbox = detection.get('bbox')
            
            if bbox:
                pts = np.array(bbox, dtype=np.int32)
                color = (0, 255, 0) if has_decimal else (0, 0, 255)
                
                # Draw bbox
                cv2.polylines(vis, [pts], True, color, 2)
                
                # Label
                label = f"{text}: {'✓ decimal' if has_decimal else '✗ no decimal'}"
                cv2.putText(vis, label, 
                           (int(np.min(pts[:, 0])), int(np.min(pts[:, 1]) - 10)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Add confidence text
        conf = debug_info.get('confidence', 0)
        cv2.putText(vis, f"Confidence: {conf:.1%}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return vis