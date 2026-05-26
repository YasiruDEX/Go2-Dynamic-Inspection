"""
Decimal Point Detector
Uses computer vision to detect decimal points between digits on gauge faces.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional


class DecimalPointDetector:
    """
    Detects decimal points on analog gauge faces to determine if 
    OCR numbers should be scaled (e.g., 15 -> 1.5).
    """
    
    def __init__(self, 
                 min_dot_radius: int = 1,
                 max_dot_radius: int = 5,
                 confidence_threshold: float = 0.6):
        """
        Args:
            min_dot_radius: Minimum pixel radius for decimal point
            max_dot_radius: Maximum pixel radius for decimal point
            confidence_threshold: Minimum ratio of numbers with decimal points
        """
        self.min_dot_radius = min_dot_radius
        self.max_dot_radius = max_dot_radius
        self.confidence_threshold = confidence_threshold
    
    def detect_decimal_scale(self, 
                            image: np.ndarray,
                            ocr_results: List[dict],
                            debug: bool = False) -> Tuple[float, dict]:
        """
        Detect if gauge uses decimal scale by finding decimal points.
        
        Args:
            image: Cropped gauge image (BGR)
            ocr_results: List of OCR detections with 'text' and 'bbox'
                        [{'text': '15', 'bbox': [[x1,y1], [x2,y2], ...]}, ...]
            debug: If True, return debug visualization
        
        Returns:
            scale_factor: 0.1 if decimals detected, 1.0 otherwise
            debug_info: Dictionary with detection details
        """
        if len(ocr_results) < 2:
            return 1.0, {"reason": "insufficient_numbers"}
        
        decimal_points_found = 0
        total_regions_checked = 0
        debug_info = {
            "regions": [],
            "decimal_points": [],
            "confidence": 0.0
        }
        
        # Check each OCR number for decimal point
        for ocr_item in ocr_results:
            text = ocr_item.get('text', '')
            bbox = ocr_item.get('bbox', [])
            
            if not text or not bbox:
                continue
            
            # Only check numbers (skip units, labels)
            if not self._is_numeric(text):
                continue
            
            total_regions_checked += 1
            
            # Define search region for decimal point (left side of number)
            search_region = self._get_decimal_search_region(bbox, image.shape)
            if search_region is None:
                continue
            
            # Extract region
            x1, y1, x2, y2 = search_region
            roi = image[y1:y2, x1:x2]
            
            # Detect decimal point in this region
            has_decimal, dot_pos = self._detect_dot_in_region(roi)
            
            debug_info["regions"].append({
                "text": text,
                "search_region": search_region,
                "has_decimal": has_decimal
            })
            
            if has_decimal:
                decimal_points_found += 1
                debug_info["decimal_points"].append({
                    "text": text,
                    "position": (x1 + dot_pos[0], y1 + dot_pos[1]) if dot_pos else None
                })
        
        # Calculate confidence
        if total_regions_checked > 0:
            confidence = decimal_points_found / total_regions_checked
            debug_info["confidence"] = confidence
            debug_info["decimal_count"] = decimal_points_found
            debug_info["total_checked"] = total_regions_checked
            
            # If most numbers have decimal points, apply scale correction
            if confidence >= self.confidence_threshold:
                return 0.1, debug_info
        
        return 1.0, debug_info
    
    def _is_numeric(self, text: str) -> bool:
        """Check if text is a number."""
        try:
            float(text.replace('O', '0'))  # Handle OCR mistakes
            return True
        except:
            return False
    
    def _get_decimal_search_region(self, 
                                   bbox: List[List[float]], 
                                   image_shape: Tuple) -> Optional[Tuple[int, int, int, int]]:
        """
        Get the region to search for decimal point (to the left of number).
        
        Args:
            bbox: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]] polygon
            image_shape: (height, width, channels)
        
        Returns:
            (x1, y1, x2, y2) search region or None
        """
        if len(bbox) < 2:
            return None
        
        # Get bounding rectangle
        xs = [p[0] for p in bbox]
        ys = [p[1] for p in bbox]
        
        min_x, max_x = int(min(xs)), int(max(xs))
        min_y, max_y = int(min(ys)), int(max(ys))
        
        # Number width and height
        num_width = max_x - min_x
        num_height = max_y - min_y
        
        # Search region: left side of number
        # Decimal point typically appears at ~20% of digit height from bottom
        search_width = int(num_width * 0.4)  # Look left
        search_x1 = max(0, min_x - search_width)
        search_x2 = min_x + int(num_width * 0.2)  # Include slight overlap
        
        # Vertical: focus on bottom half where decimal point appears
        search_y1 = min_y + int(num_height * 0.5)
        search_y2 = max_y
        
        # Clamp to image bounds
        h, w = image_shape[:2]
        search_x1 = max(0, search_x1)
        search_x2 = min(w, search_x2)
        search_y1 = max(0, search_y1)
        search_y2 = min(h, search_y2)
        
        if search_x2 <= search_x1 or search_y2 <= search_y1:
            return None
        
        return (search_x1, search_y1, search_x2, search_y2)
    
    def _detect_dot_in_region(self, roi: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int]]]:
        """
        Detect small circular dot (decimal point) in region.
        
        Args:
            roi: Region of interest (BGR image)
        
        Returns:
            (has_dot, dot_position)
        """
        if roi.size == 0 or roi.shape[0] < 3 or roi.shape[1] < 3:
            return False, None
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        
        # Apply adaptive thresholding to handle different lighting
        binary = cv2.adaptiveThreshold(
            gray, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 
            11, 2
        )
        
        # Morphological operations to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Look for small circular contours
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1:
                continue
            
            # Get bounding circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            
            # Check if it's a small dot
            if self.min_dot_radius <= radius <= self.max_dot_radius:
                # Check circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter ** 2)
                    
                    # Decimal point should be fairly circular
                    if circularity > 0.5:
                        return True, (int(x), int(y))
        
        return False, None
    
    def visualize_detection(self, 
                           image: np.ndarray, 
                           debug_info: dict,
                           ocr_results: List[dict]) -> np.ndarray:
        """
        Create debug visualization showing search regions and detected dots.
        
        Args:
            image: Original image
            debug_info: Debug information from detect_decimal_scale
            ocr_results: OCR results
        
        Returns:
            Annotated image
        """
        vis = image.copy()
        
        # Draw search regions
        for region_info in debug_info.get("regions", []):
            x1, y1, x2, y2 = region_info["search_region"]
            color = (0, 255, 0) if region_info["has_decimal"] else (0, 0, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            
            # Label
            label = f"{region_info['text']}"
            cv2.putText(vis, label, (x1, y1-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw detected decimal points
        for dot_info in debug_info.get("decimal_points", []):
            if dot_info["position"]:
                x, y = dot_info["position"]
                cv2.circle(vis, (x, y), 5, (255, 0, 255), 2)
                cv2.circle(vis, (x, y), 2, (255, 0, 255), -1)
        
        # Add confidence text
        conf = debug_info.get("confidence", 0.0)
        text = f"Decimal Confidence: {conf:.2%}"
        cv2.putText(vis, text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return vis