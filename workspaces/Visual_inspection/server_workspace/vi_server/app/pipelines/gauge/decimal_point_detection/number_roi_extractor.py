"""
Number ROI Extractor
Extracts and crops regions of interest (ROIs) containing numbers from gauge images.
Groups adjacent numbers together and provides clean cropped images for further analysis.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
import os


class NumberROIExtractor:
    """
    Extracts ROIs of numbers from gauge images for decimal point detection.
    Can group adjacent numbers together and save individual crops.
    """
    
    def __init__(self,
                 proximity_threshold: float = 1.5,
                 padding_ratio: float = 0.3,
                 min_roi_size: int = 10):
        """
        Args:
            proximity_threshold: Distance ratio to group numbers together
                                (e.g., 1.5 means numbers within 1.5x their width are grouped)
            padding_ratio: Extra padding around ROI as ratio of ROI size
            min_roi_size: Minimum ROI dimension in pixels
        """
        self.proximity_threshold = proximity_threshold
        self.padding_ratio = padding_ratio
        self.min_roi_size = min_roi_size
    
    def extract_number_rois(self,
                           image: np.ndarray,
                           ocr_results: List,
                           group_adjacent: bool = False,
                           debug: bool = False) -> Dict:
        """
        Extract ROIs containing numbers from the gauge image.
        
        Args:
            image: Full gauge image (BGR or grayscale)
            ocr_results: List of OCRReading objects with polygon, reading, confidence
            group_adjacent: If True, group nearby numbers into single ROI. Default False extracts each number label individually.
            debug: If True, include debug visualizations
        
        Returns:
            Dictionary containing:
                - 'rois': List of cropped ROI images
                - 'roi_info': List of dicts with metadata for each ROI
                - 'visualization': Annotated image showing ROIs (if debug=True)
        """
        if not ocr_results:
            return {'rois': [], 'roi_info': [], 'visualization': None}
        
        # Filter only numeric OCR results
        numeric_results = [r for r in ocr_results if r.is_number()]
        
        if not numeric_results:
            return {'rois': [], 'roi_info': [], 'visualization': None}
        
        # Get bounding boxes for all numbers
        number_boxes = []
        for ocr_item in numeric_results:
            bbox = self._get_bounding_box_from_polygon(ocr_item.polygon)
            if bbox is not None:
                number_boxes.append({
                    'bbox': bbox,
                    'text': ocr_item.reading,
                    'confidence': ocr_item.confidence,
                    'polygon': ocr_item.polygon,
                    'center': ocr_item.center
                })
        
        if not number_boxes:
            return {'rois': [], 'roi_info': [], 'visualization': None}
        
        # Group adjacent numbers if requested
        if group_adjacent:
            grouped_boxes = self._group_adjacent_numbers(number_boxes)
        else:
            grouped_boxes = [[box] for box in number_boxes]
        
        # Extract ROIs
        rois = []
        roi_info = []
        
        for group_idx, group in enumerate(grouped_boxes):
            # Merge bounding boxes in group
            merged_bbox = self._merge_bounding_boxes([item['bbox'] for item in group])
            
            # Add padding
            padded_bbox = self._add_padding(merged_bbox, image.shape, self.padding_ratio)
            
            # Extract ROI
            roi = self._crop_roi(image, padded_bbox)
            
            if roi is not None and roi.size > 0:
                # Check minimum size
                if roi.shape[0] >= self.min_roi_size and roi.shape[1] >= self.min_roi_size:
                    rois.append(roi)
                    
                    # Store metadata
                    roi_info.append({
                        'roi_id': group_idx,
                        'bbox': padded_bbox,
                        'original_bbox': merged_bbox,
                        'numbers': [item['text'] for item in group],
                        'confidences': [item['confidence'] for item in group],
                        'group_size': len(group),
                        'roi_shape': roi.shape
                    })
        
        result = {
            'rois': rois,
            'roi_info': roi_info,
            'visualization': None
        }
        
        # Create visualization if debug mode
        if debug:
            result['visualization'] = self._create_visualization(image, roi_info)
        
        return result
    
    def save_rois(self,
                  rois: List[np.ndarray],
                  roi_info: List[Dict],
                  output_dir: str,
                  prefix: str = "roi") -> List[str]:
        """
        Save extracted ROIs to disk.
        
        Args:
            rois: List of ROI images
            roi_info: List of ROI metadata
            output_dir: Directory to save ROIs
            prefix: Filename prefix
        
        Returns:
            List of saved file paths
        """
        os.makedirs(output_dir, exist_ok=True)
        
        saved_paths = []
        for idx, (roi, info) in enumerate(zip(rois, roi_info)):
            # Create filename with number information
            numbers_str = "_".join(info['numbers'])
            filename = f"{prefix}_{idx:03d}_{numbers_str}.png"
            filepath = os.path.join(output_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, roi)
            saved_paths.append(filepath)
        
        return saved_paths
    
    def _get_bounding_box_from_polygon(self, polygon: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        Convert polygon to axis-aligned bounding box.
        
        Args:
            polygon: Nx2 array of polygon points
        
        Returns:
            (x_min, y_min, x_max, y_max) or None
        """
        if polygon is None or polygon.size == 0:
            return None
        
        x_coords = polygon[:, 0]
        y_coords = polygon[:, 1]
        
        x_min = int(np.min(x_coords))
        y_min = int(np.min(y_coords))
        x_max = int(np.max(x_coords))
        y_max = int(np.max(y_coords))
        
        return (x_min, y_min, x_max, y_max)
    
    def _group_adjacent_numbers(self, number_boxes: List[Dict]) -> List[List[Dict]]:
        """
        Group numbers that are close to each other.
        
        Args:
            number_boxes: List of number box dictionaries
        
        Returns:
            List of groups, where each group is a list of number boxes
        """
        if not number_boxes:
            return []
        
        # Sort by x-coordinate (left to right)
        sorted_boxes = sorted(number_boxes, key=lambda x: x['bbox'][0])
        
        groups = []
        current_group = [sorted_boxes[0]]
        
        for i in range(1, len(sorted_boxes)):
            prev_box = sorted_boxes[i-1]['bbox']
            curr_box = sorted_boxes[i]['bbox']
            
            # Calculate distance between boxes
            prev_right = prev_box[2]
            curr_left = curr_box[0]
            
            # Calculate average width
            prev_width = prev_box[2] - prev_box[0]
            curr_width = curr_box[2] - curr_box[0]
            avg_width = (prev_width + curr_width) / 2
            
            # Check if boxes are close enough to group
            distance = curr_left - prev_right
            
            if distance < avg_width * self.proximity_threshold:
                # Add to current group
                current_group.append(sorted_boxes[i])
            else:
                # Start new group
                groups.append(current_group)
                current_group = [sorted_boxes[i]]
        
        # Add last group
        if current_group:
            groups.append(current_group)
        
        return groups
    
    def _merge_bounding_boxes(self, bboxes: List[Tuple[int, int, int, int]]) -> Tuple[int, int, int, int]:
        """
        Merge multiple bounding boxes into one.
        
        Args:
            bboxes: List of (x_min, y_min, x_max, y_max) tuples
        
        Returns:
            Merged (x_min, y_min, x_max, y_max)
        """
        x_mins = [bbox[0] for bbox in bboxes]
        y_mins = [bbox[1] for bbox in bboxes]
        x_maxs = [bbox[2] for bbox in bboxes]
        y_maxs = [bbox[3] for bbox in bboxes]
        
        return (min(x_mins), min(y_mins), max(x_maxs), max(y_maxs))
    
    def _add_padding(self,
                     bbox: Tuple[int, int, int, int],
                     image_shape: Tuple,
                     padding_ratio: float) -> Tuple[int, int, int, int]:
        """
        Add padding around bounding box.
        
        Args:
            bbox: (x_min, y_min, x_max, y_max)
            image_shape: Shape of the image (height, width, ...)
            padding_ratio: Padding as ratio of bbox size
        
        Returns:
            Padded (x_min, y_min, x_max, y_max)
        """
        x_min, y_min, x_max, y_max = bbox
        
        width = x_max - x_min
        height = y_max - y_min
        
        pad_x = int(width * padding_ratio)
        pad_y = int(height * padding_ratio)
        
        # Apply padding
        x_min_padded = max(0, x_min - pad_x)
        y_min_padded = max(0, y_min - pad_y)
        x_max_padded = min(image_shape[1], x_max + pad_x)
        y_max_padded = min(image_shape[0], y_max + pad_y)
        
        return (x_min_padded, y_min_padded, x_max_padded, y_max_padded)
    
    def _crop_roi(self, image: np.ndarray, bbox: Tuple[int, int, int, int]) -> Optional[np.ndarray]:
        """
        Crop ROI from image.
        
        Args:
            image: Full image
            bbox: (x_min, y_min, x_max, y_max)
        
        Returns:
            Cropped ROI or None
        """
        x_min, y_min, x_max, y_max = bbox
        
        # Validate bbox
        if x_max <= x_min or y_max <= y_min:
            return None
        
        # Crop
        if len(image.shape) == 3:
            roi = image[y_min:y_max, x_min:x_max, :]
        else:
            roi = image[y_min:y_max, x_min:x_max]
        
        return roi
    
    def _create_visualization(self, image: np.ndarray, roi_info: List[Dict]) -> np.ndarray:
        """
        Create visualization showing all extracted ROIs.
        
        Args:
            image: Original image
            roi_info: List of ROI metadata
        
        Returns:
            Annotated image
        """
        # Make a copy
        vis = image.copy()
        
        # Ensure BGR format for color drawing
        if len(vis.shape) == 2:
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        
        # Draw each ROI
        for info in roi_info:
            bbox = info['bbox']
            roi_id = info['roi_id']
            numbers = info['numbers']
            
            x_min, y_min, x_max, y_max = bbox
            
            # Draw rectangle
            color = self._get_color_for_id(roi_id)
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), color, 2)
            
            # Draw label
            label = f"ROI {roi_id}: {', '.join(numbers)}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            
            # Background for text
            cv2.rectangle(vis, 
                         (x_min, y_min - label_size[1] - 10),
                         (x_min + label_size[0], y_min),
                         color, -1)
            
            # Text
            cv2.putText(vis, label, (x_min, y_min - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Add summary
        summary = f"Total ROIs: {len(roi_info)}"
        cv2.putText(vis, summary, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return vis
    
    def _get_color_for_id(self, roi_id: int) -> Tuple[int, int, int]:
        """Generate a unique color for each ROI ID."""
        colors = [
            (0, 255, 0),    # Green
            (255, 0, 0),    # Blue
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
            (128, 255, 0),  # Spring Green
            (255, 128, 0),  # Sky Blue
        ]
        return colors[roi_id % len(colors)]
