"""
Pipeline Integration for Number ROI Extraction
Demonstrates how to integrate NumberROIExtractor into the main gauge reading pipeline.
"""

import cv2
import numpy as np
import os
from typing import List, Dict, Optional

from decimal_point_detection.number_roi_extractor import NumberROIExtractor


class ROIPipelineIntegration:
    """
    Integrates ROI extraction into the gauge reading pipeline.
    Provides methods to extract number ROIs at the appropriate pipeline stage.
    """
    
    def __init__(self, 
                 save_rois: bool = False,
                 output_dir: Optional[str] = None,
                 proximity_threshold: float = 1.5,
                 padding_ratio: float = 0.3):
        """
        Args:
            save_rois: Whether to save extracted ROIs to disk
            output_dir: Directory to save ROIs (if save_rois=True)
            proximity_threshold: Distance ratio for grouping adjacent numbers
            padding_ratio: Padding around ROIs as ratio of ROI size
        """
        self.save_rois = save_rois
        self.output_dir = output_dir
        
        # Initialize ROI extractor
        self.extractor = NumberROIExtractor(
            proximity_threshold=proximity_threshold,
            padding_ratio=padding_ratio,
            min_roi_size=10
        )
    
    def extract_rois_from_pipeline(self,
                                   image: np.ndarray,
                                   ocr_results: List,
                                   run_path: str,
                                   debug: bool = False) -> Dict:
        """
        Extract number ROIs from the pipeline.
        This should be called after OCR stage in the pipeline.
        
        Args:
            image: Cropped gauge image (after gauge detection)
            ocr_results: List of OCRReading objects from OCR stage
            run_path: Path to save outputs for this run
            debug: Whether to create debug visualizations
        
        Returns:
            Dictionary with ROI extraction results
        """
        # Extract individual ROIs (one per number label)
        result_individual = self.extractor.extract_number_rois(
            image=image,
            ocr_results=ocr_results,
            group_adjacent=False,
            debug=debug
        )
        
        # Also extract grouped ROIs (for advanced analysis if needed)
        result_grouped = self.extractor.extract_number_rois(
            image=image,
            ocr_results=ocr_results,
            group_adjacent=True,
            debug=False  # Only need debug for individual
        )
        
        # Save ROIs if requested
        if self.save_rois:
            roi_output_dir = os.path.join(run_path, "number_rois")
            
            # Save individual ROIs (primary output)
            individual_dir = os.path.join(roi_output_dir, "individual")
            self.extractor.save_rois(
                rois=result_individual['rois'],
                roi_info=result_individual['roi_info'],
                output_dir=individual_dir,
                prefix="individual"
            )
            
            # Save grouped ROIs (optional, for advanced analysis)
            grouped_dir = os.path.join(roi_output_dir, "grouped")
            self.extractor.save_rois(
                rois=result_grouped['rois'],
                roi_info=result_grouped['roi_info'],
                output_dir=grouped_dir,
                prefix="grouped"
            )
        
        # Save visualization if debug mode
        if debug and result_individual['visualization'] is not None:
            vis_path = os.path.join(run_path, "number_rois_visualization.png")
            cv2.imwrite(vis_path, result_individual['visualization'])
        
        return {
            'individual': result_individual,
            'grouped': result_grouped
        }
    
    def get_roi_for_decimal_detection(self, 
                                      roi_results: Dict,
                                      number_text: str) -> Optional[np.ndarray]:
        """
        Get the ROI containing a specific number for decimal detection.
        
        Args:
            roi_results: Results from extract_rois_from_pipeline
            number_text: The number text to find (e.g., "15")
        
        Returns:
            ROI image containing the number, or None if not found
        """
        grouped_results = roi_results['grouped']
        
        for roi, info in zip(grouped_results['rois'], grouped_results['roi_info']):
            if number_text in info['numbers']:
                return roi
        
        return None
    
    def prepare_rois_for_decimal_analysis(self, roi_results: Dict) -> List[Dict]:
        """
        Prepare ROIs for decimal point analysis.
        Returns a list of ROIs with metadata for further processing.
        
        Args:
            roi_results: Results from extract_rois_from_pipeline
        
        Returns:
            List of dictionaries with ROI and metadata
        """
        prepared_rois = []
        
        individual_results = roi_results['individual']
        
        for roi, info in zip(individual_results['rois'], individual_results['roi_info']):
            prepared_rois.append({
                'roi': roi,
                'numbers': info['numbers'],
                'bbox': info['bbox'],
                'roi_id': info['roi_id'],
                'confidences': info['confidences']
            })
        
        return prepared_rois


def add_roi_extraction_to_pipeline(image, ocr_readings, run_path, debug=False):
    """
    Helper function to add ROI extraction to the existing pipeline.
    
    This can be called in pipeline.py after the OCR stage.
    
    Args:
        image: Cropped gauge image
        ocr_readings: List of OCRReading objects
        run_path: Path for saving outputs
        debug: Debug mode flag
    
    Returns:
        ROI extraction results
    
    Example usage in pipeline.py:
        # After OCR stage (around line 380)
        from decimal_point_detection.pipeline_integration import add_roi_extraction_to_pipeline
        
        roi_results = add_roi_extraction_to_pipeline(
            image=cropped_img,  # or cropped_resized_img depending on requirements
            ocr_readings=number_labels,
            run_path=run_path,
            debug=debug
        )
        
        # Now roi_results contains all extracted ROIs for decimal detection
    """
    integrator = ROIPipelineIntegration(
        save_rois=debug,  # Save ROIs when in debug mode
        output_dir=run_path
    )
    
    roi_results = integrator.extract_rois_from_pipeline(
        image=image,
        ocr_results=ocr_readings,
        run_path=run_path,
        debug=debug
    )
    
    if debug:
        print(f"Extracted {len(roi_results['individual']['rois'])} individual ROIs")
        print(f"Extracted {len(roi_results['grouped']['rois'])} grouped ROIs (optional)")
    
    return roi_results
