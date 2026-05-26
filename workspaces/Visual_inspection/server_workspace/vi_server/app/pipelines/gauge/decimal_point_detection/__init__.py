"""
Decimal Point Detection Module

This module provides ROI extraction and decimal point detection for analog gauges:
- NumberROIExtractor: Extract number regions for decimal analysis
- ROIPipelineIntegration: Integration with main pipeline
- DecimalPointDetector: Detect decimal points in number ROIs
- add_roi_extraction_to_pipeline: Helper function for pipeline integration
- apply_decimal_correction: Apply decimal correction to OCR values

Legacy methods (failed approaches) are archived in the legacy_methods/ folder.
"""
from .number_roi_extractor import NumberROIExtractor
from .pipeline_integration import ROIPipelineIntegration, add_roi_extraction_to_pipeline
from .decimal_detector import DecimalPointDetector, apply_decimal_correction, save_detection_results

__all__ = [
    'NumberROIExtractor',
    'ROIPipelineIntegration',
    'add_roi_extraction_to_pipeline',
    'DecimalPointDetector',
    'apply_decimal_correction',
    'save_detection_results'
]