"""VLM (Vision Language Model) pipeline stub.

Placeholder implementation for VLM-based inspection pipelines.
Used for object types: door, fire_extinguisher, unknown.
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


def run_vlm_pipeline(roi_path: str, object_type: str) -> Dict[str, Any]:
    """
    Execute VLM pipeline on ROI image (stub implementation).
    
    This is a STUB that returns placeholder data. Will be replaced
    with actual VLM implementation in the future.
    
    Args:
        roi_path: Absolute path to the ROI image file
        object_type: Type of object being inspected
        
    Returns:
        Dictionary containing VLM inspection results
    """
    logger.info(f"Running VLM stub for object_type={object_type} on: {roi_path}")
    
    stub_result = {
        "message": "VLM pipeline not implemented yet",
        "object_type": object_type,
        "confidence": 0.0,
        "method": "vlm_stub",
        "status": "pending_implementation"
    }
    
    logger.info(f"VLM stub result: {stub_result}")
    return stub_result


# TODO: Future VLM integration
# When implementing VLM pipeline:
# 1. Replace this stub with actual VLM model inference
# 2. Consider using models like:
#    - GPT-4 Vision API
#    - LLaVA
#    - BLIP-2
#    - Custom fine-tuned models
# 3. Ensure consistent return format with gauge pipeline
# 4. Add proper error handling and timeout management
