"""VLM task router - main entry point for VLM-based reasoning."""

import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from app.pipelines.vlm.postprocess import (
    create_error_result,
    validate_and_parse_vlm_response,
)
from app.pipelines.vlm.schemas import PromptConfig
from app.pipelines.vlm.vlm_client import get_vlm_client

logger = logging.getLogger(__name__)

# Cache for loaded prompts
_PROMPT_CACHE: Dict[str, PromptConfig] = {}

# Alias map — normalises any variant name the Jetson might send to the correct prompt file
_TASK_ALIASES: Dict[str, str] = {
    "extinguisher":       "fire_extinguisher",
    "fire extinguisher":  "fire_extinguisher",
    "fire-extinguisher":  "fire_extinguisher",
    "exit":               "emergency_exit",
    "emergency exit":     "emergency_exit",
    "emergency-exit":     "emergency_exit",
    "cylinder":           "main_cylinder",
    "main cylinder":      "main_cylinder",
    "hydraulic":          "main_cylinder",
    "human":              "person",
    "people":             "person",
    "worker":             "person",
}


def load_prompt_config(task_name: str) -> PromptConfig:
    """
    Load prompt configuration for a task.
    
    Args:
        task_name: Task name (e.g., "door", "fire_extinguisher")
        
    Returns:
        PromptConfig instance
        
    Raises:
        FileNotFoundError: If prompt file doesn't exist
    """
    # Check cache
    if task_name in _PROMPT_CACHE:
        return _PROMPT_CACHE[task_name]
    
    # Load from file
    prompts_dir = Path(__file__).parent / "prompts"
    prompt_file = prompts_dir / f"{task_name}.yaml"
    
    if not prompt_file.exists():
        raise FileNotFoundError(f"Prompt file not found: {prompt_file}")
    
    logger.debug(f"Loading prompt config from: {prompt_file}")
    
    with open(prompt_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    
    # Load base rules
    base_rules_file = prompts_dir / "base_rules.md"
    if base_rules_file.exists():
        with open(base_rules_file, "r", encoding="utf-8") as f:
            base_rules = f.read()
        # Prepend base rules to system prompt
        data["system"] = f"{base_rules}\n\n{data['system']}"
    
    config = PromptConfig(**data)
    
    # Cache it
    _PROMPT_CACHE[task_name] = config
    
    return config


def run_vlm_task(
    object_type: str,
    roi_path: str,
    metadata: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Execute VLM-based reasoning task.
    
    This is the main entry point called by the queue worker.
    
    Args:
        object_type: Object type / task name
        roi_path: Absolute path to ROI image
        metadata: Optional metadata dictionary (may contain inspection_prompt)
        
    Returns:
        Dictionary with VLM result (matches VLMResult schema)
        
    Raises:
        Exception: If VLM processing fails critically
    """
    logger.info(f"Running VLM task: object_type={object_type}, roi_path={roi_path}")
    
    metadata = metadata or {}

    # Normalise object_type using alias map
    normalised = _TASK_ALIASES.get(object_type.lower().strip(), object_type.lower().strip())
    if normalised != object_type:
        logger.info(f"Alias resolved: '{object_type}' → '{normalised}'")
        object_type = normalised
    
    try:
        # Verify image exists
        if not Path(roi_path).exists():
            raise FileNotFoundError(f"ROI image not found: {roi_path}")
        
        # Load prompt configuration
        try:
            prompt_config = load_prompt_config(object_type)
        except FileNotFoundError as e:
            logger.error(f"Prompt config not found for task: {object_type}")
            return create_error_result(
                task=object_type,
                error_message=f"No prompt configuration for task: {object_type}",
                summary=f"Task '{object_type}' is not configured"
            )
        
        # Prepare prompt variables
        prompt_vars = {
            "object_type": object_type,
            "inspection_prompt": metadata.get(
                "inspection_prompt",
                "Perform general visual inspection"
            ),
            "extra_context": metadata.get("extra_context", ""),
        }
        
        # Render prompts
        system_prompt = prompt_config.system
        user_prompt = prompt_config.render_user_prompt(**prompt_vars)
        
        logger.debug(f"System prompt length: {len(system_prompt)} chars")
        logger.debug(f"User prompt length: {len(user_prompt)} chars")
        
        # Get VLM client
        vlm_client = get_vlm_client()
        
        # Call VLM
        try:
            response = vlm_client.analyze_image(
                image_path=roi_path,
                system_prompt=system_prompt,
                user_prompt=user_prompt
            )
            
            logger.debug(f"VLM raw response: {response[:300]}...")
        
        except Exception as e:
            logger.error(f"VLM client error: {e}", exc_info=True)
            return create_error_result(
                task=object_type,
                error_message=f"VLM API error: {str(e)}",
                summary="Failed to get response from VLM"
            )
        
        # Validate and parse response
        try:
            result = validate_and_parse_vlm_response(response, object_type)
            logger.info(
                f"VLM task completed: {object_type} -> {result.decision} "
                f"(confidence: {result.confidence:.2f})"
            )
            return result.to_dict()
        
        except ValueError as e:
            logger.error(f"Response validation failed: {e}")
            logger.error(f"Raw response: {response}")
            return create_error_result(
                task=object_type,
                error_message=f"Invalid VLM response format: {str(e)}",
                summary="VLM returned invalid response"
            )
    
    except Exception as e:
        logger.error(f"Unexpected error in VLM task: {e}", exc_info=True)
        return create_error_result(
            task=object_type,
            error_message=f"Unexpected error: {str(e)}",
            summary="VLM processing failed"
        )


def clear_prompt_cache():
    """Clear the prompt configuration cache (useful for testing)."""
    global _PROMPT_CACHE
    _PROMPT_CACHE.clear()
    logger.info("Prompt cache cleared")
