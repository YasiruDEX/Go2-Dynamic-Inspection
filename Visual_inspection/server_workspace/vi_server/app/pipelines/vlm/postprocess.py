"""Post-processing and validation for VLM outputs."""

import json
import logging
from typing import Any, Dict, Optional

from pydantic import ValidationError

from app.pipelines.vlm.schemas import VLMResult

logger = logging.getLogger(__name__)


def clean_json_response(response: str) -> str:
    """
    Clean VLM response to extract JSON.
    
    Handles cases where model returns JSON in markdown code blocks.
    
    Args:
        response: Raw model response
        
    Returns:
        Cleaned JSON string
    """
    response = response.strip()
    
    # Remove markdown code blocks if present
    if response.startswith("```"):
        # Find the actual JSON content
        lines = response.split("\n")
        # Skip first line (```json or ```)
        # Skip last line (```)
        if lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        response = "\n".join(lines).strip()
    
    return response


def normalize_evidence(evidence: Dict[str, Any]) -> Dict[str, Any]:
    """
    Normalize evidence dictionary.
    
    Converts string booleans to actual booleans, handles null/unknown.
    
    Args:
        evidence: Raw evidence dictionary
        
    Returns:
        Normalized evidence dictionary
    """
    normalized = {}
    
    for key, value in evidence.items():
        # Handle string booleans
        if isinstance(value, str):
            value_lower = value.lower()
            if value_lower in ["true", "yes"]:
                normalized[key] = True
            elif value_lower in ["false", "no"]:
                normalized[key] = False
            elif value_lower in ["null", "unknown", "uncertain"]:
                normalized[key] = None
            else:
                normalized[key] = value
        else:
            normalized[key] = value
    
    return normalized


def validate_and_parse_vlm_response(
    response: str,
    expected_task: str
) -> VLMResult:
    """
    Validate and parse VLM response into structured format.
    
    Args:
        response: Raw VLM response (should be JSON)
        expected_task: Expected task name for validation
        
    Returns:
        Validated VLMResult
        
    Raises:
        ValueError: If response is invalid or doesn't match schema
    """
    logger.debug(f"Validating VLM response for task: {expected_task}")
    
    # Clean response
    cleaned = clean_json_response(response)
    
    # Parse JSON
    try:
        data = json.loads(cleaned)
    except json.JSONDecodeError as e:
        logger.error(f"Failed to parse JSON: {e}")
        logger.error(f"Raw response: {response[:500]}")
        raise ValueError(f"Invalid JSON response: {str(e)}")
    
    # Normalize evidence
    if "evidence" in data and isinstance(data["evidence"], dict):
        data["evidence"] = normalize_evidence(data["evidence"])
    
    # Validate with Pydantic
    try:
        result = VLMResult(**data)
    except ValidationError as e:
        logger.error(f"Pydantic validation failed: {e}")
        logger.error(f"Data: {data}")
        raise ValueError(f"Response doesn't match schema: {str(e)}")
    
    # Verify task matches
    if result.task != expected_task:
        logger.warning(
            f"Task mismatch: expected '{expected_task}', got '{result.task}'"
        )
        # Update to expected task
        result.task = expected_task
    
    logger.info(
        f"VLM result validated: decision={result.decision}, "
        f"confidence={result.confidence:.2f}"
    )
    
    # Store raw response for verification
    result.vlm_raw_response = response
    
    return result


def create_error_result(
    task: str,
    error_message: str,
    summary: Optional[str] = None
) -> Dict[str, Any]:
    """
    Create error result when VLM processing fails.
    
    Args:
        task: Task name
        error_message: Error description
        summary: Optional summary
        
    Returns:
        Error result dictionary
    """
    return {
        "task": task,
        "decision": "UNKNOWN",
        "confidence": 0.0,
        "summary": summary or f"VLM processing failed: {error_message}",
        "findings": [f"Error: {error_message}"],
        "evidence": {},
        "extracted_objects": []
    }
