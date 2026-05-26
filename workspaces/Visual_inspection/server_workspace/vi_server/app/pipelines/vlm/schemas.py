"""Pydantic schemas for VLM pipeline outputs."""

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field, field_validator


class VLMEvidence(BaseModel):
    """Evidence dictionary with flexible fields based on task type."""
    
    # Fire extinguisher fields
    blocked: Optional[bool] = None
    present: Optional[bool] = None

    # Door fields
    open: Optional[bool] = None

    # Main cylinder fields
    oil_leak: Optional[bool] = None
    spill_visible: Optional[bool] = None

    # Person / PPE fields
    person_detected: Optional[bool] = None
    helmet_worn: Optional[bool] = None
    gloves_worn: Optional[bool] = None
    jacket_worn: Optional[bool] = None
    ppe_compliant: Optional[bool] = None

    # Unknown / auto-detect fields
    identified_object: Optional[str] = None

    # Allow any extra fields Gemini may add
    class Config:
        extra = "allow"


class VLMResult(BaseModel):
    """Structured output from VLM reasoning task."""
    
    task: str = Field(..., description="Object type / task name")
    decision: Literal["PASS", "FAIL", "UNKNOWN"] = Field(
        ..., 
        description="Final decision: PASS (compliant), FAIL (issue found), UNKNOWN (uncertain)"
    )
    confidence: float = Field(
        ..., 
        ge=0.0, 
        le=1.0, 
        description="Confidence score between 0 and 1"
    )
    summary: str = Field(..., description="Short human-readable summary")
    findings: List[str] = Field(
        default_factory=list, 
        description="List of specific observations/findings"
    )
    evidence: Dict[str, Any] = Field(
        default_factory=dict, 
        description="Task-specific evidence (e.g., blocked, open, oil_leak)"
    )
    extracted_objects: Optional[List[str]] = Field(
        default=None, 
        description="Objects detected in the image"
    )
    vlm_raw_response: Optional[str] = Field(
        default=None,
        description="Raw response from VLM (for debugging/verification)"
    )
    
    @field_validator("decision")
    @classmethod
    def validate_decision(cls, v: str) -> str:
        """Ensure decision is uppercase."""
        return v.upper()
    
    @field_validator("confidence")
    @classmethod
    def clamp_confidence(cls, v: float) -> float:
        """Clamp confidence to [0, 1] range."""
        return max(0.0, min(1.0, v))
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON storage."""
        return self.model_dump(exclude_none=True)


class PromptConfig(BaseModel):
    """Configuration for a VLM task prompt."""
    
    system: str = Field(..., description="System instructions")
    user_template: str = Field(..., description="User prompt template with placeholders")
    decision_rules: str = Field(..., description="Decision rules for the task")
    output_schema: str = Field(..., description="Description of expected JSON output")
    
    def render_user_prompt(self, **kwargs) -> str:
        """
        Render user prompt with provided variables.
        
        Args:
            **kwargs: Variables to substitute in template (e.g., object_type, inspection_prompt)
            
        Returns:
            Rendered prompt string
        """
        prompt = self.user_template
        for key, value in kwargs.items():
            placeholder = f"{{{{{key}}}}}"
            if placeholder in prompt:
                prompt = prompt.replace(placeholder, str(value))
        return prompt
