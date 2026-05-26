
import base64
import json
import logging
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, Optional

from app.settings import settings

logger = logging.getLogger(__name__)


class VLMClient(ABC):
    """Abstract base class for VLM providers."""
    
    @abstractmethod
    def analyze_image(
        self, 
        image_path: str, 
        system_prompt: str, 
        user_prompt: str
    ) -> str:
        """
        Analyze image with VLM and return response.
        
        Args:
            image_path: Path to image file
            system_prompt: System instructions
            user_prompt: User prompt/question
            
        Returns:
            Model response as string (should be JSON)
        """
        pass


class StubVLMClient(VLMClient):
    """Stub VLM client for offline testing."""
    
    def analyze_image(
        self, 
        image_path: str, 
        system_prompt: str, 
        user_prompt: str
    ) -> str:
        """Return deterministic stub response."""
        logger.info(f"[STUB] Analyzing image: {image_path}")
        logger.debug(f"[STUB] System prompt: {system_prompt[:100]}...")
        logger.debug(f"[STUB] User prompt: {user_prompt[:100]}...")
        
        # Determine task from user prompt
        task = "unknown"
        if "fire_extinguisher" in user_prompt or "fire extinguisher" in user_prompt:
            task = "fire_extinguisher"
        elif "door" in user_prompt.lower():
            task = "door"
        elif "emergency" in user_prompt.lower():
            task = "emergency_exit"
        elif "cylinder" in user_prompt.lower() or "oil" in user_prompt.lower():
            task = "main_cylinder"
        elif "ppe" in user_prompt.lower() or "helmet" in user_prompt.lower() or "person" in user_prompt.lower():
            task = "person"
        
        # Return task-specific stub response
        stub_responses = {
            "fire_extinguisher": {
                "task": "fire_extinguisher",
                "decision": "PASS",
                "confidence": 0.85,
                "summary": "[STUB] Fire extinguisher appears present and accessible.",
                "findings": [
                    "Red cylindrical object visible in image",
                    "No obvious obstructions blocking access"
                ],
                "evidence": {"present": True, "blocked": False},
                "extracted_objects": ["fire extinguisher", "wall"]
            },
            "door": {
                "task": "door",
                "decision": "PASS",
                "confidence": 0.78,
                "summary": "[STUB] Door appears to be closed.",
                "findings": [
                    "Door panel aligned with frame",
                    "No visible gap between door and frame"
                ],
                "evidence": {"open": False},
                "extracted_objects": ["door", "frame"]
            },
            "emergency_exit": {
                "task": "emergency_exit",
                "decision": "PASS",
                "confidence": 0.82,
                "summary": "[STUB] Emergency exit appears clear and accessible.",
                "findings": [
                    "Exit pathway is visible",
                    "No objects blocking the exit"
                ],
                "evidence": {"blocked": False},
                "extracted_objects": ["emergency exit sign", "door"]
            },
            "main_cylinder": {
                "task": "main_cylinder",
                "decision": "PASS",
                "confidence": 0.75,
                "summary": "[STUB] No oil leak detected near main cylinder.",
                "findings": [
                    "Floor area appears dry",
                    "No reflective puddles visible"
                ],
                "evidence": {"oil_leak": False},
                "extracted_objects": ["cylinder", "floor"]
            },
            "person": {
                "task": "person",
                "decision": "FAIL",
                "confidence": 0.80,
                "summary": "[STUB] Person detected but PPE compliance cannot be confirmed.",
                "findings": [
                    "Person visible in image",
                    "Safety helmet not clearly visible",
                    "Gloves status unclear",
                    "Safety jacket/hi-vis vest not detected"
                ],
                "evidence": {
                    "person_detected": True,
                    "helmet_worn": False,
                    "gloves_worn": None,
                    "jacket_worn": False,
                    "ppe_compliant": False
                },
                "extracted_objects": ["person"]
            },
            "unknown": {
                "task": "unknown",
                "decision": "UNKNOWN",
                "confidence": 0.60,
                "summary": "[STUB] Generic inspection - unable to determine specific criteria.",
                "findings": [
                    "Image analyzed but no specific task criteria provided",
                    "Recommend using specific object_type for better results"
                ],
                "evidence": {},
                "extracted_objects": ["various objects"]
            }
        }
        
        response = stub_responses.get(task, stub_responses["unknown"])
        return json.dumps(response, indent=2)


class OpenAIVLMClient(VLMClient):
    """OpenAI GPT-4 Vision client."""
    
    def __init__(self, api_key: str, model: str = "gpt-4o"):
        """
        Initialize OpenAI client.
        
        Args:
            api_key: OpenAI API key
            model: Model name (default: gpt-4o for vision)
        """
        try:
            from openai import OpenAI
        except ImportError:
            raise ImportError(
                "OpenAI package not installed. Install with: pip install openai"
            )
        
        self.client = OpenAI(api_key=api_key)
        self.model = model
        logger.info(f"Initialized OpenAI VLM client with model: {model}")
    
    def _encode_image(self, image_path: str) -> str:
        """Encode image to base64."""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")
    
    def analyze_image(
        self, 
        image_path: str, 
        system_prompt: str, 
        user_prompt: str,
        max_retries: int = 1
    ) -> str:
        """
        Analyze image using OpenAI Vision API.
        
        Args:
            image_path: Path to image file
            system_prompt: System instructions
            user_prompt: User prompt/question
            max_retries: Number of retry attempts for invalid JSON
            
        Returns:
            Model response as string (JSON)
        """
        logger.info(f"Analyzing image with OpenAI: {image_path}")
        
        # Encode image
        base64_image = self._encode_image(image_path)
        
        # Determine image format
        image_ext = Path(image_path).suffix.lower()
        media_type = "image/jpeg" if image_ext in [".jpg", ".jpeg"] else "image/png"
        
        # Prepare messages
        messages = [
            {
                "role": "system",
                "content": system_prompt
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": user_prompt
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:{media_type};base64,{base64_image}"
                        }
                    }
                ]
            }
        ]
        
        # Call API
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=1000,
                temperature=0.1,  # Low temperature for consistent outputs
            )
            
            content = response.choices[0].message.content
            logger.debug(f"OpenAI response: {content[:200]}...")
            
            # Validate JSON
            try:
                json.loads(content)
                return content
            except json.JSONDecodeError as e:
                logger.warning(f"Invalid JSON response: {e}")
                
                # Retry with explicit JSON request
                if max_retries > 0:
                    logger.info("Retrying with explicit JSON request...")
                    retry_prompt = (
                        "Your previous response was not valid JSON. "
                        "Please return ONLY valid JSON matching the schema. "
                        "No markdown, no code blocks, no extra text."
                    )
                    messages.append({
                        "role": "assistant",
                        "content": content
                    })
                    messages.append({
                        "role": "user",
                        "content": retry_prompt
                    })
                    
                    retry_response = self.client.chat.completions.create(
                        model=self.model,
                        messages=messages,
                        max_tokens=1000,
                        temperature=0.1,
                    )
                    
                    retry_content = retry_response.choices[0].message.content
                    json.loads(retry_content)  # Validate
                    return retry_content
                else:
                    raise ValueError(f"Invalid JSON response: {content}")
        
        except Exception as e:
            logger.error(f"OpenAI API error: {e}", exc_info=True)
            raise


class GoogleGeminiVLMClient(VLMClient):
    """Google Gemini Vision client."""
    
    def __init__(self, api_key: str, model: str = "gemini-2.0-flash-exp"):
        """
        Initialize Google Gemini client.
        
        Args:
            api_key: Google API key
            model: Model name (default: gemini-2.0-flash-exp)
        """
        try:
            import google.generativeai as genai
        except ImportError:
            raise ImportError(
                "Google Generative AI package not installed. Install with: pip install google-generativeai"
            )
        
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(model)
        self.model_name = model
        logger.info(f"Initialized Google Gemini VLM client with model: {model}")
    
    def analyze_image(
        self, 
        image_path: str, 
        system_prompt: str, 
        user_prompt: str
    ) -> str:
        """
        Analyze image using Google Gemini Vision API.
        
        Args:
            image_path: Path to image file
            system_prompt: System instructions
            user_prompt: User prompt/question
            
        Returns:
            Model response as string (JSON)
        """
        logger.info(f"Analyzing image with Google Gemini: {image_path}")
        
        try:
            from PIL import Image
            import google.generativeai as genai
            # Removed local import of json to avoid UnboundLocalError
            
            # Load image
            img = Image.open(image_path)
            
            # Combine system and user prompts
            full_prompt = f"{system_prompt}\n\n{user_prompt}"
            
            # Call API
            # Enable JSON mode for Gemini 1.5/2.0+
            generation_config = {"response_mime_type": "application/json"}
            response = self.model.generate_content(
                [full_prompt, img], 
                generation_config=generation_config
            )
            content = response.text
            
            logger.debug(f"Gemini response: {content[:200]}...")
            
            # Validate JSON
            try:
                # Helper function to remove trailing commas which are common in LLM JSON
                def clean_json_string(s):
                    import re
                    # Remove trailing commas inside objects/arrays
                    s = re.sub(r',\s*\}', '}', s)
                    s = re.sub(r',\s*\]', ']', s)
                    return s

                try:
                    json.loads(content)
                    return content
                except json.JSONDecodeError:
                    content = clean_json_string(content)
                    json.loads(content)
                    return content

            except json.JSONDecodeError:
                # Try to extract JSON from markdown code blocks
                import re
                json_match = re.search(r'```json\s*(\{.*?\})\s*```', content, re.DOTALL)
                if json_match:
                    json_content = json_match.group(1)
                    json.loads(json_content)  # Validate
                    return json_content
                
                # Try to find JSON object
                json_match = re.search(r'\{.*\}', content, re.DOTALL)
                if json_match:
                    json_content = json_match.group(0)
                    json.loads(json_content)  # Validate
                    return json_content
                
                raise ValueError(f"Invalid JSON response: {content}")
        
        except Exception as e:
            logger.error(f"Google Gemini API error: {e}", exc_info=True)
            raise


def get_vlm_client() -> VLMClient:
    """
    Factory function to get appropriate VLM client based on settings.
    
    Returns:
        VLMClient instance
    """
    provider = settings.vlm_provider.lower()
    
    if provider == "stub":
        logger.info("Using stub VLM client")
        return StubVLMClient()
    
    elif provider == "openai":
        if not settings.vlm_api_key:
            raise ValueError("VLM_API_KEY is required for OpenAI provider")
        
        logger.info(f"Using OpenAI VLM client with model: {settings.vlm_model}")
        return OpenAIVLMClient(
            api_key=settings.vlm_api_key,
            model=settings.vlm_model
        )
    
    elif provider == "google":
        if not settings.vlm_api_key:
            raise ValueError("VLM_API_KEY is required for Google provider")
        
        logger.info(f"Using Google Gemini VLM client with model: {settings.vlm_model}")
        return GoogleGeminiVLMClient(
            api_key=settings.vlm_api_key,
            model=settings.vlm_model
        )
    
    else:
        raise ValueError(
            f"Unknown VLM provider: {provider}. "
            f"Supported providers: stub, openai, google"
        )
