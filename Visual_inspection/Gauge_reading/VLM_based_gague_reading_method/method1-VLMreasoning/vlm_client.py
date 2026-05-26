"""
VLM API Client for Gauge Reading
Supports: 
- API-based: GPT-4o, Gemini Models
- Local: Qwen2-VL
"""
import os
from pathlib import Path
from dotenv import load_dotenv
import google.generativeai as genai
from openai import OpenAI
import base64
from io import BytesIO
from PIL import Image

# --- ROBUST ENV LOADING ---
# This finds the .env file even if you run the script from a different folder
current_dir = Path(__file__).resolve().parent
env_path = current_dir.parent / '.env'
load_dotenv(dotenv_path=env_path)

class VLMClient:
    def __init__(self, model_name='gemini-2.5-flash', colab_url=None):
        self.model_name = model_name
        self.colab_url = colab_url
        self.client = None
        self._setup_client()

    def _setup_client(self):
        """Initialize the appropriate client based on model name"""
        if "gemini" in self.model_name:
            # ACCEPT BOTH GEMINI_API_KEY AND GOOGLE_API_KEY
            api_key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
            
            if not api_key:
                # Try to print debug info (without exposing full key)
                print(f"[ERROR] Looking for .env at: {env_path}")
                print(f"[ERROR] Current Directory: {os.getcwd()}")
                raise ValueError("GEMINI_API_KEY or GOOGLE_API_KEY not found in environment variables")
            
            genai.configure(api_key=api_key)
            print(f"[OK] Initialized Google Gemini client with model: {self.model_name}")
            self.client = genai.GenerativeModel(self.model_name)
        
        elif "gpt" in self.model_name:
            api_key = os.environ.get("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY not found in environment variables")
            
            self.client = OpenAI(api_key=api_key)
            print(f"[OK] Initialized OpenAI client with model: {self.model_name}")

    def get_response(self, image_path, prompt):
        """Get response from the VLM"""
        if "gemini" in self.model_name:
            return self._query_gemini(image_path, prompt)
        elif "gpt" in self.model_name:
            return self._query_gpt4(image_path, prompt)
        else:
            raise ValueError(f"Model {self.model_name} not supported yet")
    
    def query(self, image_path, prompt):
        """Alias for get_response"""
        return self.get_response(image_path, prompt)

    def _query_gemini(self, image_path, prompt):
        """Query Google Gemini API"""
        try:
            # FIXED: Ensure image_path is a string path, not a PIL Image
            if isinstance(image_path, str):
                img = Image.open(image_path)
            else:
                img = image_path  # Already a PIL Image
            
            response = self.client.generate_content([prompt, img])
            return response.text
        except Exception as e:
            print(f"[ERROR] Gemini API Error: {e}")
            return None

    def _query_gpt4(self, image_path, prompt):
        """Query OpenAI GPT-4o API"""
        try:
            # Encode image
            with open(image_path, "rb") as image_file:
                base64_image = base64.b64encode(image_file.read()).decode('utf-8')

            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                        ],
                    }
                ],
                max_tokens=300
            )
            return response.choices[0].message.content
        except Exception as e:
            print(f"[ERROR] GPT-4 API Error: {e}")
            return None