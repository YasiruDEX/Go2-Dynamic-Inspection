import sys
import os
import json
import argparse
from pathlib import Path

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from app.pipelines.vlm.vlm_router import run_vlm_task
from app.settings import settings

def main():
    parser = argparse.ArgumentParser(description="Manual VLM Pipeline Test")
    parser.add_argument("--image", required=True, help="Path to image file")
    parser.add_argument("--type", required=True, 
                        choices=["door", "fire_extinguisher", "emergency_exit", "main_cylinder", "unknown"],
                        help="Object type to inspect")
    parser.add_argument("--provider", default=None, help="Override VLM provider (stub, openai, google)")
    parser.add_argument("--key", default=None, help="API Key (if using openai/google)")
    
    args = parser.parse_args()
    
    # Check if image exists
    if not os.path.exists(args.image):
        print(f"Error: Image not found at {args.image}")
        return

    # Override settings if provided
    if args.provider:
        settings.vlm_provider = args.provider
        print(f"Overriding provider to: {settings.vlm_provider}")
        
    if args.key:
        settings.vlm_api_key = args.key
        print("Overriding API key")

    print(f"Running VLM Task...")
    print(f"Image: {args.image}")
    print(f"Type:  {args.type}")
    print(f"Provider: {settings.vlm_provider}")
    print("-" * 40)

    try:
        result = run_vlm_task(args.type, os.path.abspath(args.image))
        print(json.dumps(result, indent=2))
        
        # Check if decision is present (indicates successful VLM run)
        if result.get("decision"):
            print(f"\n[SUCCESS] Inspection completed. Decision: {result['decision']}")
        else:
            print("\n[FAILED] Inspection failed (no decision returned).")
            
    except Exception as e:
        print(f"\n[ERROR] Exception occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
