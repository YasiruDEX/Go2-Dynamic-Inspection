import sys
import json
from pathlib import Path
from app.pipelines.vlm.vlm_router import run_vlm_task

def main():
    if len(sys.argv) < 3:
        print("Usage: python test_single_vlm.py <object_class> <path_to_image>")
        print("Example: python test_single_vlm.py door C:/path/to/my_door.jpg")
        sys.exit(1)

    object_class = sys.argv[1]
    image_path = sys.argv[2]

    if not Path(image_path).exists():
        print(f"Error: Image not found at {image_path}")
        sys.exit(1)

    print(f"Running VLM inspection for '{object_class}' on image: {image_path}")
    print("Waiting for response...\n")

    result = run_vlm_task(object_class, image_path, {})
    
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    main()
