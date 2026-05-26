import os
import glob
from pathlib import Path

def fix_cache():
    print("Searching for corrupted model weights...")
    
    # Common cache directories for torch/mmocr
    home = Path.home()
    search_paths = [
        home / ".cache" / "torch" / "hub" / "checkpoints",
        home / ".cache" / "mmocr",
        home / ".cache" / "mmengine",
    ]
    
    # The file causing issues
    target_filename = "dbnet_resnet18_fpnc_1200e_icdar2015_20220825_221614-7c0e94f2.pth"
    
    found = False
    for path in search_paths:
        if not path.exists():
            continue
            
        # Search recursively
        for file_path in path.rglob("*" + target_filename + "*"):
            print(f"Found potentially corrupted file: {file_path}")
            try:
                os.remove(file_path)
                print(f"Successfully deleted: {file_path}")
                found = True
            except Exception as e:
                print(f"Error deleting file: {e}")
                
        # Also look for partial downloads
        for file_path in path.rglob("*" + target_filename + "*.download"):
             print(f"Found partial download: {file_path}")
             try:
                os.remove(file_path)
                print(f"Successfully deleted: {file_path}")
                found = True
             except Exception as e:
                print(f"Error deleting file: {e}")

    if found:
        print("\nCache cleaned. Please run the pipeline test again.")
    else:
        print(f"\nCould not find file '{target_filename}' in standard cache locations.")
        print("Please manually check your cache directories.")
        print(f"Checked in: {[str(p) for p in search_paths]}")

if __name__ == "__main__":
    fix_cache()
