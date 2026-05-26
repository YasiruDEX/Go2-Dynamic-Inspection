from pathlib import Path
from dotenv import load_dotenv
import os

env_path = Path(__file__).resolve().parent.parent / '.env'
print(f"Looking for .env at: {env_path}")
print(f"File exists: {env_path.exists()}")

load_dotenv(dotenv_path=env_path)

key = os.environ.get("GEMINI_API_KEY")
if key:
    print(f"SUCCESS! Key loaded (first 10 chars): {key[:10]}...")
else:
    print("FAILED! Key not loaded")
    print("\nFile contents:")
    with open(env_path, 'r') as f:
        print(f.read())