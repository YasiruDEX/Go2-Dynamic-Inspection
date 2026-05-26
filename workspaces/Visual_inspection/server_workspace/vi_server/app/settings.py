"""Application settings — loaded from environment variables or .env file."""

import os
from pathlib import Path

# Load .env file if it exists (simple manual loader, no dependency needed)
_env_path = Path(__file__).parent.parent / ".env"
if _env_path.exists():
    for line in _env_path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, _, val = line.partition("=")
            os.environ.setdefault(key.strip(), val.strip())


class Settings:
    # VLM
    vlm_provider: str = os.getenv("VLM_PROVIDER", "stub")
    vlm_api_key:  str = os.getenv("VLM_API_KEY", "")
    vlm_model:    str = os.getenv("VLM_MODEL", "gpt-4o")

    # Server
    server_host: str = os.getenv("SERVER_HOST", "0.0.0.0")
    server_port: int = int(os.getenv("SERVER_PORT", "8000"))
    log_level:   str = os.getenv("LOG_LEVEL", "INFO")

    # Cloud DB (used by cloud_uploader.py)
    cloud_db_url: str = os.getenv("CLOUD_DB_URL", "")

    # Allowed object types
    allowed_object_types: list = [
        t.strip() for t in
        os.getenv("ALLOWED_OBJECT_TYPES",
                  "gauge,door,fire_extinguisher,unknown,emergency_exit,main_cylinder,person"
                  ).split(",")
    ]


settings = Settings()
