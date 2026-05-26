"""Image validation and processing utilities."""

from io import BytesIO
from typing import Tuple

from PIL import Image
from fastapi import HTTPException, UploadFile


ALLOWED_EXTENSIONS = {".jpg", ".jpeg", ".png"}
ALLOWED_MIME_TYPES = {"image/jpeg", "image/png"}


async def validate_image(file: UploadFile, max_size_mb: int = 10) -> None:
    """
    Validate uploaded image file.
    
    Args:
        file: Uploaded file from FastAPI
        max_size_mb: Maximum allowed file size in megabytes
        
    Raises:
        HTTPException: If validation fails
    """
    # Check content type
    if file.content_type not in ALLOWED_MIME_TYPES:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid file type. Allowed types: {', '.join(ALLOWED_MIME_TYPES)}"
        )
    
    # Read file content
    content = await file.read()
    file_size = len(content)
    
    # Check file size
    max_size_bytes = max_size_mb * 1024 * 1024
    if file_size > max_size_bytes:
        raise HTTPException(
            status_code=400,
            detail=f"File too large. Maximum size: {max_size_mb}MB"
        )
    
    # Validate image integrity
    try:
        image = Image.open(BytesIO(content))
        image.verify()
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid or corrupted image file: {str(e)}"
        )
    
    # Reset file pointer for later reading
    await file.seek(0)


def get_image_dimensions(image_path: str) -> Tuple[int, int]:
    """
    Get image dimensions.
    
    Args:
        image_path: Path to image file
        
    Returns:
        Tuple of (width, height)
    """
    with Image.open(image_path) as img:
        return img.size


def calculate_image_quality_metrics(image_path: str) -> dict:
    """
    Calculate basic image quality metrics.
    
    Args:
        image_path: Path to image file
        
    Returns:
        Dictionary with quality metrics (blur_score, brightness_mean, etc.)
    """
    # Placeholder implementation
    # In production, you might want to implement actual blur detection
    # and brightness calculation
    return {
        "blur_score": None,
        "brightness_mean": None,
    }
