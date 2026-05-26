"""Create a sample test image for testing the server."""

from PIL import Image, ImageDraw, ImageFont
from pathlib import Path

# Create sample directory
sample_dir = Path("sample_images")
sample_dir.mkdir(exist_ok=True)

# Create a sample gauge image
img = Image.new("RGB", (400, 300), color=(240, 240, 240))
draw = ImageDraw.Draw(img)

# Draw a simple gauge representation
# Outer circle
draw.ellipse([50, 50, 350, 250], outline=(0, 0, 0), width=3)

# Inner circle
draw.ellipse([180, 120, 220, 160], fill=(50, 50, 50))

# Gauge needle (pointing to ~5.2)
draw.line([200, 140, 280, 100], fill=(255, 0, 0), width=4)

# Scale markings
for i in range(0, 11):
    angle = 180 + (i * 18)  # 0-10 scale
    import math
    x1 = 200 + 140 * math.cos(math.radians(angle))
    y1 = 150 + 90 * math.sin(math.radians(angle))
    x2 = 200 + 120 * math.cos(math.radians(angle))
    y2 = 150 + 70 * math.sin(math.radians(angle))
    draw.line([x1, y1, x2, y2], fill=(0, 0, 0), width=2)

# Add text
draw.text((180, 270), "Pressure Gauge", fill=(0, 0, 0))

# Save
gauge_path = sample_dir / "sample_gauge.jpg"
img.save(gauge_path, "JPEG", quality=95)
print(f"Created: {gauge_path}")

# Create a sample door image
img_door = Image.new("RGB", (400, 300), color=(139, 90, 60))
draw_door = ImageDraw.Draw(img_door)

# Door frame
draw_door.rectangle([50, 30, 350, 270], fill=(160, 100, 70), outline=(80, 50, 30), width=3)

# Door handle
draw_door.ellipse([300, 140, 330, 160], fill=(200, 180, 50), outline=(150, 130, 30), width=2)

# Add text
draw_door.text((160, 280), "Door", fill=(255, 255, 255))

door_path = sample_dir / "sample_door.jpg"
img_door.save(door_path, "JPEG", quality=95)
print(f"Created: {door_path}")

print("\nSample images created successfully!")
print(f"Location: {sample_dir.absolute()}")
