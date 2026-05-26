from PIL import Image, ImageDraw, ImageFont, ImageFilter, ImageEnhance
import numpy as np
import random
import os
from registry import registry
from artifacts import Artifact
from generators.utils.blender_utils import resolve_path

TEMPLATE_PATH = resolve_path("generators/electricity_meter/template.jpg")


# ------------------ Data augmentation ------------------
def _border_median_color(img: Image.Image):
    arr = np.array(img)
    top = arr[0, :, :]
    bottom = arr[-1, :, :]
    left = arr[:, 0, :]
    right = arr[:, -1, :]
    border = np.concatenate([top, bottom, left, right], axis=0)
    color = np.median(border, axis=0).astype(np.uint8)
    return int(color[0]), int(color[1]), int(color[2])


def _maybe(p: float) -> bool:
    return random.random() < p


def _add_gaussian_noise(img: Image.Image, sigma: float) -> Image.Image:
    arr = np.array(img).astype(np.int16)
    noise = np.random.normal(0.0, sigma, arr.shape).astype(np.int16)
    arr = np.clip(arr + noise, 0, 255).astype(np.uint8)
    return Image.fromarray(arr, "RGB")


def _random_shear(img: Image.Image, max_shear: float = 0.02) -> Image.Image:
    shx = random.uniform(-max_shear, max_shear)
    shy = random.uniform(-max_shear, max_shear)
    coeffs = (1.0, shx, 0.0, shy, 1.0, 0.0)
    fill = _border_median_color(img)
    return img.transform(
        img.size, Image.AFFINE, coeffs, resample=Image.BICUBIC, fillcolor=fill
    )


def _augment_image(img: Image.Image) -> Image.Image:
    w, h = img.size
    fill = _border_median_color(img)

    angle = random.uniform(-20, 20)
    img = img.rotate(angle, resample=Image.BICUBIC, expand=False, fillcolor=fill)

    # Slight shear warp
    if _maybe(0.6):
        img = _random_shear(img, max_shear=0.02)

    # Random jitter of brightness/contrast/saturation/sharpness
    img = ImageEnhance.Brightness(img).enhance(random.uniform(0.9, 1.1))
    img = ImageEnhance.Contrast(img).enhance(random.uniform(0.9, 1.12))
    img = ImageEnhance.Color(img).enhance(random.uniform(0.95, 1.08))
    img = ImageEnhance.Sharpness(img).enhance(random.uniform(0.9, 1.15))

    # Apply Gaussian noise
    if _maybe(0.6):
        sigma = random.uniform(1.0, 4.0)
        img = _add_gaussian_noise(img, sigma)

    return img


# ------------------ Utility: load DSEG font ------------------
def _load_font(px_height: int) -> ImageFont.FreeTypeFont:
    path = resolve_path("generators/electricity_meter/DSEG7Classic-Bold.ttf")
    return ImageFont.truetype(path, size=px_height)


# ------------------ Main function ------------------
@registry.register(name="electricity_meter2", tags={"electricity_meter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """
    Generate an electricity meter image with a random reading and save it to img_path.
    Returns: {"reading": <string>, "unit": "kWh"}
    """
    if not os.path.exists(TEMPLATE_PATH):
        raise FileNotFoundError(f"Template image not found: {TEMPLATE_PATH}")

    base = Image.open(TEMPLATE_PATH).convert("RGB")
    x0, x1, y0, y1 = 200, 450, 420, 480
    w, h = x1 - x0, y1 - y0

    # Row containing the upper digits (roughly 42% of LCD height)
    row_top = y0 + int(h * 0.07)
    row_bottom = y0 + int(h * 0.55)
    row_left = x0 + int(w * 0.05)  # Keep a left margin to avoid touching the edge
    row_right = x1 - int(w * 0.05)  # Right margin roughly aligned with the "kWh" area
    row_h = row_bottom - row_top

    # Sample the LCD background median color to cover the old digits (upper row only)
    screen = np.array(base)[y0:y1, x0:x1]
    bg = np.median(screen.reshape(-1, 3), axis=0).astype(np.uint8)

    # Add slight grain noise so the cover-up blends better
    H, W = row_h, row_right - row_left
    noise = (np.random.normal(0, 2.2, (H, W, 3))).astype(np.int16)
    patch = np.clip(np.full((H, W, 3), bg, dtype=np.int16) + noise, 0, 255).astype(
        np.uint8
    )
    patch_img = Image.fromarray(patch, "RGB").filter(ImageFilter.GaussianBlur(0.3))
    base.paste(patch_img, (row_left, row_top))

    # -- Render the reading with the DSEG7 font, aligned to the top-right --
    left = random.choice([5, 6, 7, 8])
    reading = f"{random.randint(0, 10**left - 1):0{left}d}.{random.randint(0, 9)}"

    font = _load_font(int(row_h))

    # Draw on a transparent canvas first for easier adjustment/blur
    canvas = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(canvas)

    # Right and top padding
    pad_r = int(W * 0.02) + 2
    pad_t = max(1, int(H * 0.08))

    # Paint onto the transparent canvas (right aligned)
    d.text((W - pad_r, pad_t), reading, font=font, fill=(0, 0, 0, 255), anchor="rt")

    # Slight blur to mimic LCD edges and reduce alpha for better blending
    canvas = canvas.filter(ImageFilter.GaussianBlur(0.25))
    alpha = canvas.split()[-1]
    alpha = alpha.point(lambda a: int(a * 0.9))
    canvas.putalpha(alpha)

    # Paste back onto the upper row
    base.paste(canvas, (row_left, row_top), canvas)

    # Slightly soften the entire LCD to remove seams
    lcd = base.crop((x0, y0, x1, y1)).filter(ImageFilter.GaussianBlur(0.5))
    base.paste(lcd, (x0, y0))

    # Random data augmentation pass
    base = _augment_image(base)

    base.save(img_path, quality=95)
    print(reading)
    return Artifact(
        data=img_path,
        image_type="electricity_meter",
        design="Digital",
        evaluator_kwargs={
            "interval": [reading, reading],
            "units": ["kWh", "kW·h", "kilowatt hour", "kilowatt-hour"],
        },
    )


if __name__ == "__main__":
    print(generate("out.jpg"))
