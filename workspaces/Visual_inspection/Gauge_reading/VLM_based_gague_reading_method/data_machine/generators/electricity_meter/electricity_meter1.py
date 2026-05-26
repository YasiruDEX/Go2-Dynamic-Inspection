import math
import random
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from registry import registry
from artifacts import Artifact

# ==============================================================================

# Helper Functions

# ==============================================================================


def _get_font(size: int) -> ImageFont.FreeTypeFont:
    """Attempts to load a common system font, falling back to the default."""
    try:
        return ImageFont.truetype("Arial.ttf", size)
    except IOError:
        return ImageFont.load_default()


def _random_color(min_val: int = 0, max_val: int = 255) -> Tuple[int, int, int]:
    """Generates a random RGB color."""
    return tuple(random.randint(min_val, max_val) for _ in range(3))


def _polar_to_cartesian(
    cx: float, cy: float, r: float, theta_rad: float
) -> Tuple[float, float]:
    """Converts polar coordinates to Cartesian for Pillow's coordinate system."""
    x = cx + r * math.cos(theta_rad)
    y = cy - r * math.sin(theta_rad)  # Y-axis is inverted in Pillow
    return x, y


def _add_noise(image: Image.Image, amount: float) -> Image.Image:
    """Adds Gaussian noise to the image."""
    rgb = np.array(image).astype(np.float32)
    noise = np.random.randn(*rgb.shape) * amount
    noisy_rgb = np.clip(rgb + noise, 0, 255).astype(np.uint8)
    return Image.fromarray(noisy_rgb, "RGB")


def _draw_gradient_background(
    draw: ImageDraw, size: int, c1: Tuple[int, int, int], c2: Tuple[int, int, int]
):
    """Draws a simple vertical gradient background."""
    r1, g1, b1 = c1
    r2, g2, b2 = c2
    for i in range(size):
        progress = i / size
    r = int(r1 + (r2 - r1) * progress)
    g = int(g1 + (g2 - g1) * progress)
    b = int(b1 + (b2 - b1) * progress)
    draw.line([(0, i), (size, i)], fill=(r, g, b))


def _generate_reading_and_values() -> Tuple[float, List[float]]:
    """
    Generates a random reading and the corresponding continuous values for each dial.
    The position of each dial's needle is influenced by the dials to its right.
    """
    reading_int = random.randint(100, 9899)
    sub_unit_fraction = random.uniform(0.1, 0.9)
    target_reading = reading_int + sub_unit_fraction

    d1 = (reading_int // 1000) % 10
    d2 = (reading_int // 100) % 10
    d3 = (reading_int // 10) % 10
    d4 = reading_int % 10

    # Calculate continuous values for each dial needle
    v4 = d4 + sub_unit_fraction
    v3 = d3 + v4 / 10.0
    v2 = d2 + v3 / 10.0
    v1 = d1 + v2 / 10.0

    return target_reading, [v1, v2, v3, v4]


def _draw_dial(
    draw: ImageDraw,
    center: Tuple[float, float],
    radius: float,
    value: float,
    is_clockwise: bool,
    multiplier_text: str,
    colors: Dict[str, Tuple[int, int, int]],
    font_small: ImageFont.FreeTypeFont,
    font_large: ImageFont.FreeTypeFont,
):
    """Draws a single instrument dial with its ticks, labels, and needle."""
    cx, cy = center

    # Draw dial face
    draw.ellipse(
        (cx - radius, cy - radius, cx + radius, cy + radius),
        fill=colors["dial_bg"],
        outline=colors["tick"],
        width=max(1, int(radius * 0.02)),
    )

    # Draw ticks and labels (0-9)
    for i in range(10):
        angle_rad = (math.pi / 2) - (i / 10.0) * 2 * math.pi
        if not is_clockwise:
            angle_rad = (math.pi / 2) + (i / 10.0) * 2 * math.pi

        # Ticks
        start_pt = _polar_to_cartesian(cx, cy, radius * 0.9, angle_rad)
        end_pt = _polar_to_cartesian(cx, cy, radius, angle_rad)
        draw.line(
            [start_pt, end_pt], fill=colors["tick"], width=max(1, int(radius * 0.02))
        )

        # Labels
        label_pt = _polar_to_cartesian(cx, cy, radius * 0.75, angle_rad)
        draw.text(label_pt, str(i), fill=colors["label"], font=font_small, anchor="mm")

    # Draw multiplier text
    multiplier_pos = (cx, cy + radius * 1.25)
    draw.text(
        multiplier_pos,
        multiplier_text,
        fill=colors["label"],
        font=font_small,
        anchor="mt",
    )

    # Draw needle
    needle_angle_rad = (math.pi / 2) - (value / 10.0) * 2 * math.pi
    if not is_clockwise:
        needle_angle_rad = (math.pi / 2) + (value / 10.0) * 2 * math.pi

    needle_tip = _polar_to_cartesian(cx, cy, radius * 0.85, needle_angle_rad)
    counterweight_tip = _polar_to_cartesian(
        cx, cy, radius * 0.15, needle_angle_rad + math.pi
    )
    needle_width = max(2, int(radius * 0.04))

    # Simple shadow
    shadow_offset = needle_width * 0.3
    draw.line(
        [
            (
                counterweight_tip[0] + shadow_offset,
                counterweight_tip[1] + shadow_offset,
            ),
            (needle_tip[0] + shadow_offset, needle_tip[1] + shadow_offset),
        ],
        fill=(0, 0, 0, 80),
        width=needle_width,
    )

    # Needle itself
    draw.line(
        [counterweight_tip, needle_tip], fill=colors["needle"], width=needle_width
    )

    # Central hub
    hub_radius = radius * 0.08
    draw.ellipse(
        (cx - hub_radius, cy - hub_radius, cx + hub_radius, cy + hub_radius),
        fill=colors["needle"],
    )


# ==============================================================================

# Public Entrypoint

# ==============================================================================


@registry.register(name="electricity_meter1", tags={"electricity_meter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """
    Render a synthetic Electricity Meter image and save it to img_path.

    Returns:
        A dictionary containing the design type and evaluation parameters.
    """
    # 1. DIVERSITY SETUP
    # =================
    img_size = random.choice([384, 512, 640])

    # Colors
    is_dark_theme = random.choice([True, False])
    if is_dark_theme:
        bg1, bg2 = _random_color(10, 50), _random_color(20, 60)
        face_color = _random_color(40, 80)
        dial_bg_color = _random_color(60, 100)
        tick_color = _random_color(200, 255)
        needle_color = random.choice([(255, 80, 80), (80, 255, 80), (230, 230, 50)])
    else:
        bg1, bg2 = _random_color(200, 240), _random_color(220, 255)
        face_color = _random_color(180, 220)
        dial_bg_color = _random_color(230, 255)
        tick_color = _random_color(0, 50)
        needle_color = random.choice([(200, 0, 0), (0, 0, 0)])

    colors = {
        "bg1": bg1,
        "bg2": bg2,
        "face": face_color,
        "dial_bg": dial_bg_color,
        "tick": tick_color,
        "label": tick_color,
        "needle": needle_color,
    }

    # Structure
    padding = img_size * random.uniform(0.08, 0.15)
    face_radius = img_size * random.uniform(0.02, 0.1)

    # Effects
    add_noise = random.choice([True, False])
    noise_amount = random.uniform(5, 15)
    add_glare = random.choice([True, False])
    add_screws = random.choice([True, False])

    # Typography
    font_size_small = int(img_size * 0.04)
    font_size_large = int(img_size * 0.08)
    font_small = _get_font(font_size_small)
    font_large = _get_font(font_size_large)

    # 2. IMAGE INITIALIZATION
    # =======================
    image = Image.new("RGB", (img_size, img_size))
    draw = ImageDraw.Draw(image, "RGBA")
    _draw_gradient_background(draw, img_size, colors["bg1"], colors["bg2"])

    # 3. GENERATE READING
    # ===================
    target_reading, dial_values = _generate_reading_and_values()

    # 4. DRAW METER COMPONENTS
    # ========================
    # Main faceplate
    draw.rounded_rectangle(
        (padding, padding, img_size - padding, img_size - padding * 1.5),
        radius=face_radius,
        fill=colors["face"],
        outline=(0, 0, 0, 50),
        width=2,
    )

    # Draw "kWh" unit label
    unit_text = random.choice(["kWh", "kilowatt-hour", "kilowatt-hours"])
    unit_pos = (img_size / 2, img_size - padding * 0.75)
    draw.text(unit_pos, unit_text, fill=colors["label"], font=font_large, anchor="ms")

    # Draw the four dials
    dial_y_center = img_size / 2.2
    total_dial_width = img_size - 2 * padding
    dial_spacing = total_dial_width / 4
    dial_radius = dial_spacing * 0.38
    dial_centers_x = [padding + (i + 0.5) * dial_spacing for i in range(4)]

    dial_params = [
        {"is_cw": True, "multiplier": "1000"},
        {"is_cw": False, "multiplier": "100"},
        {"is_cw": True, "multiplier": "10"},
        {"is_cw": False, "multiplier": "1"},
    ]
    # The problem description has an ambiguity: 1k vs 1000. Let's vary it.
    if random.random() > 0.5:
        dial_params[0]["multiplier"] = "1k"

    for i, p in enumerate(dial_params):
        _draw_dial(
            draw,
            center=(dial_centers_x[i], dial_y_center),
            radius=dial_radius,
            value=dial_values[i],
            is_clockwise=p["is_cw"],
            multiplier_text=p["multiplier"],
            colors=colors,
            font_small=font_small,
            font_large=font_large,
        )

    # 5. ADD DIVERSITY ARTIFACTS
    # ==========================
    if add_screws:
        screw_radius = padding * 0.1
        screw_color = tuple(int(c * 0.5) for c in colors["face"])
        corners = [
            (padding, padding),
            (img_size - padding, padding),
            (padding, img_size - padding * 1.5),
            (img_size - padding, img_size - padding * 1.5),
        ]
        for cx, cy in corners:
            draw.ellipse(
                (
                    cx - screw_radius,
                    cy - screw_radius,
                    cx + screw_radius,
                    cy + screw_radius,
                ),
                fill=screw_color,
            )

    if add_glare:
        glare_alpha = random.randint(20, 50)
        draw.ellipse(
            (img_size * -0.1, img_size * -0.1, img_size * 0.7, img_size * 0.7),
            fill=(255, 255, 255, glare_alpha),
        )

    # Remove alpha channel before noise application
    image = image.convert("RGB")

    if add_noise:
        image = _add_noise(image, noise_amount)

    # 6. SAVE AND RETURN
    # ==================
    image.save(img_path)

    evaluator_kwargs = {
        "interval": [math.floor(target_reading), math.ceil(target_reading)],
        "units": ["kWh", "kilowatt-hour", "kilowatt-hours"],
    }

    return Artifact(
        data=img_path,
        image_type="electricity_meter",
        design="Composite",
        evaluator_kwargs=evaluator_kwargs,
    )
