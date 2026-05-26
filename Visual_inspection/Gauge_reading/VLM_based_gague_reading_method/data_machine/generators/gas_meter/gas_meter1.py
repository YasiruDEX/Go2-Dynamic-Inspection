import math
import random
from PIL import Image, ImageDraw, ImageFont, ImageFilter, ImageOps
import numpy as np
from registry import registry
from artifacts import Artifact

# --- Private Helper Functions ---


def _get_random_color_palette() -> dict:
    """Selects a random, realistic color palette for the meter."""
    palettes = [
        {  # Classic Gray/Red
            "case": (90, 95, 100),
            "face": (225, 225, 215),
            "text": (20, 20, 20),
            "needle": (210, 30, 25),
            "ticks": (40, 40, 40),
        },
        {  # Modern Black/Orange
            "case": (50, 55, 60),
            "face": (245, 245, 245),
            "text": (10, 10, 10),
            "needle": (240, 90, 0),
            "ticks": (30, 30, 30),
        },
        {  # Vintage Beige
            "case": (70, 65, 60),
            "face": (215, 205, 185),
            "text": (50, 45, 40),
            "needle": (40, 40, 40),
            "ticks": (60, 55, 50),
        },
    ]
    return random.choice(palettes)


def _add_subtle_noise(image: Image.Image) -> Image.Image:
    """Adds subtle Gaussian noise to the image."""
    np_image = np.array(image).astype(np.float32)
    noise_intensity = random.uniform(1, 5)
    noise = np.random.normal(0, noise_intensity, np_image.shape)
    np_image += noise
    np_image = np.clip(np_image, 0, 255)
    return Image.fromarray(np_image.astype(np.uint8))


def _draw_needle(
    draw: ImageDraw.ImageDraw,
    center: tuple,
    radius: float,
    value: float,
    is_clockwise: bool,
    color: tuple,
    style: str,
):
    """Draws a single needle on a dial."""
    cx, cy = center
    angle_rad = -math.pi / 2  # Start at top (0)
    sweep = 2 * math.pi

    # Adjust angle based on value and direction
    angle_rad += (value / 10.0) * sweep * (1 if is_clockwise else -1)

    length = radius * random.uniform(0.8, 0.9)
    tip_x = cx + length * math.cos(angle_rad)
    tip_y = cy + length * math.sin(angle_rad)

    # Draw needle body
    if style == "simple":
        draw.line(
            [center, (tip_x, tip_y)], fill=color, width=max(2, int(radius * 0.05))
        )
    else:  # Tapered style
        base_width = radius * random.uniform(0.08, 0.12)
        base_x1 = cx + base_width * math.cos(angle_rad + math.pi / 2)
        base_y1 = cy + base_width * math.sin(angle_rad + math.pi / 2)
        base_x2 = cx + base_width * math.cos(angle_rad - math.pi / 2)
        base_y2 = cy + base_width * math.sin(angle_rad - math.pi / 2)
        draw.polygon(
            [(base_x1, base_y1), (base_x2, base_y2), (tip_x, tip_y)], fill=color
        )

    # Draw central hub
    hub_radius = radius * random.uniform(0.08, 0.15)
    hub_outline = tuple(c // 2 for c in color)
    draw.ellipse(
        [(cx - hub_radius, cy - hub_radius), (cx + hub_radius, cy + hub_radius)],
        fill=color,
        outline=hub_outline,
    )


def _draw_dial(
    draw: ImageDraw.ImageDraw,
    center: tuple,
    radius: float,
    is_clockwise: bool,
    colors: dict,
    font: ImageFont.FreeTypeFont,
):
    """Draws the tick marks, numbers, and border for a single dial."""
    cx, cy = center

    # Add border for the dial
    border_width = max(1, int(radius * 0.03))
    draw.ellipse(
        [(cx - radius, cy - radius), (cx + radius, cy + radius)],
        outline=colors["ticks"],
        width=border_width,
    )

    label_radius_factor = random.choice([0.75, 1.2])  # Place numbers inside or outside
    label_radius = radius * label_radius_factor
    tick_len = radius * 0.1

    for i in range(10):
        angle_rad = -math.pi / 2 + (i / 10.0) * 2 * math.pi

        # Ticks
        start_tick = (
            cx + radius * math.cos(angle_rad),
            cy + radius * math.sin(angle_rad),
        )
        end_tick = (
            cx + (radius - tick_len) * math.cos(angle_rad),
            cy + (radius - tick_len) * math.sin(angle_rad),
        )
        draw.line(
            [start_tick, end_tick],
            fill=colors["ticks"],
            width=max(1, int(radius * 0.02)),
        )

        # Numbers (0, 1, 2... for CW; 0, 9, 8... for CCW)
        num_to_draw = i if is_clockwise else (10 - i) % 10
        num_str = str(num_to_draw)

        text_x = cx + label_radius * math.cos(angle_rad)
        text_y = cy + label_radius * math.sin(angle_rad)
        draw.text(
            (text_x, text_y), num_str, font=font, fill=colors["text"], anchor="mm"
        )


def _draw_dial_label(
    draw: ImageDraw.ImageDraw,
    center: tuple,
    radius: float,
    multiplier: float,
    color: tuple,
    font: ImageFont.FreeTypeFont,
):
    """Draws the unit label below a dial."""
    cx, cy = center
    label_y_offset = radius * random.uniform(1.4, 1.6)

    if multiplier == 1000:
        label_text = "1k"
    elif multiplier == 0.1:
        label_text = "0.1"
    else:
        label_text = str(int(multiplier))

    draw.text((cx, cy + label_y_offset), label_text, font=font, fill=color, anchor="ma")


# --- Public Entrypoint Function ---


@registry.register(name="gas_meter1", tags={"gas_meter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """
    Render a synthetic Gas Meter image and save it to img_path.

    Returns:
        A dictionary containing the design type, evaluation interval, and units.
    """
    # 1. DIVERSITY: Initialize over 12 random parameters
    img_size = random.choice([384, 512, 640])
    colors = _get_random_color_palette()
    case_color = tuple(
        np.clip(
            np.array(colors["case"]) + np.random.randint(-10, 10, 3), 0, 255
        ).astype(int)
    )
    face_color = tuple(
        np.clip(np.array(colors["face"]) + np.random.randint(-5, 5, 3), 0, 255).astype(
            int
        )
    )
    needle_style = random.choice(["simple", "tapered"])
    apply_blur = random.choice([True, False])
    apply_noise = random.choice([True, False])
    apply_rotation = random.choice([True, False])

    padding = img_size * random.uniform(0.05, 0.1)

    # Adjusted layout parameters for circular arrangement
    num_dials = 5
    central_ring_radius = img_size * random.uniform(
        0.2, 0.25
    )  # Radius for the circle on which dial centers lie
    dial_radius = img_size * random.uniform(
        0.07, 0.09
    )  # Individual dial radius, smaller for more space

    main_font_size = int(img_size * random.uniform(0.04, 0.06))
    dial_num_font_size = int(dial_radius * random.uniform(0.25, 0.35))
    dial_label_font_size = int(dial_radius * random.uniform(0.22, 0.28))

    try:
        base_font = ImageFont.truetype("DejaVuSans-Bold.ttf", size=10)
        main_font = base_font.font_variant(size=main_font_size)
        dial_num_font = base_font.font_variant(size=dial_num_font_size)
        dial_label_font = base_font.font_variant(size=dial_label_font_size)
    except IOError:
        # Fallback to Pillow's default font if system font is not found
        main_font = ImageFont.load_default(size=int(main_font_size * 0.75))
        dial_num_font = ImageFont.load_default(size=int(dial_num_font_size * 0.75))
        dial_label_font = ImageFont.load_default(size=int(dial_label_font_size * 0.75))

    # 2. READING & UNITS: Generate random reading
    target_reading = random.uniform(100.0, 9999.0)

    if random.choice([True, False]):
        units_list = ["cubic meter", "m³", "cu m"]
        display_unit_header = "CUBIC METERS"
    else:
        units_list = ["cubic feet", "ft³", "cu ft"]
        display_unit_header = "CUBIC FEET"

    # 3. IMAGE CREATION
    image = Image.new("RGB", (img_size, img_size), case_color)
    draw = ImageDraw.Draw(image)

    # 4. DRAW METER
    face_coords = [(padding, padding), (img_size - padding, img_size - padding)]
    draw.rounded_rectangle(
        face_coords,
        radius=padding * 0.5,
        fill=face_color,
        outline=tuple(c // 2 for c in case_color),
        width=int(padding * 0.1),
    )

    draw.text(
        (img_size / 2, padding * 1.8),
        display_unit_header,
        font=main_font,
        fill=colors["text"],
        anchor="ma",
    )

    dial_multipliers = [1000, 100, 10, 1, 0.1]

    # Calculate dial positions for circular arrangement
    center_x, center_y = (
        img_size / 2,
        img_size / 2 - 20 + img_size * random.uniform(0.05, 0.1),
    )  # Slightly offset central y for header

    # Calculate angles for dial centers, ensuring they are evenly spaced on the ring
    # Start angle to position the dials nicely, maybe slightly offset from straight horizontal
    start_angle = random.uniform(0, 2 * math.pi / num_dials)

    for i in range(num_dials):
        is_clockwise = i % 2 == 0  # Leftmost dial is CW, then alternates

        # Calculate angle for the current dial on the central ring
        angle_on_ring = start_angle + i * (2 * math.pi / num_dials)

        # Calculate dial center coordinates
        cx = center_x + central_ring_radius * math.cos(angle_on_ring)
        cy = center_y + central_ring_radius * math.sin(angle_on_ring)

        multiplier = dial_multipliers[i]
        value = (target_reading % (multiplier * 10)) / multiplier

        _draw_dial(draw, (cx, cy), dial_radius, is_clockwise, colors, dial_num_font)
        _draw_needle(
            draw,
            (cx, cy),
            dial_radius,
            value,
            is_clockwise,
            colors["needle"],
            needle_style,
        )

        _draw_dial_label(
            draw, (cx, cy), dial_radius, multiplier, colors["text"], dial_label_font
        )

    # 5. POST-PROCESSING & ARTIFACTS
    if apply_noise:
        image = _add_subtle_noise(image)
    if apply_blur:
        image = image.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.1, 0.4)))
    if apply_rotation:
        image = image.rotate(
            random.uniform(-2, 2),
            resample=Image.BICUBIC,
            expand=False,
            fillcolor=case_color,
        )
        image = ImageOps.fit(image, (img_size, img_size), Image.LANCZOS)

    # 6. SAVE IMAGE
    image.save(img_path)

    # 7. RETURN DICT with updated interval logic
    # Lower bound is the reading rounded down to one decimal place.
    lower_bound = math.floor(target_reading * 10) / 10.0
    # Upper bound is the reading rounded up to one decimal place.
    upper_bound = math.ceil(target_reading * 10) / 10.0

    evaluator_kwargs = {"interval": [lower_bound, upper_bound], "units": units_list}

    return Artifact(
        data=img_path,
        image_type="gas_meter",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )
