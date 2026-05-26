import os
import random
from PIL import Image, ImageDraw, ImageFont
import math
from registry import registry
from artifacts import Artifact

# --- Configuration ---
# Define realistic temperature ranges for each unit
TEMP_RANGES = {
    "Celsius": (-20, 50),
    "Fahrenheit": (-10, 120),
}

# Define visual properties for randomization
MATERIALS = {
    "glass": {"bg": (210, 220, 230, 180), "outline": (150, 160, 170)},
    "plastic": {"bg": (240, 240, 240, 255), "outline": (100, 100, 100)},
    "metal": {"bg": (180, 180, 190, 255), "outline": (80, 80, 90)},
}
LIQUID_COLORS = {
    "red": (255, 0, 0),
    "blue": (0, 100, 255),
    "green": (0, 255, 0),
    "yellow": (255, 255, 0),
    "orange": (255, 165, 0),
    "purple": (128, 0, 128),
    "pink": (255, 192, 203),
    "brown": (165, 42, 42),
}
BACKGROUNDS = [
    (255, 255, 255),  # White
    (230, 230, 230),  # Light Gray
    (245, 245, 220),  # Beige
    (200, 225, 245),  # Light Blue
]


# --- Helper to find a suitable font ---
def get_unicode_font(font_size):
    """
    Attempts to find a system font that supports common Unicode characters,
    especially degree symbols. Falls back to default if none found.
    """
    font_paths = [
        # Windows paths
        "C:/Windows/Fonts/segoeuib.ttf",  # Segoe UI Bold
        "C:/Windows/Fonts/arial.ttf",
        # macOS paths
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        # Linux paths (common for DejaVu, adjust if needed)
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    ]

    for font_path in font_paths:
        if os.path.exists(font_path):
            try:
                return ImageFont.truetype(font_path, font_size)
            except IOError:
                continue

    # Fallback to default if no suitable font is found
    print("Warning: Could not find a robust Unicode font. Falling back to default.")
    return ImageFont.load_default()


# --- Main Generation Function ---
def draw_thermometer(params):
    """
    Generates a single thermometer image based on the provided parameters.
    Updated to ensure scale numbers and unit symbols are always present and readable.
    """
    is_vertical = params["orientation"] == "vertical"
    width, height = (
        (params["width"], params["height"])
        if is_vertical
        else (params["height"], params["width"])
    )

    # Create image with a random background
    image = Image.new("RGB", (width, height), random.choice(BACKGROUNDS))
    draw = ImageDraw.Draw(image)

    # Dynamically adjust font size
    font_size = max(12, min(int(min(width, height) / 25), 20))
    # Use the helper function to get a Unicode-compatible font
    font = get_unicode_font(font_size)

    # Thermometer dimensions and positioning
    padding = max(30, int(min(width, height) * 0.08))  # Dynamic padding
    body_width = max(30, int(min(width, height) * 0.1))  # Dynamic body width

    if is_vertical:
        body_x0 = (width - body_width) / 2
        body_y0 = padding
        body_x1 = body_x0 + body_width
        body_y1 = height - padding
        scale_length = body_y1 - body_y0
    else:  # Horizontal
        body_x0 = padding
        body_y0 = (height - body_width) / 2
        body_x1 = width - padding
        body_y1 = body_y0 + body_width
        scale_length = body_x1 - body_x0

    # Draw thermometer body
    material = MATERIALS[params["material"]]
    draw.rectangle(
        [body_x0, body_y0, body_x1, body_y1],
        fill=material["bg"],
        outline=material["outline"],
        width=2,
    )

    # Draw bulb
    bulb_radius = body_width / 1.5
    if is_vertical:
        bulb_center_x = width / 2
        bulb_center_y = body_y1 + bulb_radius / 2
    else:  # Horizontal
        bulb_center_x = body_x0 - bulb_radius / 2
        bulb_center_y = height / 2

    bulb_box = [
        bulb_center_x - bulb_radius,
        bulb_center_y - bulb_radius,
        bulb_center_x + bulb_radius,
        bulb_center_y + bulb_radius,
    ]
    draw.ellipse(bulb_box, fill=material["bg"], outline=material["outline"], width=2)
    draw.ellipse(
        [b + 4 for b in bulb_box[:2]] + [b - 4 for b in bulb_box[2:]],
        fill=params["liquid_color"],
    )

    # Draw liquid column
    scale_min, scale_max = params["scale_range"]
    temp = params["temperature"]

    # Clamp temperature to scale range for drawing purposes
    clamped_temp = max(scale_min, min(temp, scale_max))
    temp_percentage = (clamped_temp - scale_min) / (scale_max - scale_min)

    liquid_width = max(8, int(body_width * 0.3))  # Dynamic liquid width
    if is_vertical:
        liquid_height = scale_length * temp_percentage
        liquid_x0 = (width - liquid_width) / 2
        liquid_y0 = body_y1 - liquid_height
        liquid_x1 = liquid_x0 + liquid_width
        liquid_y1 = body_y1
    else:  # Horizontal
        liquid_length = scale_length * temp_percentage
        liquid_x0 = body_x0
        liquid_y0 = (height - liquid_width) / 2
        liquid_x1 = body_x0 + liquid_length
        liquid_y1 = liquid_y0 + liquid_width

    draw.rectangle(
        [liquid_x0, liquid_y0, liquid_x1, liquid_y1], fill=params["liquid_color"]
    )

    # Draw Scale Ticks and Labels
    major_tick_interval = params["scale_resolution"]
    # Ensure integer minor ticks; avoid float step issues
    minor_tick_interval = max(1, major_tick_interval // 5)

    # 1) Draw all minor ticks (excluding those that are major)
    for val in range(scale_min, scale_max + 1, minor_tick_interval):
        if val % major_tick_interval == 0:
            continue
        tick_percentage = (val - scale_min) / (scale_max - scale_min)

        if is_vertical:
            tick_y = body_y1 - (scale_length * tick_percentage)
            tick_x_start = body_x1
            tick_len = 8
            tick_x_end = tick_x_start + tick_len
            draw.line(
                [(tick_x_start, tick_y), (tick_x_end, tick_y)], fill="black", width=1
            )
        else:  # Horizontal
            tick_x = body_x0 + (scale_length * tick_percentage)
            tick_y_start = body_y0
            tick_len = 8
            tick_y_end = tick_y_start - tick_len
            draw.line(
                [(tick_x, tick_y_start), (tick_x, tick_y_end)], fill="black", width=1
            )

    # 2) Draw major ticks and labels. Compute the first visible major tick robustly.
    first_major = int(math.ceil(scale_min / major_tick_interval) * major_tick_interval)
    for val in range(first_major, scale_max + 1, major_tick_interval):
        tick_percentage = (val - scale_min) / (scale_max - scale_min)

        if is_vertical:
            tick_y = body_y1 - (scale_length * tick_percentage)
            tick_x_start = body_x1
            tick_len = 15
            tick_x_end = tick_x_start + tick_len
            draw.line(
                [(tick_x_start, tick_y), (tick_x_end, tick_y)], fill="black", width=1
            )

            text_label = str(val)
            text_bbox = draw.textbbox((0, 0), text_label, font=font)
            text_height = text_bbox[3] - text_bbox[1]
            draw.text(
                (tick_x_end + 5, tick_y - text_height / 2),
                text_label,
                fill="black",
                font=font,
            )
        else:  # Horizontal
            tick_x = body_x0 + (scale_length * tick_percentage)
            tick_y_start = body_y0
            tick_len = 15
            tick_y_end = tick_y_start - tick_len
            draw.line(
                [(tick_x, tick_y_start), (tick_x, tick_y_end)], fill="black", width=1
            )

            text_label = str(val)
            text_bbox = draw.textbbox((0, 0), text_label, font=font)
            text_width = text_bbox[2] - text_bbox[0]
            text_height = text_bbox[3] - text_bbox[1]
            # Center text label horizontally on the tick, position above
            draw.text(
                (tick_x - text_width / 2, tick_y_end - 5 - text_height),
                text_label,
                fill="black",
                font=font,
            )

    # Draw unit symbol
    unit_symbol = params["units"]["symbol"]
    # Adjust position slightly for better visual balance

    if is_vertical:
        text_bbox = draw.textbbox((0, 0), unit_symbol, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        draw.text(
            (body_x1 + 5, body_y0 - text_width / 2 - 10),
            unit_symbol,
            fill="black",
            font=font,
        )
    else:  # Horizontal
        text_bbox = draw.textbbox((0, 0), unit_symbol, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        draw.text(
            (body_x1 - text_width / 2, body_y0 - 40 - text_height),
            unit_symbol,
            fill="black",
            font=font,
        )

    return image


def generate_thermometer_data(units):
    """
    Generates a dictionary of randomized parameters for a single thermometer.
    """
    unit_name = "Celsius" if units == "C" else "Fahrenheit"
    unit_symbol = "°C" if units == "C" else "°F"

    min_temp, max_temp = TEMP_RANGES[unit_name]

    # Generate a random temperature
    temperature = round(random.uniform(min_temp, max_temp), 1)

    # Determine a sensible scale range around the temperature
    scale_buffer = random.randint(10, 30)
    scale_min = int(temperature - random.uniform(scale_buffer, scale_buffer + 20))
    scale_max = int(temperature + random.uniform(scale_buffer, scale_buffer + 20))

    # Make scale min/max neat (multiples of 10)
    scale_min = (scale_min // 10) * 10
    scale_max = ((scale_max + 9) // 10) * 10

    params = {
        "temperature": temperature,
        "units": {"name": unit_name, "symbol": unit_symbol},
        "scale_range": (scale_min, scale_max),
        "scale_resolution": random.choice(
            [10, 20]
        ),  # Major ticks every 10 or 20 degrees
        "material": random.choice(list(MATERIALS.keys())),
        "liquid_color": random.choice(list(LIQUID_COLORS.values())),
        "orientation": random.choice(["vertical", "horizontal"]),
        "width": random.randint(150, 200),
        "height": random.randint(400, 600),
    }
    return params


@registry.register(name="normal_thermometer", tags={"thermometer"}, weight=1.0)
def generate_thermoter(img_path="thermometer.png") -> Artifact:
    unit_choice = random.choice(["C", "F"])
    # 1. Generate random parameters
    params = generate_thermometer_data(unit_choice)
    image = draw_thermometer(params)
    image.save(img_path)
    allowable_error = params["scale_resolution"] / 10
    evaluator_kwargs = {
        "interval": [
            params["temperature"] - allowable_error,
            params["temperature"] + allowable_error,
        ],
        "units": [params["units"]["name"], params["units"]["symbol"]],
    }
    print(evaluator_kwargs)
    return Artifact(
        data=img_path,
        image_type="thermometer",
        design="Linear",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    generate_thermoter()
