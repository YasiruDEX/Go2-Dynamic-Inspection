# Generator: "lab style" graduated cylinder (ticks printed on glass, orange labels on the tube).
# - Randomizes capacity, minor tick step, unit (μL/mL/L), and fill volume
# - Style inspired by the user photo: ticks inside-left of the tube, orange numbers, vertical unit text
# - Single public entrypoint: generate(img_path) -> dict
#
# Re-run to get a new randomized image.

from PIL import Image, ImageDraw, ImageFont, ImageFilter
import random
import os
from registry import registry
from artifacts import Artifact

# ---------------------------- helpers ----------------------------


def _load_font(size: int):
    for name in ["DejaVuSans.ttf", "Arial.ttf"]:
        try:
            return ImageFont.truetype(name, size=size)
        except Exception:
            continue
    return ImageFont.load_default()


def _choose_capacity_ml():
    # common sizes
    return random.choice([10, 25, 50, 100, 250, 500, 1000])


def _choose_minor_step_ml(capacity_ml: int):
    table = {
        10: [0.1, 0.2, 0.5],
        25: [0.2, 0.5, 1.0],
        50: [0.5, 1.0, 2.0],
        100: [1.0, 2.0, 5.0],
        250: [2.0, 5.0, 10.0],
        500: [5.0, 10.0, 20.0],
        1000: [10.0, 20.0, 50.0],
    }
    return random.choice(table.get(capacity_ml, [1.0]))


def _choose_display_unit(capacity_ml: int):
    # Weighted choices by capacity
    if capacity_ml <= 25:
        choices = [("mL", 0.8), ("μL", 0.2)]
    elif capacity_ml <= 250:
        choices = [("mL", 0.9), ("L", 0.1)]
    else:
        choices = [("mL", 0.6), ("L", 0.4)]
    r = random.random()
    acc = 0.0
    for u, p in choices:
        acc += p
        if r <= acc:
            return u
    return choices[-1][0]


def _to_display(value_ml: float, unit: str) -> float:
    if unit == "μL":
        return value_ml * 1000.0
    if unit == "L":
        return value_ml / 1000.0
    return value_ml


def _format_label(val: float, unit: str):
    if unit == "μL":
        return str(int(round(val)))
    if unit == "mL":
        return (
            str(int(round(val)))
            if abs(val - round(val)) < 1e-6
            else (f"{val:.1f}".rstrip("0").rstrip("."))
        )
    if unit == "L":
        s = f"{val:.2f}".rstrip("0").rstrip(".")
        return s if s else "0"
    return f"{val:g}"


# ---------------------------- drawing ----------------------------


def _draw_glass(draw: ImageDraw.ImageDraw, bbox):
    x1, y1, x2, y2 = bbox
    draw.rounded_rectangle(
        bbox, radius=16, outline=(70, 70, 70, 220), width=2, fill=(255, 255, 255, 0)
    )
    draw.line([(x1 + 6, y1 + 8), (x1 + 6, y2 - 8)], fill=(255, 255, 255, 140), width=6)
    draw.line([(x2 - 6, y1 + 8), (x2 - 6, y2 - 8)], fill=(255, 255, 255, 80), width=3)


def _draw_liquid(base_img: Image.Image, bbox, fill_top_y, color=(227, 164, 36, 220)):
    x1, y1, x2, y2 = bbox
    liquid = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    L = ImageDraw.Draw(liquid, "RGBA")
    L.rectangle([x1 + 6, fill_top_y, x2 - 6, y2 - 6], fill=color)
    meniscus_h = max(6, (x2 - x1) // 9)
    L.ellipse(
        [x1 + 6, fill_top_y - meniscus_h, x2 - 6, fill_top_y + meniscus_h], fill=color
    )
    base_img.alpha_composite(liquid)

    # highlight over meniscus
    highlight = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    H = ImageDraw.Draw(highlight, "RGBA")
    H.arc(
        [x1 + 12, fill_top_y - meniscus_h // 2, x2 - 12, fill_top_y + meniscus_h // 2],
        start=0,
        end=180,
        fill=(255, 255, 255, 190),
        width=2,
    )
    base_img.alpha_composite(highlight)


def _draw_ticks_on_glass(
    draw: ImageDraw.ImageDraw,
    tube_bbox,
    inner_top,
    inner_bottom,
    capacity_ml: float,
    minor_step_ml: float,
    unit: str,
    px_per_ml: float,
    orange=(213, 75, 43),
):
    x1, y1, x2, y2 = tube_bbox
    # We draw ticks on the left half of the tube, labels in orange beside ticks
    ticks_x_left = x1 + 16
    ticks_x_right = x1 + (x2 - x1) // 2 + 6

    n_major = random.choice([5, 10])
    major_ml = max(
        minor_step_ml, round((capacity_ml / n_major) / minor_step_ml) * minor_step_ml
    )
    mid_ml = max(minor_step_ml, round((major_ml / 2) / minor_step_ml) * minor_step_ml)

    # lengths (inside tube)
    tick_short = min(14, (ticks_x_right - ticks_x_left) - 8)
    tick_mid = tick_short + 6
    tick_long = tick_mid + 8

    label_font = _load_font(26 if unit != "μL" else 22)
    label_color = (orange[0], orange[1], orange[2])

    n_minor = int(round(capacity_ml / minor_step_ml))
    for i in range(n_minor + 1):
        ml = i * minor_step_ml
        y = int(round(inner_bottom - ml * px_per_ml))
        is_major = (
            abs((ml % major_ml)) < 1e-9
            or abs(((major_ml - (ml % major_ml)) % major_ml)) < 1e-9
        )
        is_mid = (
            abs((ml % mid_ml)) < 1e-9 or abs(((mid_ml - (ml % mid_ml)) % mid_ml)) < 1e-9
        )
        length = tick_long if is_major else (tick_mid if is_mid else tick_short)
        draw.line(
            [(ticks_x_left, y), (ticks_x_left + length, y)],
            fill=(35, 35, 35, 255),
            width=2,
        )

        if is_major:
            # label to the right of the long tick, but still inside the glass
            val = _to_display(ml, unit)
            text = _format_label(val, unit)
            tw, th = draw.textbbox((0, 0), text, font=label_font)[2:]
            tx = ticks_x_left + length + 4
            draw.text((tx, y - th // 2), text, fill=label_color, font=label_font)


def _draw_vertical_unit(draw: ImageDraw.ImageDraw, text, pos, color=(213, 75, 43)):
    """Draw vertical unit text similar to photo by stacking characters."""
    x, y = pos
    font = _load_font(28)
    dy = 0
    for ch in text:
        tw, th = draw.textbbox((0, 0), ch, font=font)[2:]
        draw.text((x, y + dy), ch, fill=color, font=font)
        dy += th + 2


# ---------------------------- public API ----------------------------


@registry.register(name="measuring_cylinder2", tags={"measuring_cylinder"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """Render a randomized, photo-style graduated cylinder (ticks on glass).
    Returns:
        {
            "volume_value": <float in chosen unit>,
            "unit": <'μL'|'mL'|'L'>,
            "capacity_value": <float in chosen unit>,
            "minor_step_value": <float in chosen unit>,
            "interval": [0, <capacity_value>],
            "image_path": <path>
        }
    """
    # Random spec
    capacity_ml = _choose_capacity_ml()
    minor_step_ml = _choose_minor_step_ml(capacity_ml)
    unit = _choose_display_unit(capacity_ml)

    # Canvas/background
    W, H = 720, 1080
    img = Image.new("RGBA", (W, H), (242, 245, 248, 255))
    # soft background to mimic bench
    bg = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    gd = ImageDraw.Draw(bg, "RGBA")
    gd.rectangle([0, int(H * 0.62), W, H], fill=(225, 232, 238, 255))
    img = Image.alpha_composite(img, bg.filter(ImageFilter.GaussianBlur(1)))

    draw = ImageDraw.Draw(img, "RGBA")

    # Tube geometry (similar proportions to the photo)
    tube_top, tube_bottom = 160, H - 180
    tube_left, tube_right = W // 2 - 70, W // 2 + 70
    tube_bbox = (tube_left, tube_top, tube_right, tube_bottom)
    _draw_glass(draw, tube_bbox)

    inner_top, inner_bottom = tube_top + 18, tube_bottom - 18
    inner_height = inner_bottom - inner_top
    px_per_ml = inner_height / float(capacity_ml)

    # Liquid (amber like photo) – volume 5%..95%
    volume_ml = round(random.uniform(0.05 * capacity_ml, 0.95 * capacity_ml), 2)
    fill_top_y = int(round(inner_bottom - volume_ml * px_per_ml))
    amber_variants = [
        (227, 164, 36, 220),
        (214, 146, 30, 220),
        (236, 175, 60, 220),
        (200, 135, 30, 220),
    ]
    _draw_liquid(
        img,
        (tube_left, inner_top, tube_right, inner_bottom),
        fill_top_y,
        color=random.choice(amber_variants),
    )

    # Ticks and numbers printed on the glass (left)
    _draw_ticks_on_glass(
        draw,
        tube_bbox,
        inner_top,
        inner_bottom,
        capacity_ml,
        minor_step_ml,
        unit,
        px_per_ml,
    )

    # Orange unit + calibration text near top-left
    orange = (213, 75, 43)
    small_font = _load_font(20)
    draw.text((tube_left + 70, tube_top + 28), "Ex20° C", fill=orange, font=small_font)
    draw.text((tube_left + 108, tube_top + 56), unit, fill=orange, font=small_font)

    # Base
    base_y = tube_bottom + 22
    draw.ellipse(
        [W // 2 - 120, base_y, W // 2 + 120, base_y + 30],
        outline=(60, 60, 60, 220),
        fill=(235, 236, 240, 255),
    )
    draw.rectangle(
        [W // 2 - 80, tube_bottom - 6, W // 2 + 80, tube_bottom + 26],
        fill=(242, 243, 246, 255),
        outline=(90, 90, 90, 220),
    )

    # Save
    img = img.convert("RGB")
    os.makedirs(os.path.dirname(img_path), exist_ok=True) if os.path.dirname(
        img_path
    ) else None
    img.save(img_path, quality=95)

    # Return info in chosen unit
    vol_val = _to_display(volume_ml, unit)
    step_val = _to_display(minor_step_ml, unit)
    return Artifact(
        data=img_path,
        image_type="measuring_cylinder",
        design="Linear",
        evaluator_kwargs={
            "interval": [vol_val - step_val / 2, vol_val + step_val / 2],
            "units": [unit],
        },
    )


# Demo
if __name__ == "__main__":
    out = generate("out.jpg")
    print(out)
