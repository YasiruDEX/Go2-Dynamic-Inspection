# Re-run the generator cell (state was reset last run).

from PIL import Image, ImageDraw, ImageFont, ImageFilter
import random
import os

from registry import registry
from artifacts import Artifact


def _load_font(size: int):
    for name in ["DejaVuSans.ttf", "Arial.ttf"]:
        try:
            return ImageFont.truetype(name, size=size)
        except Exception:
            continue
    return ImageFont.load_default()


def _choose_capacity_ml():
    candidates = [10, 25, 50, 100, 250, 500, 1000]  # mL
    return random.choice(candidates)


def _choose_minor_step_ml(capacity_ml: int):
    table = {
        10: [0.1, 0.2, 0.5],
        25: [0.2, 0.5, 1.0],
        50: [0.5, 1.0, 2.0],
        100: [1.0, 2.0],
        250: [2.0, 5.0],
        500: [5.0, 10.0],
        1000: [10.0, 20.0],
    }
    return random.choice(table.get(capacity_ml, [1.0]))


def _choose_display_unit(capacity_ml: int):
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
    return value_ml  # mL


def _format_label(val: float, unit: str):
    if unit == "μL":
        return str(int(round(val)))
    if unit == "mL":
        if abs(val - round(val)) < 1e-6:
            return f"{int(round(val))}"
        return f"{val:.1f}".rstrip("0").rstrip(".")
    if unit == "L":
        s = f"{val:.2f}"
        s = s.rstrip("0").rstrip(".")
        return s if s else "0"
    return f"{val:g}"


def _draw_vertical_ticks(
    draw,
    origin,
    height_px,
    capacity_ml: float,
    minor_step_ml: float,
    unit: str,
    px_per_ml: float,
    color=(30, 30, 30),
    font=None,
):
    x0, y_bottom = origin
    n_major = random.choice([5, 10])
    major_interval_ml = capacity_ml / n_major
    major_interval_ml = max(
        minor_step_ml, round(major_interval_ml / minor_step_ml) * minor_step_ml
    )
    mid_interval_ml = max(
        minor_step_ml, round((major_interval_ml / 2) / minor_step_ml) * minor_step_ml
    )

    tick_short, tick_mid, tick_long = 16, 24, 34

    n_minor = int(round(capacity_ml / minor_step_ml))
    for i in range(n_minor + 1):
        ml = i * minor_step_ml
        y = int(round(y_bottom - ml * px_per_ml))

        is_major = (
            abs((ml % major_interval_ml)) < 1e-9
            or abs(((major_interval_ml - (ml % major_interval_ml)) % major_interval_ml))
            < 1e-9
        )
        is_mid = (
            abs((ml % mid_interval_ml)) < 1e-9
            or abs(((mid_interval_ml - (ml % mid_interval_ml)) % mid_interval_ml))
            < 1e-9
        )

        tick = tick_long if is_major else (tick_mid if is_mid else tick_short)
        draw.line([(x0, y), (x0 + tick, y)], fill=color, width=2)
        if is_major and font is not None:
            label_val = _to_display(ml, unit)
            label = _format_label(label_val, unit)
            tw, th = draw.textbbox((0, 0), label, font=font)[2:]
            draw.text((x0 + tick + 6, y - th // 2), label, fill=color, font=font)


def _draw_glass(draw, bbox):
    x1, y1, x2, y2 = bbox
    draw.rounded_rectangle(
        bbox, radius=16, outline=(60, 60, 60, 200), width=2, fill=(255, 255, 255, 0)
    )
    draw.line([(x1 + 6, y1 + 6), (x1 + 6, y2 - 6)], fill=(255, 255, 255, 120), width=6)
    draw.line([(x2 - 6, y1 + 6), (x2 - 6, y2 - 6)], fill=(255, 255, 255, 60), width=3)


def _draw_liquid(base_img, bbox, fill_top_y, color=(86, 140, 220, 220)):
    x1, y1, x2, y2 = bbox
    liquid = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    L = ImageDraw.Draw(liquid, "RGBA")
    L.rectangle([x1 + 6, fill_top_y, x2 - 6, y2 - 6], fill=color)
    meniscus_h = max(6, (x2 - x1) // 8)
    L.ellipse(
        [x1 + 6, fill_top_y - meniscus_h, x2 - 6, fill_top_y + meniscus_h], fill=color
    )
    base_img.alpha_composite(liquid)
    highlight = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    H = ImageDraw.Draw(highlight, "RGBA")
    H.arc(
        [x1 + 12, fill_top_y - meniscus_h // 2, x2 - 12, fill_top_y + meniscus_h // 2],
        start=0,
        end=180,
        fill=(255, 255, 255, 180),
        width=2,
    )
    base_img.alpha_composite(highlight)


@registry.register(name="measuring_cylinder1", tags={"measuring_cylinder"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    capacity_ml = _choose_capacity_ml()
    minor_step_ml = _choose_minor_step_ml(capacity_ml)
    unit = _choose_display_unit(capacity_ml)

    W, H = 720, 1080
    img = Image.new("RGBA", (W, H), (245, 246, 250, 255))

    bg = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    gd = ImageDraw.Draw(bg, "RGBA")
    for r in range(0, max(W, H), 8):
        alpha = max(0, 90 - r // 10)
        gd.ellipse(
            [W // 2 - r, H // 2 - r, W // 2 + r, H // 2 + r],
            outline=(0, 0, 0, alpha),
            width=2,
        )
    img = Image.alpha_composite(img, bg.filter(ImageFilter.GaussianBlur(18)))

    draw = ImageDraw.Draw(img, "RGBA")

    tube_top, tube_bottom = 140, H - 180
    tube_left, tube_right = W // 2 - 80, W // 2 + 80
    tube_bbox = (tube_left, tube_top, tube_right, tube_bottom)
    _draw_glass(draw, tube_bbox)

    inner_top, inner_bottom = tube_top + 18, tube_bottom - 16
    inner_height = inner_bottom - inner_top
    px_per_ml = inner_height / float(capacity_ml)

    volume_ml = round(random.uniform(0.05 * capacity_ml, 0.95 * capacity_ml), 2)
    fill_top_y = int(round(inner_bottom - volume_ml * px_per_ml))
    liquid_color = random.choice(
        [
            (86, 140, 220, 220),
            (70, 170, 150, 220),
            (210, 120, 120, 220),
            (160, 160, 210, 220),
        ]
    )
    _draw_liquid(
        img,
        (tube_left, inner_top, tube_right, inner_bottom),
        fill_top_y,
        color=liquid_color,
    )

    scale_x = tube_right + 10
    font_size = 26
    if unit == "μL" and capacity_ml >= 25:
        font_size = 22
    font_major = _load_font(font_size)

    _draw_vertical_ticks(
        draw,
        (scale_x, inner_bottom),
        inner_height,
        capacity_ml=capacity_ml,
        minor_step_ml=minor_step_ml,
        unit=unit,
        px_per_ml=px_per_ml,
        color=(30, 30, 30),
        font=font_major,
    )

    unit_font = _load_font(28)
    draw.text((scale_x + 48, inner_top - 36), unit, fill=(40, 40, 40), font=unit_font)
    small_font = _load_font(20)
    draw.text(
        (scale_x - 2, tube_top - 56), "Ex 20°C", fill=(110, 110, 110), font=small_font
    )

    base_y = tube_bottom + 26
    draw.ellipse(
        [W // 2 - 140, base_y, W // 2 + 140, base_y + 34],
        outline=(60, 60, 60, 220),
        fill=(230, 230, 235, 255),
    )
    draw.rectangle(
        [W // 2 - 90, tube_bottom - 6, W // 2 + 90, tube_bottom + 28],
        fill=(240, 240, 245, 255),
        outline=(90, 90, 90, 220),
    )

    img = img.convert("RGB")
    os.makedirs(os.path.dirname(img_path), exist_ok=True) if os.path.dirname(
        img_path
    ) else None
    img.save(img_path, quality=95)

    def _to_display(value_ml: float, unit: str) -> float:
        if unit == "μL":
            return value_ml * 1000.0
        if unit == "L":
            return value_ml / 1000.0
        return value_ml

    volume_val = _to_display(volume_ml, unit)
    minor_step_val = _to_display(minor_step_ml, unit)
    return Artifact(
        data=img_path,
        image_type="measuring_cylinder",
        design="Linear",
        evaluator_kwargs={
            "interval": [
                volume_val - minor_step_val / 2,
                volume_val + minor_step_val / 2,
            ],
            "units": [unit],
        },
    )


if __name__ == "__main__":
    out = generate("out.jpg")
    print(out)
