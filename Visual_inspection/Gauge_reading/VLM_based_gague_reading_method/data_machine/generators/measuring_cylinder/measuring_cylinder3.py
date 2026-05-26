# Fix: use Image.composite with mask before alpha_composite

from PIL import Image, ImageDraw, ImageFont, ImageFilter
import random
import os
from registry import registry
from artifacts import Artifact


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


def _load_font(size: int):
    for name in ["DejaVuSans.ttf", "Arial.ttf"]:
        try:
            return ImageFont.truetype(name, size=size)
        except Exception:
            continue
    return ImageFont.load_default()


def _rounded_mask(size, bbox, radius):
    mask = Image.new("L", size, 0)
    d = ImageDraw.Draw(mask)
    d.rounded_rectangle(bbox, radius=radius, fill=255)
    return mask


def _hgrad(width, height, left_alpha, right_alpha):
    grad = Image.new("L", (width, 1))
    px = grad.load()
    for x in range(width):
        a = int(left_alpha + (right_alpha - left_alpha) * x / max(1, width - 1))
        px[x, 0] = max(0, min(255, a))
    return grad.resize((width, height))


def _vgrad(width, height, top_alpha, bottom_alpha):
    grad = Image.new("L", (1, height))
    px = grad.load()
    for y in range(height):
        a = int(top_alpha + (bottom_alpha - top_alpha) * y / max(1, height - 1))
        px[0, y] = max(0, min(255, a))
    return grad.resize((width, height))


def _apply_glass_effects(img: Image.Image, tube_bbox):
    x1, y1, x2, y2 = tube_bbox
    inner = (x1 + 6, y1 + 6, x2 - 6, y2 - 6)
    W, H = img.size
    w = x2 - x1
    h = y2 - y1

    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))

    left_grad = _hgrad(w // 2, h, 110, 0)
    right_grad = _hgrad(w // 2, h, 0, 90)
    L = Image.new("RGBA", (w // 2, h), (255, 255, 255, 0))
    L.putalpha(left_grad)
    R = Image.new("RGBA", (w // 2, h), (255, 255, 255, 0))
    R.putalpha(right_grad)
    overlay.alpha_composite(L, (x1, y1))
    overlay.alpha_composite(R, (x1 + w // 2, y1))

    sheen_w = max(6, w // 14)
    sheen = Image.new("RGBA", (sheen_w, h), (255, 255, 255, 0))
    sheen.putalpha(_vgrad(sheen_w, h, 0, 70))
    overlay.alpha_composite(sheen, (x1 + (w - sheen_w) // 2, y1))

    stripes = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    sd = ImageDraw.Draw(stripes, "RGBA")
    for _ in range(2):
        sx = random.randint(8, w // 3)
        stripe_w = random.randint(6, 12)
        alpha = random.randint(70, 110)
        sd.rectangle([sx, 10, sx + stripe_w, h - 10], fill=(255, 255, 255, alpha))
    stripes = stripes.filter(ImageFilter.GaussianBlur(2))
    overlay.alpha_composite(stripes, (x1, y1))

    scratch = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    sd = ImageDraw.Draw(scratch, "RGBA")
    for _ in range(random.randint(3, 7)):
        sx = random.randint(10, w - 10)
        y0 = random.randint(20, h - 60)
        y1s = y0 + random.randint(30, 120)
        sd.line(
            [(sx, y0), (sx, min(h - 10, y1s))],
            fill=(255, 255, 255, random.randint(20, 35)),
            width=1,
        )
    scratch = scratch.filter(ImageFilter.GaussianBlur(0.6))
    overlay.alpha_composite(scratch, (x1, y1))

    mask = _rounded_mask(img.size, inner, radius=12)
    clipped = Image.composite(overlay, Image.new("RGBA", img.size, (0, 0, 0, 0)), mask)
    img.alpha_composite(clipped)


# Bring back other functions from previous cell by redefining quickly
def _draw_glass_base(img: Image.Image, tube_bbox):
    d = ImageDraw.Draw(img, "RGBA")
    x1, y1, x2, y2 = tube_bbox
    d.rounded_rectangle(
        tube_bbox,
        radius=16,
        outline=(80, 80, 80, 220),
        width=2,
        fill=(255, 255, 255, 0),
    )
    d.line([(x1 + 6, y1 + 8), (x1 + 6, y2 - 8)], fill=(255, 255, 255, 170), width=6)
    d.line([(x2 - 6, y1 + 8), (x2 - 6, y2 - 8)], fill=(255, 255, 255, 90), width=3)


def _draw_liquid(base_img: Image.Image, bbox, fill_top_y, color=(227, 164, 36, 220)):
    x1, y1, x2, y2 = bbox
    liquid = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    L = ImageDraw.Draw(liquid, "RGBA")
    L.rectangle([x1 + 6, fill_top_y, x2 - 6, y2 - 6], fill=color)
    meniscus_h = max(6, (x2 - x1) // 10)
    L.ellipse(
        [x1 + 6, fill_top_y - meniscus_h, x2 - 6, fill_top_y + meniscus_h], fill=color
    )
    base_img.alpha_composite(liquid)
    fx = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    H = ImageDraw.Draw(fx, "RGBA")
    H.arc(
        [x1 + 12, fill_top_y - meniscus_h // 2, x2 - 12, fill_top_y + meniscus_h // 2],
        start=0,
        end=180,
        fill=(255, 255, 255, 200),
        width=2,
    )
    H.arc(
        [
            x1 + 14,
            fill_top_y - meniscus_h // 2 + 4,
            x2 - 14,
            fill_top_y + meniscus_h // 2 + 4,
        ],
        start=0,
        end=180,
        fill=(50, 50, 50, 80),
        width=2,
    )
    base_img.alpha_composite(fx)


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
    ticks_x_left = x1 + 16
    ticks_x_right = x1 + (x2 - x1) // 2 + 6

    n_major = 10
    major_ml = max(
        minor_step_ml, round((capacity_ml / n_major) / minor_step_ml) * minor_step_ml
    )
    mid_ml = max(minor_step_ml, round((major_ml / 2) / minor_step_ml) * minor_step_ml)

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
            # label every major step; keep density reasonable
            val = ml if minor_step_ml >= 1 else round(ml, 1)
            val = _to_display(val, unit)
            text = _format_label(val, unit)
            tw, th = draw.textbbox((0, 0), text, font=label_font)[2:]
            tx = ticks_x_left + length + 4
            draw.text((tx, y - th // 2), text, fill=label_color, font=label_font)


def _draw_vertical_unit(draw: ImageDraw.ImageDraw, text, pos, color=(213, 75, 43)):
    x, y = pos
    font = _load_font(28)
    dy = 0
    for ch in text:
        tw, th = draw.textbbox((0, 0), ch, font=font)[2:]
        draw.text((x, y + dy), ch, fill=color, font=font)
        dy += th + 2


def _drop_shadow(img: Image.Image, bbox):
    x1, y1, x2, y2 = bbox
    shadow = Image.new("RGBA", img.size, (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow, "RGBA")
    off = 10
    sd.rounded_rectangle(
        [x1 + off, y1 + off, x2 + off, y2 + off], radius=16, fill=(0, 0, 0, 130)
    )
    shadow = shadow.filter(ImageFilter.GaussianBlur(18))
    img.alpha_composite(shadow)


def _top_rim_specular(img: Image.Image, tube_bbox):
    x1, y1, x2, y2 = tube_bbox
    rim = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(rim, "RGBA")
    d.arc(
        [x1 + 8, y1 + 6, x2 - 8, y1 + 26],
        start=200,
        end=-20,
        fill=(255, 255, 255, 220),
        width=3,
    )
    d.arc(
        [x1 + 7, y1 + 5, x2 - 7, y1 + 25],
        start=200,
        end=-20,
        fill=(255, 210, 210, 90),
        width=1,
    )
    d.arc(
        [x1 + 9, y1 + 7, x2 - 9, y1 + 27],
        start=200,
        end=-20,
        fill=(210, 210, 255, 90),
        width=1,
    )
    img.alpha_composite(rim)


@registry.register(name="measuring_cylinder3", tags={"measuring_cylinder"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    capacity_ml = _choose_capacity_ml()
    minor_step_ml = _choose_minor_step_ml(capacity_ml)
    unit = _choose_display_unit(capacity_ml)
    W, H = 720, 1080
    img = Image.new("RGBA", (W, H), (242, 245, 248, 255))
    bg = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    gd = ImageDraw.Draw(bg, "RGBA")
    gd.rectangle([0, int(H * 0.62), W, H], fill=(225, 232, 238, 255))
    for r in range(0, max(W, H), 8):
        alpha = max(0, 90 - r // 10)
        gd.ellipse(
            [W // 2 - r, H // 2 - r, W // 2 + r, H // 2 + r],
            outline=(0, 0, 0, alpha),
            width=2,
        )
    img = Image.alpha_composite(img, bg.filter(ImageFilter.GaussianBlur(12)))
    draw = ImageDraw.Draw(img, "RGBA")

    tube_top, tube_bottom = 160, H - 180
    tube_left, tube_right = W // 2 - 70, W // 2 + 70
    tube_bbox = (tube_left, tube_top, tube_right, tube_bottom)

    _drop_shadow(img, tube_bbox)
    _draw_glass_base(img, tube_bbox)
    _apply_glass_effects(img, tube_bbox)

    inner_top, inner_bottom = tube_top + 18, tube_bottom - 18
    inner_height = inner_bottom - inner_top
    px_per_ml = inner_height / float(capacity_ml)
    volume_ml = round(random.uniform(0.05 * capacity_ml, 0.95 * capacity_ml), 2)
    fill_top_y = int(round(inner_bottom - volume_ml * px_per_ml))

    amber_variants = [
        (227, 164, 36, 220),
        (214, 146, 30, 220),
        (236, 175, 60, 220),
        (200, 135, 30, 220),
        (140, 160, 220, 210),
        (160, 200, 180, 210),
    ]
    _draw_liquid(
        img,
        (tube_left, inner_top, tube_right, inner_bottom),
        fill_top_y,
        color=random.choice(amber_variants),
    )

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

    _top_rim_specular(img, tube_bbox)

    orange = (213, 75, 43)
    small_font = _load_font(20)
    draw.text((tube_left + 18, tube_top + 28), "Ex20° C", fill=orange, font=small_font)
    draw.text((tube_left + 108, tube_top + 56), unit, fill=orange, font=small_font)
    # _draw_vertical_unit(draw, unit.lower(), (tube_left + 18, tube_top + 54), color=orange)

    base_y = tube_bottom + 22
    base = Image.new("RGBA", img.size, (0, 0, 0, 0))
    bd = ImageDraw.Draw(base, "RGBA")
    bd.ellipse(
        [W // 2 - 120, base_y, W // 2 + 120, base_y + 30],
        outline=(60, 60, 60, 220),
        fill=(235, 236, 240, 255),
    )
    bd.ellipse(
        [W // 2 - 118, base_y + 2, W // 2 + 118, base_y + 28],
        outline=(255, 255, 255, 110),
        fill=None,
        width=2,
    )
    base = base.filter(ImageFilter.GaussianBlur(0.5))
    img.alpha_composite(base)

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


# Demo run
if __name__ == "__main__":
    out = generate("out.jpg")
    print(out)
