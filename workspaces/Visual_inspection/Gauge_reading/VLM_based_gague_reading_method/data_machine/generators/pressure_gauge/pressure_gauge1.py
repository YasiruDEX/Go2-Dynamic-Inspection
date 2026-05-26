"""
Synthetic Analog Pressure Gauge Image Generator
Offline-only: uses Pillow (PIL), numpy, random, math.

Public API:
    def generate(img_path: str) -> dict

Behavior:
- Randomizes many independent visual + metrological factors (size, palette, bezel, start/sweep,
  direction, units/range, tick spacing, label density, needle style, colored zones, background,
  materials, glare/noise, etc.).
- Encodes the chosen target_reading via the needle angle on a circular dial.
- Draws only ticks/unit labels/brand marks (never the exact randomized reading as text).
- Returns evaluator metadata with an interval equal to ±(smallest resolvable step), adapting to
  tick density and rendering resolution.
"""

import math
import random
from typing import Tuple, List

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from registry import registry
from artifacts import Artifact

# ----------------------------- Utility helpers (internal) -----------------------------


def _rand_choice(pairs):
    return random.choice(pairs)


def _deg2rad(a):
    return a * math.pi / 180.0


def _rad2deg(a):
    return a * 180.0 / math.pi


def _polar(
    center: Tuple[float, float], r: float, ang_rad: float
) -> Tuple[float, float]:
    cx, cy = center
    return (cx + r * math.cos(ang_rad), cy + r * math.sin(ang_rad))


def _rounded_rectangle(
    draw: ImageDraw.ImageDraw, xy, radius, fill=None, outline=None, width=1
):
    # Simple rounded rectangle using pieslice + rectangles
    (x0, y0, x1, y1) = xy
    r = min(radius, (x1 - x0) / 2, (y1 - y0) / 2)
    draw.rounded_rectangle(xy, radius=r, fill=fill, outline=outline, width=width)


def _soft_vignette(im: Image.Image, strength=0.25):
    w, h = im.size
    y, x = np.ogrid[-1 : 1 : h * 1j, -1 : 1 : w * 1j]
    mask = x**2 + y**2
    mask = (mask - mask.min()) / (mask.max() - mask.min() + 1e-6)
    mask = np.clip(mask, 0, 1)
    mask = (mask**1.5) * strength  # emphasis at corners
    arr = np.array(im).astype(np.float32)
    arr *= 1 - mask[..., None]
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    return Image.fromarray(arr)


def _grain(im: Image.Image, sigma=6, amount=0.06):
    # Add gentle film-like grain
    w, h = im.size
    noise = np.random.normal(0, 255 * amount, (h, w, 1)).astype(np.float32)
    arr = np.array(im).astype(np.float32)
    arr = np.clip(arr + noise, 0, 255).astype(np.uint8)
    out = Image.fromarray(arr)
    if sigma > 0:
        out = out.filter(ImageFilter.GaussianBlur(radius=max(0, sigma * 0.12)))
    return out


def _ring(
    draw: ImageDraw.ImageDraw,
    bbox,
    inner_ratio=0.84,
    base=(180, 180, 185),
    highlight=(230, 230, 235),
    shadow=(120, 120, 125),
):
    # Simple metallic bezel ring illusion using concentric ovals
    x0, y0, x1, y1 = bbox
    draw.ellipse(bbox, fill=base)
    # inner cut
    mx = (x0 + x1) / 2
    my = (y0 + y1) / 2
    rx = (x1 - x0) * inner_ratio / 2
    ry = (y1 - y0) * inner_ratio / 2
    inner_bbox = (mx - rx, my - ry, mx + rx, my + ry)
    draw.ellipse(inner_bbox, fill=highlight)
    # subtle shadow inner lip
    lip = 0.94
    rx2 = (x1 - x0) * inner_ratio * lip / 2
    ry2 = (y1 - y0) * inner_ratio * lip / 2
    inner2 = (mx - rx2, my - ry2, mx + rx2, my + ry2)
    draw.ellipse(inner2, outline=shadow, width=max(1, int((x1 - x0) * 0.004)))


def _draw_arc_thick(draw: ImageDraw.ImageDraw, bbox, start_deg, end_deg, color, width):
    draw.arc(bbox, start=start_deg, end=end_deg, fill=color, width=width)


def _text(draw: ImageDraw.ImageDraw, xy, text, fill, anchor="mm", font=None):
    try:
        draw.text(xy, text, fill=fill, anchor=anchor, font=font)
    except TypeError:
        # Older Pillow may not support anchor; fallback approximate centering
        w, h = draw.textsize(text, font=font)
        x, y = xy
        draw.text((x - w / 2, y - h / 2), text, fill=fill, font=font)


def _unit_synonyms(unit_key: str) -> List[str]:
    table = {
        "psi": ["psi", "PSI"],
        "kPa": ["kPa"],
        "bar": ["bar", "bars"],
        "MPa": ["MPa"],
    }
    return table.get(unit_key, [unit_key])


def _clip(v, a, b):
    return max(a, min(b, v))


# ----------------------------- Main generator -----------------------------


@registry.register(name="pressure_gauge1", tags={"pressure_gauge"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """
    Render a synthetic Pressure Gauge image and save it to img_path.
    Returns:
        Artifact:
            "design": <short textual summary of the final visual/scale design>,
            "evaluator_kwargs": {
                "interval": [<lower_bound>, <upper_bound>],
                "units": [<list of acceptable unit strings>]
            },
            "evaluator": <evaluator name>,
            "image_type": "pressure_gauge",
            "design": "Dial",
        }
    """
    # ------------------ Global image + camera/layout randomization ------------------
    img_size = random.choice([384, 512, 640])
    im = Image.new("RGB", (img_size, img_size), (240, 240, 240))
    draw = ImageDraw.Draw(im)

    # Background style
    bg_mode = random.choice(["radial_grad", "linear_grad", "noisy"])
    base_bg = tuple(random.randint(210, 245) for _ in range(3))
    if bg_mode in ("radial_grad", "linear_grad"):
        # simple gradient via numpy
        h, w = img_size, img_size
        if bg_mode == "radial_grad":
            y, x = np.ogrid[-1 : 1 : h * 1j, -1 : 1 : w * 1j]
            d = np.sqrt(x**2 + y**2)
            g = 1 - np.clip(d, 0, 1)  # center bright
        else:
            x = np.linspace(0, 1, w)[None, :]
            g = np.repeat(x, h, axis=0)  # left-to-right
            if random.random() < 0.5:
                g = g[:, ::-1]
        tint = np.array(base_bg, dtype=np.float32)
        low = tint - np.array([15, 15, 18], dtype=np.float32)
        high = tint + np.array([8, 8, 10], dtype=np.float32)
        grad = (
            low[None, None, :] * (1 - g[..., None]) + high[None, None, :] * g[..., None]
        )
        grad = np.clip(grad, 0, 255).astype(np.uint8)
        im = Image.fromarray(grad)
        draw = ImageDraw.Draw(im)
    else:
        im = _grain(im, sigma=4, amount=0.04)
        draw = ImageDraw.Draw(im)

    # Panel cutout or wall plate (rounded rectangle)
    if random.random() < 0.65:
        pad = int(img_size * random.uniform(0.04, 0.08))
        plate_bbox = (pad, pad, img_size - pad, img_size - pad)
        plate_radius = int(img_size * random.uniform(0.04, 0.08))
        plate_fill = tuple(random.randint(210, 230) for _ in range(3))
        plate_edge = tuple(random.randint(150, 170) for _ in range(3))
        _rounded_rectangle(
            draw,
            plate_bbox,
            plate_radius,
            fill=plate_fill,
            outline=plate_edge,
            width=max(2, img_size // 160),
        )

    # Slight perspective/rotation
    tilt_deg = random.uniform(-3.0, 3.0)

    # ------------------ Dial + bezel geometry ------------------
    cx = cy = img_size // 2
    center = (cx, cy)
    outer_r = int(img_size * random.uniform(0.43, 0.47))  # bezel outer radius
    face_r = int(outer_r * random.uniform(0.78, 0.84))  # inner face radius

    # Bezel ring
    bezel_bbox = (cx - outer_r, cy - outer_r, cx + outer_r, cy + outer_r)
    _ring(
        draw,
        bezel_bbox,
        inner_ratio=random.uniform(0.84, 0.90),
        base=tuple(random.randint(165, 190) for _ in range(3)),
        highlight=tuple(random.randint(215, 235) for _ in range(3)),
        shadow=tuple(random.randint(95, 115) for _ in range(3)),
    )

    # Faceplate
    face_color = random.choice(
        [(248, 248, 245), (252, 252, 252), (245, 248, 252), (250, 246, 240)]
    )
    draw.ellipse(
        (cx - face_r, cy - face_r, cx + face_r, cy + face_r),
        fill=face_color,
        outline=(180, 180, 180),
        width=max(1, img_size // 256),
    )

    # Optional screws
    if random.random() < 0.65:
        screw_r = max(2, img_size // 64)
        for ang in np.linspace(0, 2 * math.pi, 5)[:-1]:
            sx, sy = _polar(
                center, outer_r * 0.88, ang + _deg2rad(random.choice([0, 45]))
            )
            draw.ellipse(
                (sx - screw_r, sy - screw_r, sx + screw_r, sy + screw_r),
                fill=(150, 150, 150),
                outline=(90, 90, 90),
            )

    # ------------------ Scale design: units, range, ticks, sweep ------------------
    # Choose primary unit and scale presets common to pressure gauges
    unit_choice = random.choice(["psi", "kPa", "bar", "MPa"])

    # Preset ranges/major steps (realistic)
    presets = {
        "psi": [(0, 60, 10), (0, 100, 20), (0, 160, 20), (0, 200, 20), (0, 300, 50)],
        "kPa": [(0, 400, 50), (0, 600, 100), (0, 1000, 100), (0, 1600, 200)],
        "bar": [(0, 6, 1), (0, 10, 1), (0, 16, 2)],
        "MPa": [(0, 0.6, 0.1), (0, 1.0, 0.1), (0, 1.6, 0.2)],
    }
    scale_min, scale_max, major_step = random.choice(presets[unit_choice])

    # minor subdiv per major tick (3–5 typically)
    minor_div = random.choice([3, 4, 5])
    minor_step = major_step / minor_div

    # Start angle & sweep (diverse but readable)
    start_deg = random.uniform(200, 240)  # left-bottom quadrant
    sweep_deg = random.uniform(240, 300)  # big arc across the top
    clockwise = random.random() < 0.6  # many industrial dials are clockwise increasing
    direction = -1.0 if clockwise else +1.0

    # Label density (show every Nth major)
    label_every = random.choice([1, 1, 2])  # occasionally sparser labels
    label_inside = random.random() < 0.5  # label placement

    # Avoid zero-width range
    assert scale_max > scale_min

    def angle_from_value(val: float) -> float:
        t = (val - scale_min) / (scale_max - scale_min)
        return _deg2rad(start_deg + direction * (sweep_deg * t))

    def value_from_angle(theta: float) -> float:
        # inverse mapping
        a0 = _deg2rad(start_deg)
        dt = (theta - a0) / _deg2rad(sweep_deg)
        val = scale_min + (dt * (scale_max - scale_min) * (1.0 / direction))
        return val

    # ------------------ Target reading (avoid exact labeled ticks) ------------------
    labeled_values = set()
    v = scale_min
    while v <= scale_max + 1e-9:
        labeled_values.add(round(v, 6))
        v += major_step

    # sample reading strictly inside bounds, and not exactly on a labeled major tick
    for _ in range(999):
        target = random.uniform(scale_min, scale_max)
        # keep inside by at least a half-minor step to avoid needle clipping labels
        margin = minor_step * 0.5
        if target <= scale_min + margin or target >= scale_max - margin:
            continue
        if round(target, 6) in labeled_values:
            continue
        break
    else:
        # Fallback mid-scale
        target = (scale_min + scale_max) / 2.0 + 0.33 * minor_step

    theta = angle_from_value(target)

    # ------------------ Colored zones (green/yellow/red bands) ------------------
    zone_style = random.choice(["none", "gyr", "ok_high"])
    arc_w = max(4, int(face_r * 0.08))
    ring_bbox = (
        cx - int(face_r * 0.90),
        cy - int(face_r * 0.90),
        cx + int(face_r * 0.90),
        cy + int(face_r * 0.90),
    )

    if zone_style == "gyr":
        # 0–60% green, 60–85% yellow, 85–100% red (approx, but shuffled)
        splits = [0.0, random.uniform(0.5, 0.65), random.uniform(0.75, 0.9), 1.0]
        cols = [(80, 160, 80), (220, 200, 60), (200, 80, 70)]
        # optionally shuffle colors/zones slightly
        if random.random() < 0.35:
            cols = [(80, 150, 170), (230, 165, 60), (200, 70, 70)]
        for (a, b), c in zip(zip(splits[:-1], splits[1:]), cols):
            s = start_deg + direction * (sweep_deg * a)
            e = start_deg + direction * (sweep_deg * b)
            _draw_arc_thick(draw, ring_bbox, s, e, c, arc_w)
    elif zone_style == "ok_high":
        # acceptable band midrange & a red high-risk band
        ok_a = random.uniform(0.25, 0.4)
        ok_b = random.uniform(0.6, 0.75)
        hi_a = random.uniform(0.85, 0.9)
        hi_b = 1.0
        for (a, b), c in [((ok_a, ok_b), (70, 150, 90)), ((hi_a, hi_b), (200, 60, 60))]:
            s = start_deg + direction * (sweep_deg * a)
            e = start_deg + direction * (sweep_deg * b)
            _draw_arc_thick(draw, ring_bbox, s, e, c, arc_w)

    # ------------------ Ticks & labels ------------------
    tick_outer = face_r * 0.86
    major_len = face_r * 0.14
    minor_len = face_r * 0.08
    tick_inner_major = tick_outer - major_len
    tick_inner_minor = tick_outer - minor_len

    tick_color = (30, 30, 30)
    label_color = (20, 20, 20)
    unit_color = (25, 25, 25)

    # Ticks
    # Minor ticks
    total_steps = int(round((scale_max - scale_min) / minor_step))
    for i in range(total_steps + 1):
        val = scale_min + i * minor_step
        # avoid drawing a minor tick where a major sits (to keep styles distinct)
        major_mod = (
            abs((val - scale_min) / major_step - round((val - scale_min) / major_step))
            < 1e-6
        )
        is_major = major_mod
        theta_i = angle_from_value(val)
        p1 = _polar(center, tick_outer, theta_i)
        p2 = _polar(center, tick_inner_major if is_major else tick_inner_minor, theta_i)
        draw.line([p1, p2], fill=tick_color, width=max(1, img_size // 256))

    # Major labels
    font = ImageFont.load_default()
    label_r = face_r * (0.62 if label_inside else 0.95)
    lab_index = 0
    v = scale_min
    while v <= scale_max + 1e-9:
        if lab_index % label_every == 0:
            th = angle_from_value(v)
            lx, ly = _polar(center, label_r, th)
            text = f"{v:.0f}" if abs(v) >= 1 else f"{v:.2g}"
            _text(draw, (lx, ly), text, label_color, anchor="mm", font=font)
        v += major_step
        lab_index += 1

    # Unit label(s)
    primary_units_str = random.choice(_unit_synonyms(unit_choice))
    _text(
        draw,
        (cx, cy + face_r * random.uniform(0.18, 0.26)),
        primary_units_str,
        unit_color,
        anchor="mm",
        font=font,
    )

    # Branding mark (generic)
    if random.random() < 0.75:
        brand = random.choice(
            ["FLAGEVAL", "ACME", "OMEGA", "GAU-TEK", "NEXA", "VARIOMETER"]
        )
        _text(
            draw,
            (cx, cy - face_r * random.uniform(0.22, 0.16)),
            brand,
            (30, 30, 30),
            anchor="mm",
            font=font,
        )

    # ------------------ Needle ------------------
    # Style choices
    needle_color = random.choice(
        [(180, 30, 30), (25, 25, 25), (180, 60, 20), (25, 80, 140)]
    )
    hub_color = random.choice([(20, 20, 20), (160, 160, 160), (50, 50, 50)])
    needle_len = face_r * random.uniform(0.74, 0.82)
    base_w = max(3, int(face_r * random.uniform(0.02, 0.03)))
    tip_w = max(1, int(base_w * random.uniform(0.25, 0.45)))

    # Build needle polygon (tapered)
    def _rot(p, ang):
        x, y = p
        ca, sa = math.cos(ang), math.sin(ang)
        return (x * ca - y * sa, x * sa + y * ca)

    # Needle defined in its local frame, then rotated to theta
    body_len = needle_len * 0.92
    tip_len = needle_len * 0.08
    pts_local = [
        (-base_w, 0),
        (body_len, tip_w),
        (body_len + tip_len, 0),
        (body_len, -tip_w),
    ]
    # Rotate and translate
    pts = []
    for x, y in pts_local:
        xr, yr = _rot((x, y), theta)
        pts.append((cx + xr, cy + yr))
    draw.polygon(pts, fill=needle_color)

    # Counterweight (optional)
    if random.random() < 0.6:
        cw_r = face_r * random.uniform(0.06, 0.09)
        cw_off = face_r * random.uniform(0.02, 0.03)
        opp = theta + math.pi
        cwx, cwy = _polar(center, cw_off, opp)
        draw.ellipse(
            (cwx - cw_r, cwy - cw_r, cwx + cw_r, cwy + cw_r), fill=needle_color
        )

    # Hub
    hub_r = face_r * random.uniform(0.045, 0.06)
    draw.ellipse(
        (cx - hub_r, cy - hub_r, cx + hub_r, cy + hub_r),
        fill=hub_color,
        outline=(20, 20, 20),
        width=max(1, img_size // 256),
    )

    # ------------------ Glass glare & finishing ------------------
    if random.random() < 0.85:
        glare = Image.new("L", (img_size, img_size), 0)
        gd = ImageDraw.Draw(glare)
        glare_r = face_r * random.uniform(0.85, 1.05)
        glare_bbox = (cx - glare_r, cy - glare_r, cx + glare_r, cy + glare_r)
        # soft diagonal glare
        for i in range(12):
            alpha = int(12 - i) * random.randint(6, 10)
            start = start_deg + i * 2
            end = start + sweep_deg * random.uniform(0.08, 0.14)
            _draw_arc_thick(
                gd, glare_bbox, start, end, alpha, width=max(2, img_size // 200)
            )
        glare = glare.filter(ImageFilter.GaussianBlur(radius=max(1, img_size // 80)))
        # apply as highlight
        arr = np.array(im).astype(np.float32)
        gla = np.array(glare)[..., None].astype(np.float32) / 255.0
        arr = np.clip(arr * (1 - 0.12 * gla) + 255 * (0.12 * gla), 0, 255).astype(
            np.uint8
        )
        im = Image.fromarray(arr)
        draw = ImageDraw.Draw(im)

    im = _soft_vignette(im, strength=random.uniform(0.18, 0.28))
    if random.random() < 0.6:
        im = _grain(im, sigma=5, amount=random.uniform(0.02, 0.05))

    # Subtle image-level rotation to simulate camera perspective
    if abs(tilt_deg) > 0.5:
        im = im.rotate(
            tilt_deg, resample=Image.BICUBIC, expand=False, fillcolor=base_bg
        )

    # ------------------ Evaluator tolerance (resolution-aware) ------------------
    # Instrument resolution limited by minor tick spacing and by angular pixels near the needle tip
    sweep_rad = _deg2rad(abs(sweep_deg))
    r_tip = needle_len
    # 1 pixel along arc corresponds to an angle of ~1/r radians
    reading_per_pixel = (scale_max - scale_min) * (1.0 / r_tip) / sweep_rad
    smallest_step = max(minor_step, reading_per_pixel)  # conservative
    lower = _clip(target - smallest_step / 4, scale_min, scale_max)
    upper = _clip(target + smallest_step / 4, scale_min, scale_max)

    # ------------------ Sanity checks ------------------
    assert scale_min < target < scale_max
    recovered = value_from_angle(theta)
    assert (
        abs(recovered - target) <= smallest_step + 1e-6
    ), "Inverse map mismatch vs. instrument resolution"

    # ------------------ Save ------------------
    im.save(img_path)

    result = Artifact(
        data=img_path,
        design="Dial",
        evaluator_kwargs={
            "interval": [round(lower, 2), round(upper, 2)],
            "units": _unit_synonyms(unit_choice),
        },
        evaluator="interval_matching",
        image_type="pressure_gauge",
    )
    return result


# If you want a quick local smoke test, you could run:
if __name__ == "__main__":
    meta = generate("pressure_gauge.png")
    print(meta)
