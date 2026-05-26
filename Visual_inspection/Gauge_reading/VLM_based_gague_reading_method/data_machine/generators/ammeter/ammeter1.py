# -*- coding: utf-8 -*-
"""
Synthetic Ammeter (Dial) Image Generator
Offline-only dependencies: Pillow (PIL), numpy, random, math
The module exposes exactly one public entrypoint: generate(img_path: str) -> dict
"""

import math
import random
from typing import Tuple, List

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

from registry import registry
from artifacts import Artifact

# ---------------------------- helpers (module-internal) ----------------------------


def _rand_choice(seq):
    return seq[random.randrange(len(seq))]


def _text_size(
    draw: ImageDraw.ImageDraw, text: str, font: ImageFont.FreeTypeFont
) -> Tuple[int, int]:
    # Robust text measurement across Pillow versions
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0], bbox[3] - bbox[1]


def _value_to_angle(
    v: float,
    vmin: float,
    vmax: float,
    start_deg: float,
    sweep_deg: float,
    clockwise: bool,
) -> float:
    t = (v - vmin) / float(vmax - vmin)
    t = max(0.0, min(1.0, t))
    if clockwise:
        return start_deg - sweep_deg * t
    else:
        return start_deg + sweep_deg * t


def _angle_to_value(
    angle_deg: float,
    vmin: float,
    vmax: float,
    start_deg: float,
    sweep_deg: float,
    clockwise: bool,
) -> float:
    if clockwise:
        t = (start_deg - angle_deg) / float(sweep_deg)
    else:
        t = (angle_deg - start_deg) / float(sweep_deg)
    t = max(0.0, min(1.0, t))
    return vmin + t * (vmax - vmin)


def _draw_round_rect(
    draw: ImageDraw.ImageDraw,
    box: Tuple[int, int, int, int],
    r: int,
    fill=None,
    outline=None,
    width: int = 1,
):
    x0, y0, x1, y1 = box
    r = max(0, min(r, (x1 - x0) // 2, (y1 - y0) // 2))
    draw.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=width)


def _polar_to_cart(
    cx: float, cy: float, r: float, ang_deg: float
) -> Tuple[float, float]:
    ang = math.radians(ang_deg)
    return cx + r * math.cos(ang), cy + r * math.sin(ang)


def _polyline(
    draw: ImageDraw.ImageDraw, pts: List[Tuple[float, float]], fill, width: int = 1
):
    if len(pts) >= 2:
        draw.line(pts, fill=fill, width=width, joint="curve")


def _soft_vignette(img: Image.Image):
    w, h = img.size
    yy, xx = np.mgrid[0:h, 0:w]
    cx, cy = w / 2.0, h / 2.0
    rr = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    mask = rr / rr.max()
    mask = np.clip((mask - 0.2) / (1.0 - 0.2), 0, 1)  # start vignette near 20% radius
    vignette = np.uint8(255 * (1 - 0.1 * mask))  # subtle 10% darkening
    arr = np.array(img).astype(np.float32)
    arr *= vignette[..., None] / 255.0
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    return Image.fromarray(arr)


def _grain(img: Image.Image, sigma: float = 6.0):
    if sigma <= 0:
        return img
    arr = np.array(img).astype(np.float32)
    noise = np.random.normal(0, sigma, size=arr.shape).astype(np.float32)
    arr = np.clip(arr + noise, 0, 255).astype(np.uint8)
    return Image.fromarray(arr)


def _ring(
    draw: ImageDraw.ImageDraw,
    center: Tuple[int, int],
    r_outer: int,
    r_inner: int,
    fill_outer,
    fill_inner,
):
    cx, cy = center
    bbox_out = [cx - r_outer, cy - r_outer, cx + r_outer, cy + r_outer]
    bbox_in = [cx - r_inner, cy - r_inner, cx + r_inner, cy + r_inner]
    draw.ellipse(bbox_out, fill=fill_outer)
    draw.ellipse(bbox_in, fill=fill_inner)


def _draw_glass_glare(
    img: Image.Image, center: Tuple[int, int], radius: int, intensity: float
):
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    owdraw = ImageDraw.Draw(overlay)
    cx, cy = center
    # a soft diagonal glare
    glare_r = int(radius * random.uniform(0.75, 1.0))
    bbox = [cx - glare_r, cy - glare_r, cx + glare_r, cy + glare_r]
    grad_steps = 80
    for i in range(grad_steps):
        a = int(255 * intensity * (1 - i / grad_steps) * 0.5)
        col = (255, 255, 255, a)
        owdraw.arc(bbox, start=30, end=120, fill=col, width=max(1, glare_r // 60))
        # shrink bbox a bit each step for soft gradient
        bbox = [bbox[0] + 1, bbox[1] + 1, bbox[2] - 1, bbox[3] - 1]
    img.alpha_composite(overlay)


def _draw_screws(
    draw: ImageDraw.ImageDraw,
    rect: Tuple[int, int, int, int],
    n: int,
    screw_r: int,
    color,
):
    x0, y0, x1, y1 = rect
    pts = []
    if n == 4:
        pts = [
            (x0 + 18, y0 + 18),
            (x1 - 18, y0 + 18),
            (x0 + 18, y1 - 18),
            (x1 - 18, y1 - 18),
        ]
    elif n == 2:
        pts = [(x0 + 18, y0 + 18), (x1 - 18, y1 - 18)]
    for sx, sy in pts:
        draw.ellipse(
            [sx - screw_r, sy - screw_r, sx + screw_r, sy + screw_r],
            fill=color,
            outline=(30, 30, 30),
        )
        # slot
        draw.line(
            [(sx - screw_r + 2, sy), (sx + screw_r - 2, sy)], fill=(30, 30, 30), width=1
        )


def _draw_terminals(
    draw: ImageDraw.ImageDraw,
    panel_box: Tuple[int, int, int, int],
    base_y: int,
    spacing: int,
    size: int,
):
    # Two binding posts at bottom (red / black)
    px = (panel_box[0] + panel_box[2]) // 2
    x_left = px - spacing // 2
    x_right = px + spacing // 2
    for x, col in [(x_left, (200, 40, 40)), (x_right, (20, 20, 20))]:
        # post base
        draw.rectangle(
            [x - size, base_y - size // 2, x + size, base_y + size // 2],
            fill=(80, 80, 80),
        )
        # nut
        draw.ellipse(
            [x - size // 2, base_y - size // 2, x + size // 2, base_y + size // 2],
            fill=col,
            outline=(230, 230, 230),
        )
        # tiny highlight
        draw.ellipse(
            [x - size // 4, base_y - size // 4, x + size // 4, base_y + size // 4],
            outline=(255, 255, 255),
        )


def _draw_colored_zones(
    draw: ImageDraw.ImageDraw,
    center: Tuple[int, int],
    r0: int,
    r1: int,
    start_deg: float,
    sweep_deg: float,
    vmin: float,
    vmax: float,
    clockwise: bool,
    zones: List[Tuple[float, float, Tuple[int, int, int]]],
):
    # zones expressed as [(low_val, high_val, color), ...]
    cx, cy = center
    for lo, hi, col in zones:
        lo = max(vmin, lo)
        hi = min(vmax, hi)
        if hi <= lo:
            continue
        ang0 = _value_to_angle(lo, vmin, vmax, start_deg, sweep_deg, clockwise)
        ang1 = _value_to_angle(hi, vmin, vmax, start_deg, sweep_deg, clockwise)
        # ensure ang0 < ang1 for drawing; handle clockwise by swapping if needed
        a0, a1 = (ang1, ang0) if ang1 < ang0 else (ang0, ang1)
        # draw multiple rings for thickness
        for rr in range(r0, r1 + 1):
            bbox_rr = [cx - rr, cy - rr, cx + rr, cy + rr]
            draw.arc(bbox_rr, start=a0, end=a1, fill=col, width=2)


def _draw_ticks_and_labels(
    draw: ImageDraw.ImageDraw,
    center: Tuple[int, int],
    radius: int,
    start_deg: float,
    sweep_deg: float,
    vmin: float,
    vmax: float,
    major_step: float,
    minor_step: float,
    clockwise: bool,
    label_every: float,
    font: ImageFont.ImageFont,
    label_ring: float,
    tick_col=(20, 20, 20),
    label_col=(20, 20, 20),
    target_value: float = None,
):
    cx, cy = center
    # minor ticks
    val = vmin
    while val <= vmax + 1e-9:
        ang = _value_to_angle(val, vmin, vmax, start_deg, sweep_deg, clockwise)
        inner = radius * 0.86
        outer = radius * 0.96
        is_major = abs((val - vmin) % major_step) < 1e-6 or abs((val - vmax)) < 1e-6
        if is_major:
            inner = radius * 0.80
        x0, y0 = _polar_to_cart(cx, cy, inner, ang)
        x1, y1 = _polar_to_cart(cx, cy, outer, ang)
        draw.line([(x0, y0), (x1, y1)], fill=tick_col, width=2 if is_major else 1)
        val += minor_step

    # labels
    val = vmin
    while val <= vmax + 1e-9:
        is_label = abs((val - vmin) % label_every) < 1e-6 or abs(val - vmax) < 1e-6
        if is_label:
            # Never place a label equal to the exact randomized reading (if it coincides)
            if target_value is not None and abs(val - target_value) < 1e-6:
                val += label_every
                continue
            ang = _value_to_angle(val, vmin, vmax, start_deg, sweep_deg, clockwise)
            tx, ty = _polar_to_cart(cx, cy, radius * label_ring, ang)
            txt = f"{int(val)}"
            tw, th = _text_size(draw, txt, font)
            draw.text((tx - tw / 2, ty - th / 2), txt, font=font, fill=label_col)
        val += label_every


def _needle_polygon(
    center: Tuple[int, int], ang_deg: float, r_tip: float, r_tail: float, width: float
) -> List[Tuple[float, float]]:
    cx, cy = center
    # create a tapered needle with an arrow/spade tip
    tip = _polar_to_cart(cx, cy, r_tip, ang_deg)
    left = _polar_to_cart(cx, cy, width, ang_deg - 90)
    right = _polar_to_cart(cx, cy, width, ang_deg + 90)
    tail = _polar_to_cart(cx, cy, r_tail, ang_deg + 180)
    return [tip, left, tail, right]


def _palette():
    # Diverse palettes; each call should vary >12 independent factors across the pipeline
    face_colors = [
        (245, 246, 242),  # off-white
        (250, 250, 250),  # bright white
        (238, 240, 245),  # cool gray
        (246, 244, 240),  # warm paper
    ]
    bezel_colors = [
        (40, 40, 45),  # matte black metal
        (70, 70, 75),  # dark gray steel
        (110, 90, 70),  # aged bronze
        (160, 160, 165),  # brushed aluminum
    ]
    accent_colors = [
        (200, 40, 40),  # red
        (30, 120, 200),  # blue
        (20, 140, 60),  # green
        (220, 140, 30),  # orange
    ]
    bg_modes = ["studio_gradient", "panel_plate", "plain"]
    return {
        "face": _rand_choice(face_colors),
        "bezel": _rand_choice(bezel_colors),
        "accent": _rand_choice(accent_colors),
        "ticks": (25, 25, 25),
        "labels": (30, 30, 30),
        "bg_mode": _rand_choice(bg_modes),
    }


def _draw_background(img: Image.Image, palette, face_rect: Tuple[int, int, int, int]):
    draw = ImageDraw.Draw(img)
    w, h = img.size
    mode = palette["bg_mode"]
    if mode == "studio_gradient":
        # vertical gradient
        top = np.array([random.randint(200, 235)] * 3, dtype=np.uint8)
        bot = np.array([random.randint(120, 160)] * 3, dtype=np.uint8)
        grad = np.linspace(0, 1, h)[:, None]
        arr = (top * (1 - grad) + bot * grad).astype(np.uint8)
        arr = np.repeat(arr, w, axis=1).T.swapaxes(0, 1)
        img.paste(Image.fromarray(arr))
    elif mode == "panel_plate":
        # subtle noise panel
        base = (210, 210, 210)
        draw.rectangle([0, 0, w, h], fill=base)
        img_blur = _grain(img, sigma=8.0).filter(ImageFilter.GaussianBlur(0.5))
        img.paste(img_blur)
    else:
        draw.rectangle([0, 0, w, h], fill=(235, 235, 240))

    # faint drop shadow for the instrument body
    shadow = Image.new("RGBA", img.size, (0, 0, 0, 0))
    sdraw = ImageDraw.Draw(shadow)
    x0, y0, x1, y1 = face_rect
    pad = 18
    sbox = [x0 - pad, y0 - pad, x1 + pad, y1 + pad]
    sdraw.rounded_rectangle(sbox, radius=20, fill=(0, 0, 0, 80))
    shadow = shadow.filter(ImageFilter.GaussianBlur(14))
    img.alpha_composite(shadow)


# ---------------------------- main entrypoint ----------------------------
@registry.register(name="ammeter1", tags={"ammeter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """Render a synthetic Ammeter image and save it to img_path.
    Returns:
        {
            "design": <short textual summary of the final visual/scale design>,
            "evaluator_kwargs": {
                "interval": [<lower_bound>, <upper_bound>],
                "units": [<list of acceptable unit strings>]
            }
        }
    """
    # ---------------- basic randomized setup ----------------
    size = _rand_choice([384, 512, 640])
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    # Face geometry & diversity knobs
    face_margin = random.randint(size // 12, size // 9)
    face_rect = (face_margin, face_margin, size - face_margin, size - face_margin)
    face_shape = _rand_choice(["circle", "rounded_square"])  # instrument face shape
    corner_radius = random.randint(16, 26)
    palette = _palette()

    # Background & body
    _draw_background(img, palette, face_rect)

    # Instrument body / panel
    body = Image.new("RGBA", img.size, (0, 0, 0, 0))
    bdraw = ImageDraw.Draw(body)
    # outer bezel
    _draw_round_rect(bdraw, face_rect, r=corner_radius, fill=palette["bezel"])
    body = body.filter(ImageFilter.GaussianBlur(0.3))
    img.alpha_composite(body)

    # inner face cutout
    inner_pad = random.randint(size // 36, size // 24)
    inner_rect = (
        face_rect[0] + inner_pad,
        face_rect[1] + inner_pad,
        face_rect[2] - inner_pad,
        face_rect[3] - inner_pad,
    )

    face_layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    fdraw = ImageDraw.Draw(face_layer)
    if face_shape == "circle":
        # draw circular face within inner_rect
        cx = (inner_rect[0] + inner_rect[2]) // 2
        cy = (inner_rect[1] + inner_rect[3]) // 2
        r = min(inner_rect[2] - inner_rect[0], inner_rect[3] - inner_rect[1]) // 2
        fdraw.ellipse(
            [cx - r, cy - r, cx + r, cy + r],
            fill=palette["face"],
            outline=(200, 200, 200),
        )
    else:
        _draw_round_rect(
            fdraw,
            inner_rect,
            r=corner_radius - 6,
            fill=palette["face"],
            outline=(200, 200, 200),
        )
    img.alpha_composite(face_layer)

    # screws and terminals (metallic casing + two terminals at bottom)
    _draw_screws(
        draw,
        face_rect,
        n=_rand_choice([2, 4]),
        screw_r=random.randint(5, 7),
        color=(180, 180, 180),
    )
    term_y = face_rect[3] + random.randint(size // 32, size // 24)
    _draw_terminals(
        draw,
        face_rect,
        base_y=term_y,
        spacing=random.randint(size // 4, size // 3),
        size=random.randint(size // 28, size // 22),
    )

    # ---------------- scale logic (linear dial 0–100 as requested) ----------------
    vmin, vmax = 0.0, 100.0
    unit = _rand_choice(["A", "mA"])  # per user requirement
    units_full = (
        ["A", "Ampere", "ampere", "amps"]
        if unit == "A"
        else ["mA", "milliampere", "milliamps", "milliamp"]
    )

    # Dial geometry: start & sweep (linear mapping), clockwise or counter-clockwise
    start_deg = random.uniform(200, 230)  # where the minimum value appears
    sweep_deg = random.uniform(210, 270)  # total sweep
    clockwise = _rand_choice([True, False])

    # Ticks & labels
    # Root cause of misalignment: label spacing was independent of major ticks,
    # which could place numbers (e.g., every 25) on non-major ticks (e.g., every 20).
    # Fix by tying label spacing to major tick spacing and ensuring minor ticks
    # divide the major step exactly.
    major_step = _rand_choice([10.0, 20.0, 25.0])
    # choose a minor step that divides major_step
    candidate_minors = [
        s
        for s in [1.0, 2.0, 5.0]
        if abs((major_step / s) - round(major_step / s)) < 1e-6
    ]
    if not candidate_minors:
        candidate_minors = [1.0]
    minor_step = _rand_choice(candidate_minors)
    # labels align with major ticks
    label_every = major_step

    # Sample a target reading strictly inside (avoid needle exactly on end stops)
    target = random.uniform(vmin + 0.5, vmax - 0.5)

    # Mapping to angle & inverse check
    ang_target = _value_to_angle(target, vmin, vmax, start_deg, sweep_deg, clockwise)
    inv = _angle_to_value(ang_target, vmin, vmax, start_deg, sweep_deg, clockwise)
    smallest_resolvable = 0.1  # as requested; matches evaluator interval ±0.1
    assert vmin < target < vmax, "Target out of bounds"
    assert (
        abs(inv - target) <= smallest_resolvable
    ), "Inverse mapping exceeds resolution tolerance"

    # ---------------- draw dial: ticks, colored zones, labels ----------------
    cx = (inner_rect[0] + inner_rect[2]) // 2
    cy = (inner_rect[1] + inner_rect[3]) // 2
    dial_r = int(0.42 * size)

    # optional colored safety zones (typical ammeter: green/yellow/red)
    zone_style = _rand_choice(["none", "safe_yellow_red", "green_red"])
    zones = []
    if zone_style == "safe_yellow_red":
        zones = [
            (0, 65, (40, 160, 70)),
            (65, 85, (230, 170, 30)),
            (85, 100, (200, 50, 50)),
        ]
    elif zone_style == "green_red":
        split = random.uniform(70, 85)
        zones = [
            (0, split, (40, 160, 70)),
            (split, 100, (200, 50, 50)),
        ]
    if zones:
        _draw_colored_zones(
            draw,
            (cx, cy),
            int(dial_r * 0.72),
            int(dial_r * 0.78),
            start_deg,
            sweep_deg,
            vmin,
            vmax,
            clockwise,
            zones,
        )

    # ticks & labels
    label_ring = random.uniform(0.58, 0.68)
    _draw_ticks_and_labels(
        draw,
        (cx, cy),
        dial_r,
        start_deg,
        sweep_deg,
        vmin,
        vmax,
        major_step,
        minor_step,
        clockwise,
        label_every,
        font,
        label_ring,
        tick_col=palette["ticks"],
        label_col=palette["labels"],
        target_value=target,
    )

    # Units & brand (allowed, but never the exact reading)
    brand = _rand_choice(["AMMETER", "METERWORKS", "FLAGLAB", "ELECTROTECH", "VOLTEX"])
    # place unit
    unit_txt = unit
    uw, uh = _text_size(draw, unit_txt, font)
    draw.text((cx - uw / 2, cy + dial_r * 0.15), unit_txt, font=font, fill=(50, 50, 50))
    # brand text near top center
    bw, bh = _text_size(draw, brand, font)
    draw.text((cx - bw / 2, cy - dial_r * 0.35), brand, font=font, fill=(70, 70, 70))

    # ---------------- indicator (needle, hub, counterweight) ----------------
    needle_style = _rand_choice(["tapered", "spade"])
    needle_col = _rand_choice([(210, 30, 30), (30, 30, 30), (30, 100, 180)])
    hub_style = _rand_choice(["cap_solid", "cap_ring"])

    # Needle polygon
    r_tip = dial_r * random.uniform(0.80, 0.88)
    r_tail = dial_r * random.uniform(0.12, 0.18)
    width = max(2, int(dial_r * random.uniform(0.020, 0.030)))
    poly = _needle_polygon((cx, cy), ang_target, r_tip, r_tail, width)

    # Spade tip decoration if chosen
    if needle_style == "spade":
        tipx, tipy = _polar_to_cart(cx, cy, r_tip + width * 0.6, ang_target)
        poly.insert(0, (tipx, tipy))

    draw.polygon(poly, fill=needle_col, outline=(20, 20, 20))

    # hub
    hub_r_outer = int(dial_r * random.uniform(0.05, 0.07))
    if hub_style == "cap_ring":
        _ring(
            draw,
            (cx, cy),
            hub_r_outer,
            int(hub_r_outer * 0.55),
            fill_outer=(120, 120, 120),
            fill_inner=(230, 230, 230),
        )
    else:
        draw.ellipse(
            [cx - hub_r_outer, cy - hub_r_outer, cx + hub_r_outer, cy + hub_r_outer],
            fill=(160, 160, 160),
            outline=(40, 40, 40),
        )

    # subtle glass glare (kept mild to avoid occluding the needle)
    if random.random() < 0.9:
        _draw_glass_glare(
            img, (cx, cy), int(dial_r * 1.05), intensity=random.uniform(0.15, 0.35)
        )

    # overall tilt / composition diversity
    if random.random() < 0.7:
        angle = random.uniform(-8, 8)
        bg_col = (235, 235, 240, 255)
        img = img.rotate(angle, resample=Image.BICUBIC, expand=False, fillcolor=bg_col)

    # mild vignette and film grain for realism (never hiding the indicator)
    if random.random() < 0.8:
        img = _soft_vignette(img)
    if random.random() < 0.8:
        img = _grain(img, sigma=random.uniform(2.0, 6.0))

    # Final composite to RGB and save
    out = Image.new("RGB", img.size, (240, 240, 242))
    out.paste(img.convert("RGBA"), mask=img.split()[-1] if img.mode == "RGBA" else None)
    out = out.filter(ImageFilter.UnsharpMask(radius=1.2, percent=90, threshold=3))
    out = out.resize((size, size), resample=Image.BICUBIC)
    out.save(img_path, quality=95)

    # evaluator interval (± smallest resolvable step = 0.1 as requested)
    lo = max(vmin, round(target - minor_step / 4, 1))
    hi = min(vmax, round(target + minor_step / 4, 1))
    print(lo, hi)
    return Artifact(
        data=img_path,
        image_type="ammeter",
        design="Dial",
        evaluator_kwargs={"interval": [lo, hi], "units": units_full},
    )


if __name__ == "__main__":
    generate("ammeter1.png")
