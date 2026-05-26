# Round-bezel voltmeter style derived from ammeter3.
# One public entrypoint: generate(img_path) -> Artifact
# - Circular black bezel with screw holes
# - White circular dial with rail-style scale (0..100), major every 20, readable tick marks
# - Random unit among V / mV / µV (center unit text)
# - Red needle pivoting from a lower position, bottom mechanical window & screws

from PIL import Image, ImageDraw, ImageFont, ImageFilter
import math
import random

from registry import registry
from artifacts import Artifact


def _measure(draw, text, font):
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0], bbox[3] - bbox[1]


def _polar(cx, cy, r, ang_deg):
    a = math.radians(ang_deg)
    return cx + r * math.cos(a), cy + r * math.sin(a)


def _screw(draw, x, y, r, fill_outer=(55, 55, 60), fill_inner=(170, 170, 175)):
    draw.ellipse((x - r, y - r, x + r, y + r), fill=fill_outer)
    r2 = int(r * 0.68)
    draw.ellipse((x - r2, y - r2, x + r2, y + r2), fill=fill_inner)
    # Cross slot
    draw.line(
        (x - r2 * 0.8, y, x + r2 * 0.8, y),
        fill=(90, 90, 95),
        width=max(1, int(r * 0.15)),
    )
    draw.line(
        (x, y - r2 * 0.8, x, y + r2 * 0.8),
        fill=(90, 90, 95),
        width=max(1, int(r * 0.15)),
    )


def _soft_highlight(base_img, circle_bbox):
    # Gentle radial highlight near top-left
    x0, y0, x1, y1 = circle_bbox
    w, h = x1 - x0, y1 - y0
    layer = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    # Clip: simple ellipse feathering
    d.ellipse(circle_bbox, fill=(255, 255, 255, 0), outline=None)
    poly = [
        (x0 + 0.18 * w, y0 + 0.18 * h),
        (x0 + 0.60 * w, y0 + 0.12 * h),
        (x0 + 0.82 * w, y0 + 0.22 * h),
        (x0 + 0.38 * w, y0 + 0.28 * h),
    ]
    d.polygon(poly, fill=(255, 255, 255, 120))
    layer = layer.filter(ImageFilter.GaussianBlur(radius=int(0.03 * (w + h) / 2)))
    base_img.alpha_composite(layer)


@registry.register(name="voltmeter_circle", tags={"voltmeter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    S = 2
    W, H = 900 * S, 900 * S
    img = Image.new("RGBA", (W, H), (230, 232, 234, 255))
    draw = ImageDraw.Draw(img)

    cx, cy = W // 2, H // 2

    # --- black bezel ---
    R_outer = int(0.44 * W)
    draw.ellipse(
        (cx - R_outer, cy - R_outer, cx + R_outer, cy + R_outer),
        fill=(35, 36, 38),
        outline=(20, 20, 22),
        width=6 * S,
    )
    # inner chamfer ring
    R_mid = int(R_outer * 0.86)
    draw.ellipse(
        (cx - R_mid, cy - R_mid, cx + R_mid, cy + R_mid),
        outline=(90, 92, 96),
        width=8 * S,
    )

    # screw holes on bezel (12 and 6 o'clock)
    _screw(draw, cx, cy - int(R_outer * 0.78), int(12 * S))
    _screw(draw, cx, cy + int(R_outer * 0.78), int(12 * S))

    # --- white dial circle ---
    R_dial = int(R_outer * 0.74)
    dial_bbox = (cx - R_dial, cy - R_dial, cx + R_dial, cy + R_dial)
    draw.ellipse(dial_bbox, fill=(250, 250, 252), outline=(190, 192, 196), width=5 * S)

    # soft glass highlight under ticks/text
    _soft_highlight(img, dial_bbox)

    # --- scale geometry (rail + ticks) ---
    # Pivot is slightly below the dial center
    pivot_y_offset = int(R_dial * 0.22)
    px, py = cx, cy + pivot_y_offset
    start_deg, end_deg = 210, 330
    span = end_deg - start_deg

    # Rail band
    R_arc_outer = int(R_dial * 0.52)
    band_w = int(0.06 * R_dial)
    R_arc_inner = R_arc_outer - band_w
    draw.arc(
        (px - R_arc_outer, py - R_arc_outer, px + R_arc_outer, py + R_arc_outer),
        start=start_deg,
        end=end_deg,
        fill=(55, 58, 62),
        width=3 * S,
    )
    draw.arc(
        (px - R_arc_inner, py - R_arc_inner, px + R_arc_inner, py + R_arc_inner),
        start=start_deg,
        end=end_deg,
        fill=(55, 58, 62),
        width=3 * S,
    )

    # Fixed scale like the reference: 0..100, major every 20, 5 minors
    unit = random.choice(["V", "mV", "µV"])
    FS = 100.0
    major = 20.0
    minors = 5
    smallest_div = major / minors
    total_minors = int(FS / smallest_div)

    # ticks
    for i in range(total_minors + 1):
        v = i * smallest_div
        ang = start_deg + span * (v / FS)
        # tick anchored between inner/outer rail
        is_major = abs((v % major)) < 1e-9 or abs(major - (v % major)) < 1e-9
        r0 = R_arc_inner
        r1 = R_arc_outer
        # Make ticks intrude into the band
        inner = r0 + (band_w * 0.18 if is_major else band_w * 0.36)
        outer = r1 - (band_w * 0.18 if is_major else band_w * 0.10)
        x1, y1 = _polar(px, py, inner, ang)
        x2, y2 = _polar(px, py, outer, ang)
        draw.line(
            (x1, y1, x2, y2), fill=(30, 30, 30), width=4 * S if is_major else 2 * S
        )

    # labels at majors
    try:
        font_num = ImageFont.truetype("DejaVuSans.ttf", 22 * S)
        font_mid = ImageFont.truetype("DejaVuSans.ttf", 26 * S)
        font_small = ImageFont.truetype("DejaVuSans.ttf", 18 * S)
    except Exception as e:
        print(e)
        font_num = font_mid = font_small = ImageFont.load_default()

    for j in range(int(FS / major) + 1):
        v = j * major
        ang = start_deg + span * (v / FS)
        r_text = R_arc_outer + int(0.13 * R_dial)
        tx, ty = _polar(px, py, r_text, ang)
        label = f"{int(v)}"
        tw, th = _measure(draw, label, font_num)
        draw.text((tx - tw / 2, ty - th / 2), label, fill=(20, 20, 20), font=font_num)

    # center unit (V/mV/µV) with underscore
    uw, uh = _measure(draw, unit, font_mid)
    draw.text(
        (cx - uw / 2, cy - int(0.22 * R_dial) - uh // 2),
        unit,
        fill=(25, 25, 25),
        font=font_mid,
    )
    # underscore
    draw.line(
        (
            cx - uw * 0.35,
            cy - int(0.22 * R_dial) + uh * 0.65,
            cx + uw * 0.35,
            cy - int(0.22 * R_dial) + uh * 0.65,
        ),
        fill=(25, 25, 25),
        width=2 * S,
    )

    # branding and small legends
    brand = "VOLTEX"
    model = "VO-65"
    draw.text(
        (cx - int(0.52 * R_dial), cy + int(0.05 * R_dial)),
        brand,
        fill=(70, 72, 76),
        font=font_small,
    )
    draw.text(
        (cx - int(0.52 * R_dial), cy + int(0.12 * R_dial)),
        model,
        fill=(70, 72, 76),
        font=font_small,
    )

    right1 = "2.5  CE"
    fs_str = {"V": "DC50V", "mV": "DC50mV", "µV": "DC50µV"}[unit]
    right2 = f"F.S: {fs_str}"
    draw.text(
        (cx + int(0.20 * R_dial), cy + int(0.05 * R_dial)),
        right1,
        fill=(70, 72, 76),
        font=font_small,
    )
    draw.text(
        (cx + int(0.10 * R_dial), cy + int(0.12 * R_dial)),
        right2,
        fill=(70, 72, 76),
        font=font_small,
    )

    # bottom mechanical window and hardware
    cut_r = int(0.26 * R_dial)
    cut_bbox = (
        cx - cut_r,
        cy + int(0.24 * R_dial) - cut_r,
        cx + cut_r,
        cy + int(0.24 * R_dial) + cut_r,
    )
    draw.pieslice(cut_bbox, start=200, end=340, fill=(38, 40, 42))
    # two silver screws
    _screw(
        draw,
        cx - int(0.18 * R_dial),
        cy + int(0.33 * R_dial),
        int(7 * S),
        fill_outer=(70, 72, 75),
        fill_inner=(190, 190, 195),
    )
    _screw(
        draw,
        cx + int(0.18 * R_dial),
        cy + int(0.33 * R_dial),
        int(7 * S),
        fill_outer=(70, 72, 75),
        fill_inner=(190, 190, 195),
    )
    # center red LED-like cap
    draw.ellipse(
        (
            cx - 9 * S,
            cy + int(0.30 * R_dial) - 9 * S,
            cx + 9 * S,
            cy + int(0.30 * R_dial) + 9 * S,
        ),
        fill=(210, 35, 38),
        outline=(110, 0, 0),
    )

    # random reading & needle
    reading = round(random.uniform(0.04 * FS, 0.96 * FS), 6)
    ang = start_deg + span * (reading / FS)
    needle_len = R_arc_outer - int(0.07 * R_dial)
    nx, ny = _polar(px, py, needle_len, ang)
    draw.line((px, py, nx, ny), fill=(210, 40, 45), width=5 * S)
    draw.ellipse(
        (px - 12 * S, py - 12 * S, px + 12 * S, py + 12 * S), fill=(20, 20, 22)
    )

    # export
    out = img.resize((W // S, H // S), Image.Resampling.LANCZOS).convert("RGB")
    out.save(img_path, quality=95)

    # evaluator interval around reading, with unit synonyms
    lo = round(max(0.0, reading - smallest_div / 4), 2)
    hi = round(min(FS, reading + smallest_div / 4), 2)
    unit_map = {
        "V": ["V", "Volt"],
        "mV": ["mV", "millivolt"],
        "µV": ["µV", "microvolt"],
    }
    return Artifact(
        data=img_path,
        image_type="voltmeter",
        design="Dial",
        evaluator_kwargs={"interval": [lo, hi], "units": unit_map[unit]},
    )


if __name__ == "__main__":
    result = generate("out.jpg")
    print(result)
