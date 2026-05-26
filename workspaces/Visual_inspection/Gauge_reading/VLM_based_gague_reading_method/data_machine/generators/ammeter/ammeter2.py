# Fix: put the glass highlight *beneath* ticks/labels/needle so it never occludes marks.
# Also soften the highlight and confine it to the dial area.
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import math
import random
from registry import registry
from artifacts import Artifact


def _measure(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont):
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0], bbox[3] - bbox[1]


def _polar(cx, cy, r, ang_deg):
    ang = math.radians(ang_deg)
    return cx + r * math.cos(ang), cy + r * math.sin(ang)


def _draw_soft_highlight(base_img, rect):
    # Draw a soft triangular highlight on a separate layer, blurred then composited.
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    layer = Image.new("RGBA", base_img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    poly = [
        (x0 + 0.10 * w, y0 + 0.18 * h),
        (x0 + 0.52 * w, y0 + 0.06 * h),
        (x0 + 0.88 * w, y0 + 0.25 * h),
        (x0 + 0.40 * w, y0 + 0.35 * h),
    ]
    d.polygon(poly, fill=(255, 255, 255, 140))
    layer = layer.filter(ImageFilter.GaussianBlur(radius=int(0.025 * (w + h) / 2)))
    base_img.alpha_composite(layer)


@registry.register(name="ammeter2", tags={"ammeter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    S = 2
    W, H = 900 * S, 560 * S
    img = Image.new("RGBA", (W, H), (225, 228, 232, 255))
    draw = ImageDraw.Draw(img)

    margin = 40 * S
    panel = (margin, margin, W - margin, H - margin)
    draw.rounded_rectangle(
        panel,
        radius=30 * S,
        fill=(240, 241, 243, 255),
        outline=(180, 182, 186, 255),
        width=4 * S,
    )

    base_h = int(0.38 * H)
    base = (margin + 10 * S, H - base_h - margin, W - margin - 10 * S, H - margin)
    draw.rectangle(base, fill=(25, 26, 28, 255))

    dial = (margin + 20 * S, margin + 20 * S, W - margin - 20 * S, base[1] - 15 * S)
    draw.rounded_rectangle(
        dial,
        radius=20 * S,
        fill=(250, 250, 252, 255),
        outline=(200, 202, 205, 255),
        width=3 * S,
    )

    # ---- place the soft highlight NOW (beneath ticks & labels) ----
    _draw_soft_highlight(img, dial)

    # choose a scale
    def _choose_config():
        opts = [
            ("A", 5.0, 1.0, 5),
            ("A", 10.0, 2.0, 5),
            ("A", 50.0, 10.0, 5),
            ("mA", 100.0, 20.0, 4),
            ("mA", 500.0, 100.0, 5),
            ("µA", 500.0, 100.0, 5),
        ]
        unit, fs, major, minors = random.choice(opts)
        return unit, fs, major, minors, major / minors

    unit, FS, major, minors, smallest_div = _choose_config()

    cx = (dial[0] + dial[2]) // 2
    cy = dial[3] - int(0.15 * (dial[3] - dial[1]))
    R_outer = int(0.80 * (dial[3] - dial[1]))
    R_ticks = int(0.78 * (dial[3] - dial[1]))
    start_deg, end_deg = 210, 330
    span = end_deg - start_deg

    draw.arc(
        [cx - R_outer, cy - R_outer, cx + R_outer, cy + R_outer],
        start=start_deg,
        end=end_deg,
        fill=(60, 60, 60, 255),
        width=3 * S,
    )

    tick_color = (30, 30, 30, 255)
    minor_step_val = smallest_div
    total_minors = int(FS / minor_step_val)

    for i in range(total_minors + 1):
        v = i * minor_step_val
        ang = start_deg + span * (v / FS)
        is_major = abs((v % major)) < 1e-9 or abs(major - (v % major)) < 1e-9
        length = int(22 * S if is_major else 12 * S)
        width = 4 * S if is_major else 2 * S
        x1, y1 = _polar(cx, cy, R_ticks, ang)
        x2, y2 = _polar(cx, cy, R_ticks - length, ang)
        draw.line((x1, y1, x2, y2), fill=tick_color, width=width)

    try:
        font_big = ImageFont.truetype("DejaVuSans.ttf", 26 * S)
        font_mid = ImageFont.truetype("DejaVuSans.ttf", 22 * S)
    except Exception as e:
        print(e)
        font_big = ImageFont.load_default()
        font_mid = ImageFont.load_default()

    num_majors = int(FS / major)
    for j in range(num_majors + 1):
        v = j * major
        ang = start_deg + span * (v / FS)
        rl = R_ticks - int(48 * S)
        tx, ty = _polar(cx, cy, rl, ang)
        lab = f"{int(v) if v.is_integer() else v:g}"
        tw, th = _measure(draw, lab, font_mid)
        draw.text(
            (tx - tw / 2, ty - th / 2), lab, fill=(20, 20, 20, 255), font=font_mid
        )

    uw, uh = _measure(draw, unit, font_big)
    draw.text(
        (cx - uw / 2, cy - uh - 28 * S), unit, fill=(25, 25, 25, 255), font=font_big
    )

    reading = round(random.uniform(0.03 * FS, 0.97 * FS), 6)
    ang_r = start_deg + span * (reading / FS)
    needle_len = R_ticks - int(22 * S)
    nx, ny = _polar(cx, cy, needle_len, ang_r)
    draw.line((cx, cy, nx, ny), fill=(192, 30, 38, 255), width=5 * S)
    tx, ty = _polar(cx, cy, int(0.20 * needle_len), ang_r + 180)
    draw.line((tx, ty, cx, cy), fill=(192, 30, 38, 200), width=3 * S)
    draw.ellipse(
        (cx - 12 * S, cy - 12 * S, cx + 12 * S, cy + 12 * S), fill=(20, 20, 22, 255)
    )

    brand = random.choice(["DH-670", "MX-55", "AM-110", "DT-51A"])
    bw, bh = _measure(draw, brand, font_mid)
    draw.text(
        (dial[0] + 20 * S, dial[3] - bh - 12 * S),
        brand,
        fill=(50, 50, 50, 255),
        font=font_mid,
    )

    out = img.resize((W // S, H // S), Image.Resampling.LANCZOS).convert("RGB")
    out.save(img_path, quality=95)

    return Artifact(
        data=img_path,
        image_type="ammeter",
        design="Dial",
        evaluator_kwargs={
            "interval": [reading - smallest_div / 4, reading + smallest_div / 4],
            "units": [unit],
        },
    )


if __name__ == "__main__":
    result_fixed = generate("out.jpg")
    print(result_fixed)
