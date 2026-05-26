# Vintage octagon clock generator styled after the provided photo.
# - Input: time string like "HH:MM[:SS]"
# - Output: PNG image mimicking a brass octagonal wall clock with railway minute track.
#
# Dependencies: Pillow (PIL)
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import math
import random
from datetime import datetime, timedelta
from typing import Tuple, Union, Optional
from registry import registry
from artifacts import Artifact


def _time_to_string(hours, minutes, seconds):
    """Convert time components back to string format"""
    return f"{hours}:{minutes:02d}:{seconds:02d}"


def _parse_time_v2(
    t: Union[str, Tuple[int, int, int], datetime],
) -> Tuple[int, int, int]:
    if isinstance(t, tuple) and len(t) == 3:
        return int(t[0]), int(t[1]), int(t[2])
    if isinstance(t, datetime):
        return t.hour, t.minute, t.second
    if isinstance(t, str):
        parts = t.strip().split(":")
        if len(parts) == 2:
            h, m = parts
            s = 0
        elif len(parts) == 3:
            h, m, s = parts
        else:
            raise ValueError("Time string must be 'HH:MM' or 'HH:MM:SS'")
        return int(h), int(m), int(s)
    raise TypeError("Unsupported time input")


def _add_seconds_to_time_string(time_str, seconds_delta):
    """Add seconds to a time string and return the new time string"""
    h, m, s = _parse_time_v2(time_str)
    # Create a datetime object for easy arithmetic
    dt = datetime.now().replace(hour=h, minute=m, second=s)
    new_dt = dt + timedelta(seconds=seconds_delta)
    return _time_to_string(new_dt.hour, new_dt.minute, new_dt.second)


def _regular_polygon(cx, cy, r, sides, rotation_deg=0):
    pts = []
    for i in range(sides):
        ang = math.radians(rotation_deg + 360 * i / sides - 90)  # start at top
        pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
    return pts


def _polar(cx, cy, r, deg):
    theta = math.radians(deg - 90)
    return cx + r * math.cos(theta), cy + r * math.sin(theta)


def draw_vintage_octagon_clock(
    time_input: Union[str, Tuple[int, int, int], datetime],
    size: int = 900,
    out_path: Optional[str] = None,
    brand_text: Optional[str] = None,
    show_seconds_hand: bool = True,
):
    """Render a brass octagonal clock reminiscent of a vintage wall clock."""
    h, m, s = _parse_time_v2(time_input)
    SS = 4  # supersample for crisp edges
    W = H = size * SS
    cx, cy = W // 2, H // 2
    img = Image.new("RGB", (W, H), (35, 28, 20))  # dark wood-ish background
    draw = ImageDraw.Draw(img)

    # Outer brass bezel (octagon with bevel)
    bezel_r = int(W * 0.44)
    octo_outer = _regular_polygon(cx, cy, bezel_r, 8, rotation_deg=0)
    octo_inner = _regular_polygon(cx, cy, int(bezel_r * 0.88), 8, rotation_deg=0)
    # base brass color
    brass_dark = (165, 134, 60)
    brass_light = (218, 191, 110)
    draw.polygon(octo_outer, fill=brass_dark)
    draw.polygon(octo_inner, fill=(0, 0, 0))  # carve inner via mask next

    # Create a mask to punch inner hole from bezel
    bezel = Image.new("L", (W, H), 0)
    bz = ImageDraw.Draw(bezel)
    bz.polygon(octo_outer, fill=255)
    bz.polygon(octo_inner, fill=0)
    # Bevel highlight by blurring the mask and compositing a lighter color
    bevel = Image.new("RGB", (W, H), brass_light)
    bevel = Image.composite(bevel, Image.new("RGB", (W, H), (0, 0, 0)), bezel)
    img = Image.blend(img, bevel, 0.34)

    # Dial plate (octagon slightly inside)
    dial_r = int(bezel_r * 0.80)
    octo_dial = _regular_polygon(cx, cy, dial_r, 8, rotation_deg=0)

    # Build a golden radial gradient and clip to octagon
    dial = Image.new("RGB", (W, H), (0, 0, 0))
    # radial gradient
    grad = Image.new("L", (W, H), 0)
    gpx = grad.load()
    for y in range(H):
        for x in range(W):
            # distance from center normalized
            d = math.hypot(x - cx, y - cy) / (dial_r * 1.05)
            v = int(max(0, min(255, (1.0 - d) * 255)))
            gpx[x, y] = v
    bright = (242, 218, 130)
    edge = (200, 170, 90)
    dial_img = Image.new("RGB", (W, H), edge)
    dial_bright = Image.new("RGB", (W, H), bright)
    dial = Image.composite(dial_bright, dial_img, grad)

    # Clip gradient to octagon mask
    mask = Image.new("L", (W, H), 0)
    md = ImageDraw.Draw(mask)
    md.polygon(octo_dial, fill=255)
    img.paste(dial, (0, 0), mask)

    # Inner stepped octagon (like the photo)
    inner_r = int(dial_r * 0.58)
    octo_inner_plate = _regular_polygon(cx, cy, inner_r, 8, rotation_deg=0)
    inner_color = (225, 200, 120)
    inner_plate = Image.new("RGB", (W, H), inner_color)
    inner_mask = Image.new("L", (W, H), 0)
    ImageDraw.Draw(inner_mask).polygon(octo_inner_plate, fill=255)
    img.paste(inner_plate, (0, 0), inner_mask)

    draw = ImageDraw.Draw(img)

    # Minute railway track — small blocks along a circle inside the dial
    track_outer = int(dial_r * 0.92)
    track_inner = int(dial_r * 0.84)
    for i in range(60):
        deg = i * 6
        thickness = 0.018 * W if i % 5 == 0 else 0.010 * W
        # create small radial rectangles
        r1 = track_inner
        r2 = track_outer
        x1, y1 = _polar(cx, cy, r1, deg)
        x2, y2 = _polar(cx, cy, r2, deg)
        # perpendicular offset for rectangle width
        phi = math.radians(deg - 90)
        dx = math.cos(phi + math.pi / 2) * thickness / 2
        dy = math.sin(phi + math.pi / 2) * thickness / 2
        poly = [
            (x1 - dx, y1 - dy),
            (x1 + dx, y1 + dy),
            (x2 + dx, y2 + dy),
            (x2 - dx, y2 - dy),
        ]
        draw.polygon(poly, fill=(25, 25, 25))

    # Numerals (bold vintage-like)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", int(74 * SS))
    except Exception:
        font = ImageFont.load_default()
    num_r = int(dial_r * 0.65)
    text_color = (15, 15, 15)
    for n in range(1, 13):
        deg = n * 30
        tx, ty = _polar(cx, cy, num_r, deg)
        label = str(n)
        bbox = draw.textbbox((0, 0), label, font=font)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.text((tx - tw / 2, ty - th / 2), label, fill=text_color, font=font)

    # Optional brand text under 12
    if brand_text:
        try:
            brand_font = ImageFont.truetype("DejaVuSans.ttf", int(40 * SS))
        except Exception:
            brand_font = ImageFont.load_default()
        bx, by = _polar(cx, cy, int(num_r * 0.58), 0)  # under 12
        bb = draw.textbbox((0, 0), brand_text, font=brand_font)
        draw.text(
            (bx - (bb[2] - bb[0]) / 2, by - (bb[3] - bb[1]) / 2),
            brand_text,
            fill=(30, 30, 30),
            font=brand_font,
        )

    # Winding holes at ~4:30 and ~7:30
    for ang in (225, 315):
        px, py = _polar(cx, cy, int(inner_r * 0.58), ang)
        r_out = int(18 * SS)
        r_in = int(10 * SS)
        draw.ellipse(
            (px - r_out, py - r_out, px + r_out, py + r_out), fill=(40, 40, 40)
        )
        draw.ellipse((px - r_in, py - r_in, px + r_in, py + r_in), fill=(120, 120, 120))

    # Compute hand angles
    hour_deg = ((h % 12) + m / 60.0 + s / 3600.0) * 30.0
    min_deg = (m + s / 60.0) * 6.0
    sec_deg = s * 6.0

    # Hands — art-deco-ish (hour broader, minute slimmer)
    def draw_hand(deg, r_len, width, color, tail=0.08, spear=False):
        x_end, y_end = _polar(cx, cy, r_len, deg)
        x_tail, y_tail = _polar(cx, cy, -r_len * tail, deg)
        # For a paddle shape, draw a polygon with a diamond/spear tip
        phi = math.radians(deg - 90)
        dx = math.cos(phi + math.pi / 2) * width / 2
        dy = math.sin(phi + math.pi / 2) * width / 2
        base1 = (x_tail - dx, y_tail - dy)
        base2 = (x_tail + dx, y_tail + dy)
        tip_left = (x_end - dx, y_end - dy)
        tip_right = (x_end + dx, y_end + dy)
        if spear:
            # add a small triangle spear
            spear_len = width * 1.2
            sx, sy = _polar(cx, cy, r_len + spear_len, deg)
            poly = [base1, base2, tip_right, (sx, sy), tip_left]
        else:
            poly = [base1, base2, tip_right, tip_left]
        draw.polygon(poly, fill=color)

    hand_dark = (45, 35, 30)
    draw_hand(hour_deg, int(inner_r * 0.92), int(24 * SS), hand_dark, spear=True)
    draw_hand(min_deg, int(dial_r * 0.86), int(16 * SS), hand_dark, spear=True)

    if show_seconds_hand:
        # slender seconds hand with counterweight
        x_end, y_end = _polar(cx, cy, int(dial_r * 0.90), sec_deg)
        draw.line((cx, cy, x_end, y_end), fill=(90, 70, 60), width=int(4 * SS))
        x_tail, y_tail = _polar(cx, cy, -int(dial_r * 0.18), sec_deg)
        draw.line((x_tail, y_tail, cx, cy), fill=(90, 70, 60), width=int(4 * SS))
        draw.ellipse(
            (x_tail - 10 * SS, y_tail - 10 * SS, x_tail + 10 * SS, y_tail + 10 * SS),
            fill=(90, 70, 60),
        )

    # Center cap
    cap_r = int(12 * SS)
    draw.ellipse((cx - cap_r, cy - cap_r, cx + cap_r, cy + cap_r), fill=(70, 55, 45))

    # Subtle vignette for depth
    vignette = Image.new("L", (W, H), 0)
    vd = ImageDraw.Draw(vignette)
    vd.ellipse(
        (
            cx - int(W * 0.44),
            cy - int(H * 0.44),
            cx + int(W * 0.44),
            cy + int(H * 0.44),
        ),
        fill=220,
    )
    vignette = vignette.filter(ImageFilter.GaussianBlur(int(50 * SS)))
    img = Image.composite(img, Image.new("RGB", (W, H), (25, 20, 15)), vignette)

    # Downsample
    out = img.resize((size, size), Image.Resampling.LANCZOS)
    if out_path:
        out.save(out_path, "PNG")
        return out_path
    return out


@registry.register(name="vintage_octagon_clock", tags={"clock"})
def draw_random_clock(img_path="clock.png"):
    time_input = (
        f"{random.randint(1, 12)}:{random.randint(1, 59)}:{random.randint(1, 59)}"
    )
    draw_vintage_octagon_clock(
        time_input, size=900, out_path=img_path, brand_text=None, show_seconds_hand=True
    )
    # Create time range: time_input ± 1 second
    time_minus_1s = _add_seconds_to_time_string(time_input, -1)
    time_plus_1s = _add_seconds_to_time_string(time_input, 1)

    evaluator_kwargs = {"interval": [time_minus_1s, time_plus_1s], "units": [""]}
    # print(evaluator_kwargs)
    return Artifact(
        data=img_path,
        image_type="clock",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    draw_random_clock()
