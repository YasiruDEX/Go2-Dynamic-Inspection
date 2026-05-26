# Re-run after state reset (the notebook environment restarted between cells).
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import math
import random
from datetime import datetime
from typing import Optional
from registry import registry
from artifacts import Artifact
from generators.clock.utils import add_seconds_to_time_string


def _parse_time(t):
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


def _polar(cx, cy, r, deg):
    theta = math.radians(deg - 90)
    return cx + r * math.cos(theta), cy + r * math.sin(theta)


def draw_station_roman_clock(
    time_input,
    size: int = 900,
    out_path: Optional[str] = None,
    top_text: Optional[str] = "VICTORIA",
    mid_text: Optional[str] = "STATION",
    bottom_text: Optional[str] = "1747\nLONDON",
    use_roman_IIII: bool = True,
    show_seconds_hand: bool = True,
):
    h, m, s = _parse_time(time_input)
    SS = 4
    W = H = size * SS
    cx, cy = W // 2, H // 2
    img = Image.new("RGB", (W, H), (235, 232, 225))
    draw = ImageDraw.Draw(img)

    R = int(W * 0.44)
    bezel_outer = R
    bezel_inner = int(R * 0.90)
    draw.ellipse(
        (cx - bezel_outer, cy - bezel_outer, cx + bezel_outer, cy + bezel_outer),
        fill=(22, 22, 24),
    )
    draw.ellipse(
        (cx - bezel_inner, cy - bezel_inner, cx + bezel_inner, cy + bezel_inner),
        fill=(0, 0, 0),
    )

    dial_r = int(R * 0.86)
    dial = Image.new("RGB", (W, H), (212, 170, 110))
    grad = Image.new("L", (W, H), 0)
    gpx = grad.load()
    for y in range(H):
        for x in range(W):
            d = math.hypot(x - cx, y - cy) / (dial_r * 1.05)
            v = int(max(0, min(255, (1.1 - d) * 255)))
            gpx[x, y] = v
    base = Image.new("RGB", (W, H), (230, 186, 120))
    dial = Image.composite(base, dial, grad)
    blot = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    bd = ImageDraw.Draw(blot)
    for _ in range(220):
        rr = random.randint(int(6 * SS), int(28 * SS))
        x = random.randint(cx - dial_r, cx + dial_r)
        y = random.randint(cy - dial_r, cy + dial_r)
        col = random.choice(
            [
                (170, 120, 70, random.randint(30, 60)),
                (150, 100, 60, random.randint(30, 60)),
                (200, 160, 100, random.randint(20, 45)),
            ]
        )
        bd.ellipse((x - rr, y - rr, x + rr, y + rr), fill=col)
    blot = blot.filter(ImageFilter.GaussianBlur(int(10 * SS)))
    dial = Image.alpha_composite(dial.convert("RGBA"), blot).convert("RGB")
    mask = Image.new("L", (W, H), 0)
    ImageDraw.Draw(mask).ellipse(
        (cx - dial_r, cy - dial_r, cx + dial_r, cy + dial_r), fill=255
    )
    img.paste(dial, (0, 0), mask)

    draw = ImageDraw.Draw(img)
    track_outer = int(dial_r * 0.93)
    track_inner = int(dial_r * 0.84)
    for i in range(60):
        deg = i * 6
        base_ang = 9 if i % 5 == 0 else 5
        tip_r = track_inner
        base_r = track_outer
        phi1 = deg - base_ang / 2
        phi2 = deg + base_ang / 2
        b1 = _polar(cx, cy, base_r, phi1)
        b2 = _polar(cx, cy, base_r, phi2)
        tip = _polar(cx, cy, tip_r, deg)
        draw.polygon([b1, b2, tip], fill=(40, 30, 20))

    draw.ellipse(
        (
            cx - int(dial_r * 0.78),
            cy - int(dial_r * 0.78),
            cx + int(dial_r * 0.78),
            cy + int(dial_r * 0.78),
        ),
        outline=(40, 30, 20),
        width=int(2 * SS),
    )

    numerals = [
        "XII",
        "I",
        "II",
        "III",
        "IIII" if use_roman_IIII else "IV",
        "V",
        "VI",
        "VII",
        "VIII",
        "IX",
        "X",
        "XI",
    ]
    try:
        font_num = ImageFont.truetype("DejaVuSerif.ttf", int(66 * SS))
    except Exception:
        font_num = ImageFont.truetype("DejaVuSans.ttf", int(66 * SS))
    num_r = int(dial_r * 0.67)
    for idx, label in enumerate(numerals):
        deg = idx * 30
        tx, ty = _polar(cx, cy, num_r, deg)
        bbox = draw.textbbox((0, 0), label, font=font_num)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.text((tx - tw / 2, ty - th / 2), label, fill=(30, 25, 20), font=font_num)

    def draw_center_text(text, y_offset_ratio, size_px):
        if not text:
            return
        try:
            f = ImageFont.truetype("DejaVuSerif.ttf", size_px)
        except Exception:
            f = ImageFont.truetype("DejaVuSans.ttf", size_px)
        lines = str(text).split("\n")
        y0 = cy + int(dial_r * y_offset_ratio)
        for i, line in enumerate(lines):
            bb = draw.textbbox((0, 0), line, font=f)
            draw.text(
                (
                    cx - (bb[2] - bb[0]) // 2,
                    y0 + i * int(size_px * 1.1) - (bb[3] - bb[1]) // 2,
                ),
                line,
                fill=(40, 32, 24),
                font=f,
            )

    draw_center_text(top_text, -0.28, int(38 * SS))
    draw_center_text(mid_text, -0.18, int(28 * SS))
    draw_center_text(bottom_text, 0.22, int(34 * SS))

    hour_deg = ((h % 12) + m / 60.0 + s / 3600.0) * 30.0
    min_deg = (m + s / 60.0) * 6.0
    sec_deg = s * 6.0

    def draw_hand_spade(deg, r_len, width, color, tail=0.10):
        x_end, y_end = _polar(cx, cy, r_len, deg)
        x_tail, y_tail = _polar(cx, cy, -r_len * tail, deg)
        phi = math.radians(deg - 90)
        dx = math.cos(phi + math.pi / 2) * width / 2
        dy = math.sin(phi + math.pi / 2) * width / 2
        base1, base2 = (x_tail - dx, y_tail - dy), (x_tail + dx, y_tail + dy)
        tip_l, tip_r = (x_end - dx, y_end - dy), (x_end + dx, y_end + dy)
        draw.polygon([base1, base2, tip_r, tip_l], fill=color)
        sp = int(width * 1.1)
        sx, sy = _polar(cx, cy, r_len + sp * 0.8, deg)
        left = _polar(sx, sy, sp, deg - 90)
        right = _polar(sx, sy, sp, deg + 90)
        tip = _polar(sx, sy, sp, deg)
        tailp = _polar(sx, sy, sp, deg + 180)
        draw.polygon([left, tip, right, tailp], fill=color)
        hx, hy = _polar(cx, cy, r_len - width * 0.6, deg)
        draw.ellipse(
            (
                hx - int(width * 0.35),
                hy - int(width * 0.35),
                hx + int(width * 0.35),
                hy + int(width * 0.35),
            ),
            fill=(230, 186, 120),
        )

    def draw_hand_pointer(deg, r_len, width, color, tail=0.10):
        x_end, y_end = _polar(cx, cy, r_len, deg)
        x_tail, y_tail = _polar(cx, cy, -r_len * tail, deg)
        phi = math.radians(deg - 90)
        dx = math.cos(phi + math.pi / 2) * width / 2
        dy = math.sin(phi + math.pi / 2) * width / 2
        base1, base2 = (x_tail - dx, y_tail - dy), (x_tail + dx, y_tail + dy)
        tip = _polar(cx, cy, r_len + width * 0.9, deg)
        draw.polygon([base1, base2, tip], fill=color)

    hand_dark = (25, 20, 15)
    draw_hand_spade(hour_deg, int(dial_r * 0.50), int(22 * SS), hand_dark)
    draw_hand_pointer(min_deg, int(dial_r * 0.74), int(14 * SS), hand_dark)

    if show_seconds_hand:
        x_end, y_end = _polar(cx, cy, int(dial_r * 0.80), sec_deg)
        draw.line((cx, cy, x_end, y_end), fill=(190, 30, 30), width=int(4 * SS))
        x_tail, y_tail = _polar(cx, cy, -int(dial_r * 0.18), sec_deg)
        draw.line((x_tail, y_tail, cx, cy), fill=(190, 30, 30), width=int(4 * SS))

    draw.ellipse(
        (cx - int(10 * SS), cy - int(10 * SS), cx + int(10 * SS), cy + int(10 * SS)),
        fill=(210, 210, 210),
        outline=(60, 60, 60),
        width=int(2 * SS),
    )

    glass = Image.new("L", (W, H), 0)
    gd = ImageDraw.Draw(glass)
    gd.ellipse(
        (
            cx - int(W * 0.10),
            cy - int(H * 0.38),
            cx + int(W * 0.36),
            cy - int(H * 0.02),
        ),
        fill=140,
    )
    glass = glass.filter(ImageFilter.GaussianBlur(int(42 * SS)))
    img = Image.composite(Image.new("RGB", (W, H), (255, 255, 255)), img, glass)

    out = img.resize((size, size), Image.Resampling.LANCZOS)
    if out_path:
        out.save(out_path, "PNG")
        return out_path
    return out


@registry.register(name="roman_station_clock", tags={"clock"})
def draw_roman_station_clock(img_path="roman_station_clock.png"):
    time_input = (
        f"{random.randint(1, 12)}:{random.randint(1, 59)}:{random.randint(1, 59)}"
    )
    top_text = random.choice(
        [
            "VICTORIA",
            "PHILIP",
            "JOYCE",
            "JASON",
            "CHRIS",
            "GARY",
            "HARRY",
            "JOHN",
            "MIKE",
            "NICK",
            "PETER",
            "QUINN",
            "RICHARD",
            "SCOTT",
            "TOM",
            "WALTER",
            "XAVIER",
            "YOUNG",
        ]
    )
    mid_text = random.choice(
        [
            "STATION",
            "PARK",
            "ROAD",
            "SCHOOL",
            "LIBRARY",
            "BUSINESS",
            "GROCERY",
            "MARKET",
            "MUSEUM",
            "THEATER",
            "PARK",
            "LIBRARY",
            "MUSEUM",
            "THEATER",
        ]
    )
    bottom_text = random.choice(
        [
            "1747\nLONDON",
            "1867\nTORONTO",
            "1903\nVANCOUVER",
            "1923\nCALGARY",
            "2014\nEDMONTON",
            "2015\nCALGARY",
            "2016\nEDMONTON",
            "2017\nCALGARY",
            "2018\nEDMONTON",
            "2019\nCALGARY",
            "2020\nEDMONTON",
        ]
    )
    use_roman_IIII = random.choice([True, False])
    show_seconds_hand = random.choice([True, False, True])
    draw_station_roman_clock(
        time_input,
        size=880,
        out_path=img_path,
        top_text=top_text,
        mid_text=mid_text,
        bottom_text=bottom_text,
        use_roman_IIII=use_roman_IIII,
        show_seconds_hand=show_seconds_hand,
    )

    if show_seconds_hand:
        # Create time range: time_input ± 1 seconds
        allow_error = 1
    else:
        # Create time range: time_input ± 60 second
        allow_error = 60
    time_minus_1s = add_seconds_to_time_string(time_input, -allow_error)
    time_plus_1s = add_seconds_to_time_string(time_input, allow_error)

    evaluator_kwargs = {"interval": [time_minus_1s, time_plus_1s], "units": [""]}
    print(evaluator_kwargs)
    return Artifact(
        data=img_path,
        image_type="clock",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    # Generate and show demo
    draw_roman_station_clock()
