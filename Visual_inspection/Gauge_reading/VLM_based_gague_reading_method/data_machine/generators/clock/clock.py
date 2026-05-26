# Analog clock generator
# - Input: time string like "HH:MM" or "HH:MM:SS" (24h)
# - Output: PNG image file
#
# You can copy this entire cell to a .py file and run it.
# Example CLI usage to add if needed:
#   python clock.py --time "10:10:30" --size 800 --theme dark --out clock.png

from PIL import Image, ImageDraw, ImageFont
import math
from datetime import datetime
from typing import Tuple, Union, Optional
import random
from registry import registry
from artifacts import Artifact
from generators.clock.utils import add_seconds_to_time_string


def _parse_time(t: Union[str, Tuple[int, int, int], datetime]) -> Tuple[int, int, int]:
    if isinstance(t, tuple) and len(t) == 3:
        h, m, s = t
        return int(h), int(m), int(s)
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
    raise TypeError("Unsupported time input. Use 'HH:MM[:SS]', (h,m,s), or datetime.")


def _polar(cx, cy, r, deg):
    # Convert clock degrees (0 at 12 o'clock, clockwise) to image coords
    theta = math.radians(deg - 90)  # shift so 0 deg is at 12 o'clock
    return cx + r * math.cos(theta), cy + r * math.sin(theta)


def draw_analog_clock(
    time_input: Union[str, Tuple[int, int, int], datetime],
    size: int = 800,
    theme: str = "light",
    show_numbers: bool = True,
    out_path: Optional[str] = None,
):
    """
    Draw an analog clock for the given time.

    Args:
        time_input: 'HH:MM' or 'HH:MM:SS' (24h), or (h,m,s) tuple, or datetime
        size: output image size in pixels (square)
        theme: 'light' or 'dark'
        show_numbers: whether to draw 1..12 numerals
        out_path: where to save PNG. If None, returns a PIL.Image instead

    Returns:
        out_path if provided, else PIL.Image.Image
    """
    h, m, s = _parse_time(time_input)
    # Super-sample for crisp anti-aliased output
    SS = 4
    W = H = size * SS
    pad = int(0.06 * W)
    cx, cy = W // 2, H // 2
    R = min(cx, cy) - pad  # dial radius

    # Colors by theme
    if theme.lower() == "dark":
        bg = (16, 16, 20)
        face = (28, 28, 35)
        ring = (90, 90, 105)
        tick = (210, 210, 220)
        hand_hour = (230, 230, 240)
        hand_min = (230, 230, 240)
        hand_sec = (180, 60, 60)
        text_color = (235, 235, 240)
        center_cap = (230, 230, 240)
    else:
        bg = (250, 250, 252)
        face = (240, 242, 245)
        ring = (170, 170, 180)
        tick = (50, 50, 60)
        hand_hour = (30, 30, 35)
        hand_min = (30, 30, 35)
        hand_sec = (190, 30, 30)
        text_color = (30, 30, 35)
        center_cap = (30, 30, 35)

    img = Image.new("RGB", (W, H), bg)
    draw = ImageDraw.Draw(img)

    # Face
    draw.ellipse(
        (cx - R, cy - R, cx + R, cy + R), fill=face, outline=ring, width=int(4 * SS)
    )

    # Minute & hour ticks
    outer = R - int(0.02 * W)
    for i in range(60):
        deg = i * 6
        if i % 5 == 0:
            # hour tick
            inner = outer - int(0.065 * W)
            width = int(4.5 * SS)
        else:
            # minute tick
            inner = outer - int(0.035 * W)
            width = int(2.5 * SS)
        x1, y1 = _polar(cx, cy, inner, deg)
        x2, y2 = _polar(cx, cy, outer, deg)
        draw.line((x1, y1, x2, y2), fill=tick, width=width, joint="curve")

    # Numbers 1..12
    if show_numbers:
        try:
            # Try a nicer font if present
            font = ImageFont.truetype("DejaVuSans.ttf", int(42 * SS))
        except Exception:
            font = ImageFont.load_default()
        r_num = R - int(0.13 * W)
        for n in range(1, 13):
            deg = n * 30
            tx, ty = _polar(cx, cy, r_num, deg)
            text = str(n)
            # center text
            bbox = draw.textbbox((0, 0), text, font=font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            draw.text((tx - tw / 2, ty - th / 2), text, fill=text_color, font=font)

    # Compute hand angles
    hour_deg = ((h % 12) + m / 60.0 + s / 3600.0) * 30.0
    min_deg = (m + s / 60.0) * 6.0
    sec_deg = s * 6.0

    # Hand lengths and widths
    r_hour = R * 0.52
    r_min = R * 0.74
    r_sec = R * 0.84

    w_hour = int(10 * SS)
    w_min = int(6 * SS)
    w_sec = int(3 * SS)

    # Draw hands (with small tail for style)
    def draw_hand(deg, r_len, width, color, tail=0.12):
        x_end, y_end = _polar(cx, cy, r_len, deg)
        x_tail, y_tail = _polar(cx, cy, -r_len * tail, deg)
        draw.line(
            (x_tail, y_tail, x_end, y_end), fill=color, width=width, joint="curve"
        )

    draw_hand(hour_deg, r_hour, w_hour, hand_hour)
    draw_hand(min_deg, r_min, w_min, hand_min)
    draw_hand(sec_deg, r_sec, w_sec, hand_sec)

    # Center cap
    cap_r = int(7 * SS)
    draw.ellipse(
        (cx - cap_r, cy - cap_r, cx + cap_r, cy + cap_r), fill=center_cap, outline=None
    )

    # Downsample to target size for anti-aliasing
    out_img = img.resize((size, size), Image.LANCZOS)

    if out_path:
        out_img.save(out_path, "PNG")
        return out_path
    return out_img


@registry.register(name="normal_clock", tags={"clock"})
def draw_random_clock(img_path="clock.png"):
    time_input = (
        f"{random.randint(1, 12)}:{random.randint(1, 59)}:{random.randint(1, 59)}"
    )
    theme = random.choice(["light", "dark"])
    draw_analog_clock(time_input, size=640, theme=theme, out_path=img_path)

    # Create time range: time_input ± 1 second
    time_minus_1s = add_seconds_to_time_string(time_input, -1)
    time_plus_1s = add_seconds_to_time_string(time_input, 1)

    evaluator_kwargs = {"interval": [time_minus_1s, time_plus_1s], "units": [""]}
    # print(evaluator_kwargs, theme)
    return Artifact(
        data=img_path,
        image_type="clock",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    # --- Create a demo image ---
    draw_random_clock()
