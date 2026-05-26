from typing import Tuple
from PIL import Image, ImageDraw, ImageFont
import random
from registry import registry
from artifacts import Artifact

# ---------- Style randomization pool ----------
RULER_BODY_COLORS = [
    (245, 236, 210),  # Wood tone
    (235, 235, 235),  # Light gray
    (250, 245, 230),  # Off-white
    (255, 248, 220),  # Cornsilk
]
RULER_OUTLINES = [(60, 60, 60), (30, 30, 30), (80, 80, 80)]
TICK_COLORS = [(0, 0, 0), (20, 20, 20)]
OBJECT_COLORS = [
    (120, 170, 240),
    (250, 140, 120),
    (130, 220, 170),
    (220, 160, 240),
    (250, 200, 120),
    (150, 150, 240),
]
OBJECT_OUTLINES = [
    (20, 50, 120),
    (120, 40, 30),
    (30, 110, 80),
    (90, 60, 140),
    (110, 90, 30),
]


def _draw_ruler(
    draw: ImageDraw.ImageDraw,
    origin: Tuple[int, int],
    length_cm: float,
    ppcm: int,
    height: int,
    body_color,
    edge_color,
    tick_color,
) -> Tuple[int, int, int, int]:
    """
    Draw a ruler with centimeters as major ticks and millimeters as minor ticks,
    labeling centimeter numbers and the unit.
    Return ruler bounding box pixels as (x0, y0, x1, y1).
    """
    x0, y0 = origin
    x1 = x0 + int(length_cm * ppcm)
    y1 = y0 + height

    # Ruler body
    draw.rectangle([x0, y0, x1, y1], fill=body_color, outline=edge_color, width=2)

    # Tick parameters
    cm_len = int(height * 0.55)
    mid_len = int(height * 0.40)
    mm_len = int(height * 0.28)

    font = ImageFont.load_default()

    for c in range(int(length_cm) + 1):
        x = x0 + c * ppcm
        # Centimeter major tick
        draw.line([(x, y0), (x, y0 + cm_len)], fill=tick_color, width=2)
        # Number label
        draw.text((x + 2, y0 + cm_len + 2), str(c), fill=tick_color, font=font)

        # Millimeter minor ticks
        if c < int(length_cm):
            for mm in range(1, 10):
                xm = x + int(ppcm * (mm / 10.0))
                if mm == 5:
                    t = mid_len
                else:
                    t = mm_len
                draw.line([(xm, y0), (xm, y0 + t)], fill=tick_color, width=1)

    # Annotate unit at the top-left of the ruler
    unit_label = "cm"
    draw.text((x0 + 4, y0 - 16), unit_label, fill=tick_color, font=font)

    return x0, y0, x1, y1


def _draw_object(
    draw: ImageDraw.ImageDraw,
    left_x: int,
    right_x: int,
    baseline_y: int,
    color,
    edge_color,
) -> None:
    """Draw a random object above the ruler."""
    w = max(6, right_x - left_x)
    h = random.randint(26, 48)
    top = baseline_y - h - random.randint(6, 14)
    bottom = top + h

    shape = random.choice(["rect", "ellipse", "capsule"])
    if shape == "rect":
        draw.rectangle(
            [left_x, top, right_x, bottom], fill=color, outline=edge_color, width=2
        )
    elif shape == "ellipse":
        draw.ellipse(
            [left_x, top, right_x, bottom], fill=color, outline=edge_color, width=2
        )
    elif shape == "capsule":
        r = min(h // 2, w // 4)
        draw.rectangle([left_x + r, top, right_x - r, bottom], fill=color, outline=None)
        draw.pieslice(
            [left_x, top, left_x + 2 * r, bottom],
            90,
            270,
            fill=color,
            outline=edge_color,
        )
        draw.pieslice(
            [right_x - 2 * r, top, right_x, bottom],
            -90,
            90,
            fill=color,
            outline=edge_color,
        )
        draw.line([(left_x + r, top), (right_x - r, top)], fill=edge_color, width=2)
        draw.line(
            [(left_x + r, bottom), (right_x - r, bottom)], fill=edge_color, width=2
        )

    # # Alignment guide lines
    # drop_h = 14
    # draw.line([(left_x, bottom + 2), (left_x, bottom + 2 + drop_h)], fill=edge_color, width=2)
    # draw.line([(right_x, bottom + 2), (right_x, bottom + 2 + drop_h)], fill=edge_color, width=2)


@registry.register(name="ruler_1", tags={"ruler"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """Generate an image with a random ruler (tick values and unit labels) and an object."""
    W, H = 1024, 512
    base = Image.new("RGB", (W, H), (255, 255, 255))
    layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)

    # Random ruler parameters
    ppcm = random.randint(40, 70)
    ruler_len_cm = random.uniform(14.0, 20.0)
    ruler_height = random.randint(70, 100)
    ruler_left = random.randint(80, 140)
    ruler_top = random.randint(H // 3 - 30, H // 3 + 30)

    # Keep the ruler entirely within the canvas
    right_margin = 12
    bottom_margin = 12
    max_len_px = max(1, W - right_margin - ruler_left)
    ruler_len_cm = min(ruler_len_cm, max_len_px / ppcm)
    if ruler_top + ruler_height > H - bottom_margin:
        ruler_top = max(8, H - bottom_margin - ruler_height)
    body_color = random.choice(RULER_BODY_COLORS)
    edge_color = random.choice(RULER_OUTLINES)
    tick_color = random.choice(TICK_COLORS)

    x0, y0, x1, y1 = _draw_ruler(
        d,
        (ruler_left, ruler_top),
        ruler_len_cm,
        ppcm,
        ruler_height,
        body_color,
        edge_color,
        tick_color,
    )

    # Random object length and position
    max_obj_len = min(10.0, ruler_len_cm - 2.0)
    obj_len_cm = round(random.uniform(2.0, max_obj_len), 1)
    left_margin_cm = random.uniform(0.5, ruler_len_cm - obj_len_cm - 0.5)
    right_cm = left_margin_cm + obj_len_cm

    left_x = x0 + int(left_margin_cm * ppcm)
    right_x = x0 + int(right_cm * ppcm)

    obj_color = random.choice(OBJECT_COLORS)
    obj_edge = random.choice(OBJECT_OUTLINES)
    _draw_object(
        d, left_x, right_x, baseline_y=y0, color=obj_color, edge_color=obj_edge
    )

    # Rotation
    angle = random.uniform(-5, 5)
    layer_rot = layer.rotate(
        angle, resample=Image.BICUBIC, expand=False, center=(W // 2, H // 2)
    )
    base = Image.alpha_composite(base.convert("RGBA"), layer_rot).convert("RGB")

    base.save(img_path)

    # Return the reading (rounded to 1 mm)
    mm_per_px = 10.0 / ppcm
    length_px = right_x - left_x
    length_mm = length_px * mm_per_px
    length_mm_rounded = round(length_mm)
    length_cm = length_mm_rounded / 10.0

    question_candidates = [
        "What's the length of the object?",
        "How long is the object?",
        "What is the length of the object?",
        "How long is the object?",
    ]
    return Artifact(
        data=img_path,
        image_type="ruler",
        question=random.choice(question_candidates),
        design="Linear",
        evaluator="multi_interval_matching",
        evaluator_kwargs={
            "intervals": [
                [length_cm - 0.15, length_cm + 0.15],
                [length_mm_rounded - 1.5, length_mm_rounded + 1.5],
            ],
            "units": [["cm", "centimeter"], ["mm", "millimeter"]],
        },
    )


if __name__ == "__main__":
    info = generate("ruler_1.png")
    print(info)
