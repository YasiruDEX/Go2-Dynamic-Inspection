from typing import Tuple
from PIL import Image, ImageDraw, ImageFont
import random
from registry import registry
from artifacts import Artifact


# --------- Helper functions ---------
def _draw_metal_ruler(
    draw: ImageDraw.ImageDraw,
    origin: Tuple[int, int],
    length_cm: float,
    ppcm: int,
    height: int,
    unit_font: ImageFont.ImageFont,
    num_font: ImageFont.ImageFont,
) -> Tuple[int, int, int, int]:
    """Draw a metal ruler with a blue band, ticks, and unit labels."""
    x0, y0 = origin
    x1 = x0 + int(length_cm * ppcm)
    y1 = y0 + height

    # Ruler body & top blue band
    metal = (230, 232, 235)
    edge = (90, 90, 95)
    blue = (60, 140, 190)
    draw.rectangle([x0, y0, x1, y1], fill=metal, outline=edge, width=2)
    band_h = max(10, int(height * 0.16))
    draw.rectangle([x0, y0, x1, y0 + band_h], fill=blue, outline=None)

    # Ticks
    cm_len = int(height * 0.60)
    mid_len = int(height * 0.42)
    mm_len = int(height * 0.28)
    tick = (20, 20, 22)

    for c in range(int(length_cm) + 1):
        xpos = x0 + c * ppcm
        draw.line([(xpos, y0), (xpos, y0 + cm_len)], fill=tick, width=2)
        draw.text((xpos + 2, y0 + cm_len + 2), str(c), fill=tick, font=num_font)
        if c < int(length_cm):
            for mm in range(1, 10):
                xm = xpos + int(ppcm * (mm / 10.0))
                tlen = mid_len if mm == 5 else mm_len
                draw.line([(xm, y0), (xm, y0 + tlen)], fill=tick, width=1)

    return x0, y0, x1, y1


def _draw_pencil(
    draw: ImageDraw.ImageDraw, left_x: int, right_x: int, baseline_y: int
) -> None:
    """Draw a pencil (black body, wood tip, lead, ferrule, eraser) within bounds."""
    thickness = 12
    y_mid = baseline_y - 8
    y0 = y_mid - thickness // 2
    y1 = y_mid + thickness // 2

    # Colors
    body = (20, 22, 26)
    wood = (206, 160, 110)
    lead = (40, 40, 40)
    ferrule = (180, 185, 190)
    eraser = (170, 50, 60)
    outline = (15, 15, 18)

    # Determine tail widths so the overall right edge equals right_x
    ferrule_w = 18
    eraser_w = 16
    ferrule_x0 = right_x - (ferrule_w + eraser_w)

    # Black body (ends at the start of the ferrule)
    draw.rectangle([left_x + 28, y0, ferrule_x0, y1], fill=body, outline=outline)

    # Wood tip
    wood_tip = [(left_x + 4, y_mid), (left_x + 28, y0), (left_x + 28, y1)]
    draw.polygon(wood_tip, fill=wood, outline=outline)
    # Lead tip
    lead_tip = [(left_x, y_mid), (left_x + 8, y0 + 2), (left_x + 8, y1 - 2)]
    draw.polygon(lead_tip, fill=lead, outline=lead)

    # Ferrule
    draw.rectangle(
        [ferrule_x0, y0, ferrule_x0 + ferrule_w, y1], fill=ferrule, outline=outline
    )
    for i in range(3):
        yy = y0 + 2 + i * 4
        draw.line(
            [(ferrule_x0, yy), (ferrule_x0 + ferrule_w, yy)], fill=outline, width=1
        )
    # Eraser
    draw.rectangle(
        [ferrule_x0 + ferrule_w, y0, right_x, y1], fill=eraser, outline=outline
    )


@registry.register(name="ruler_2", tags={"ruler"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """Generate an image: metal ruler + pencil constrained within the tick range."""
    W, H = 1100, 420
    bg = (245, 243, 238)
    base = Image.new("RGB", (W, H), bg)
    layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)

    # Fonts
    unit_font = ImageFont.load_default()
    num_font = ImageFont.load_default()

    # Ruler parameters
    ppcm = random.randint(45, 60)
    ruler_len_cm = random.uniform(14.0, 20.0)
    ruler_height = random.randint(80, 100)
    ruler_left = random.randint(70, 130)
    ruler_top = random.randint(H // 3 - 20, H // 3 + 20)

    # Ensure the ruler fits the canvas horizontally and vertically
    right_margin = 12
    bottom_margin = 12
    max_len_px = max(1, W - right_margin - ruler_left)
    # Clamp the length so x0 + length <= W - right_margin
    ruler_len_cm = min(ruler_len_cm, max_len_px / ppcm)
    # Clamp vertical position to stay inside the canvas
    if ruler_top + ruler_height > H - bottom_margin:
        ruler_top = max(8, H - bottom_margin - ruler_height)

    x0, y0, x1, y1 = _draw_metal_ruler(
        d,
        (ruler_left, ruler_top),
        ruler_len_cm,
        ppcm,
        ruler_height,
        unit_font,
        num_font,
    )

    # Pencil length; ensure fully within the tick range
    min_len = 3.0
    left_margin_cm = random.uniform(0.0, ruler_len_cm - min_len - 2.0)
    right_cm = random.uniform(left_margin_cm + min_len, ruler_len_cm - 2.0)

    left_x = x0 + int(left_margin_cm * ppcm)
    right_x = x0 + int(right_cm * ppcm)

    _draw_pencil(d, left_x, right_x, baseline_y=y0)

    # Slight rotation
    angle = random.uniform(-4, 4)
    layer_rot = layer.rotate(
        angle, resample=Image.BICUBIC, expand=False, center=(W // 2, H // 2)
    )
    comp = Image.alpha_composite(base.convert("RGBA"), layer_rot).convert("RGB")
    comp.save(img_path)

    # Compute reading
    mm_per_px = 10.0 / ppcm
    length_px = right_x - left_x
    length_mm = length_px * mm_per_px
    value_cm = round(length_mm) / 10.0
    question_candidates = [
        "What's the length of the pencil?",
        "How long is the pencil?",
        "What is the length of the pencil?",
        "How long is the pencil?",
    ]
    return Artifact(
        data=img_path,
        image_type="ruler",
        question=random.choice(question_candidates),
        design="Linear",
        evaluator="interval_matching",
        evaluator_kwargs={"interval": [value_cm - 0.15, value_cm + 0.15], "units": []},
    )


# Example
if __name__ == "__main__":
    info = generate("ruler_2.png")
    print(info)
