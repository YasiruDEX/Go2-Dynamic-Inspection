import math
import random
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from registry import registry
from artifacts import Artifact


@registry.register(name="water_meter1", tags={"water_meter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    """
    Renders a synthetic Water Meter image and saves it to img_path.

    Returns:
        A dictionary containing the design type, reading interval, and acceptable units.
    """

    class SyntheticWaterMeter:
        """
        Manages the generation of a synthetic water meter image, handling all randomization
        and drawing logic.
        """

        def __init__(self, size: int):
            self.size = size
            self.center = (size // 2, size // 2)
            self._randomize_parameters()
            self.image = Image.new("RGB", (size, size), self.bg_color)
            self.draw = ImageDraw.Draw(self.image, "RGBA")

        def _randomize_parameters(self):
            """Randomizes over 12 independent visual and measurement parameters for diversity."""
            # 1. Image Size is pre-determined and passed to __init__

            # 2. Unit System Selection
            units_options = [
                ("CUBIC FEET", ["cubic feet", "ft^3"]),
                ("CUBIC METERS", ["cubic meters", "m^3"]),
                ("GALLONS", ["gallons", "gal"]),
            ]
            self.unit_text, self.units_list = random.choice(units_options)

            # 3. Reading and Interval Calculation
            self.billing_read_val = random.randint(1, 99999)
            self.dial_reading_val = random.uniform(0.0, 1.0)
            self.total_reading = self.billing_read_val + self.dial_reading_val
            self.interval = [
                math.floor(self.total_reading * 100) / 100.0,
                math.ceil(self.total_reading * 100) / 100.0,
            ]

            # 4. Color Palette (Theme)
            is_dark_theme = random.random() < 0.3
            self.text_color = "#E0E0E0" if is_dark_theme else "#101010"
            self.face_color = "#101010" if is_dark_theme else "#FEFEFE"
            self.bg_color = "#202020" if is_dark_theme else "#F0F0F0"
            self.pointer_color = (
                random.choice(["#FF3030", "#D0D0D0"])
                if is_dark_theme
                else random.choice(["#D00000", "#101010"])
            )
            self.bezel_color = (
                random.choice(["#303040", "#454545"])
                if is_dark_theme
                else random.choice(["#2C5B8A", "#505050"])
            )

            # 5. Bezel Style
            self.bezel_width_ratio = random.uniform(0.08, 0.15)

            # 6. Pointer Style
            self.pointer_shape = random.choice(["tapered", "straight", "arrow"])
            self.has_counterweight = random.random() < 0.4

            # 7. Font Selection and Sizing
            self.font_size_ratio = random.uniform(0.045, 0.06)
            try:
                # Use a common system font if available for better quality, otherwise fallback
                self.font_major = ImageFont.truetype(
                    "DejaVuSans.ttf", int(self.size * self.font_size_ratio)
                )
                self.font_minor = ImageFont.truetype(
                    "DejaVuSans.ttf", int(self.size * 0.035)
                )
                self.font_brand = ImageFont.truetype(
                    "DejaVuSans-Bold.ttf", int(self.size * self.font_size_ratio)
                )
            except IOError:
                self.font_major = ImageFont.load_default()
                self.font_minor = ImageFont.load_default()
                self.font_brand = ImageFont.load_default()

            # 8. Brand Name
            self.brand_name = random.choice(
                ["Hersey", "Badger", "Sensus", "Neptune", "Metron"]
            )

            # 9. Finishing Effects (Glare/Reflection)
            self.has_glare = random.random() < 0.8

            # 10. Camera View (Rotation)
            self.rotation_angle = random.uniform(-5, 5)

            # 11. Background Style
            self.bg_style = random.choice(["solid", "gradient"])

            # 12. Noise and Artifacts
            self.noise_level = random.uniform(0, 0.03)

            # 13. Tick Style
            self.major_tick_thickness = random.randint(
                max(1, int(self.size / 200)), max(2, int(self.size / 150))
            )
            self.minor_tick_thickness = max(1, self.major_tick_thickness // 2)

            # 14. Additional Markings
            self.pipe_size_text = random.choice(["5/8", "3/4", '1"'])
            self.has_low_flow_indicator = random.random() < 0.7

        def _draw_bezel_and_face(self):
            """Draws the outer casing and the dial face."""
            bezel_outer_radius = self.size / 2
            self.draw.ellipse([(0, 0), (self.size, self.size)], fill=self.bezel_color)

            face_radius = bezel_outer_radius * (1 - self.bezel_width_ratio - 0.02)
            box = [
                (self.center[0] - face_radius, self.center[1] - face_radius),
                (self.center[0] + face_radius, self.center[1] + face_radius),
            ]
            self.draw.ellipse(box, fill=self.face_color)
            self.face_radius = face_radius

        def _draw_ticks_and_numbers(self):
            """Draws the major/minor ticks and numeric labels on the dial."""
            number_radius = self.face_radius * 0.85
            for i in range(100):
                angle = math.radians(270 + i * 3.6)  # 0 at the top
                is_major = i % 10 == 0
                tick_len = self.face_radius * (0.1 if is_major else 0.05)
                start_r, end_r = self.face_radius - tick_len, self.face_radius

                pt1 = (
                    self.center[0] + start_r * math.cos(angle),
                    self.center[1] + start_r * math.sin(angle),
                )
                pt2 = (
                    self.center[0] + end_r * math.cos(angle),
                    self.center[1] + end_r * math.sin(angle),
                )
                self.draw.line(
                    [pt1, pt2],
                    fill=self.text_color,
                    width=self.major_tick_thickness
                    if is_major
                    else self.minor_tick_thickness,
                )

                if is_major:
                    num = str(i // 10)
                    pos = (
                        self.center[0] + number_radius * math.cos(angle),
                        self.center[1] + number_radius * math.sin(angle),
                    )
                    self.draw.text(
                        pos,
                        num,
                        font=self.font_major,
                        fill=self.text_color,
                        anchor="mm",
                    )

        def _draw_billing_read(self):
            """Draws the odometer-style integer reading."""
            width, height = self.size * 0.35, self.size * 0.09
            x0, y0 = self.center[0] - width / 2, self.center[1] - self.size * 0.25
            self.draw.rectangle(
                [x0, y0, x0 + width, y0 + height], fill="#F0F0F0", outline="#323232"
            )

            billing_str = f"{self.billing_read_val:06d}"
            digit_width = width / len(billing_str)
            for i, digit in enumerate(billing_str):
                dx = x0 + i * digit_width + digit_width / 2
                dy = y0 + height / 2 + random.uniform(-height * 0.1, height * 0.1)
                self.draw.text(
                    (dx, dy), digit, font=self.font_major, fill="#0A0A0A", anchor="mm"
                )

        def _draw_static_elements(self):
            """Draws all fixed text and symbols like brand, units, etc."""
            # Unit text
            self.draw.text(
                (self.center[0], self.center[1] - self.size * 0.1),
                self.unit_text,
                font=self.font_minor,
                fill=self.text_color,
                anchor="mm",
            )
            # Brand text
            self.draw.text(
                (self.center[0], self.center[1] + self.size * 0.2),
                self.brand_name,
                font=self.font_brand,
                fill=self.text_color,
                anchor="mm",
            )
            # 1/10 Scale text
            self.draw.text(
                (self.center[0], self.center[1] + self.size * 0.3),
                "1/10",
                font=self.font_minor,
                fill=self.text_color,
                anchor="mm",
            )
            # Pipe size
            self.draw.text(
                (self.center[0] - self.size * 0.2, self.center[1] + self.size * 0.1),
                self.pipe_size_text,
                font=self.font_minor,
                fill=self.text_color,
                anchor="mm",
            )

            # Low-flow indicator
            if self.has_low_flow_indicator:
                r = self.face_radius * 0.4
                pos = (self.center[0] + r, self.center[1] + self.size * 0.1)
                s = self.size * 0.03
                self.draw.polygon(
                    [pos, (pos[0] + s, pos[1]), (pos[0] + s / 2, pos[1] - s * 0.866)],
                    fill="#FF0000",
                )

        def _draw_pointer(self):
            """Draws the main indicator pointer."""
            angle_rad = math.radians(270 + self.dial_reading_val * 360)
            length = self.face_radius * 0.8
            width = self.size * 0.015
            tip = (
                self.center[0] + length * math.cos(angle_rad),
                self.center[1] + length * math.sin(angle_rad),
            )

            if self.pointer_shape == "tapered":
                base_angle1, base_angle2 = (
                    angle_rad + math.pi / 2,
                    angle_rad - math.pi / 2,
                )
                base_pt1 = (
                    self.center[0] + width * math.cos(base_angle1),
                    self.center[1] + width * math.sin(base_angle1),
                )
                base_pt2 = (
                    self.center[0] + width * math.cos(base_angle2),
                    self.center[1] + width * math.sin(base_angle2),
                )
                self.draw.polygon([tip, base_pt1, base_pt2], fill=self.pointer_color)
            elif self.pointer_shape == "arrow":
                head_len = self.size * 0.05
                base_x, base_y = (
                    self.center[0] + (length - head_len) * math.cos(angle_rad),
                    self.center[1] + (length - head_len) * math.sin(angle_rad),
                )
                self.draw.line(
                    [self.center, (base_x, base_y)],
                    fill=self.pointer_color,
                    width=max(1, int(width)),
                )
                self.draw.polygon(
                    [
                        tip,
                        (
                            base_x + width * 2 * math.sin(angle_rad),
                            base_y - width * 2 * math.cos(angle_rad),
                        ),
                        (
                            base_x - width * 2 * math.sin(angle_rad),
                            base_y + width * 2 * math.cos(angle_rad),
                        ),
                    ],
                    fill=self.pointer_color,
                )
            else:  # straight
                self.draw.line(
                    [self.center, tip],
                    fill=self.pointer_color,
                    width=max(1, int(width * 1.5)),
                )

            if self.has_counterweight:
                cw_len = length * 0.2
                cw_pt = (
                    self.center[0] - cw_len * math.cos(angle_rad),
                    self.center[1] - cw_len * math.sin(angle_rad),
                )
                self.draw.line(
                    [self.center, cw_pt],
                    fill=self.pointer_color,
                    width=max(2, int(width * 2)),
                )

            hub_radius = self.size * random.uniform(0.02, 0.04)
            self.draw.ellipse(
                [
                    (self.center[0] - hub_radius, self.center[1] - hub_radius),
                    (self.center[0] + hub_radius, self.center[1] + hub_radius),
                ],
                fill=self.bezel_color,
                outline=self.text_color,
            )

        def build(self) -> Image.Image:
            """Constructs the image by layering all components and applying final effects."""
            self._draw_bezel_and_face()
            self._draw_ticks_and_numbers()
            self._draw_billing_read()
            self._draw_static_elements()
            self._draw_pointer()

            self.image = self.image.rotate(
                self.rotation_angle, resample=Image.BICUBIC, center=self.center
            )

            if self.has_glare:
                glare_layer = Image.new(
                    "RGBA", (self.size, self.size), (255, 255, 255, 0)
                )
                glare_draw = ImageDraw.Draw(glare_layer)
                radius = self.size / 2 * random.uniform(0.9, 1.1)
                offset = (
                    self.size * random.uniform(-0.2, 0.2),
                    self.size * random.uniform(-0.2, 0.2),
                )
                glare_box = [
                    (
                        self.center[0] - radius + offset[0],
                        self.center[1] - radius + offset[1],
                    ),
                    (
                        self.center[0] + radius + offset[0],
                        self.center[1] + radius + offset[1],
                    ),
                ]
                glare_draw.ellipse(
                    glare_box, fill=(255, 255, 255, random.randint(30, 70))
                )
                self.image = Image.alpha_composite(
                    self.image.convert("RGBA"), glare_layer
                ).convert("RGB")

            if self.noise_level > 0:
                noise_arr = np.array(self.image)
                noise = np.random.normal(0, self.noise_level * 40, noise_arr.shape)
                noisy_arr = np.clip(noise_arr + noise, 0, 255).astype(np.uint8)
                self.image = Image.fromarray(noisy_arr)

            if random.random() < 0.4:
                self.image = self.image.filter(
                    ImageFilter.GaussianBlur(radius=random.uniform(0.2, 0.6))
                )

            return self.image

    img_size = random.choice([384, 512, 640])
    generator = SyntheticWaterMeter(img_size)
    image = generator.build()
    image.save(img_path)

    evaluator_kwargs = {"interval": generator.interval, "units": generator.units_list}
    return Artifact(
        data=img_path,
        image_type="water_meter",
        design="Composite",
        evaluator_kwargs=evaluator_kwargs,
    )
