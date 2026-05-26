import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import math
import random
from registry import registry
from artifacts import Artifact


class DialCaliperSimulator:
    def __init__(self):
        self.fig_width = 14
        self.fig_height = 6

    def create_caliper(self, reading, unit="mm", output_path="caliper.png"):
        fig, ax = plt.subplots(1, 1, figsize=(self.fig_width, self.fig_height))
        ax.set_xlim(0, 14)
        ax.set_ylim(0, 6)
        ax.set_aspect("equal")
        ax.axis("off")

        fig.patch.set_facecolor("white")
        self._draw_main_frame(ax)
        self._draw_main_scale(ax, reading, unit)
        self._draw_dial(ax, reading, unit)
        self._draw_jaws(ax, reading, unit)

        plt.tight_layout()
        plt.savefig(output_path, dpi=300, bbox_inches="tight", facecolor="white")
        plt.close()

    def _draw_main_frame(self, ax):
        """Draw main frame"""
        main_body = Rectangle(
            (0.9, 2.5), 10, 0.8, facecolor="#E8E8E8", edgecolor="black", linewidth=1.5
        )
        ax.add_patch(main_body)
        dial_housing = Circle(
            (9, 4.2), 1.3, facecolor="#D0D0D0", edgecolor="black", linewidth=2
        )
        ax.add_patch(dial_housing)
        dial_inner = Circle(
            (9, 4.2), 1.1, facecolor="white", edgecolor="black", linewidth=1
        )
        ax.add_patch(dial_inner)

    def _draw_main_scale(self, ax, reading, unit):
        scale_y = 2.9
        scale_start_x = 0.9
        if unit == "mm":
            max_scale = min(100, int(reading) + 20)
            jaw_offset = reading * 0.08
            moving_jaw_left = 0.9 + jaw_offset
            moving_jaw_right = 10.2

            for i in range(max_scale + 1):
                x_pos = scale_start_x + i * 0.08
                if x_pos > 10.8:
                    break

                if moving_jaw_left <= x_pos <= moving_jaw_right:
                    continue

                if i % 10 == 0:
                    ax.plot(
                        [x_pos, x_pos], [scale_y, scale_y + 0.25], "k-", linewidth=1.5
                    )
                    # FIX 2: Removed "if i > 0" to ensure '0' is drawn
                    ax.text(
                        x_pos,
                        scale_y + 0.35,
                        str(i),
                        ha="center",
                        va="bottom",
                        fontsize=7,
                        weight="bold",
                    )
                elif i % 5 == 0:
                    ax.plot(
                        [x_pos, x_pos], [scale_y, scale_y + 0.15], "k-", linewidth=1
                    )
                else:
                    ax.plot(
                        [x_pos, x_pos], [scale_y, scale_y + 0.08], "k-", linewidth=0.8
                    )

            ax.text(2, 2.4, "mm", ha="left", va="center", fontsize=9, weight="bold")

        else:  # inch
            max_scale_tenth = min(60, int(reading * 10) + 20)

            jaw_offset = reading * 2.0
            moving_jaw_left = 0.9 + jaw_offset
            moving_jaw_right = 10.2

            for i in range(max_scale_tenth + 1):
                x_pos = scale_start_x + i * 0.2
                if x_pos > 10.8:
                    break

                if moving_jaw_left <= x_pos <= moving_jaw_right:
                    continue

                if i % 10 == 0:
                    ax.plot(
                        [x_pos, x_pos], [scale_y, scale_y + 0.25], "k-", linewidth=2
                    )
                    ax.text(
                        x_pos,
                        scale_y + 0.35,
                        str(i // 10),
                        ha="center",
                        va="bottom",
                        fontsize=7,
                        weight="bold",
                    )
                elif i % 5 == 0:
                    ax.plot(
                        [x_pos, x_pos], [scale_y, scale_y + 0.18], "k-", linewidth=1.5
                    )
                else:
                    ax.plot([x_pos, x_pos], [scale_y, scale_y + 0.1], "k-", linewidth=1)

            ax.text(2, 2.4, "INCH", ha="left", va="center", fontsize=9, weight="bold")

    def _draw_dial(self, ax, reading, unit):
        center_x, center_y = 9, 4.2
        outer_radius = 1.0

        dial_face = Circle(
            (center_x, center_y),
            outer_radius,
            facecolor="white",
            edgecolor="black",
            linewidth=1.5,
        )
        ax.add_patch(dial_face)

        if unit == "mm":
            self._draw_mm_dial(ax, center_x, center_y, outer_radius, reading)
        else:
            self._draw_inch_dial(ax, center_x, center_y, outer_radius, reading)

        center_dot = Circle((center_x, center_y), 0.05, facecolor="black")
        ax.add_patch(center_dot)

    def _draw_mm_dial(self, ax, center_x, center_y, radius, reading):
        for i in range(100):
            angle = 2 * math.pi * i / 100 - math.pi / 2
            if i % 10 == 0:
                inner_r, outer_r = radius - 0.15, radius - 0.05
                text_r = radius - 0.25
                inner_x, inner_y = (
                    center_x + inner_r * math.cos(angle),
                    center_y + inner_r * math.sin(angle),
                )
                outer_x, outer_y = (
                    center_x + outer_r * math.cos(angle),
                    center_y + outer_r * math.sin(angle),
                )
                ax.plot([inner_x, outer_x], [inner_y, outer_y], "k-", linewidth=2)
                text_x, text_y = (
                    center_x + text_r * math.cos(angle),
                    center_y + text_r * math.sin(angle),
                )
                ax.text(
                    text_x,
                    text_y,
                    str(i),
                    ha="center",
                    va="center",
                    fontsize=8,
                    weight="bold",
                )
            elif i % 5 == 0:
                inner_r, outer_r = radius - 0.1, radius - 0.05
                inner_x, inner_y = (
                    center_x + inner_r * math.cos(angle),
                    center_y + inner_r * math.sin(angle),
                )
                outer_x, outer_y = (
                    center_x + outer_r * math.cos(angle),
                    center_y + outer_r * math.sin(angle),
                )
                ax.plot([inner_x, outer_x], [inner_y, outer_y], "k-", linewidth=1.5)
            else:
                inner_r, outer_r = radius - 0.07, radius - 0.05
                inner_x, inner_y = (
                    center_x + inner_r * math.cos(angle),
                    center_y + inner_r * math.sin(angle),
                )
                outer_x, outer_y = (
                    center_x + outer_r * math.cos(angle),
                    center_y + outer_r * math.sin(angle),
                )
                ax.plot([inner_x, outer_x], [inner_y, outer_y], "k-", linewidth=0.8)

        # Corrected Logic
        main_scale_mm = math.floor(reading)
        dial_reading = reading - main_scale_mm
        pointer_position = (dial_reading * 100) % 100
        pointer_angle = 2 * math.pi * (pointer_position / 100) - math.pi / 2
        pointer_length = radius - 0.15
        pointer_x = center_x + pointer_length * math.cos(pointer_angle)
        pointer_y = center_y + pointer_length * math.sin(pointer_angle)
        ax.plot([center_x, pointer_x], [center_y, pointer_y], "red", linewidth=3)

        ax.text(
            center_x,
            center_y - 0.4,
            "0.01mm",
            ha="center",
            va="center",
            fontsize=7,
            weight="bold",
        )
        ax.text(
            center_x,
            center_y - 0.6,
            "SHOCK-PROOF",
            ha="center",
            va="center",
            fontsize=6,
        )

    def _draw_inch_dial(self, ax, center_x, center_y, radius, reading):
        for i in range(100):
            angle = 2 * math.pi * i / 100 - math.pi / 2
            if i % 10 == 0:
                inner_r, outer_r = radius - 0.15, radius - 0.05
                text_r = radius - 0.25
                inner_x, inner_y = (
                    center_x + inner_r * math.cos(angle),
                    center_y + inner_r * math.sin(angle),
                )
                outer_x, outer_y = (
                    center_x + outer_r * math.cos(angle),
                    center_y + outer_r * math.sin(angle),
                )
                ax.plot([inner_x, outer_x], [inner_y, outer_y], "k-", linewidth=2)
                text_x, text_y = (
                    center_x + text_r * math.cos(angle),
                    center_y + text_r * math.sin(angle),
                )
                ax.text(
                    text_x,
                    text_y,
                    str(i),
                    ha="center",
                    va="center",
                    fontsize=8,
                    weight="bold",
                )
            elif i % 5 == 0:
                inner_r, outer_r = radius - 0.1, radius - 0.05
                inner_x, inner_y = (
                    center_x + inner_r * math.cos(angle),
                    center_y + inner_r * math.sin(angle),
                )
                outer_x, outer_y = (
                    center_x + outer_r * math.cos(angle),
                    center_y + outer_r * math.sin(angle),
                )
                ax.plot([inner_x, outer_x], [inner_y, outer_y], "k-", linewidth=1.5)
            else:
                inner_r, outer_r = radius - 0.07, radius - 0.05
                inner_x, inner_y = (
                    center_x + inner_r * math.cos(angle),
                    center_y + inner_r * math.sin(angle),
                )
                outer_x, outer_y = (
                    center_x + outer_r * math.cos(angle),
                    center_y + outer_r * math.sin(angle),
                )
                ax.plot([inner_x, outer_x], [inner_y, outer_y], "k-", linewidth=0.8)

        # Corrected Logic
        main_scale_inch = math.floor(reading * 10) / 10.0
        dial_reading = reading - main_scale_inch
        pointer_position = (dial_reading / 0.1) * 100 % 100
        pointer_angle = 2 * math.pi * (pointer_position / 100) - math.pi / 2
        pointer_length = radius - 0.15
        pointer_x, pointer_y = (
            center_x + pointer_length * math.cos(pointer_angle),
            center_y + pointer_length * math.sin(pointer_angle),
        )
        ax.plot([center_x, pointer_x], [center_y, pointer_y], "red", linewidth=3)

        ax.text(
            center_x,
            center_y - 0.4,
            '0.001"',
            ha="center",
            va="center",
            fontsize=7,
            weight="bold",
        )
        ax.text(
            center_x,
            center_y - 0.6,
            "SHOCK-PROOF",
            ha="center",
            va="center",
            fontsize=6,
        )

    def _draw_jaws(self, ax, reading, unit):
        """绘制卡爪"""
        fixed_jaw = Rectangle(
            (0.5, 1.8), 0.4, 2, facecolor="#C0C0C0", edgecolor="black", linewidth=1.5
        )
        ax.add_patch(fixed_jaw)
        ax.plot([0.9, 0.9], [1.5, 4.1], "k-", linewidth=3)

        if unit == "mm":
            jaw_offset = reading * 0.08
        else:
            jaw_offset = reading * 2.0
        moving_jaw_x = 0.9 + jaw_offset
        moving_jaw = Rectangle(
            (moving_jaw_x, 1.8),
            0.4,
            2,
            facecolor="#C0C0C0",
            edgecolor="black",
            linewidth=1.5,
        )
        ax.add_patch(moving_jaw)
        ax.plot([moving_jaw_x, moving_jaw_x], [1.5, 4.1], "k-", linewidth=3)

        long_bar = Rectangle(
            (moving_jaw_x + 0.4, 2.4),
            10.2 - (moving_jaw_x + 0.4),
            0.6,
            facecolor="#C0C0C0",
            edgecolor="black",
            linewidth=1.5,
        )
        ax.add_patch(long_bar)


def generate_random_reading():
    unit = random.choice(["mm", "inch"])

    if unit == "mm":
        reading = round(random.uniform(0, 90), 2)
    else:  # inch
        reading = round(random.uniform(0, 2.5), 3)

    return [reading, unit]


@registry.register(name="dial_caliper_1", tags={"dial_caliper"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    reading, unit = generate_random_reading()
    simulator = DialCaliperSimulator()
    simulator.create_caliper(reading, unit, img_path)

    if unit == "mm":
        reading_pulse_1 = round(reading + 0.01, 2)
        reading_sub_1 = round(reading - 0.01, 2)
    else:
        reading_pulse_1 = round(reading + 0.001, 3)
        reading_sub_1 = round(reading - 0.001, 3)
    evaluator_kwargs = {
        "interval": [reading_sub_1, reading_pulse_1],
        "units": ["mm", "millimeter"] if unit == "mm" else ["in", "inch", "inches"],
    }

    return Artifact(
        data=img_path,
        image_type="dial_caliper",
        design="Composite",
        evaluator_kwargs=evaluator_kwargs,
    )
