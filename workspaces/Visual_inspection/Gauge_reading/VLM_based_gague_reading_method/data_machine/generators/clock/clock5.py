import matplotlib.pyplot as plt
import numpy as np
import random
from registry import registry
from artifacts import Artifact


def get_random_color():
    """Generate a random color."""
    return (random.random(), random.random(), random.random())


def draw_clock(hour, minute, second, filename, use_roman=False):
    """Draw a clock image with given time and style."""
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect("equal")
    ax.axis("off")

    bg_color = get_random_color()
    number_color = get_random_color()
    hour_hand_color = get_random_color()
    minute_hand_color = get_random_color()
    second_hand_color = get_random_color()
    tick_color = get_random_color()

    fig.patch.set_facecolor(bg_color)  # Set background color

    circle = plt.Circle((0, 0), 1, color="white", alpha=0.9)
    ax.add_patch(circle)

    roman_numerals = [
        "XII",
        "I",
        "II",
        "III",
        "IV",
        "V",
        "VI",
        "VII",
        "VIII",
        "IX",
        "X",
        "XI",
    ]
    arabic_numerals = ["12", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"]

    for i in range(12):
        angle = np.pi / 2 - i * np.pi / 6  # Start from 12 o'clock, counter-clockwise
        x_outer = 0.95 * np.cos(angle)
        y_outer = 0.95 * np.sin(angle)
        x_inner = 0.85 * np.cos(angle)
        y_inner = 0.85 * np.sin(angle)
        # Draw major tick
        ax.plot([x_inner, x_outer], [y_inner, y_outer], color=tick_color, linewidth=3)
        # Draw number
        number_text = roman_numerals[i] if use_roman else arabic_numerals[i]
        x_text = 0.75 * np.cos(angle)
        y_text = 0.75 * np.sin(angle)
        ax.text(
            x_text,
            y_text,
            number_text,
            fontsize=16,
            ha="center",
            va="center",
            color=number_color,
            weight="bold",
        )

    # Draw minor ticks (one per second, 60 in total)
    for i in range(60):
        angle = (
            np.pi / 2 - i * np.pi / 30
        )  # Start from 12 o'clock, 6 degrees per second
        x_outer = 0.95 * np.cos(angle)
        y_outer = 0.95 * np.sin(angle)
        # Every 5 seconds, draw a slightly longer tick (except at 15s, 30s, 45s, 0s)
        if i % 5 == 0 and i % 15 != 0:
            x_inner = 0.88 * np.cos(angle)
            y_inner = 0.88 * np.sin(angle)
            linewidth = 1.5
        elif i % 15 != 0:  # Minor tick, skip positions covered by major ticks
            x_inner = 0.90 * np.cos(angle)
            y_inner = 0.90 * np.sin(angle)
            linewidth = 0.8
        else:
            continue  # Skip positions covered by major ticks

        ax.plot(
            [x_inner, x_outer],
            [y_inner, y_outer],
            color=tick_color,
            linewidth=linewidth,
            alpha=0.7,
        )
    # Second hand only points to integer seconds (60 positions)
    second_degree = second * 6

    # Calculate hand angles (hour and minute hands move smoothly)
    hour_angle = np.pi / 2 - (hour % 12 + minute / 60 + second / 3600) * np.pi / 6
    minute_angle = np.pi / 2 - (minute + second / 60) * np.pi / 30
    second_angle = np.pi / 2 - second_degree * np.pi / 180

    use_arrow_style = random.choice([True, False])

    if use_arrow_style:
        # Arrow style hands
        # Hour hand (thick, short, arrow)
        hour_x = 0.5 * np.cos(hour_angle)
        hour_y = 0.5 * np.sin(hour_angle)
        ax.annotate(
            "",
            xy=(hour_x, hour_y),
            xytext=(0, 0),
            arrowprops=dict(
                arrowstyle="->", color=hour_hand_color, lw=8, shrinkA=0, shrinkB=0
            ),
        )
        # Minute hand (medium thickness, medium length, arrow)
        minute_x = 0.75 * np.cos(minute_angle)
        minute_y = 0.75 * np.sin(minute_angle)
        ax.annotate(
            "",
            xy=(minute_x, minute_y),
            xytext=(0, 0),
            arrowprops=dict(
                arrowstyle="->", color=minute_hand_color, lw=5, shrinkA=0, shrinkB=0
            ),
        )
        # Second hand (thin, long, no arrow)
        second_x = 0.85 * np.cos(second_angle)
        second_y = 0.85 * np.sin(second_angle)
        ax.plot(
            [0, second_x],
            [0, second_y],
            color=second_hand_color,
            linewidth=2,
            solid_capstyle="round",
        )
    else:
        # Regular style hands
        # Hour hand (thickest, shortest)
        hour_x = 0.5 * np.cos(hour_angle)
        hour_y = 0.5 * np.sin(hour_angle)
        ax.plot(
            [0, hour_x],
            [0, hour_y],
            color=hour_hand_color,
            linewidth=8,
            solid_capstyle="round",
        )
        # Minute hand (medium thickness, medium length)
        minute_x = 0.75 * np.cos(minute_angle)
        minute_y = 0.75 * np.sin(minute_angle)
        ax.plot(
            [0, minute_x],
            [0, minute_y],
            color=minute_hand_color,
            linewidth=5,
            solid_capstyle="round",
        )
        # Second hand (thinnest, longest)
        second_x = 0.85 * np.cos(second_angle)
        second_y = 0.85 * np.sin(second_angle)
        ax.plot(
            [0, second_x],
            [0, second_y],
            color=second_hand_color,
            linewidth=2,
            solid_capstyle="round",
        )
    # Draw center dot
    center_circle = plt.Circle((0, 0), 0.05, color="black", zorder=10)
    ax.add_patch(center_circle)
    # Save image
    plt.tight_layout()
    plt.savefig(
        filename, dpi=100, bbox_inches="tight", facecolor=bg_color, edgecolor="none"
    )
    plt.close()


def generate_random_time():
    """Generate a random time between 00:00:00 and 11:59:59, and also return 24-hour format by adding 12 hours."""
    hour_12 = random.randint(0, 11)
    minute = random.randint(0, 59)
    second = random.randint(0, 59)
    hour_24 = hour_12 + 12

    return hour_12, hour_24, minute, second


def format_time(hour, minute, second):
    """Format time as hh:mm:ss."""
    return f"{hour}:{minute:02d}:{second:02d}"


@registry.register(name="lff_synthetic_clock", tags={"clock"}, weight=1)
def draw_lff_synthetic_clock(img_path="clock.png"):
    hour_12, hour_24, minute, second = generate_random_time()
    use_roman = random.choice(
        [True, False]
    )  # Randomly choose number type (Arabic or Roman)
    draw_clock(hour_12, minute, second, img_path, use_roman)

    time_12h_lower = format_time(hour_12, minute, second - 1)
    time_12h_upper = format_time(hour_12, minute, second + 1)

    time_24h_lower = format_time(hour_24, minute, second - 1)
    time_24h_upper = format_time(hour_24, minute, second + 1)

    evaluator_kwargs = {
        "intervals": [
            [time_12h_lower, time_12h_upper],
            [time_24h_lower, time_24h_upper],
        ],
        "units": [""],
    }

    return Artifact(
        data=img_path,
        image_type="clock",
        design="Dial",
        evaluator="multi_interval_matching",
        evaluator_kwargs=evaluator_kwargs,
        generator="lff-synthesized",
    )
