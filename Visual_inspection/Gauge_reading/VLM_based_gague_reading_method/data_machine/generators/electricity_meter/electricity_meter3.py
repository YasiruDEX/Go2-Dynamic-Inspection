import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import random
from io import BytesIO
from PIL import Image
from registry import registry
from artifacts import Artifact


def generate_electric_meter(reading=None):
    """
    Generate a simulated electric meter image.

    Args:
        reading (list): List of 5 digits. If None, randomly generated.

    Returns:
        matplotlib.figure.Figure: Generated electric meter image.
    """
    if reading is None:
        reading = [random.randint(0, 9) for _ in range(5)]
    if len(reading) != 5:
        reading = (
            reading[:5] if len(reading) > 5 else reading + [0] * (5 - len(reading))
        )
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor("#D2B48C")
    meter_body = FancyBboxPatch(
        (1, 2),
        8,
        5,
        boxstyle="round,pad=0.1",
        facecolor="#2F4F4F",
        edgecolor="black",
        linewidth=2,
    )
    ax.add_patch(meter_body)
    display_area = FancyBboxPatch(
        (1.5, 4),
        7,
        2.5,
        boxstyle="round,pad=0.05",
        facecolor="white",
        edgecolor="gray",
        linewidth=1,
    )
    ax.add_patch(display_area)
    ax.text(2, 6, "EC 86521", fontsize=10, fontweight="bold", ha="left")
    ax.text(6.5, 6, "kWh", fontsize=12, fontweight="bold", ha="center")

    for i in range(5):
        bg_color = "red" if i == 4 else "black"
        digit_bg = patches.Rectangle(
            (2.5 + i * 0.8, 5),
            0.7,
            0.8,
            facecolor=bg_color,
            edgecolor="gray",
            linewidth=1,
        )
        ax.add_patch(digit_bg)
    for i, digit in enumerate(reading):
        text_color = "white" if i == 4 else "lime"
        ax.text(
            2.85 + i * 0.8,
            5.4,
            str(digit),
            fontsize=20,
            fontweight="bold",
            color=text_color,
            ha="center",
            va="center",
            family="monospace",
        )
    ax.text(
        5,
        4.5,
        "SINGLE-PHASE WATTHOUR METER",
        fontsize=8,
        ha="center",
        fontweight="bold",
    )
    ax.text(2, 3.8, "TYPE DD26   1PHASE   2WIRE", fontsize=7, ha="left")
    ax.text(2, 3.6, "220V   50Hz   600rev/kWh", fontsize=7, ha="left")
    ax.text(2, 3.4, "10(20)A  Class 1", fontsize=7, ha="left")
    ax.text(5, 3.1, "MADE IN CHINA", fontsize=8, ha="center", fontweight="bold")
    terminal_area = FancyBboxPatch(
        (3, 1.2),
        4,
        0.6,
        boxstyle="round,pad=0.05",
        facecolor="#404040",
        edgecolor="black",
        linewidth=1,
    )
    ax.add_patch(terminal_area)
    for i in range(4):
        terminal = patches.Circle(
            (3.5 + i * 0.8, 1.5), 0.15, facecolor="silver", edgecolor="black"
        )
        ax.add_patch(terminal)
        screw = patches.Circle((3.5 + i * 0.8, 1.5), 0.05, facecolor="black")
        ax.add_patch(screw)
    plt.tight_layout()
    return fig


def generate_meter_image(reading=None, save_path=None):
    """
    Generate electric meter image and return PIL Image object, optionally save to path.

    Args:
        reading (list): List of 5 digits. If None, randomly generated.
        save_path (str): Save path. If None, do not save.

    Returns:
        PIL.Image: Generated electric meter image object.
    """
    fig = generate_electric_meter(reading)
    buf = BytesIO()
    fig.savefig(
        buf,
        format="png",
        dpi=300,
        bbox_inches="tight",
        facecolor=fig.get_facecolor(),
        edgecolor="none",
    )
    buf.seek(0)
    img = Image.open(buf)
    plt.close(fig)
    if img.mode == "RGBA":
        img = img.convert("RGB")
    img.save(save_path)
    return img


@registry.register(name="electricity_meter3", tags={"electricity_meter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    random_reading = [random.randint(0, 9) for _ in range(5)]
    generate_meter_image(random_reading, save_path=img_path)
    reading = (
        random_reading[0] * 1000
        + random_reading[1] * 100
        + random_reading[2] * 10
        + random_reading[3] * 1
        + random_reading[4] * 0.1
    )
    evaluator_kwargs = {
        "interval": [reading, reading],
        "units": ["kWh", "kilowatt-hour", "kilowatt-hours"],
    }
    return Artifact(
        data=img_path,
        image_type="electricity_meter",
        design="Digital",
        evaluator_kwargs=evaluator_kwargs,
    )
