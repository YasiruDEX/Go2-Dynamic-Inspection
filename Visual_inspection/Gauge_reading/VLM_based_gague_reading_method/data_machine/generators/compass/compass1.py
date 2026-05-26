import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from registry import registry
from artifacts import Artifact


def generate_random_compass():
    compass_styles = [
        {
            "name": "classic_navy",
            "background": "#1e3a8a",
            "compass_bg": "#fef3c7",
            "compass_border": "#1f2937",
            "ring_color": "#92400e",
            "face_color": "#fbbf24",
            "text_color": "#1f2937",
            "needle_color": "#dc2626",
        },
        {
            "name": "vintage_brass",
            "background": "#92400e",
            "compass_bg": "#fed7aa",
            "compass_border": "#451a03",
            "ring_color": "#78350f",
            "face_color": "#fb923c",
            "text_color": "#451a03",
            "needle_color": "#dc2626",
        },
        {
            "name": "modern_minimal",
            "background": "#374151",
            "compass_bg": "#f9fafb",
            "compass_border": "#111827",
            "ring_color": "#374151",
            "face_color": "#e5e7eb",
            "text_color": "#111827",
            "needle_color": "#ef4444",
        },
        {
            "name": "military_green",
            "background": "#365314",
            "compass_bg": "#ecfccb",
            "compass_border": "#1a2e05",
            "ring_color": "#365314",
            "face_color": "#bef264",
            "text_color": "#1a2e05",
            "needle_color": "#dc2626",
        },
        {
            "name": "rose_gold",
            "background": "#be185d",
            "compass_bg": "#fce7f3",
            "compass_border": "#831843",
            "ring_color": "#be185d",
            "face_color": "#f9a8d4",
            "text_color": "#831843",
            "needle_color": "#dc2626",
        },
        {
            "name": "deep_ocean",
            "background": "#0c4a6e",
            "compass_bg": "#e0f2fe",
            "compass_border": "#0c2c42",
            "ring_color": "#0c4a6e",
            "face_color": "#7dd3fc",
            "text_color": "#0c2c42",
            "needle_color": "#dc2626",
        },
        {
            "name": "vintage_copper",
            "background": "#7c2d12",
            "compass_bg": "#fecaca",
            "compass_border": "#450a0a",
            "ring_color": "#7c2d12",
            "face_color": "#f87171",
            "text_color": "#450a0a",
            "needle_color": "#dc2626",
        },
        {
            "name": "moonlight_silver",
            "background": "#4b5563",
            "compass_bg": "#e2e8f0",
            "compass_border": "#1f2937",
            "ring_color": "#4b5563",
            "face_color": "#cbd5e1",
            "text_color": "#1f2937",
            "needle_color": "#dc2626",
        },
        {
            "name": "forest_green",
            "background": "#14532d",
            "compass_bg": "#dcfce7",
            "compass_border": "#052e16",
            "ring_color": "#14532d",
            "face_color": "#86efac",
            "text_color": "#052e16",
            "needle_color": "#dc2626",
        },
        {
            "name": "sunset_orange",
            "background": "#c2410c",
            "compass_bg": "#fed7aa",
            "compass_border": "#7c2d12",
            "ring_color": "#c2410c",
            "face_color": "#fdba74",
            "text_color": "#7c2d12",
            "needle_color": "#dc2626",
        },
    ]
    needle_styles = [
        {
            "name": "traditional_arrow",
            "north_style": "arrow",
            "south_style": "arrow",
            "north_color": "#dc2626",
            "south_color": "#374151",
            "width": 6,
        },
        {
            "name": "diamond_tip",
            "north_style": "diamond",
            "south_style": "diamond",
            "north_color": "#ef4444",
            "south_color": "#6b7280",
            "width": 5,
        },
        {
            "name": "triangle_tip",
            "north_style": "triangle",
            "south_style": "triangle",
            "north_color": "#dc2626",
            "south_color": "#4b5563",
            "width": 7,
        },
        {
            "name": "circle_tip",
            "north_style": "circle",
            "south_style": "circle",
            "north_color": "#f87171",
            "south_color": "#9ca3af",
            "width": 5,
        },
        {
            "name": "leaf_tip",
            "north_style": "leaf",
            "south_style": "leaf",
            "north_color": "#dc2626",
            "south_color": "#374151",
            "width": 6,
        },
        {
            "name": "square_tip",
            "north_style": "square",
            "south_style": "square",
            "north_color": "#ef4444",
            "south_color": "#6b7280",
            "width": 5,
        },
    ]
    needle_angle = random.randint(0, 359)
    compass_rotation = random.randint(0, 359)
    compass_style = random.choice(compass_styles)
    needle_style = random.choice(needle_styles)
    return compass_style, needle_style, needle_angle, compass_rotation


def draw_needle_tip(ax, x, y, angle, tip_style, color, size=0.08):
    """Draw different styles of needle tips."""
    perp_angle = angle + np.pi / 2

    if tip_style == "arrow":
        left_x = x - size * np.cos(angle) + size / 2 * np.cos(perp_angle)
        left_y = y - size * np.sin(angle) + size / 2 * np.sin(perp_angle)
        right_x = x - size * np.cos(angle) - size / 2 * np.cos(perp_angle)
        right_y = y - size * np.sin(angle) - size / 2 * np.sin(perp_angle)
        arrow = plt.Polygon(
            [(x, y), (left_x, left_y), (right_x, right_y)], color=color, alpha=0.9
        )
        ax.add_patch(arrow)

    elif tip_style == "diamond":
        tip1_x = x + size / 2 * np.cos(angle)
        tip1_y = y + size / 2 * np.sin(angle)
        tip2_x = x - size * np.cos(angle) + size / 3 * np.cos(perp_angle)
        tip2_y = y - size * np.sin(angle) + size / 3 * np.sin(perp_angle)
        tip3_x = x - size / 2 * np.cos(angle)
        tip3_y = y - size / 2 * np.sin(angle)
        tip4_x = x - size * np.cos(angle) - size / 3 * np.cos(perp_angle)
        tip4_y = y - size * np.sin(angle) - size / 3 * np.sin(perp_angle)
        diamond = plt.Polygon(
            [(tip1_x, tip1_y), (tip2_x, tip2_y), (tip3_x, tip3_y), (tip4_x, tip4_y)],
            color=color,
            alpha=0.9,
        )
        ax.add_patch(diamond)

    elif tip_style == "triangle":
        left_x = x - size * np.cos(angle) + size * 0.6 * np.cos(perp_angle)
        left_y = y - size * np.sin(angle) + size * 0.6 * np.sin(perp_angle)
        right_x = x - size * np.cos(angle) - size * 0.6 * np.cos(perp_angle)
        right_y = y - size * np.sin(angle) - size * 0.6 * np.sin(perp_angle)
        triangle = plt.Polygon(
            [(x, y), (left_x, left_y), (right_x, right_y)], color=color, alpha=0.9
        )
        ax.add_patch(triangle)

    elif tip_style == "circle":
        circle = Circle((x, y), size / 2, color=color, alpha=0.9)
        ax.add_patch(circle)

    elif tip_style == "leaf":
        from matplotlib.patches import Ellipse

        ellipse = Ellipse(
            (x, y), size, size * 0.6, angle=np.degrees(angle), color=color, alpha=0.9
        )
        ax.add_patch(ellipse)


def create_compass_matplotlib(
    compass_style, needle_style, needle_angle, compass_rotation, img_path
):
    """Draw a compass with random style and needle."""
    fig, ax = plt.subplots(1, 1, figsize=(8, 8), facecolor=compass_style["background"])
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor(compass_style["background"])
    outer_ring = Circle(
        (0, 0), 1.1, color=compass_style["compass_border"], linewidth=8, fill=False
    )
    ax.add_patch(outer_ring)
    compass_body = Circle((0, 0), 1.0, color=compass_style["compass_bg"], alpha=0.9)
    ax.add_patch(compass_body)
    inner_ring = Circle(
        (0, 0),
        0.95,
        color=compass_style["ring_color"],
        linewidth=4,
        fill=False,
        alpha=0.7,
    )
    ax.add_patch(inner_ring)
    face_circle = Circle((0, 0), 0.9, color=compass_style["face_color"], alpha=0.6)
    ax.add_patch(face_circle)
    for i in range(0, 360, 5):
        display_angle_rad = np.radians(90 - i - compass_rotation)
        if i % 90 == 0:
            r1, r2 = 0.7, 0.9
            linewidth = 3
        elif i % 30 == 0:
            r1, r2 = 0.75, 0.9
            linewidth = 2
        else:
            r1, r2 = 0.8, 0.9
            linewidth = 1
        x1, y1 = r1 * np.cos(display_angle_rad), r1 * np.sin(display_angle_rad)
        x2, y2 = r2 * np.cos(display_angle_rad), r2 * np.sin(display_angle_rad)
        ax.plot(
            [x1, x2],
            [y1, y2],
            color=compass_style["text_color"],
            linewidth=linewidth,
            alpha=0.8,
        )
    for i in range(0, 360, 30):
        if i % 90 != 0:
            display_angle_rad = np.radians(90 - i - compass_rotation)
            x, y = 0.65 * np.cos(display_angle_rad), 0.65 * np.sin(display_angle_rad)
            ax.text(
                x,
                y,
                str(i),
                fontsize=14,
                ha="center",
                va="center",
                color=compass_style["text_color"],
                weight="bold",
                bbox=dict(boxstyle="round,pad=0.1", facecolor="white", alpha=0.7),
            )
    directions = [
        (0, "N", "#dc2626"),
        (90, "E", "#2563eb"),
        (180, "S", "#16a34a"),
        (270, "W", "#ca8a04"),
    ]
    for deg, label, color in directions:
        display_angle_rad = np.radians(90 - deg - compass_rotation)
        x, y = 0.55 * np.cos(display_angle_rad), 0.55 * np.sin(display_angle_rad)
        ax.text(
            x,
            y,
            label,
            fontsize=20,
            ha="center",
            va="center",
            color=color,
            weight="bold",
            bbox=dict(
                boxstyle="circle,pad=0.3",
                facecolor="white",
                alpha=0.9,
                edgecolor=color,
                linewidth=2,
            ),
        )
    for i in range(0, 360, 45):
        display_angle_rad = np.radians(90 - i - compass_rotation)
        x1, y1 = 0.3 * np.cos(display_angle_rad), 0.3 * np.sin(display_angle_rad)
        x2, y2 = 0.4 * np.cos(display_angle_rad), 0.4 * np.sin(display_angle_rad)
        ax.plot(
            [x1, x2],
            [y1, y2],
            color=compass_style["text_color"],
            linewidth=1,
            alpha=0.3,
        )
    needle_length = 0.7
    needle_angle_rad = np.radians(90 - needle_angle)
    north_x = needle_length * np.cos(needle_angle_rad)
    north_y = needle_length * np.sin(needle_angle_rad)
    south_x = -needle_length * np.cos(needle_angle_rad)
    south_y = -needle_length * np.sin(needle_angle_rad)
    ax.plot(
        [0, north_x],
        [0, north_y],
        color=needle_style["north_color"],
        linewidth=needle_style["width"],
        solid_capstyle="round",
        alpha=0.9,
    )
    ax.plot(
        [0, south_x],
        [0, south_y],
        color=needle_style["south_color"],
        linewidth=needle_style["width"],
        solid_capstyle="round",
        alpha=0.9,
    )
    draw_needle_tip(
        ax,
        north_x,
        north_y,
        needle_angle_rad,
        needle_style["north_style"],
        needle_style["north_color"],
    )
    draw_needle_tip(
        ax,
        south_x,
        south_y,
        needle_angle_rad + np.pi,
        needle_style["south_style"],
        needle_style["south_color"],
    )
    center_dot = Circle((0, 0), 0.06, color="white", zorder=10)
    ax.add_patch(center_dot)
    center_dot_border = Circle(
        (0, 0),
        0.06,
        color=compass_style["text_color"],
        fill=False,
        linewidth=2,
        zorder=11,
    )
    ax.add_patch(center_dot_border)
    shadow_circle = Circle((0.02, -0.02), 0.95, color="black", alpha=0.2, zorder=-1)
    ax.add_patch(shadow_circle)
    plt.tight_layout()
    plt.savefig(
        img_path, dpi=100, bbox_inches="tight", facecolor=compass_style["background"]
    )
    plt.close()


@registry.register(name="lff_synthetic_compass", tags={"compass"}, weight=1)
def draw_lff_synthetic_compass(img_path="compass.png"):
    compass_style, needle_style, needle_angle, compass_rotation = (
        generate_random_compass()
    )
    create_compass_matplotlib(
        compass_style, needle_style, needle_angle, compass_rotation, img_path
    )

    # Calculate actual north direction (angle relative to image)
    actual_north_angle = (needle_angle - compass_rotation) % 360
    lower_angle = (actual_north_angle // 5) * 5
    upper_angle = lower_angle + 5 if actual_north_angle % 5 != 0 else lower_angle
    evaluator_kwargs = {
        "interval": [lower_angle, upper_angle],
        "units": ["degree", "°"],
    }
    return Artifact(
        data=img_path,
        image_type="compass",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
        generator="lff-synthesized",
    )
