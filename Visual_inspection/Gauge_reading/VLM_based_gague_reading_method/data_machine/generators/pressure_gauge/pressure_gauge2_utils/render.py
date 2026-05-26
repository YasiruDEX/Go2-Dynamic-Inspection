import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
import numpy as np
from .config import PressureGaugeConfig, ScaleConfig


class PressureGaugeRenderer:
    """压力计渲染器"""

    def __init__(self, config: PressureGaugeConfig):
        self.config = config
        self.center = (0, 0)
        self.radius = 1.4
        # Change angles for clockwise orientation with opening at the bottom
        self.scale_start_angle = 225  # Start at bottom-left (approx 8 o'clock)
        self.scale_end_angle = (
            -45
        )  # End at bottom-right (approx 4 o'clock) (270-degree sweep)

    def render(self, save_path: str):
        """渲染压力计并保存"""
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect("equal")
        ax.axis("off")

        # 设置背景
        fig.patch.set_facecolor(self.config.background_color)

        # 绘制表盘背景
        self._draw_dial_background(ax)

        # 根据样式绘制不同的背景区域
        self._draw_style_background(ax)

        # 绘制刻度和标签
        if self.config.is_dual_scale:
            self._draw_dual_scales(ax)
        else:
            self._draw_single_scale(ax)

        # 绘制单位
        self._draw_center_unit_labels(ax)

        # 绘制指针
        self._draw_pointer(ax)

        # 绘制中心圆
        self._draw_center_circle(ax)

        # 保存图片
        plt.tight_layout()
        plt.savefig(
            save_path,
            dpi=100,
            bbox_inches="tight",
            facecolor=self.config.background_color,
        )
        plt.close()

    def _draw_dial_background(self, ax):
        """绘制表盘背景"""
        # 外壳
        shell = Circle(
            self.center,
            self.radius,
            facecolor="lightgray",
            edgecolor="black",
            linewidth=3,
        )
        ax.add_patch(shell)

        # 内盘面
        dial_face = Circle(
            self.center,
            self.radius * 0.9,
            facecolor="white",
            edgecolor="black",
            linewidth=1,
        )
        ax.add_patch(dial_face)

    def _draw_style_background(self, ax):
        """根据样式绘制背景区域"""
        # sweep_angle = self.scale_end_angle - self.scale_start_angle
        if self.config.style == 1:
            # 第一种：相同背景色
            pass
        elif self.config.style == 2:
            # 第二种：高数值区域红色背景
            self._draw_danger_zone(ax, 0.7)  # 70%以上为危险区域
        elif self.config.style == 3:
            # 第三种：黄绿红三色背景
            self._draw_color_zones(ax)

    def _draw_danger_zone(self, ax, threshold_ratio: float):
        """绘制危险区域（红色背景）"""
        sweep_angle = self.scale_end_angle - self.scale_start_angle
        start = self.scale_start_angle + sweep_angle * threshold_ratio
        end = self.scale_end_angle

        wedge = Wedge(
            self.center,
            self.radius * 0.9,
            start,
            end,
            facecolor="lightcoral",
            alpha=0.3,
        )
        ax.add_patch(wedge)

    def _draw_color_zones(self, ax):
        """绘制三色区域"""
        sweep_angle = self.scale_end_angle - self.scale_start_angle

        # 绿色区域 (0-40%)
        wedge1 = Wedge(
            self.center,
            self.radius * 0.9,
            self.scale_start_angle,
            self.scale_start_angle + sweep_angle * 0.40,
            facecolor="lightgreen",
            alpha=0.2,
        )
        ax.add_patch(wedge1)

        # 黄色区域 (40-75%)
        wedge2 = Wedge(
            self.center,
            self.radius * 0.9,
            self.scale_start_angle + sweep_angle * 0.40,
            self.scale_start_angle + sweep_angle * 0.75,
            facecolor="yellow",
            alpha=0.2,
        )
        ax.add_patch(wedge2)

        # 红色区域 (75-100%)
        wedge3 = Wedge(
            self.center,
            self.radius * 0.9,
            self.scale_start_angle + sweep_angle * 0.75,
            self.scale_end_angle,
            facecolor="red",
            alpha=0.3,
        )
        ax.add_patch(wedge3)

    def _draw_single_scale(self, ax):
        """绘制单刻度"""
        scale = self.config.scales[0]
        self._draw_scale(
            ax,
            scale,
            tick_start_radius=self.radius * 0.8,
            tick_end_radius=self.radius * 0.9,
            label_radius=self.radius * 0.7,
        )

    def _draw_dual_scales(self, ax):
        """绘制双刻度"""
        outer_scale = self.config.scales[0]
        inner_scale = self.config.scales[1]

        # 外圈刻度：向外延伸，数字在刻度之外
        self._draw_scale(
            ax,
            outer_scale,
            tick_start_radius=self.radius * 0.8,
            tick_end_radius=self.radius * 0.9,
            label_radius=self.radius * 1.02,
        )

        # 内圈刻度：向内延伸，数字在刻度之内
        self._draw_scale(
            ax,
            inner_scale,
            tick_start_radius=self.radius * 0.75,
            tick_end_radius=self.radius * 0.6,
            label_radius=self.radius * 0.5,
        )

    def _draw_scale(
        self,
        ax,
        scale: ScaleConfig,
        tick_start_radius: float,
        tick_end_radius: float,
        label_radius: float,
        show_numbers: bool = True,
    ):
        """绘制通用刻度"""
        # Handle cases where min/max are the same to avoid division by zero
        if scale.max_val == scale.min_val:
            return

        values = np.arange(
            scale.min_val, scale.max_val + scale.min_unit / 2, scale.min_unit
        )

        # Determine tick direction and length
        tick_direction = np.sign(tick_end_radius - tick_start_radius)
        major_tick_length = abs(tick_end_radius - tick_start_radius)
        minor_tick_length = major_tick_length * 0.6

        for value in values:
            ratio = (value - scale.min_val) / (scale.max_val - scale.min_val)
            angle = self.scale_start_angle + ratio * (
                self.scale_end_angle - self.scale_start_angle
            )
            angle_rad = np.radians(angle)

            # Check for major tick or labeled tick
            is_major = (
                abs(value % scale.major_tick) < scale.min_unit / 2
                if scale.major_tick > 0
                else False
            )
            is_label = (
                abs(value % scale.label_tick) < scale.min_unit / 2
                if scale.label_tick > 0
                else False
            )

            # Set tick length and width
            if is_major or is_label:
                current_tick_end = tick_end_radius
                linewidth = 1.5
            else:
                current_tick_end = (
                    tick_start_radius + tick_direction * minor_tick_length
                )
                linewidth = 1

            x1 = tick_start_radius * np.cos(angle_rad)
            y1 = tick_start_radius * np.sin(angle_rad)
            x2 = current_tick_end * np.cos(angle_rad)
            y2 = current_tick_end * np.sin(angle_rad)
            ax.plot([x1, x2], [y1, y2], color=scale.color, linewidth=linewidth)

            # 绘制数字标签
            if is_label and show_numbers:
                label_x = label_radius * 0.95 * np.cos(angle_rad)
                label_y = label_radius * 0.95 * np.sin(angle_rad)

                # Format number to remove unnecessary decimals
                if abs(value - round(value)) < 1e-9:
                    label_text = str(int(round(value)))
                else:
                    label_text = f"{value:.1f}"

                # Rotate text to be upright
                text_angle = angle
                if 90 < (text_angle % 360) < 270:
                    text_angle += 180

                ax.text(
                    label_x,
                    label_y,
                    label_text,
                    ha="center",
                    va="center",
                    fontsize=10,
                    color=scale.color,
                    weight="bold",
                    rotation=text_angle,
                )

    def _draw_center_unit_labels(self, ax):
        """在表盘中心绘制单位标识"""
        unit_names_list = self.config.unit_display_names

        if self.config.is_dual_scale and len(unit_names_list) > 1:
            unit1 = unit_names_list[0][0]
            unit2 = unit_names_list[1][0]
            color1 = self.config.scales[0].color
            color2 = self.config.scales[1].color

            ax.text(
                0,
                0.15,
                unit1,
                ha="center",
                va="center",
                fontsize=12,
                color=color1,
                weight="bold",
            )
            ax.text(
                0,
                -0.15,
                unit2,
                ha="center",
                va="center",
                fontsize=12,
                color=color2,
                weight="bold",
            )
        else:
            unit = unit_names_list[0][0]
            color = self.config.scales[0].color
            ax.text(
                0,
                0.15,
                unit,
                ha="center",
                va="center",
                fontsize=14,
                color=color,
                weight="bold",
            )

    def _draw_pointer(self, ax):
        """绘制指针"""
        primary_scale = self.config.scales[0]
        # Handle cases where min/max are the same
        if primary_scale.max_val == primary_scale.min_val:
            ratio = 0
        else:
            value = np.clip(
                self.config.actual_value, primary_scale.min_val, primary_scale.max_val
            )
            ratio = (value - primary_scale.min_val) / (
                primary_scale.max_val - primary_scale.min_val
            )

        angle = self.scale_start_angle + ratio * (
            self.scale_end_angle - self.scale_start_angle
        )
        angle_rad = np.radians(angle)

        pointer_length = self.radius * 0.9
        x_end = pointer_length * np.cos(angle_rad)
        y_end = pointer_length * np.sin(angle_rad)

        # 根据指针样式绘制
        if self.config.pointer_style == 1:
            # 简单直线指针
            ax.plot(
                [0, x_end],
                [0, y_end],
                color=self.config.pointer_color,
                linewidth=3,
                solid_capstyle="round",
            )
        elif self.config.pointer_style == 2:
            # 箭头指针
            ax.annotate(
                "",
                xy=(x_end, y_end),
                xytext=(0, 0),
                arrowprops=dict(
                    arrowstyle="->,head_length=0.6,head_width=0.4",
                    lw=3,
                    color=self.config.pointer_color,
                ),
            )
        elif self.config.pointer_style == 3:
            # 带尾巴的指针
            tail_length = pointer_length * 0.2
            x_tail = -tail_length * np.cos(angle_rad)
            y_tail = -tail_length * np.sin(angle_rad)
            ax.plot(
                [x_tail, x_end],
                [y_tail, y_end],
                color=self.config.pointer_color,
                linewidth=3,
            )
        else:
            # 粗指针
            ax.plot(
                [0, x_end],
                [0, y_end],
                color=self.config.pointer_color,
                linewidth=5,
                solid_capstyle="round",
            )

    def _draw_center_circle(self, ax):
        """绘制中心圆"""
        center_circle = Circle(
            self.center,
            0.05,
            facecolor=self.config.pointer_color,
            edgecolor="black",
            linewidth=1,
        )
        ax.add_patch(center_circle)
