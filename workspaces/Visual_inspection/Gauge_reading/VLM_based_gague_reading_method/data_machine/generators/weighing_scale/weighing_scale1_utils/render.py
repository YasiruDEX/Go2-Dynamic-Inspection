import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import os
from .config import ScaleConfig


class WeighingScaleRenderer:
    """圆盘称渲染器"""

    def __init__(self, config: ScaleConfig):
        self.config = config
        self.fig_size = (8, 8)
        self.center = (0, 0)

    def calculate_angle(self, value: float) -> float:
        """根据数值计算角度"""
        if self.config.scale_type == 0:
            # 0刻度和最大刻度重叠的情况
            if value == 0 or value == self.config.max_value:
                return 90  # 12点位置
            else:
                # 从12点开始，顺时针方向
                angle_per_unit = 360 / self.config.max_value
                angle = 90 - (value * angle_per_unit)
                return angle % 360
        else:
            # 0刻度不重叠的情况，0在12点，最大值在55分位置（330度）
            angle_per_unit = 330 / self.config.max_value
            angle = 90 - (value * angle_per_unit)
            return angle % 360

    def draw_dial(self, ax):
        """绘制表盘"""
        # 绘制表盘圆形
        dial_circle = patches.Circle(
            self.center,
            self.config.dial_radius,
            facecolor=self.config.dial_color,
            edgecolor=self.config.tick_color,
            linewidth=3,
        )
        ax.add_patch(dial_circle)

        # 绘制内圈装饰
        inner_circle = patches.Circle(
            self.center,
            self.config.dial_radius * 0.15,
            facecolor=self.config.tick_color,
            alpha=0.3,
        )
        ax.add_patch(inner_circle)

    def draw_ticks_and_labels(self, ax):
        """绘制刻度和标签"""
        max_val = self.config.max_value
        min_unit = self.config.min_unit
        major_tick = self.config.major_tick
        labeled_tick = self.config.labeled_tick

        # 计算角度范围
        if self.config.scale_type == 0:
            angle_range = 360
            start_angle = 90
        else:
            angle_range = 330
            start_angle = 90

        # 首先绘制所有小刻度
        num_minor_ticks = int(round(max_val / min_unit))

        for i in range(num_minor_ticks + 1):
            value = round(i * min_unit, 6)  # 避免浮点精度问题

            # 对于重叠式刻度，跳过最后一个刻度（因为与0重叠）
            if self.config.scale_type == 0 and i == num_minor_ticks:
                continue

            angle_deg = start_angle - (value / max_val * angle_range)
            angle_rad = math.radians(angle_deg)

            # 绘制小刻度
            outer_radius = self.config.dial_radius - 5
            inner_radius = outer_radius - self.config.minor_tick_length

            x1 = self.center[0] + inner_radius * math.cos(angle_rad)
            y1 = self.center[1] + inner_radius * math.sin(angle_rad)
            x2 = self.center[0] + outer_radius * math.cos(angle_rad)
            y2 = self.center[1] + outer_radius * math.sin(angle_rad)

            ax.plot(
                [x1, x2],
                [y1, y2],
                color=self.config.tick_color,
                linewidth=self.config.tick_width,
            )

        # 然后绘制大刻度（覆盖小刻度）
        num_major_ticks = int(round(max_val / major_tick))

        for i in range(num_major_ticks + 1):
            value = round(i * major_tick, 6)  # 避免浮点精度问题

            # 对于重叠式刻度，跳过最后一个刻度（因为与0重叠）
            if self.config.scale_type == 0 and i == num_major_ticks:
                continue

            angle_deg = start_angle - (value / max_val * angle_range)
            angle_rad = math.radians(angle_deg)

            # 绘制大刻度
            outer_radius = self.config.dial_radius - 5
            inner_radius = outer_radius - self.config.major_tick_length

            x1 = self.center[0] + inner_radius * math.cos(angle_rad)
            y1 = self.center[1] + inner_radius * math.sin(angle_rad)
            x2 = self.center[0] + outer_radius * math.cos(angle_rad)
            y2 = self.center[1] + outer_radius * math.sin(angle_rad)

            ax.plot(
                [x1, x2],
                [y1, y2],
                color=self.config.tick_color,
                linewidth=self.config.major_tick_width,
            )

        # 最后绘制标签（放在圆盘内侧）
        num_labeled_ticks = int(round(max_val / labeled_tick))

        for i in range(num_labeled_ticks + 1):
            value = round(i * labeled_tick, 6)  # 避免浮点精度问题

            # 对于重叠式刻度，跳过最后一个刻度（因为与0重叠）
            if self.config.scale_type == 0 and i == num_labeled_ticks:
                continue

            angle_deg = start_angle - (value / max_val * angle_range)
            angle_rad = math.radians(angle_deg)

            # 将标签放在圆盘内侧，大刻度的内侧
            label_radius = self.config.dial_radius - self.config.major_tick_length - 25
            label_x = self.center[0] + label_radius * math.cos(angle_rad)
            label_y = self.center[1] + label_radius * math.sin(angle_rad)

            # 特殊处理0和最大值重叠的情况
            if self.config.scale_type == 0 and value == 0:
                label_text = f"{int(max_val)}"
            else:
                if value >= 1:
                    label_text = f"{int(value)}"
                else:
                    label_text = f"{value:.1f}"

            ax.text(
                label_x,
                label_y,
                label_text,
                ha="center",
                va="center",
                fontsize=11,
                color=self.config.text_color,
                weight="bold",
            )

        # 添加单位标识
        unit_y = self.center[1] - self.config.dial_radius * 0.6
        ax.text(
            self.center[0],
            unit_y,
            "Kg",
            ha="center",
            va="center",
            fontsize=16,
            color=self.config.text_color,
            weight="bold",
        )

    def draw_pointer(self, ax, value: float):
        """绘制指针"""
        angle_deg = self.calculate_angle(value)
        angle_rad = math.radians(angle_deg)

        pointer_length = self.config.dial_radius * 0.75

        if self.config.pointer_style == "arrow":
            self._draw_arrow_pointer(ax, angle_rad, pointer_length)
        elif self.config.pointer_style == "line":
            self._draw_line_pointer(ax, angle_rad, pointer_length)
        else:  # triangle
            self._draw_triangle_pointer(ax, angle_rad, pointer_length)

        # 绘制中心圆
        center_circle = patches.Circle(
            self.center,
            8,
            facecolor=self.config.pointer_color,
            edgecolor="black",
            linewidth=2,
        )
        ax.add_patch(center_circle)

    def _draw_arrow_pointer(self, ax, angle_rad: float, length: float):
        """绘制箭头指针"""
        end_x = self.center[0] + length * math.cos(angle_rad)
        end_y = self.center[1] + length * math.sin(angle_rad)

        # 主指针线
        ax.plot(
            [self.center[0], end_x],
            [self.center[1], end_y],
            color=self.config.pointer_color,
            linewidth=self.config.pointer_width,
        )

        # 箭头
        arrow_length = 15
        arrow_angle = math.pi / 6

        arrow_x1 = end_x - arrow_length * math.cos(angle_rad + arrow_angle)
        arrow_y1 = end_y - arrow_length * math.sin(angle_rad + arrow_angle)
        arrow_x2 = end_x - arrow_length * math.cos(angle_rad - arrow_angle)
        arrow_y2 = end_y - arrow_length * math.sin(angle_rad - arrow_angle)

        ax.plot(
            [end_x, arrow_x1],
            [end_y, arrow_y1],
            color=self.config.pointer_color,
            linewidth=self.config.pointer_width,
        )
        ax.plot(
            [end_x, arrow_x2],
            [end_y, arrow_y2],
            color=self.config.pointer_color,
            linewidth=self.config.pointer_width,
        )

    def _draw_line_pointer(self, ax, angle_rad: float, length: float):
        """绘制直线指针"""
        end_x = self.center[0] + length * math.cos(angle_rad)
        end_y = self.center[1] + length * math.sin(angle_rad)

        ax.plot(
            [self.center[0], end_x],
            [self.center[1], end_y],
            color=self.config.pointer_color,
            linewidth=self.config.pointer_width,
            solid_capstyle="round",
        )

    def _draw_triangle_pointer(self, ax, angle_rad: float, length: float):
        """绘制三角形指针"""
        end_x = self.center[0] + length * math.cos(angle_rad)
        end_y = self.center[1] + length * math.sin(angle_rad)

        # 计算三角形的三个顶点
        base_width = 10
        base_angle1 = angle_rad + math.pi / 2
        base_angle2 = angle_rad - math.pi / 2

        base_x1 = self.center[0] + base_width * math.cos(base_angle1)
        base_y1 = self.center[1] + base_width * math.sin(base_angle1)
        base_x2 = self.center[0] + base_width * math.cos(base_angle2)
        base_y2 = self.center[1] + base_width * math.sin(base_angle2)

        triangle = patches.Polygon(
            [(end_x, end_y), (base_x1, base_y1), (base_x2, base_y2)],
            facecolor=self.config.pointer_color,
            edgecolor="black",
            linewidth=1,
        )
        ax.add_patch(triangle)

    def render(self, value: float, save_path: str = None) -> tuple:
        """渲染完整的圆盘称"""
        fig, ax = plt.subplots(1, 1, figsize=self.fig_size)

        # 设置背景颜色
        fig.patch.set_facecolor(self.config.background_color)
        ax.set_facecolor(self.config.background_color)

        # 设置坐标轴
        ax.set_xlim(-300, 300)
        ax.set_ylim(-300, 300)
        ax.set_aspect("equal")
        ax.axis("off")

        # 绘制各个组件
        self.draw_dial(ax)
        self.draw_ticks_and_labels(ax)
        self.draw_pointer(ax, value)

        # 添加标题或品牌标识
        title_y = self.center[1] + self.config.dial_radius * 0.4
        ax.text(
            self.center[0],
            title_y,
            "SCALE",
            ha="center",
            va="center",
            fontsize=14,
            color=self.config.text_color,
            weight="bold",
        )

        plt.tight_layout()

        if save_path:
            plt.savefig(
                save_path,
                dpi=100,
                bbox_inches="tight",
                facecolor=self.config.background_color,
            )
            plt.close()

        # 计算实际的区间值
        min_unit = self.config.min_unit
        lower_bound = math.floor(value / min_unit) * min_unit
        upper_bound = math.ceil(value / min_unit) * min_unit

        if abs(value - lower_bound) < 1e-6:
            # 正好在刻度上
            interval = [value, value]
        else:
            interval = [lower_bound, upper_bound]

        return fig, interval


def test_render():
    """测试渲染功能"""
    from config import ConfigGenerator

    # 创建输出目录
    os.makedirs("WeighingScale/img", exist_ok=True)

    # 生成测试配置
    config = ConfigGenerator.generate_random_config()
    renderer = WeighingScaleRenderer(config)

    # 生成随机测试值
    test_value = np.random.uniform(0, config.max_value)

    # 渲染并保存
    fig, interval = renderer.render(test_value, "WeighingScale/img/test.jpg")
    print(f"Rendered scale with value: {test_value:.3f}")
    print(f"Interval: {interval}")


if __name__ == "__main__":
    test_render()
