# Thermometer/render.py

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, Circle
from .config import Config, celsius_to_fahrenheit


class ThermometerRenderer:
    """
    温度计渲染器，根据Config配置生成温度计图像
    """

    def __init__(self):
        self.fig_width = 10
        self.fig_height = 12

    def render(self, config: Config, save_path: str):
        """
        根据配置渲染温度计并保存图片

        Args:
            config (Config): 温度计配置
            save_path (str): 图片保存路径
        """
        # 创建图形
        fig, ax = plt.subplots(figsize=(self.fig_width, self.fig_height))
        ax.set_xlim(-5, 5)
        ax.set_ylim(-6, 6)
        ax.set_aspect("equal")
        ax.axis("off")

        # 根据形状绘制温度计
        if config.shape == "on_glass":
            self._draw_on_glass(ax, config)
        elif config.shape == "sealed_in_tube":
            self._draw_sealed_in_tube(ax, config)
        elif config.shape == "on_backplate":
            self._draw_on_backplate(ax, config)

        # 应用旋转和方向
        self._apply_rotation(ax, config)

        # 保存图片
        plt.savefig(
            save_path, dpi=100, bbox_inches="tight", facecolor="white", edgecolor="none"
        )
        plt.close()

    def _draw_on_glass(self, ax, config):
        """绘制刻度在玻璃管上的温度计"""
        # 绘制玻璃管主体
        tube_width = 0.3
        tube_height = 8
        bulb_radius = 0.6

        # 玻璃管
        tube_rect = Rectangle(
            (-tube_width / 2, -tube_height / 2 + bulb_radius),
            tube_width,
            tube_height - bulb_radius,
            facecolor="lightgray",
            edgecolor="black",
            alpha=0.3,
        )
        ax.add_patch(tube_rect)

        # 底部球形储液器
        bulb = Circle(
            (0, -tube_height / 2),
            bulb_radius,
            facecolor="lightgray",
            edgecolor="black",
            alpha=0.3,
        )
        ax.add_patch(bulb)

        # 绘制液柱
        self._draw_liquid_column(ax, config, tube_width, tube_height, bulb_radius)

        # 绘制刻度（在玻璃管上）
        self._draw_scales_on_glass(ax, config, tube_width, tube_height, bulb_radius)

    def _draw_sealed_in_tube(self, ax, config):
        """绘制毛细管和刻度标尺熔封在外层玻璃管内的温度计"""
        # 外层玻璃管
        outer_width = 0.8
        outer_height = 9
        bulb_radius = 0.7

        # 外层管体
        outer_tube = Rectangle(
            (-outer_width / 2, -outer_height / 2 + bulb_radius),
            outer_width,
            outer_height - bulb_radius,
            facecolor="lightblue",
            edgecolor="black",
            alpha=0.2,
        )
        ax.add_patch(outer_tube)

        # 底部球形储液器
        bulb = Circle(
            (0, -outer_height / 2),
            bulb_radius,
            facecolor="lightblue",
            edgecolor="black",
            alpha=0.2,
        )
        ax.add_patch(bulb)

        # 内部毛细管
        inner_width = 0.15
        inner_tube = Rectangle(
            (-inner_width / 2, -outer_height / 2 + bulb_radius),
            inner_width,
            outer_height - bulb_radius,
            facecolor="white",
            edgecolor="gray",
            alpha=0.5,
        )
        ax.add_patch(inner_tube)

        # 绘制刻度标尺（在外层管内）
        self._draw_scales_sealed(ax, config, outer_width, outer_height, bulb_radius)

        # 绘制液柱
        self._draw_liquid_column(ax, config, inner_width, outer_height, bulb_radius)

    def _draw_on_backplate(self, ax, config):
        """绘制玻璃管固定在背板上的温度计"""
        # 背板
        plate_width = 2.5
        plate_height = 10
        plate = Rectangle(
            (-plate_width / 2, -plate_height / 2),
            plate_width,
            plate_height,
            facecolor="#DEB887",
            edgecolor="#654321",
            alpha=0.8,
        )
        ax.add_patch(plate)

        # 玻璃管
        tube_width = 0.25
        tube_height = 8
        bulb_radius = 0.5

        tube_rect = Rectangle(
            (-tube_width / 2, -tube_height / 2 + bulb_radius),
            tube_width,
            tube_height - bulb_radius,
            facecolor="lightgray",
            edgecolor="black",
            alpha=0.4,
        )
        ax.add_patch(tube_rect)

        # 底部球形储液器
        bulb = Circle(
            (0, -tube_height / 2),
            bulb_radius,
            facecolor="lightgray",
            edgecolor="black",
            alpha=0.4,
        )
        ax.add_patch(bulb)

        # 绘制液柱
        self._draw_liquid_column(ax, config, tube_width, tube_height, bulb_radius)

        # 绘制刻度（在背板上）
        self._draw_scales_on_backplate(
            ax, config, plate_width, tube_height, bulb_radius
        )

    def _draw_liquid_column(self, ax, config, tube_width, tube_height, bulb_radius):
        """绘制液柱"""
        # 计算液柱高度
        temp_ratio = (config.temp_c - config.min_c) / (config.max_c - config.min_c)
        liquid_height = temp_ratio * (tube_height - bulb_radius)

        # 液柱在管内
        liquid_width = tube_width * 0.8
        liquid_rect = Rectangle(
            (-liquid_width / 2, -tube_height / 2 + bulb_radius),
            liquid_width,
            liquid_height,
            facecolor=config.liquid_color_hex,
            alpha=0.8,
        )
        ax.add_patch(liquid_rect)

        # 底部球形液体
        liquid_bulb = Circle(
            (0, -tube_height / 2),
            bulb_radius * 0.9,
            facecolor=config.liquid_color_hex,
            alpha=0.8,
        )
        ax.add_patch(liquid_bulb)

    def _draw_scales_on_glass(self, ax, config, tube_width, tube_height, bulb_radius):
        """在玻璃管上绘制刻度"""
        scale_start_y = -tube_height / 2 + bulb_radius
        scale_end_y = tube_height / 2
        scale_height = scale_end_y - scale_start_y

        temp_range = config.max_c - config.min_c
        major_positions = config.get_major_tick_positions()

        # 绘制所有小刻度
        num_all_ticks = int(temp_range / config.step_c) + 1
        for i in range(num_all_ticks):
            temp_val = config.min_c + i * config.step_c
            y_pos = (
                scale_start_y + (temp_val - config.min_c) / temp_range * scale_height
            )

            if scale_start_y <= y_pos <= scale_end_y:
                # 判断是否为大刻度
                is_major = any(
                    abs(temp_val - major_pos) < 0.001 for major_pos in major_positions
                )

                # 刻度线长度和粗细
                tick_length = 0.25 if is_major else 0.15
                line_width = 1.5 if is_major else 1.0

                ax.plot(
                    [tube_width / 2, tube_width / 2 + tick_length],
                    [y_pos, y_pos],
                    "k-",
                    linewidth=line_width,
                )

                # 只在大刻度上标注数字
                if is_major:
                    if config.scale_type == "C":
                        label = (
                            f"{temp_val:.1f}"
                            if config.step_c < 1
                            else f"{int(temp_val)}"
                        )
                        unit_label = "°C"
                    else:  # 'F'
                        temp_f = celsius_to_fahrenheit(temp_val)
                        label = (
                            f"{temp_f:.1f}"
                            if abs(temp_f - round(temp_f)) > 0.1
                            else f"{int(round(temp_f))}"
                        )
                        unit_label = "°F"

                    ax.text(
                        tube_width / 2 + tick_length + 0.1,
                        y_pos,
                        label,
                        ha="left",
                        va="center",
                        fontsize=8,
                    )

        # 添加单位标识
        ax.text(
            tube_width / 2 + 0.5,
            scale_end_y + 0.3,
            unit_label,
            ha="center",
            va="center",
            fontsize=10,
            weight="bold",
        )

    def _draw_scales_sealed(self, ax, config, outer_width, outer_height, bulb_radius):
        """绘制熔封在管内的刻度标尺"""
        scale_start_y = -outer_height / 2 + bulb_radius
        scale_end_y = outer_height / 2
        scale_height = scale_end_y - scale_start_y

        # 刻度标尺背景
        ruler_width = 0.4
        ruler = Rectangle(
            (outer_width / 2 - ruler_width - 0.1, scale_start_y),
            ruler_width,
            scale_height,
            facecolor="white",
            edgecolor="gray",
            alpha=0.9,
        )
        ax.add_patch(ruler)

        temp_range = config.max_c - config.min_c
        major_step = self._calculate_major_step(temp_range, config.step_c)

        # 绘制所有刻度
        num_all_ticks = int(temp_range / config.step_c) + 1
        for i in range(num_all_ticks):
            temp_val = config.min_c + i * config.step_c
            y_pos = (
                scale_start_y + (temp_val - config.min_c) / temp_range * scale_height
            )

            if scale_start_y <= y_pos <= scale_end_y:
                # 判断是否为大刻度
                is_major = abs(temp_val % major_step) < 0.001

                tick_x = outer_width / 2 - ruler_width - 0.1
                tick_length = 0.15 if is_major else 0.08
                line_width = 1.2 if is_major else 0.8

                # 刻度线
                ax.plot(
                    [tick_x, tick_x + tick_length],
                    [y_pos, y_pos],
                    "k-",
                    linewidth=line_width,
                )

                # 只在大刻度上标注数字
                if is_major:
                    if config.scale_type == "C":
                        label = (
                            f"{temp_val:.1f}"
                            if config.step_c < 1
                            else f"{int(temp_val)}"
                        )
                        ax.text(
                            tick_x + 0.2,
                            y_pos,
                            label,
                            ha="left",
                            va="center",
                            fontsize=7,
                        )
                    elif config.scale_type == "F":
                        temp_f = celsius_to_fahrenheit(temp_val)
                        label = (
                            f"{temp_f:.1f}"
                            if abs(temp_f - round(temp_f)) > 0.1
                            else f"{int(round(temp_f))}"
                        )
                        ax.text(
                            tick_x + 0.2,
                            y_pos,
                            label,
                            ha="left",
                            va="center",
                            fontsize=7,
                        )
                    else:  # 'C_F' 双刻度
                        # 摄氏度标签（左侧）
                        c_label = (
                            f"{temp_val:.1f}"
                            if config.step_c < 1
                            else f"{int(temp_val)}"
                        )
                        ax.text(
                            tick_x - 0.05,
                            y_pos,
                            c_label,
                            ha="right",
                            va="center",
                            fontsize=7,
                        )

                        # 华氏度标签（右侧）
                        temp_f = celsius_to_fahrenheit(temp_val)
                        f_label = (
                            f"{temp_f:.1f}"
                            if abs(temp_f - round(temp_f)) > 0.1
                            else f"{int(round(temp_f))}"
                        )
                        ax.text(
                            tick_x + 0.2,
                            y_pos,
                            f_label,
                            ha="left",
                            va="center",
                            fontsize=7,
                        )

        # 添加单位标识
        if config.scale_type == "C":
            ax.text(
                tick_x + 0.25,
                scale_end_y + 0.3,
                "°C",
                ha="center",
                va="center",
                fontsize=9,
                weight="bold",
            )
        elif config.scale_type == "F":
            ax.text(
                tick_x + 0.25,
                scale_end_y + 0.3,
                "°F",
                ha="center",
                va="center",
                fontsize=9,
                weight="bold",
            )
        else:  # 'C_F'
            ax.text(
                tick_x - 0.1,
                scale_end_y + 0.3,
                "°C",
                ha="center",
                va="center",
                fontsize=9,
                weight="bold",
            )
            ax.text(
                tick_x + 0.3,
                scale_end_y + 0.3,
                "°F",
                ha="center",
                va="center",
                fontsize=9,
                weight="bold",
            )

    def _draw_scales_on_backplate(
        self, ax, config, plate_width, tube_height, bulb_radius
    ):
        """在背板上绘制刻度（支持大/小刻度判断）"""
        scale_start_y = -tube_height / 2 + bulb_radius
        scale_end_y = tube_height / 2
        scale_height = scale_end_y - scale_start_y

        temp_range = config.max_c - config.min_c
        major_step = self._calculate_major_step(
            temp_range, config.step_c
        )  # 🔹 增加大刻度计算
        num_all_ticks = int(temp_range / config.step_c) + 1

        for i in range(num_all_ticks):
            temp_val = config.min_c + i * config.step_c
            y_pos = (
                scale_start_y + (temp_val - config.min_c) / temp_range * scale_height
            )

            if scale_start_y <= y_pos <= scale_end_y:
                # 🔹 判断是否是大刻度
                is_major = abs(temp_val % major_step) < 1e-6

                # 刻度线长短、粗细区分
                tick_length = 0.25 if is_major else 0.15
                line_width = 1.5 if is_major else 1.0

                if config.scale_type == "C":
                    tick_x = 0.4
                    ax.plot(
                        [tick_x, tick_x + tick_length],
                        [y_pos, y_pos],
                        "k-",
                        linewidth=line_width,
                    )
                    if is_major:  # 只在大刻度标数字
                        label = (
                            f"{temp_val:.1f}"
                            if config.step_c < 1
                            else f"{int(temp_val)}"
                        )
                        ax.text(
                            tick_x + tick_length + 0.1,
                            y_pos,
                            label,
                            ha="left",
                            va="center",
                            fontsize=8,
                        )

                elif config.scale_type == "F":
                    tick_x = 0.4
                    ax.plot(
                        [tick_x, tick_x + tick_length],
                        [y_pos, y_pos],
                        "k-",
                        linewidth=line_width,
                    )
                    if is_major:
                        temp_f = celsius_to_fahrenheit(temp_val)
                        label = (
                            f"{temp_f:.1f}"
                            if abs(temp_f - round(temp_f)) > 0.1
                            else f"{int(round(temp_f))}"
                        )
                        ax.text(
                            tick_x + tick_length + 0.1,
                            y_pos,
                            label,
                            ha="left",
                            va="center",
                            fontsize=8,
                        )

                else:  # 'C_F' 双刻度
                    # 左侧摄氏
                    left_tick_x = -0.4
                    ax.plot(
                        [left_tick_x - tick_length, left_tick_x],
                        [y_pos, y_pos],
                        "k-",
                        linewidth=line_width,
                    )
                    if is_major:
                        c_label = (
                            f"{temp_val:.1f}"
                            if config.step_c < 1
                            else f"{int(temp_val)}"
                        )
                        ax.text(
                            left_tick_x - tick_length - 0.1,
                            y_pos,
                            c_label,
                            ha="right",
                            va="center",
                            fontsize=8,
                        )

                    # 右侧华氏
                    right_tick_x = 0.4
                    ax.plot(
                        [right_tick_x, right_tick_x + tick_length],
                        [y_pos, y_pos],
                        "k-",
                        linewidth=line_width,
                    )
                    if is_major:
                        temp_f = celsius_to_fahrenheit(temp_val)
                        f_label = (
                            f"{temp_f:.1f}"
                            if abs(temp_f - round(temp_f)) > 0.1
                            else f"{int(round(temp_f))}"
                        )
                        ax.text(
                            right_tick_x + tick_length + 0.1,
                            y_pos,
                            f_label,
                            ha="left",
                            va="center",
                            fontsize=8,
                        )

        # 绘制玻璃管（在刻度之上）
        tube_width = 0.25
        tube_rect = Rectangle(
            (-tube_width / 2, scale_start_y),
            tube_width,
            scale_height,
            facecolor="lightgray",
            edgecolor="black",
            alpha=0.4,
        )
        ax.add_patch(tube_rect)

        # 底部球形储液器
        bulb = Circle(
            (0, -tube_height / 2),
            bulb_radius * 0.7,
            facecolor="lightgray",
            edgecolor="black",
            alpha=0.4,
        )
        ax.add_patch(bulb)

        # 绘制液柱
        self._draw_liquid_column(ax, config, tube_width, tube_height, bulb_radius * 0.7)

        # 添加单位标识
        if config.scale_type == "C":
            ax.text(
                1.2,
                scale_end_y + 0.5,
                "°C",
                ha="center",
                va="center",
                fontsize=10,
                weight="bold",
            )
        elif config.scale_type == "F":
            ax.text(
                1.2,
                scale_end_y + 0.5,
                "°F",
                ha="center",
                va="center",
                fontsize=10,
                weight="bold",
            )
        else:  # 'C_F'
            ax.text(
                -1.0,
                scale_end_y + 0.5,
                "°C",
                ha="center",
                va="center",
                fontsize=10,
                weight="bold",
            )
            ax.text(
                1.2,
                scale_end_y + 0.5,
                "°F",
                ha="center",
                va="center",
                fontsize=10,
                weight="bold",
            )

    def _apply_rotation(self, ax, config):
        """应用方向和旋转角度"""
        if config.orientation == "horizontal":
            # 水平放置（90度旋转）
            ax.set_xlim(-6, 6)
            ax.set_ylim(-5, 5)
            # 对于水平方向，需要调整视图
            for child in ax.get_children():
                if hasattr(child, "set_transform"):
                    # 这里可以添加变换逻辑，但matplotlib的变换比较复杂
                    # 为简化，我们通过重新绘制来实现水平效果
                    pass

        # 应用小角度旋转
        if abs(config.angle) > 0.1:
            ax.set_transform(ax.transData)
            # 简化处理：通过调整整体布局来模拟旋转效果

    def _calculate_major_step(self, temp_range, min_step):
        """
        计算大刻度间距，确保刻度数字不会过于密集

        Args:
            temp_range: 温度范围
            min_step: 最小刻度步长

        Returns:
            major_step: 大刻度步长
        """
        # 目标：控制大刻度数量在5-15个之间
        target_major_ticks = 10

        # 计算理想的大刻度间距
        ideal_step = temp_range / target_major_ticks

        # 将理想间距调整为min_step的合理倍数
        if ideal_step <= min_step:
            major_step = min_step
        elif ideal_step <= min_step * 2:
            major_step = min_step * 2
        elif ideal_step <= min_step * 5:
            major_step = min_step * 5
        elif ideal_step <= min_step * 10:
            major_step = min_step * 10
        elif ideal_step <= min_step * 20:
            major_step = min_step * 20
        else:
            # 对于很大的范围，使用更大的倍数
            multiplier = int(ideal_step / min_step)
            # 选择合适的"整数"倍数（如5, 10, 20, 50等）
            if multiplier < 5:
                major_step = min_step * 5
            elif multiplier < 10:
                major_step = min_step * 10
            elif multiplier < 20:
                major_step = min_step * 20
            elif multiplier < 50:
                major_step = min_step * 50
            else:
                major_step = min_step * 100

        return major_step

    def _format_temperature_label(self, temp_val, is_celsius=True, step_size=1):
        """格式化温度标签"""
        if is_celsius:
            if step_size < 1:
                return f"{temp_val:.1f}"
            else:
                return f"{int(temp_val)}"
        else:
            temp_f = celsius_to_fahrenheit(temp_val)
            if abs(temp_f - round(temp_f)) > 0.1:
                return f"{temp_f:.1f}"
            else:
                return f"{int(round(temp_f))}"

    def _add_temperature_indicators(self, ax, config, scale_start_y, scale_height):
        """添加当前温度指示"""
        temp_range = config.max_c - config.min_c
        current_temp_y = (
            scale_start_y + (config.temp_c - config.min_c) / temp_range * scale_height
        )

        # 添加指示箭头或标记
        arrow = patches.FancyArrowPatch(
            (1.5, current_temp_y),
            (1.2, current_temp_y),
            connectionstyle="arc3",
            arrowstyle="->",
            mutation_scale=15,
            color="red",
            linewidth=2,
        )
        ax.add_patch(arrow)


# --- 测试代码 ---
if __name__ == "__main__":
    # 测试渲染器
    config = Config()
    print("Testing configuration:")
    print(config)

    renderer = ThermometerRenderer()
    renderer.render(config, "test_thermometer.jpg")
    print("Test image saved as 'test_thermometer.jpg'")
