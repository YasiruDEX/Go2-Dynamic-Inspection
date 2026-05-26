import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
from .config import Config, VesselType, ConfigGenerator


class VesselRenderer:
    """容器渲染器"""

    def __init__(self, config: Config):
        self.config = config
        self.fig_width = 10
        self.fig_height = 12

    def render(self, output_path: str):
        """渲染容器并保存图片"""
        fig, ax = plt.subplots(1, 1, figsize=(self.fig_width, self.fig_height))
        ax.set_xlim(-8, 8)
        ax.set_ylim(-2, 14)
        ax.set_aspect("equal")
        ax.axis("off")

        # 绘制容器轮廓
        self._draw_vessel_outline(ax)

        # 绘制刻度
        self._draw_scales(ax)

        # 先绘制刻度，再绘制液体（因为刻度会进行缩放，液体高度也要进行对应缩放）
        self._draw_liquid(ax)

        # 添加标题和单位标识
        self._add_labels(ax)

        # 保存图片
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(
            output_path,
            dpi=100,
            bbox_inches="tight",
            facecolor="white",
            edgecolor="none",
        )
        plt.close()

    def _draw_vessel_outline(self, ax):
        """绘制容器轮廓"""
        vessel_type = self.config.vessel_type
        dimensions = self.config.dimensions

        if vessel_type == VesselType.GRADUATED_CYLINDER:
            self._draw_cylinder_outline(ax, dimensions)
        elif vessel_type == VesselType.CONICAL_FLASK:
            self._draw_flask_outline(ax, dimensions)
        else:  # BEAKER
            self._draw_beaker_outline(ax, dimensions)

    def _draw_cylinder_outline(self, ax, dimensions):
        """绘制量筒轮廓 - 2D正视图"""
        height = dimensions.height
        diameter = dimensions.diameter

        # 主体矩形 - 正视图
        cylinder = patches.Rectangle(
            (-diameter / 2, 0),
            diameter,
            height,
            linewidth=2,
            edgecolor="black",
            facecolor="none",
        )
        ax.add_patch(cylinder)

        # 底部水平线
        ax.plot([-diameter / 2, diameter / 2], [0, 0], "k-", linewidth=2)

        # 嘴部 - 简化为右上角的小突出
        spout_width = diameter * 0.2
        spout_height = height * 0.08
        spout = patches.Rectangle(
            (diameter / 2, height - spout_height / 2),
            spout_width,
            spout_height,
            linewidth=2,
            edgecolor="black",
            facecolor="none",
        )
        ax.add_patch(spout)

    def _draw_flask_outline(self, ax, dimensions):
        """绘制锥形瓶轮廓 - 2D正视图"""
        height = dimensions.height
        diameter = dimensions.diameter
        neck_diameter = dimensions.neck_diameter

        # 锥形瓶正视图轮廓
        neck_height = height * 0.3
        body_height = height - neck_height

        # 轮廓点 - 2D正视图
        points = [
            (-diameter / 2, 0),  # 左下
            (-diameter / 2, body_height * 0.1),  # 左下圆弧点
            (-neck_diameter / 2, body_height),  # 左上锥形顶点
            (-neck_diameter / 2, height),  # 左颈部顶点
            (neck_diameter / 2, height),  # 右颈部顶点
            (neck_diameter / 2, body_height),  # 右上锥形顶点
            (diameter / 2, body_height * 0.1),  # 右下圆弧点
            (diameter / 2, 0),  # 右下
            (-diameter / 2, 0),  # 回到起点
        ]

        flask_outline = patches.Polygon(
            points, linewidth=2, edgecolor="black", facecolor="none"
        )
        ax.add_patch(flask_outline)

        # 底部弧线
        ax.plot([-diameter / 2, diameter / 2], [0, 0], "k-", linewidth=2)

    def _draw_beaker_outline(self, ax, dimensions):
        """绘制烧杯轮廓 - 2D正视图"""
        height = dimensions.height
        diameter = dimensions.diameter

        # 主体矩形
        beaker = patches.Rectangle(
            (-diameter / 2, 0),
            diameter,
            height,
            linewidth=2,
            edgecolor="black",
            facecolor="none",
        )
        ax.add_patch(beaker)

        # 底部水平线
        ax.plot([-diameter / 2, diameter / 2], [0, 0], "k-", linewidth=2)

        # 倾倒嘴 - 简化为右上角小缺口
        spout_points = [
            (diameter / 2, height * 0.95),
            (diameter / 2 + 0.2, height * 0.98),
            (diameter / 2 + 0.15, height),
            (diameter / 2, height),
        ]
        spout = patches.Polygon(
            spout_points, linewidth=2, edgecolor="black", facecolor="none"
        )
        ax.add_patch(spout)

    def _draw_liquid(self, ax):
        """绘制液体 - 2D正视图"""
        liquid_level = self.config.liquid_level
        max_volume = self.config.spec.max_volume
        dimensions = self.config.dimensions
        vessel_type = self.config.vessel_type

        # 计算液体高度比例
        liquid_ratio = liquid_level / max_volume

        if vessel_type == VesselType.GRADUATED_CYLINDER:
            # 量筒中的液体 - 矩形
            liquid_height = liquid_ratio * dimensions.max_scale_height
            liquid_rect = patches.Rectangle(
                (-dimensions.diameter / 2 + 0.02, 0),
                dimensions.diameter - 0.04,
                liquid_height,
                facecolor=self.config.liquid_color,
                alpha=0.6,
                edgecolor="none",
            )
            ax.add_patch(liquid_rect)

            # 液面线 - 清晰的水平线
            ax.plot(
                [-dimensions.diameter / 2 + 0.02, dimensions.diameter / 2 - 0.02],
                [liquid_height, liquid_height],
                color=self.config.liquid_color,
                linewidth=2.5,
                alpha=0.9,
            )

        elif vessel_type == VesselType.CONICAL_FLASK:
            # 锥形瓶中的液体
            body_height = dimensions.max_scale_height
            liquid_height = liquid_ratio * body_height  # 液体主要在锥体部分

            # 计算液面处的宽度
            if liquid_height <= body_height:
                width_ratio = 1.0 - (liquid_height / body_height) * (
                    1.0 - dimensions.neck_diameter / dimensions.diameter
                )
                liquid_surface_width = dimensions.diameter * width_ratio
            else:
                liquid_surface_width = dimensions.neck_diameter

            # 绘制液体梯形
            liquid_points = [
                (-dimensions.diameter / 2 + 0.02, 0),  # 左下
                (dimensions.diameter / 2 - 0.02, 0),  # 右下
                (liquid_surface_width / 2 - 0.01, liquid_height),  # 右上
                (-liquid_surface_width / 2 + 0.01, liquid_height),  # 左上
            ]

            liquid_poly = patches.Polygon(
                liquid_points,
                facecolor=self.config.liquid_color,
                alpha=0.6,
                edgecolor="none",
            )
            ax.add_patch(liquid_poly)

            # 液面线
            ax.plot(
                [-liquid_surface_width / 2 + 0.01, liquid_surface_width / 2 - 0.01],
                [liquid_height, liquid_height],
                color=self.config.liquid_color,
                linewidth=2.5,
                alpha=0.9,
            )

        else:  # BEAKER
            # 烧杯中的液体 - 矩形
            liquid_height = liquid_ratio * dimensions.max_scale_height
            liquid_rect = patches.Rectangle(
                (-dimensions.diameter / 2 + 0.02, 0),
                dimensions.diameter - 0.04,
                liquid_height,
                facecolor=self.config.liquid_color,
                alpha=0.6,
                edgecolor="none",
            )
            ax.add_patch(liquid_rect)

            # 液面线
            ax.plot(
                [-dimensions.diameter / 2 + 0.02, dimensions.diameter / 2 - 0.02],
                [liquid_height, liquid_height],
                color=self.config.liquid_color,
                linewidth=2.5,
                alpha=0.9,
            )

    def _draw_scales(self, ax):
        """绘制刻度"""
        marks = ConfigGenerator.get_scale_marks(self.config)
        dimensions = self.config.dimensions
        max_volume = self.config.spec.max_volume
        vessel_type = self.config.vessel_type

        # 计算刻度范围，确保不超出容器且留出标签空间
        if vessel_type == VesselType.CONICAL_FLASK:
            # 锥形瓶：刻度只在锥体部分，不在颈部
            dimensions.max_scale_height = dimensions.height * 0.6
        else:
            # 量筒和烧杯：刻度在瓶口下方留出空间
            dimensions.max_scale_height = dimensions.height * 0.85  # 留出15%空间给标签

        # 找到最大刻度的位置，用于放置标签
        max_scale_y = 0

        for volume, mark_type, label in marks:
            # 计算刻度高度位置
            if vessel_type == VesselType.CONICAL_FLASK:
                # 锥形瓶的刻度映射到锥体部分
                y_pos = (volume / max_volume) * dimensions.max_scale_height
            else:
                # 其他容器：映射到有效区域
                y_pos = (volume / max_volume) * dimensions.max_scale_height

            # 记录最大刻度位置
            max_scale_y = max(max_scale_y, y_pos)

            # 计算当前高度的容器内部宽度（考虑壁厚）
            current_width = self._get_vessel_inner_width_at_height(y_pos)

            # 根据刻度类型设置长度和粗细 - 大幅缩小刻度长度
            if mark_type == "min":
                line_length = current_width * 0.06  # 从0.1减小到0.06
                line_width = 0.8
            elif mark_type == "major":
                line_length = current_width * 0.10  # 从0.15减小到0.10
                line_width = 1.0
            else:  # 'number'
                line_length = current_width * 0.10  # 从0.15减小到0.10
                line_width = 1.0

            # 确保刻度不超出容器内壁
            max_line_length = current_width * 0.12  # 最大不超过内径的12%
            line_length = min(line_length, max_line_length)

            # 绘制左侧刻度线（在容器内部）
            left_start = -current_width / 2 + 0.05  # 从内壁稍微向内
            left_end = left_start + line_length
            ax.plot(
                [left_start, left_end],
                [y_pos, y_pos],
                color=self.config.scale_color,
                linewidth=line_width,
            )

            # 绘制右侧刻度线（在容器内部）
            right_end = current_width / 2 - 0.05  # 到内壁稍微向内
            right_start = right_end - line_length
            ax.plot(
                [right_start, right_end],
                [y_pos, y_pos],
                color=self.config.scale_color,
                linewidth=line_width,
            )

            # 添加标数刻度的标签（标在容器中间）
            if mark_type == "number" and label:
                ax.text(
                    0,
                    y_pos,
                    label,
                    ha="center",
                    va="center",
                    fontsize=9,
                    color=self.config.scale_color,
                    bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.9),
                )

        # 存储最大刻度位置供标签使用
        self._max_scale_height = max_scale_y

    def _get_vessel_inner_width_at_height(self, height):
        """获取容器在指定高度处的内部宽度（考虑壁厚）"""
        dimensions = self.config.dimensions
        vessel_type = self.config.vessel_type

        # 壁厚
        wall_thickness = 0.04

        if (
            vessel_type == VesselType.GRADUATED_CYLINDER
            or vessel_type == VesselType.BEAKER
        ):
            # 量筒和烧杯是直筒，宽度恒定
            return dimensions.diameter - wall_thickness
        elif vessel_type == VesselType.CONICAL_FLASK:
            # 锥形瓶需要根据高度计算宽度
            body_height = dimensions.height * 0.6
            if height <= body_height:
                # 在锥体部分，线性变化
                width_ratio = 1.0 - (height / body_height) * (
                    1.0 - dimensions.neck_diameter / dimensions.diameter
                )
                return (dimensions.diameter * width_ratio) - wall_thickness
            else:
                # 在颈部，宽度为颈部直径
                return dimensions.neck_diameter - wall_thickness

        return dimensions.diameter - wall_thickness

    def _get_vessel_width_at_height(self, height):
        """获取容器在指定高度处的宽度"""
        dimensions = self.config.dimensions
        vessel_type = self.config.vessel_type

        if (
            vessel_type == VesselType.GRADUATED_CYLINDER
            or vessel_type == VesselType.BEAKER
        ):
            # 量筒和烧杯是直筒，宽度恒定
            return dimensions.diameter
        elif vessel_type == VesselType.CONICAL_FLASK:
            # 锥形瓶需要根据高度计算宽度
            body_height = dimensions.height * 0.6
            if height <= body_height:
                # 在锥体部分，线性变化
                width_ratio = 1.0 - (height / body_height) * (
                    1.0 - dimensions.neck_diameter / dimensions.diameter
                )
                return dimensions.diameter * width_ratio
            else:
                # 在颈部，宽度为颈部直径
                return dimensions.neck_diameter

        return dimensions.diameter

    def _add_labels(self, ax):
        """添加单位标识和最大容量标识"""
        dimensions = self.config.dimensions
        vessel_type = self.config.vessel_type

        # 获取最大刻度高度
        max_scale_height = getattr(self, "_max_scale_height", 0)

        # 计算标签位置：在最大刻度上方，瓶口下方
        if vessel_type == VesselType.CONICAL_FLASK:
            # 锥形瓶：标签在锥体顶部和颈部之间
            label_height = (
                max_scale_height + (dimensions.height * 0.7 - max_scale_height) / 2
            )
        else:
            # 量筒和烧杯：标签在最大刻度上方
            available_space = dimensions.height * 0.92 - max_scale_height
            label_height = max_scale_height + available_space * 0.6  # 在60%的位置

        # 最大容量标识
        max_vol = self.config.spec.max_volume
        unit_text = f"{int(max_vol) if max_vol == int(max_vol) else max_vol} ml"

        ax.text(
            0,
            label_height,
            unit_text,
            ha="center",
            va="center",
            fontsize=10,
            fontweight="bold",
            color=self.config.scale_color,
            bbox=dict(
                boxstyle="round,pad=0.25",
                facecolor="white",
                alpha=0.95,
                edgecolor=self.config.scale_color,
                linewidth=0.5,
            ),
        )


def render_vessel(config: Config, output_path: str):
    """渲染容器的便捷函数"""
    renderer = VesselRenderer(config)
    renderer.render(output_path)
