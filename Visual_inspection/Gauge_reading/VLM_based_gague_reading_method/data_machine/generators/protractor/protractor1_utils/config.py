import random
from dataclasses import dataclass
from typing import Tuple, Dict, Any
from enum import Enum


class ProtractorType(Enum):
    HALF_CIRCLE = "180"  # 180度量角器
    FULL_CIRCLE = "360"  # 360度量角器


class ScaleType(Enum):
    CLOCKWISE = "clockwise"  # 顺时针从0°-180°
    COUNTERCLOCKWISE = "counterclockwise"  # 逆时针从0-180°
    DUAL = "dual"  # 双刻度


class StyleType(Enum):
    RADIAL_LINES = "radial_lines"  # 每个标数刻度有从圆心出来的线
    HOLLOW_CENTER = "hollow_center"  # 中间空缺的小半圆周
    CENTER_MARKS = "center_marks"  # 圆心处每隔30度有小提示线


@dataclass
class ScaleConfig:
    """刻度配置"""

    min_scale: float = 1.0  # 最小刻度
    major_scale: float = 5.0  # 大刻度
    number_scale: float = 10.0  # 标数刻度

    # 线条粗细
    min_scale_width: float = 0.8
    major_scale_width: float = 1.5
    number_scale_width: float = 1.5

    # 线条长度
    min_scale_length: float = 8
    major_scale_length: float = 15
    number_scale_length: float = 22


@dataclass
class ColorConfig:
    """颜色配置"""

    protractor_color: Tuple[int, int, int] = (0, 0, 0)  # 量角器主体颜色
    scale_color: Tuple[int, int, int] = (0, 0, 0)  # 刻度颜色
    number_color: Tuple[int, int, int] = (0, 0, 0)  # 数字颜色
    line_color: Tuple[int, int, int] = (255, 0, 0)  # 测量线颜色
    arrow_color: Tuple[int, int, int] = (255, 0, 0)  # 箭头颜色
    background_color: Tuple[int, int, int] = (255, 255, 255)  # 背景颜色


@dataclass
class ProtractorConfig:
    """量角器完整配置"""

    # 基本属性
    protractor_type: ProtractorType
    scale_type: ScaleType
    style_type: StyleType

    # 几何属性
    center: Tuple[int, int]  # 圆心位置
    radius: int  # 半径
    rotation_angle: float  # 旋转角度(-180到180度)

    # 刻度配置
    scale_config: ScaleConfig

    # 颜色配置
    color_config: ColorConfig

    # 测量角度
    angle1: float  # 第一条线的角度
    angle2: float  # 第二条线的角度

    # 图像尺寸
    image_width: int = 900  # 图像宽度
    image_height: int = 400  # 图像高度


class ConfigGenerator:
    """配置生成器"""

    def __init__(self):
        self.protractor_type_weights = [8, 2]  # 半圆:圆周 = 8:2
        self.scale_type_weights = [1, 1, 1]  # 三种刻度类型等概率
        self.style_type_weights = [1, 1, 1]  # 三种样式等概率

    def generate_random_config(
        self,
        image_width: int = 900,  # 增大默认尺寸
        image_height: int = 600,
    ) -> ProtractorConfig:
        """生成随机配置"""

        # 随机选择量角器类型
        protractor_type = random.choices(
            list(ProtractorType), weights=self.protractor_type_weights
        )[0]

        # 随机选择刻度类型
        scale_type = random.choices(list(ScaleType), weights=self.scale_type_weights)[0]

        # 随机选择样式类型
        style_type = random.choices(list(StyleType), weights=self.style_type_weights)[0]

        # 设置几何属性 - 进一步放大量角器尺寸
        center = (image_width // 2, image_height // 2)
        radius = random.randint(
            int(min(image_width, image_height) * 0.35),
            int(min(image_width, image_height) * 0.45),
        )  # 进一步增大半径范围
        rotation_angle = random.uniform(-180, 180)

        # 生成刻度配置
        scale_config = self._generate_scale_config()

        # 生成颜色配置
        color_config = self._generate_color_config()

        # 生成测量角度 - 考虑旋转角度
        angle1, angle2 = self._generate_measurement_angles(
            protractor_type, rotation_angle
        )

        return ProtractorConfig(
            protractor_type=protractor_type,
            scale_type=scale_type,
            style_type=style_type,
            center=center,
            radius=radius,
            rotation_angle=rotation_angle,
            scale_config=scale_config,
            color_config=color_config,
            angle1=angle1,
            angle2=angle2,
            image_width=image_width,
            image_height=image_height,
        )

    def _generate_scale_config(self) -> ScaleConfig:
        """生成刻度配置"""
        # 基本刻度保持固定，确保标准化
        return ScaleConfig(
            min_scale=1.0,
            major_scale=5.0,
            number_scale=10.0,
            min_scale_width=1.0,  # 稍微加粗
            major_scale_width=1.8,
            number_scale_width=1.8,
            min_scale_length=random.uniform(8, 12),  # 增加长度
            major_scale_length=random.uniform(15, 22),
            number_scale_length=random.uniform(25, 32),
        )

    def _generate_color_config(self) -> ColorConfig:
        """生成颜色配置"""
        # 大部分情况使用标准黑色，偶尔使用其他颜色
        use_standard = random.random() < 0.8

        if use_standard:
            return ColorConfig()  # 使用默认颜色
        else:
            # 生成随机但合理的颜色
            protractor_color = self._random_dark_color()
            scale_color = protractor_color
            number_color = protractor_color
            line_color = self._random_bright_color()
            arrow_color = line_color

            return ColorConfig(
                protractor_color=protractor_color,
                scale_color=scale_color,
                number_color=number_color,
                line_color=line_color,
                arrow_color=arrow_color,
            )

    def _random_dark_color(self) -> Tuple[int, int, int]:
        """生成深色"""
        return tuple(random.randint(0, 100) for _ in range(3))

    def _random_bright_color(self) -> Tuple[int, int, int]:
        """生成亮色"""
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 165, 0), (128, 0, 128)]
        return random.choice(colors)

    def _generate_measurement_angles(
        self, protractor_type: ProtractorType, rotation_angle: float
    ) -> Tuple[float, float]:
        """生成测量角度 - 已经考虑旋转"""
        if protractor_type == ProtractorType.HALF_CIRCLE:
            # 180度量角器，角度范围0-180
            max_angle = 180
        else:
            # 360度量角器，角度范围0-360
            max_angle = 360

        # 生成两个角度，确保有一定的角度差
        min_diff = 10  # 最小角度差
        max_diff = min(140, max_angle - min_diff)  # 最大角度差

        angle1 = random.uniform(0, max_angle - max_diff)
        angle_diff = random.uniform(min_diff, min(max_diff, max_angle - angle1))
        angle2 = angle1 + angle_diff

        # 注意：这里返回的是相对于量角器的角度，后续渲染时会自动应用旋转
        return angle1, angle2

    def calculate_reading_interval(
        self, actual_angle: float, min_scale: float
    ) -> Tuple[float, float]:
        """计算读数区间

        Args:
            actual_angle: 实际角度
            min_scale: 最小刻度

        Returns:
            (min_reading, max_reading): 读数区间
        """
        # 找到实际角度两侧最近的刻度线
        # lower_scale = (actual_angle // min_scale) * min_scale
        # upper_scale = lower_scale + min_scale

        # 读数误差范围为半个最小刻度
        half_scale = min_scale / 2

        min_reading = max(0, actual_angle - half_scale)
        max_reading = actual_angle + half_scale

        return min_reading, max_reading

    def get_angle_measurement_info(self, config: ProtractorConfig) -> Dict[str, Any]:
        """获取角度测量信息，用于生成标注文件"""
        angle_diff = abs(config.angle2 - config.angle1)

        # 计算读数区间
        min_reading, max_reading = self.calculate_reading_interval(
            angle_diff, config.scale_config.min_scale
        )

        return {
            "actual_angle": angle_diff,
            "reading_interval": [min_reading, max_reading],
            "angle1": config.angle1,
            "angle2": config.angle2,
            "protractor_type": config.protractor_type.value,
            "scale_type": config.scale_type.value,
            "style_type": config.style_type.value,
        }
