import random
from dataclasses import dataclass
from typing import Tuple
import colorsys


@dataclass
class ScaleConfig:
    """圆盘称配置类"""

    # 取值范围配置 [最小值,最大值,最小单位,大刻度,标数刻度]
    min_value: float
    max_value: float
    min_unit: float
    major_tick: float
    labeled_tick: float

    # 刻度形式 (0: 0刻度和最大刻度重叠, 1: 0刻度不重叠)
    scale_type: int

    # 指针配置
    pointer_color: Tuple[float, float, float]  # RGB
    pointer_style: str  # 'arrow', 'line', 'triangle'
    pointer_width: float

    # 外观配置
    dial_color: Tuple[float, float, float]  # 表盘颜色
    tick_color: Tuple[float, float, float]  # 刻度颜色
    text_color: Tuple[float, float, float]  # 文字颜色
    background_color: Tuple[float, float, float]  # 背景颜色

    # 尺寸配置
    dial_radius: float
    major_tick_length: float
    minor_tick_length: float
    tick_width: float
    major_tick_width: float


class ConfigGenerator:
    """配置生成器"""

    # 预设的取值范围
    SCALE_RANGES = [
        [0, 1, 0.005, 0.1, 0.1],  # [0,1kg,5g,100g,100g]
        [0, 2, 0.01, 0.1, 0.2],  # [0,2kg,10g,100g,200g]
        [0, 5, 0.02, 0.1, 0.5],  # [0,5kg,20g,100g,500g]
        [0, 10, 0.05, 0.5, 1],  # [0,10kg,50g,500g,1kg]
        [0, 20, 0.1, 1, 1],  # [0,20kg,100g,1kg,1kg]
        [0, 30, 0.1, 1, 2],  # [0,30kg,100g,1kg,2kg]
    ]

    # 指针样式选项
    POINTER_STYLES = ["arrow", "line", "triangle"]

    @staticmethod
    def generate_random_color(brightness_range=(0.3, 0.9), saturation_range=(0.5, 1.0)):
        """生成随机颜色"""
        hue = random.random()
        saturation = random.uniform(*saturation_range)
        brightness = random.uniform(*brightness_range)
        rgb = colorsys.hsv_to_rgb(hue, saturation, brightness)
        return rgb

    @staticmethod
    def generate_contrasting_color(base_color, min_contrast=0.5):
        """生成与基础颜色对比度足够的颜色"""
        base_brightness = sum(base_color) / 3
        if base_brightness > 0.5:
            # 基础颜色较亮，生成较暗的颜色
            target_brightness = random.uniform(0.1, base_brightness - min_contrast)
        else:
            # 基础颜色较暗，生成较亮的颜色
            target_brightness = random.uniform(base_brightness + min_contrast, 0.9)

        hue = random.random()
        saturation = random.uniform(0.3, 1.0)
        rgb = colorsys.hsv_to_rgb(hue, saturation, target_brightness)
        return rgb

    @classmethod
    def generate_random_config(cls) -> ScaleConfig:
        """生成随机配置"""
        # 随机选择取值范围
        scale_range = random.choice(cls.SCALE_RANGES)
        min_val, max_val, min_unit, major_tick, labeled_tick = scale_range

        # 随机选择刻度形式
        scale_type = random.choice([0, 1])

        # 生成颜色方案
        background_color = cls.generate_random_color(
            brightness_range=(0.9, 1.0), saturation_range=(0.0, 0.3)
        )
        dial_color = cls.generate_random_color(
            brightness_range=(0.7, 0.95), saturation_range=(0.1, 0.5)
        )
        tick_color = cls.generate_contrasting_color(dial_color, min_contrast=0.3)
        text_color = cls.generate_contrasting_color(dial_color, min_contrast=0.4)
        pointer_color = cls.generate_random_color(
            brightness_range=(0.2, 0.8), saturation_range=(0.6, 1.0)
        )

        # 随机选择指针样式
        pointer_style = random.choice(cls.POINTER_STYLES)
        pointer_width = random.uniform(2.0, 6.0)

        # 随机生成尺寸参数
        dial_radius = random.uniform(180, 220)
        major_tick_length = random.uniform(15, 25)
        minor_tick_length = random.uniform(8, 15)
        tick_width = random.uniform(1.0, 2.5)
        major_tick_width = random.uniform(2.0, 4.0)

        return ScaleConfig(
            min_value=min_val,
            max_value=max_val,
            min_unit=min_unit,
            major_tick=major_tick,
            labeled_tick=labeled_tick,
            scale_type=scale_type,
            pointer_color=pointer_color,
            pointer_style=pointer_style,
            pointer_width=pointer_width,
            dial_color=dial_color,
            tick_color=tick_color,
            text_color=text_color,
            background_color=background_color,
            dial_radius=dial_radius,
            major_tick_length=major_tick_length,
            minor_tick_length=minor_tick_length,
            tick_width=tick_width,
            major_tick_width=major_tick_width,
        )

    @classmethod
    def get_scale_info(cls, config: ScaleConfig) -> dict:
        """获取刻度信息"""
        return {
            "range": f"{config.min_value}-{config.max_value}kg",
            "min_unit": f"{config.min_unit * 1000}g",
            "major_tick": f"{config.major_tick * 1000}g",
            "labeled_tick": f"{config.labeled_tick * 1000}g",
            "scale_type": "overlap" if config.scale_type == 0 else "separate",
        }


if __name__ == "__main__":
    # 测试配置生成
    config = ConfigGenerator.generate_random_config()
    print("Generated config:")
    print(f"Range: {config.min_value}-{config.max_value}kg")
    print(f"Scale type: {config.scale_type}")
    print(f"Pointer: {config.pointer_style}, color: {config.pointer_color}")
    print(ConfigGenerator.get_scale_info(config))
