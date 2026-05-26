import random
from dataclasses import dataclass
from typing import List


@dataclass
class ScaleConfig:
    """刻度配置"""

    min_val: float
    max_val: float
    min_unit: float
    major_tick: float
    label_tick: float
    color: str = "black"


@dataclass
class PressureGaugeConfig:
    """压力计配置"""

    unit_type: str
    scales: List[ScaleConfig]
    is_dual_scale: bool
    scale_form: int  # 1或2，对应两种双刻度形式
    style: int  # 1,2,3 对应三种样式
    pointer_color: str
    pointer_style: int
    background_color: str
    actual_value: float  # 指针指向的实际值
    actual_values: List[float]  # 双刻度时的两个实际值
    unit_display_names: List[List[str]]  # 单位的显示名称


class ConfigGenerator:
    """配置生成器"""

    def __init__(self):
        # 定义所有可能的单位配置
        self.unit_configs = {
            "kgf/cm^2": [
                ScaleConfig(0, 7, 0.1, 0.5, 1, "black"),
                ScaleConfig(0, 16, 0.2, 2, 4, "red"),
            ],
            "psi": [
                ScaleConfig(0, 100, 2.5, 5, 10, "black"),
                ScaleConfig(0, 5000, 100, 200, 1000, "black"),
            ],
            "MPa": [ScaleConfig(0, 1.6, 0.05, 0.1, 0.4, "black")],
            "psi+bar": [
                ScaleConfig(0, 60, 1, 5, 10, "black"),
                ScaleConfig(0, 4, 0.1, 0.5, 1, "red"),
            ],
            "bar+psi": [
                ScaleConfig(0, 80, 2, 10, 20, "black"),
                ScaleConfig(0, 1160, 20, 100, 200, "red"),
            ],
            "psi+kPa": [
                ScaleConfig(0, 200, 2, 10, 20, "black"),
                ScaleConfig(0, 1400, 20, 100, 200, "red"),
            ],
            "kgf/cm2+psi": [
                ScaleConfig(0, 35, 0.2, 1, 5, "black"),
                ScaleConfig(0, 500, 5, 25, 50, "red"),
            ],
            "psi+kg/cm2": [
                ScaleConfig(0, 55, 1, 5, 5, "black"),
                ScaleConfig(0, 4, 0.1, 0.5, 0.5, "red"),
            ],
            "mmHg+inHg": [
                ScaleConfig(-800, 0, 10, 50, 100, "black"),
                ScaleConfig(-30, 0, 0.5, 1, 5, "red"),
            ],
        }

        # 指针颜色和样式
        self.pointer_colors = ["red", "black", "blue", "orange", "green"]
        self.pointer_styles = [1, 2, 3, 4]  # 不同的指针样式

        # 背景颜色
        self.background_colors = ["white", "lightgray", "ivory"]

    def generate_random_config(self) -> PressureGaugeConfig:
        """生成随机配置"""
        # 选择单位类型，mmHg+inHg出现概率为5%
        if random.random() < 0.05:
            unit_type = "mmHg+inHg"
        else:
            unit_types = list(self.unit_configs.keys())
            unit_types.remove("mmHg+inHg")
            unit_type = random.choice(unit_types)

        scales = self.unit_configs[unit_type].copy()
        is_dual_scale = "+" in unit_type

        # 设置刻度颜色
        if is_dual_scale:
            scales[0].color = "black"
            scales[1].color = "red"

        # 生成实际值
        if is_dual_scale:
            # 双刻度需要生成两个相关联的值
            if scales[0].max_val == scales[0].min_val:
                primary_value = scales[0].min_val
                secondary_value = scales[1].min_val
            else:
                ratio = (scales[1].max_val - scales[1].min_val) / (
                    scales[0].max_val - scales[0].min_val
                )
                primary_value = self._generate_realistic_value(scales[0])
                secondary_value = (primary_value - scales[0].min_val) * ratio + scales[
                    1
                ].min_val

            actual_values = [primary_value, secondary_value]
            actual_value = primary_value
        else:
            actual_value = self._generate_realistic_value(scales[0])
            actual_values = [actual_value]

        # 获取单位显示名称
        unit_display_names = self.get_unit_display_names(unit_type)

        config = PressureGaugeConfig(
            unit_type=unit_type,
            scales=scales,
            is_dual_scale=is_dual_scale,
            scale_form=random.choice([1, 2]) if is_dual_scale else 1,
            style=random.choice([1, 2, 3]),
            pointer_color=random.choice(self.pointer_colors),
            pointer_style=random.choice(self.pointer_styles),
            background_color=random.choice(self.background_colors),
            actual_value=actual_value,
            actual_values=actual_values,
            unit_display_names=unit_display_names,
        )

        return config

    def _generate_realistic_value(self, scale: ScaleConfig) -> float:
        """生成符合真实情况的数值"""
        # 可以比最小刻度小一位，指针可以指向两个最小刻度之间
        min_precision = scale.min_unit / 10

        # 生成在范围内的随机值
        value = random.uniform(
            scale.min_val + min_precision, scale.max_val - min_precision
        )

        # 四舍五入到合理的精度
        num_decimal_places = 0
        if "." in str(scale.min_unit):
            num_decimal_places = len(str(scale.min_unit).split(".")[1])
        return round(value, num_decimal_places + 1)

    def get_unit_display_names(self, unit_type: str) -> List[List[str]]:
        """获取单位的显示名称和全名"""
        unit_mappings = {
            "kgf/cm^2": [["kgf/cm²", "kilogram-force per square centimeter"]],
            "psi": [["psi", "pound per square inch"]],
            "MPa": [["MPa", "megapascal"]],
            "psi+bar": [["psi", "pound per square inch"], ["bar"]],
            "bar+psi": [["bar"], ["psi", "pound per square inch"]],
            "psi+kPa": [["psi", "pound per square inch"], ["kPa", "kilopascal"]],
            "kgf/cm2+psi": [
                ["kgf/cm²", "kilogram-force per square centimeter"],
                ["psi", "pound per square inch"],
            ],
            "psi+kg/cm2": [
                ["psi", "pound per square inch"],
                ["kg/cm²", "kilogram per square centimeter"],
            ],
            "mmHg+inHg": [
                ["mmHg", "millimeter of mercury"],
                ["inHg", "inch of mercury"],
            ],
        }

        return unit_mappings.get(unit_type, [[unit_type, unit_type]])
