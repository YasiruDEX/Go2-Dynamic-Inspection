import random
from dataclasses import dataclass
from typing import Tuple, List
from enum import Enum


class VesselType(Enum):
    GRADUATED_CYLINDER = "Graduated Cylinder"
    CONICAL_FLASK = "Conical Flask"
    BEAKER = "Beaker"


@dataclass
class VesselSpec:
    """容器规格定义"""

    max_volume: float  # 最大值 (ml)
    min_scale: float  # 最小刻度 (ml)
    major_scale: float  # 大刻度 (ml)
    number_scale: float  # 标数刻度 (ml)


@dataclass
class VesselDimensions:
    """容器尺寸"""

    height: float
    diameter: float
    neck_diameter: float = None  # 锥形瓶需要颈部直径
    max_scale_height: float = None  # 最大刻度高度=height*不同仪器比例


@dataclass
class Config:
    """量筒配置类"""

    vessel_type: VesselType
    spec: VesselSpec
    dimensions: VesselDimensions
    scale_color: str
    liquid_color: str
    liquid_level: float  # 液体高度对应的体积值
    start_scale_type: str  # 起始刻度类型：'major' 或 'number'


class ConfigGenerator:
    """配置生成器"""

    # 容器规格定义
    VESSEL_SPECS = {
        VesselType.GRADUATED_CYLINDER: [
            VesselSpec(10, 0.1, 0.5, 1),
            VesselSpec(25, 0.5, 2.5, 5),
            VesselSpec(50, 1, 5, 10),
            VesselSpec(100, 1, 5, 10),
        ],
        VesselType.CONICAL_FLASK: [
            VesselSpec(50, 10, 10, 10),
            VesselSpec(250, 50, 50, 50),
        ],
        VesselType.BEAKER: [
            VesselSpec(100, 10, 20, 20),
            VesselSpec(150, 30, 30, 30),
            VesselSpec(1800, 200, 400, 400),
        ],
    }

    # 刻度颜色选项
    SCALE_COLORS = ["black", "darkblue", "#2F2F2F"]  # 黑色，深蓝色，深白色

    # 液体颜色选项
    LIQUID_COLORS = [
        "#FF6B6B",
        "#4ECDC4",
        "#45B7D1",
        "#96CEB4",
        "#FFEAA7",
        "#DDA0DD",
        "#98D8C8",
        "#F7DC6F",
        "#BB8FCE",
        "#85C1E9",
    ]

    @staticmethod
    def _calculate_dimensions(
        vessel_type: VesselType, max_volume: float
    ) -> VesselDimensions:
        """根据容器类型和最大容量计算尺寸"""

        if vessel_type == VesselType.GRADUATED_CYLINDER:
            # 量筒：高细长，直径相对较小
            if max_volume <= 10:
                return VesselDimensions(height=9, diameter=1.5)
            elif max_volume <= 25:
                return VesselDimensions(height=10, diameter=2.0)
            elif max_volume <= 50:
                return VesselDimensions(height=12, diameter=2.5)
            else:  # 100ml
                return VesselDimensions(height=13, diameter=3.0)

        elif vessel_type == VesselType.CONICAL_FLASK:
            # 锥形瓶：梨形，底部直径大，颈部直径小
            if max_volume <= 50:
                return VesselDimensions(height=10, diameter=6, neck_diameter=2)
            else:  # 250ml
                return VesselDimensions(height=13, diameter=8, neck_diameter=2.5)

        else:  # BEAKER
            # 烧杯：圆柱形，直径相对较大
            if max_volume <= 100:
                return VesselDimensions(height=8, diameter=5)
            elif max_volume <= 150:
                return VesselDimensions(height=9, diameter=5.5)
            else:  # 1800ml
                return VesselDimensions(height=12, diameter=12)

    @staticmethod
    def _generate_liquid_level(spec: VesselSpec) -> float:
        """生成随机液体液面高度"""
        # 液体液面可以在最小刻度之间，精度比最小刻度小一位
        precision = spec.min_scale / 10
        min_level = spec.major_scale
        max_level = spec.max_volume * 0.9  # 不要超过90%容量

        # 生成随机液面高度
        liquid_level = random.uniform(min_level, max_level)
        # 四舍五入到指定精度
        liquid_level = round(liquid_level / precision) * precision

        return liquid_level

    @classmethod
    def generate_random_config(cls) -> Config:
        """生成随机配置"""
        # 随机选择容器类型
        vessel_type = random.choice(list(VesselType))

        # 随机选择该类型的规格
        spec = random.choice(cls.VESSEL_SPECS[vessel_type])

        # 计算尺寸
        dimensions = cls._calculate_dimensions(vessel_type, spec.max_volume)

        # 随机选择颜色
        scale_color = random.choice(cls.SCALE_COLORS)
        liquid_color = random.choice(cls.LIQUID_COLORS)

        # 生成液体液面
        liquid_level = cls._generate_liquid_level(spec)

        # 随机选择起始刻度类型
        start_scale_type = random.choice(["major", "number"])

        return Config(
            vessel_type=vessel_type,
            spec=spec,
            dimensions=dimensions,
            scale_color=scale_color,
            liquid_color=liquid_color,
            liquid_level=liquid_level,
            start_scale_type=start_scale_type,
        )

    @classmethod
    def get_scale_marks(cls, config: Config) -> List[Tuple[float, str, str]]:
        """获取刻度标记列表
        返回: [(刻度值, 刻度类型, 标签文本), ...]
        刻度类型: 'min'(最小刻度), 'major'(大刻度), 'number'(标数刻度)
        """
        spec = config.spec
        marks = []

        # 确定起始刻度
        if config.start_scale_type == "major":
            start_value = spec.major_scale
        else:  # 'number'
            start_value = spec.number_scale

        # 生成所有刻度
        current = start_value
        while current <= spec.max_volume:
            # 判断刻度类型
            if current % spec.number_scale == 0:
                mark_type = "number"
                label = str(int(current)) if current == int(current) else str(current)
            elif current % spec.major_scale == 0:
                mark_type = "major"
                label = ""
            else:
                mark_type = "min"
                label = ""

            marks.append((current, mark_type, label))
            current += spec.min_scale
            current = round(current, 3)  # 避免浮点数精度问题

        return marks
