import random
from typing import List
from .weighing_scale1_utils.config import ConfigGenerator, ScaleConfig
from .weighing_scale1_utils.render import WeighingScaleRenderer
from registry import registry
from artifacts import Artifact


def generate_random_value(config: ScaleConfig) -> float:
    """Generate a random value, possibly between two minimum ticks."""
    max_val = config.max_value
    min_unit = config.min_unit
    precision = min_unit / 10
    # Randomly choose whether the value is exactly on a tick
    if random.random() < 0.3:  # 30% chance exactly on a tick
        num_ticks = int(max_val / min_unit)
        tick_index = random.randint(0, num_ticks)
        value = tick_index * min_unit
    else:  # 70% chance between ticks
        value = random.uniform(0, max_val)
        value = round(value / precision) * precision
    return min(value, max_val)


def calculate_interval(value: float, config: ScaleConfig) -> List[float]:
    """Calculate the interval value."""
    min_unit = config.min_unit
    # Find adjacent ticks, extend by one min_unit, keep 2 decimals
    lower_tick = round((int(value / min_unit) - 1) * min_unit, 2)
    upper_tick = round((int(value / min_unit) + 2) * min_unit, 2)
    return [lower_tick, upper_tick]


@registry.register(
    name="lff_synthetic_weighing_scale", tags={"weighing_scale"}, weight=1.0
)
def generate(img_path: str) -> Artifact:
    config = ConfigGenerator.generate_random_config()
    value = generate_random_value(config)

    renderer = WeighingScaleRenderer(config)
    renderer.render(value, img_path)

    interval = calculate_interval(value, config)
    evaluator_kwargs = {"interval": interval, "units": ["kg", "kilogram"]}

    return Artifact(
        data=img_path,
        image_type="weighing_scale",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )
