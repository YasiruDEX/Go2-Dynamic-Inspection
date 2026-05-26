from typing import List
from .pressure_gauge2_utils.config import ConfigGenerator
from .pressure_gauge2_utils.render import PressureGaugeRenderer
from registry import registry
from artifacts import Artifact


def calculate_interval(value: float, min_unit: float) -> List[float]:
    """Calculate the value interval, considering the pointer may be between two ticks."""
    # Find the two nearest tick values
    lower_tick = (value // min_unit) * min_unit
    upper_tick = lower_tick + min_unit

    # If the pointer is exactly on a tick, the interval is that value
    tolerance = min_unit / 20  # Allowed error range

    if abs(value - lower_tick) < tolerance:
        return [lower_tick, lower_tick]
    elif abs(value - upper_tick) < tolerance:
        return [upper_tick, upper_tick]
    else:
        # If the pointer is between two ticks, return the interval
        return [lower_tick, upper_tick]


@registry.register(
    name="lff_synthetic_pressure_gauge", tags={"pressure_gauge"}, weight=1.0
)
def generate(img_path: str) -> Artifact:
    config_generator = ConfigGenerator()
    config = config_generator.generate_random_config()
    PressureGaugeRenderer(config).render(img_path)

    evaluator = ""
    evaluator_kwargs = {}
    unit_names = config_generator.get_unit_display_names(config.unit_type)

    if config.is_dual_scale:  # Dual scale
        intervals = []
        units = []
        for i, (value, scale) in enumerate(zip(config.actual_values, config.scales)):
            interval = calculate_interval(value, scale.min_unit)
            intervals.append(interval)
            units.append(unit_names[i])

        evaluator_kwargs = {"intervals": intervals, "units": units}
        evaluator = "multi_interval_matching"
    else:  # Single scale
        scale = config.scales[0]
        interval = calculate_interval(config.actual_value, scale.min_unit)
        evaluator_kwargs = {"interval": interval, "units": unit_names[0]}
        evaluator = "interval_matching"
    return Artifact(
        data=img_path,
        image_type="pressure_gauge",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
        evaluator=evaluator,
    )
