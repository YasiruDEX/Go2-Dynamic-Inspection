from typing import List
from .measuring_cylinder4_utils.config import ConfigGenerator, Config
from .measuring_cylinder4_utils.render import render_vessel
from registry import registry
from artifacts import Artifact


def calculate_reading_interval(config: Config) -> List[float]:
    """Calculate the interval range of the liquid level reading."""
    liquid_level = config.liquid_level
    min_scale = config.spec.min_scale

    # Find the two scale values near the liquid level
    lower_mark = (liquid_level // min_scale) * min_scale
    upper_mark = lower_mark + min_scale

    # If the liquid level is exactly on the scale, the interval is that scale value
    if abs(liquid_level - lower_mark) < 1e-10:
        return [lower_mark, lower_mark]
    elif abs(liquid_level - upper_mark) < 1e-10:
        return [upper_mark, upper_mark]
    else:
        # If the liquid level is between two scales, return the interval
        return [lower_mark, upper_mark]


@registry.register(
    name="lff_synthetic_measuring_cylinder", tags={"measuring_cylinder"}, weight=1.0
)
def generate(img_path: str) -> Artifact:
    config = ConfigGenerator.generate_random_config()
    render_vessel(config, img_path)

    evaluator_kwargs = {
        "interval": calculate_reading_interval(config),
        "units": ["ml", "milliliter"],
    }

    return Artifact(
        data=img_path,
        image_type="measuring_cylinder",
        design="Linear",
        evaluator_kwargs=evaluator_kwargs,
    )
