from typing import List
import random

from .protractor1_utils.config import ConfigGenerator, ProtractorConfig
from .protractor1_utils.render import ProtractorRenderer
from registry import registry
from artifacts import Artifact


def calculate_interval(config: ProtractorConfig, actual_angle: float) -> List[float]:
    min_scale = config.scale_config.min_scale

    # Consider human reading error, usually within ±0.5 of the minimum scale
    reading_tolerance = min_scale * 0.5
    min_reading = max(0, actual_angle - reading_tolerance)
    max_reading = actual_angle + reading_tolerance

    # For 360-degree protractors, consider angles possibly exceeding 180 degrees
    if config.protractor_type.value == "360":
        max_possible_angle = 360
    else:
        max_possible_angle = 180

    max_reading = min(max_reading, max_possible_angle)

    return [round(min_reading, 1), round(max_reading, 1)]


@registry.register(name="lff_synthetic_protractor", tags={"protractor"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    config_generator = ConfigGenerator()
    renderer = ProtractorRenderer()

    config = config_generator.generate_random_config()
    img = renderer.render_protractor(config)
    renderer.save_image(img, img_path)

    angle_info = config_generator.get_angle_measurement_info(config)
    evaluator_kwargs = {
        "interval": calculate_interval(config, angle_info["actual_angle"]),
        "units": ["degree", "°"],
    }
    question_candidates = [
        "What is the angle reading on this protractor?",
        "What angle does this protractor show?",
        "What is the angle measurement?",
        "What angle is displayed?",
        "What is the angle reading indicated by the protractor?",
        "What angle is being measured?",
    ]
    return Artifact(
        data=img_path,
        image_type="protractor",
        design="Dial",
        question=random.choice(question_candidates),
        evaluator_kwargs=evaluator_kwargs,
    )
