from .thermometer2_utils.config import Config
from .thermometer2_utils.render import ThermometerRenderer
from registry import registry
from artifacts import Artifact


@registry.register(name="lff_synthetic_thermometer", tags={"thermometer"}, weight=1.0)
def generate_thermometer(img_path="thermometer.png") -> Artifact:
    config = Config()
    ThermometerRenderer().render(config, img_path)

    if config.scale_type == "C_F":
        evaluator = "multi_interval_matching"
        evaluator_kwargs = {
            "intervals": [config.celsius_interval, config.fahrenheit_interval],
            "units": [["Celsius", "°C"], ["fahrenheit", "°F"]],
        }
    elif config.scale_type == "C":
        evaluator = "interval_matching"
        evaluator_kwargs = {
            "interval": config.celsius_interval,
            "units": ["Celsius", "°C"],
        }
    else:  # 'F'
        evaluator = "interval_matching"
        evaluator_kwargs = {
            "interval": config.fahrenheit_interval,
            "units": ["fahrenheit", "°F"],
        }

    return Artifact(
        data=img_path,
        image_type="thermometer",
        design="Linear",
        evaluator_kwargs=evaluator_kwargs,
        evaluator=evaluator,
    )
