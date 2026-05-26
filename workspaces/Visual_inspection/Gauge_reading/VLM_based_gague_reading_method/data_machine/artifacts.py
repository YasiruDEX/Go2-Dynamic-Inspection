from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class Artifact:
    data: Any  # 例如图像（PIL.Image/np.ndarray/路径）、标注、json 等
    image_type: str
    design: str
    evaluator_kwargs: Dict
    evaluator: str = "interval_matching"
    generator: Optional[str] = None
    question: Optional[str] = None

    def __post_init__(self):
        if not isinstance(self.evaluator_kwargs, dict):
            raise TypeError("evaluator_kwargs must be a dict")

        # accept alias "intervers" and normalize to "interval"
        if (
            self.evaluator == "interval_matching"
            and "interval" not in self.evaluator_kwargs
        ):
            raise ValueError("interval must be in evaluator_kwargs")
        elif (
            self.evaluator == "multi_interval_matching"
            and "intervals" not in self.evaluator_kwargs
        ):
            raise ValueError("intervals must be in evaluator_kwargs")

        if "units" not in self.evaluator_kwargs:
            raise ValueError("units must be in evaluator_kwargs")
