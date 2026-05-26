import json
import argparse

from prompt.prompt_template import PROMPT_TEMPLATE


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="prompt/configs/ammeter.json")
    return parser.parse_args()


args = parse_args()

with open(args.config, "r") as f:
    config = json.load(f)

prompt = PROMPT_TEMPLATE.format(
    INSTRUMENT_NAME=config["name"],
    INSTRUMENT_DESCRIPTION=config["description"],
    INSTRUMENT_DESIGN=config["design"],
    INSTRUMENT_OTHER_REQUIREMENTS=config["other_requirements"],
)

with open(f"{config['name']}.txt", "w") as f:
    f.write(prompt)
