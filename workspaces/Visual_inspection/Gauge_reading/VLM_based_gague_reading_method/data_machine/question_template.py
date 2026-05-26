import random
from artifacts import Artifact

COMMON_TEMPLATES = [
    "What is the measurement shown on this {image_type}?",
    "Tell me the reading of the {image_type}.",
    "What is the {image_type} reading?",
    "What does the {image_type} read?",
    "What current measurement is displayed?",
    "What current value does the meter display?",
    "What current is being measured?",
    "What is the reading of the {image_type}?",
]
MEASUREMENT_SPECIFIC_TEMPLATES = {
    "measuring_cylinder": [
        "What is the volume reading on this {image_type}?",
        "What volume does this {image_type} show?",
        "What is the liquid level reading?",
        "How much liquid is in this {image_type}?",
        "What volume reading is displayed on this graduated cylinder?",
        "What is the liquid volume measurement?",
        "What does the scale indicate for the liquid volume?",
        "How much liquid is measured in this {image_type}?",
        "What volume reading can you see?",
        "What is the liquid level measurement?",
    ],
    "ammeter": [
        "What is the current reading on this {image_type}?",
        "What current does this {image_type} show?",
        "What is the electrical current reading on this ammeter?",
        "What amperage is indicated by the needle?",
    ],
    "voltmeter": [
        "What is the voltage reading on this {image_type}?",
        "What voltage does this {image_type} show?",
        "What is the voltage measurement?",
        "What voltage is displayed?",
        "What is the voltage reading indicated by the needle?",
    ],
    "thermometer": [
        "What is the temperature reading on this {image_type}?",
        "What temperature does this {image_type} show?",
        "What is the temperature measurement?",
        "What temperature is displayed?",
        "What is the temperature reading indicated by the liquid level?",
        "What temperature does the thermometer display?",
        "What is the current temperature measurement?",
        "What temperature is being measured?",
    ],
    "clock": [
        "What time does this {image_type} show?",
        "What is the time reading on this {image_type}?",
        "What time is displayed?",
        "What time is it?",
        "What time is it now?",
        "What is the current time shown on this clock?",
        "What time reading is indicated by the hands?",
        "What time does the clock display?",
        "What's the time?",
        "Tell me the time.",
    ],
    "pressure_gauge": [
        "What is the pressure reading on this {image_type}?",
        "What pressure does this {image_type} show?",
        "What is the pressure measurement?",
        "What pressure is displayed?",
        "What is the pressure reading indicated by the gauge?",
        "What pressure value does the meter display?",
        "What is the current pressure measurement?",
        "What pressure is being measured?",
        "What does the pressure gauge read?",
    ],
}

DESIGN_SPECIFIC_TEMPLATES = {
    "Linear": [
        "What is the reading on this linear scale?",
        "What value is indicated on the scale?",
        "What measurement is shown on this linear instrument?",
    ],
    "Dial": [
        "What is the reading on this dial?",
        "What value does the needle point to?",
        "What measurement is indicated by the pointer?",
    ],
}


def get_question_template(artifact: Artifact, rng: random.Random) -> str:
    image_type = artifact.image_type
    design = artifact.design
    template_candicates = []
    template_candicates.extend(COMMON_TEMPLATES)
    template_candicates.extend(MEASUREMENT_SPECIFIC_TEMPLATES.get(image_type, []))
    template_candicates.extend(DESIGN_SPECIFIC_TEMPLATES.get(design, []))
    template = rng.choice(template_candicates)
    return template.format(image_type=image_type.replace("_", " "))
