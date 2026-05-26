# Usage

This guide explains how to create and register new instrument generators for the data machine.

## 1. Define a Configuration

Create a configuration file for your instrument in JSON format. See `data_machine/configs` for examples.

## 2. Generate the Prompt

Run the following command to generate a prompt for your instrument:

```bash
python prompt/gen_prompt.py --config <config_path>
```

## 3. Generate Code

1. Copy the generated prompt and paste it into a chat LLM (e.g., GPT, Gemini, etc.)
2. Create a new file in `data_machine/generators/<instrument_name>/` and copy the generated code into it
3. Ensure there is an `__init__.py` file in the folder
4. Test the generated code to verify it works correctly and that the image and `evaluator_kwargs` are properly configured

## 4. Register the Generator

`data_machine/generators/ammeter/ammeter2.py` is a example of a registered generator.

1. Modify the `generate(img_path: str)` function by adding a registry decorator:

```python
@registry.register(name="ammeter2", tags={"ammeter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
```

2. At the end of the function, return an `Artifact` object:

```python
return Artifact(data=img_path, image_type="ammeter", design="Dial", evaluator_kwargs=evaluator_kwargs)
```

## Example

Here's a complete example of a registered generator:

```python
from registry import registry
from artifacts import Artifact

@registry.register(name="ammeter2", tags={"ammeter"}, weight=1.0)
def generate(img_path: str) -> Artifact:
    # Your generator code here
    evaluator_kwargs = {
        # Your evaluator configuration
    }
    return Artifact(data=img_path, image_type="ammeter", design="Dial", evaluator_kwargs=evaluator_kwargs)
```

# 5. Run batch generation

To test the new generator, run the following command:

```bash
export PYTHONPATH=<your data_machine path>
python generate_data.py --tag <tag> -g <registered generator names> --num <num> --output <output_path>
# for example
python generate_data.py --tag clock -g blender_clock1 roman_station_clock --num 20 --output output
```

To randomly generate data from all registered generators, run the following command:

```bash
python generate_data.py --tag <tag> --num <num> --output <output_path>
# for example
python generate_data.py --tag ammeter --num 10 --output output
```
