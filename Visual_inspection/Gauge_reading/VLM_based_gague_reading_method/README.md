# MeasureBench

🏠[Project Page](https://flageval-baai.github.io/MeasureBenchPage/) | 💻[Code](https://github.com/flageval-baai/MeasureBench) | 📖[Paper](https://arxiv.org/abs/2510.26865/) | 🤗[Data](https://huggingface.co/datasets/FlagEval/MeasureBench)

This is the codebase for paper: [Do Vision-Language Models Measure Up? Benchmarking Visual Measurement Reading with MeasureBench](https://arxiv.org/abs/2510.26865)

## Evaluation

The input model result file must be a **JSON list** with the following structure:

```json
[
  {
    "question_id": "question_id",
    "answer": "model-generated response"
  }
]
```

You can refer to the [example file](result_example/gpt-5_real.json) for the expected format.

To run the evaluation, use the following command:

```bash
python evaluation/evaluate.py \
  --split <split> \
  --result-file <path_to_result_json> \
  --output-dir <output_directory>
```

**Example:**

```bash
python evaluation/evaluate.py \
  --split synthetic_test \
  --result-file result_example/gemini-2.5-pro_syth.json
```

## Data Generation
### Install Dependencies
First, install the required dependencies for data generation:

```bash
pip install -r data_machine/requirements.txt

# Install dependencies for Blender
sudo apt-get install -y libxi6 libxrender1 libxext6 libsm6 libxrandr2 libxcursor1 libxkbcommon0 libxkbcommon-x11-0

# Install fonts
sudo apt-get install -y fonts-dejavu
```
Download the required .blend files from Hugging Face:
```bash
pip install -U "huggingface_hub[cli]"

 huggingface-cli download \
  --repo-type dataset \
  philokey/MeasureBench \
  --include "blend_files/*" \
  --local-dir ./data_machine/generators \
  --local-dir-use-symlinks False
```

### List Available Generators

To view all registered generators and tags:
```bash
python data_machine/generate_data.py -l
```

### Generate Synthetic Data
```bash
python data_machine/generate_data.py --tag <tag> -g <generator_names> --num <num_images> --output <output_path>
```

**Examples:**

* Generate 20 clock images from `blender_clock1` and `roman_station_clock`:

  ```bash
  python data_machine/generate_data.py -g blender_clock1 roman_station_clock --num 20 --output output
  ```

* Generate 10 samples from all registered generators:

  ```bash
  python data_machine/generate_data.py --num 10 --output output
  ```

* Generate 10 images for all generators tagged as “ammeter”:

  ```bash
  python data_machine/generate_data.py --tag ammeter --num 10 --output output
  ```

## Development Guidelines

This repository uses pre-commit to run basic sanity checks and Python style (Ruff) before each commit.

Setup (once per environment):

```bash
pip install pre-commit ruff
pre-commit install --install-hooks
```

Run hooks manually on the entire codebase:

```bash
pre-commit run --all-files
```

Update hook versions to the latest revisions:

```bash
pre-commit autoupdate
```

Use Ruff directly (optional):

```bash
ruff format .
ruff check .
```

## Citation

```
@misc{lin2025measurebench,
        title={Do Vision-Language Models Measure Up? Benchmarking Visual Measurement Reading with MeasureBench},
        author={Fenfen Lin and Yesheng Liu and Haiyu Xu and Chen Yue and Zheqi He and Mingxuan Zhao and Miguel Hu Chen and Jiakang Liu and JG Yao and Xi Yang},
        year={2025},
        eprint={2510.26865},
        archivePrefix={arXiv},
        primaryClass={cs.CV},
        url={https://arxiv.org/abs/2510.26865},
}
```
