# analog_gauge_reader

<p align="center">
<img src=method_overview.png>
</p>

This is the code for the paper [Under Pressure: Learning-Based Analog Gauge Reading In The Wild](https://arxiv.org/abs/2404.08785) by Maurits Reitsma, Julian Keller, Kenneth Blomqvist and Roland Siegwart. 

## File Structure

```
analog_gauge_reader/
├── angle_reading_fit/          # Angle conversion and line fitting utilities
│   ├── angle_converter.py
│   └── line_fit.py
│
├── decimal_point_detection/    # Decimal point detection module (NEW)
│   ├── decimal_detector.py     # Main decimal point detection logic
│   ├── number_roi_extractor.py # ROI extraction for number regions
│   ├── pipeline_integration.py # Integration with main pipeline
│   ├── test_decimal_detector.py
│   ├── test_roi_extractor.py
│   ├── test_with_real_data.py
│   ├── legacy_methods/         # Previous detection approaches
│   └── README.md
│
├── dependencies/               # External model weights and dependencies
│   ├── abinet_20e_st-an_mj_20221005_012617-ead8c139.pth
│   ├── dbnet_resnet18_fpnc_1200e_icdar2015_20220825_221614-7c0e94f2.pth
│   └── facebookresearch-dinov2-c3c2683.zip
│
├── evaluation/                 # Evaluation scripts and metrics
│   ├── constants.py
│   ├── eval_plots.py
│   ├── evaluation.py
│   ├── full_evaluation.py
│   ├── evaluation_file_gen.py
│   ├── evaluation_file.ipynb
│   └── README.md
│
├── gauge_detection/            # Gauge detection (YOLOv8)
│   ├── detection_inference.py
│   ├── detection_training_local.py
│   ├── detection_training_colab.ipynb
│   └── README.md
│
├── geometry/                   # Geometric transformations
│   ├── ellipse.py
│   ├── warp_ellipse.py
│   └── ellipse_demo.ipynb
│
├── key_point_detection/        # Key point detection module
│   ├── key_point_extraction.py
│   ├── key_point_inference.py
│   ├── key_point_validator.py
│   ├── key_point_dataset.py
│   ├── model.py
│   ├── train.py
│   ├── data_preparation/
│   └── README.md
│
├── models/                     # Trained model weights
│   ├── gauge_detection_model.pt
│   ├── key_point_model.pt
│   └── segmentation_model.pt
│
├── ocr/                        # OCR inference and reading
│   ├── ocr_inference.py
│   ├── ocr_reading.py
│   ├── ocr_inference_colab.ipynb
│   └── README.md
│
├── segmentation/               # Gauge segmentation module
│   ├── segmenation_inference.py
│   ├── line_fit_test.ipynb
│   └── README.md
│
├── launch/                     # ROS launch files
│   └── analog_gauge_reader.launch
│
├── msg/                        # ROS message definitions
│   ├── GaugeReading.msg
│   └── GaugeReadings.msg
│
├── srv/                        # ROS service definitions
│   └── GaugeReader.srv
│
├── scripts/                    # Utility scripts
│   ├── analog_gauge_reader_ros.sh
│   └── analog_gauge_reader_ros_poetry.sh
│
├── pipeline.py                 # Main pipeline script
├── plots.py                    # Visualization utilities
├── ros_node.py                 # ROS node implementation
├── test_complete_pipeline.py   # End-to-end pipeline testing
├── experiments.sh              # Batch experiment runner
├── evaluations.sh              # Batch evaluation runner
├── full_evaluations.sh         # Comprehensive evaluation runner
├── pyproject.toml              # Poetry dependencies
├── poetry.lock                 # Locked dependencies
├── package.xml                 # ROS package configuration
├── CMakeLists.txt              # CMake build configuration
├── PRESENTATION_GUIDE.md       # Presentation materials
├── FINAL_PRESENTATION.md       # Final presentation content
├── RESULTS.md                  # Results and analysis
└── README.md                   # This file
```

## Setup installation (Poetry, automatic)

Install Poetry

```shell
curl -sSL https://install.python-poetry.org | python3 -
```

Install the project dependencies

```shell
poetry install
```

Enter Poetry shell

```shell
poetry shell
```

## Setup installation (manual)

To setup the conda environment to run all scripts follow the following instruction:

### Install miniconda
```shell
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

### Activate conda environment
```shell
conda create --name gauge_reader python=3.8 -y
conda activate gauge_reader
```

### install pytorch

We use torch version 2.0.0.

```shell
conda install pytorch==2.0.0 torchvision==0.15.0 torchaudio==2.0.0 -c pytorch -c nvidia
```

### install mmocr

Refer to this page for installation <https://mmocr.readthedocs.io/en/dev-1.x/get_started/install.html>
We use the version dev-1.x

```shell
pip install -U openmim
mim install mmengine==0.7.2
mim install mmcv==2.0.0
mim install mmdet==3.0.0
mim install mmocr==1.0.0
```

We use the following versions: mmocr 1.0.0, mmdet 3.0.0, mmcv 2.0.0, mmengine 0.7.2.
If for some reason the installation fails refer to https://github.com/open-mmlab/mmcv/issues/2938.
We found that it is essential that we have Pytorch version 2.0.0

#### install yolov8

We use ultralytics version 8.0.66

```shell
pip install ultralytics
```

#### install sklearn

We use scikit-learn version 1.2.2

```shell
pip install -U scikit-learn
```

## Run pipeline script

The pipeline script can be run with the following command:

```shell
python pipeline.py --detection_model path/to/detection_model --segmentation_model /path/to/segmentation_model --key_point_model path/to/key_point_model --base_path path/to/results --input path/to/test_image_folder/images --debug --eval
```

For the input you can either choose an entire folder of images or a single image. Both times the result will be saved to a new run folder created in the `base_path` folder. For each image in the input folder a separate folder will be created.

In each such folder the reading is stored inside the `result.json` file. If there is no such reading, one of the pipeline stages failed before a reading could be computed. Best check the log file which is saved inside the run folder, to see where the error came up. There will also be a `error.json` file saved to the image folder, which computes some metrics to check without any labels how good our estimate is.

Additionally if the `debug` flag is set then the plots of all pipeline stages will be added to this folder. If the `eval` flag is set then there will also be a `result_full.json` file created. This file contains the data of the individual stages of the pipeline, which is used when evaluating in the script `full_evaluation.py`.

## Run experiments

I prepared two scripts to automatically run the pipeline and evaluations on multiple folders with one command. This allows us to easily conduct experiments for images that we group by their characteristics in different folders.

If they want to be used, make sure to modify the paths inside the scripts, to match with your data.
