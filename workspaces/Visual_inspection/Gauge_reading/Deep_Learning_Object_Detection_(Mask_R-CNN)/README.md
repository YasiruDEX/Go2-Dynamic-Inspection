All code is inside source code directory

## Create environment
```bash
conda create --name gauge_reading python==3.8
conda activate gauge_reading
```

## Install Required Dependencies

```bash
pip install -r requirements.txt
```
## Trained Weights
Please find the trained weights on folder ```weights/finetunned_realdata_11.pth```

## For inference
```bash
  python api.py
```
Upload gauge_meter reading and hit the submit button

# From DockerFile
## Build Image from Dockerfile

```bash
  docker build -t <TAG> .
  docker run -p 5000:5000 <TAG> 
```
## Download docker from dockerhub
```bash
  docker run -p 5000:5000 sangamman/gaugereading:latest
```
