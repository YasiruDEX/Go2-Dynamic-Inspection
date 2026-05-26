# YOLO Weights Directory

## Instructions

Place your YOLOv11 weight file here and rename it to:
- `yolo11n.pt` (for the main model)

## Supported Model Types

You mentioned having:
- `yolo11n.pt` - YOLOv11 nano (standard bounding box detection)
- `yolo11n-obb.pt` - YOLOv11 nano with oriented bounding boxes
- `yolo11s-obb.pt` - YOLOv11 small with oriented bounding boxes

For now, place the **yolo11n.pt** file here.

## TensorRT Engine Export

After placing the `.pt` file, we'll export it to TensorRT engine:
```bash
# This will be done in Step 3
python test_scripts/export_to_tensorrt.py
```

This will create `yolo11n.engine` for accelerated inference on Jetson.

## Detected Classes

Your model is trained to detect:
- fire_extinguisher
- door
- gauges

(More classes can be added later)
