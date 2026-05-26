# --------------------------------------------------------------
# train_finetune.py — memory-safe YOLOv8s fine-tuning on Windows
# --------------------------------------------------------------
from ultralytics import YOLO

if __name__ == "__main__":
    model = YOLO("yolov8s.pt")

    model.train(
        data=r"E:\sem7\FYP\9_24\objectdetetction\datasets\custom_merged\data.yaml",
        epochs=50,
        imgsz=512,              
        batch=4,               
        device=0,               # use GPU
        name="industrial_detector_v1",
        workers=0,              # Windows-safe
        patience=15,
        lr0=0.001,
        optimizer='SGD',
        pretrained=True,
        deterministic=True,
        exist_ok=True,
        amp=False,              # disable mixed precision
        cache=False,            # disable image caching
        verbose=True,
        save=True
    )

    # evaluate after training
    metrics = model.val()
    print(metrics)
