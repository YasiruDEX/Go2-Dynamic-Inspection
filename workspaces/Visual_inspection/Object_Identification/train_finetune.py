# --------------------------------------------------------------
# train_finetune.py — memory-safe YOLOv8s fine-tuning on Windows
# --------------------------------------------------------------

from ultralytics import YOLO
import torch, gc

if __name__ == "__main__":
    # clear any cached memory first
    gc.collect()
    torch.cuda.empty_cache()

    # load YOLOv8 OBB model
    model = YOLO("yolov8s-obb.pt")

    # train the model
    model.train(
        data=r"E:\sem7\FYP\9_24\objectdetetction\datasets\custom_merged\data.yaml",
        epochs=50,
        imgsz=416,           # smaller for VRAM balance
        batch=2,             # fits better on 6 GB GPU
        device=0,            # GPU
        workers=0,           # Windows-safe
        amp=True,            # mixed precision
        mosaic=0.0,          # completely disable mosaic (safe)
        mixup=0.0,           # disable mixup too
        erasing=0.0,         # reduce CPU augmentation
        cos_lr=False,
        lr0=0.001,
        optimizer="SGD",
        name="yolov8s_custom_merged_vramfix",
        patience=10,
        verbose=True,
    )

    # evaluate after training
    metrics = model.val()
    print(metrics)
