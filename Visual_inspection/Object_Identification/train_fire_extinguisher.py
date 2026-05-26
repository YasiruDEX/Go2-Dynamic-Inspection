from ultralytics import YOLO

# Load YOLOv8 OBB small model
model = YOLO("yolov8s-obb.pt")   # try yolov8n-obb.pt if VRAM is low

# Train
model.train(
    data=r"E:\sem7\FYP\9_24\objectdetetction\datasets\FireExtinguisher.v1-2025.yolov8-obb\data.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    device=0,
    name="fire_extinguisher_obb"
)

