from ultralytics import YOLO

# Load fine-tuned model
model = YOLO(r"E:\sem7\FYP\9_24\objectdetetction\runs\obb\yolov8s_custom_merged_vramfix2\weights\best.pt")

# Path to your recorded video
video_path = r"E:\sem7\FYP\9_24\objectdetetction\record\insta360_record_20251007_221239.mp4"

# Run inference
results = model.predict(source=video_path, show=True, save=True, conf=0.5)

# All output videos will be saved under:
# runs/detect/predictX/
