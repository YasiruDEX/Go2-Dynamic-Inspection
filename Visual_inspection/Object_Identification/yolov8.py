from ultralytics import YOLO
import torch, cv2, os, glob, datetime, numpy as np

video_dir = r"E:\sem7\FYP\9_24\objectdetetction\record"
out_dir = os.path.join(video_dir, "annotated")
os.makedirs(out_dir, exist_ok=True)
candidates = sorted(glob.glob(os.path.join(video_dir, "*.mp4")), key=os.path.getmtime)
if not candidates:
    raise SystemExit("No .mp4 files found in the record folder.")
video_path = candidates[-1]

model_path = "yolov8s.pt"
device = 0 if torch.cuda.is_available() else "cpu"
model = YOLO(model_path)
names = model.names if isinstance(model.names, dict) else {i:n for i,n in enumerate(model.names)}
want = {"person","chair","car","bottle","clock","fire hydrant"}
cls_ids = [i for i,n in names.items() if n in want]

cap = cv2.VideoCapture(video_path)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0: fps = 30.0
mid = w // 2
ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
out_path = os.path.join(out_dir, f"annotated_{ts}.mp4")
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))

while True:
    ok, frame = cap.read()
    if not ok: break
    left = frame[:, :mid]
    right = frame[:, mid:]
    res_l = model.predict(left, imgsz=640, conf=0.25, device=device, classes=cls_ids, verbose=False)
    res_r = model.predict(right, imgsz=640, conf=0.25, device=device, classes=cls_ids, verbose=False)
    ann_l = res_l[0].plot()
    ann_r = res_r[0].plot()
    if ann_l.shape[0] != h or ann_l.shape[1] != mid:
        ann_l = cv2.resize(ann_l, (mid, h))
    if ann_r.shape[0] != h or ann_r.shape[1] != (w - mid):
        ann_r = cv2.resize(ann_r, (w - mid, h))
    combined = np.concatenate([ann_l, ann_r], axis=1)
    writer.write(combined)
 
cap.release()
writer.release()
print(out_path)

