import os
import cv2
import numpy as np
import random

# Paths
base = "E:\sem7\FYP\9_24\objectdetetction\datasets\custom_merged_obb"
image_dir = os.path.join(base, "images/train")
label_dir = os.path.join(base, "labels/train")
out_dir = "E:\sem7\FYP\9_24\objectdetetction\datasets"
os.makedirs(out_dir, exist_ok=True)

# CLASS ID of gauge (change if different)
GAUGE_ID = 2

# Get all label files
label_files = [f for f in os.listdir(label_dir) if f.endswith(".txt")]

# Filter only those that contain gauge class
gauge_files = []
for f in label_files:
    with open(os.path.join(label_dir, f)) as lf:
        for line in lf:
            if line.strip().startswith(str(GAUGE_ID) + " "):
                gauge_files.append(f)
                break

# Randomly pick 100 (or fewer if less exist)
sample_files = random.sample(gauge_files, min(100, len(gauge_files)))

def draw_obb(image, cx, cy, w, h, angle, color=(0, 255, 0)):
    """Draw oriented bounding box"""
    # Convert normalized coords (YOLO format) back to pixels
    H, W = image.shape[:2]
    cx, cy, w, h = cx * W, cy * H, w * W, h * H

    # Convert angle in radians or degrees depending on dataset format
    # Try assuming angle in radians (for OBB)
    rect = ((cx, cy), (w, h), np.degrees(angle))
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.polylines(image, [box], True, color, 2)

# Process each gauge label
for f in sample_files:
    img_path = os.path.join(image_dir, f.replace(".txt", ".jpg"))
    if not os.path.exists(img_path):
        img_path = os.path.join(image_dir, f.replace(".txt", ".png"))
        if not os.path.exists(img_path):
            continue

    img = cv2.imread(img_path)
    if img is None:
        continue

    with open(os.path.join(label_dir, f)) as lf:
        for line in lf:
            parts = line.strip().split()
            if not parts or int(parts[0]) != GAUGE_ID:
                continue
            if len(parts) < 6:
                continue  # skip malformed lines

            # YOLO-OBB format: class cx cy w h theta ...
            cls, cx, cy, w, h, theta = map(float, parts[:6])
            draw_obb(img, cx, cy, w, h, theta, color=(0,255,0))

    save_path = os.path.join(out_dir, f.replace(".txt", ".jpg"))
    cv2.imwrite(save_path, img)

print(f"Saved {len(sample_files)} annotated gauge images to {out_dir}")
