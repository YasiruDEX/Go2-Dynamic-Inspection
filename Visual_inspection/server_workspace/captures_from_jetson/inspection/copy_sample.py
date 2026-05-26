import os
import shutil
import glob
import random

inspection_dir = r"E:\sem7\FYP\Main repo github upload\Visual_inspection\server_workspace\captures_from_jetson\inspection"
output_dir = os.path.join(inspection_dir, "gauge_20_samples")

if os.path.exists(output_dir):
    shutil.rmtree(output_dir)
os.makedirs(output_dir)

# Find all timestamp folders
timestamp_folders = [f for f in os.listdir(inspection_dir) if os.path.isdir(os.path.join(inspection_dir, f)) and f != "gauge_20_samples"]

gauge_images_to_copy = []

# Walk through each timestamp folder
for tf in timestamp_folders:
    tf_path = os.path.join(inspection_dir, tf)
    # Check if 'gauge' class folder exists inside the timestamp folder
    gauge_path = os.path.join(tf_path, "gauge")
    if os.path.exists(gauge_path):
        # Inside gauge, there are instance folders like 'instance_1', 'instance_2', etc.
        instances = [i for i in os.listdir(gauge_path) if os.path.isdir(os.path.join(gauge_path, i))]
        for inst in instances:
            inst_path = os.path.join(gauge_path, inst)
            # Find images in this instance folder
            images = glob.glob(os.path.join(inst_path, "*.jpg")) + glob.glob(os.path.join(inst_path, "*.png"))
            if images:
                # Take ONE picture from this gauge instance
                selected = random.choice(images)
                gauge_images_to_copy.append(selected)

# Copy and rename them
for i, src_path in enumerate(gauge_images_to_copy):
    ext = os.path.splitext(src_path)[1]
    new_name = f"gauge_20_{i+1}{ext}"
    dst_path = os.path.join(output_dir, new_name)
    shutil.copy2(src_path, dst_path)
    print(f"Copied {os.path.basename(src_path)} from {os.path.basename(os.path.dirname(os.path.dirname(src_path)))}/{os.path.basename(os.path.dirname(src_path))} -> {new_name}")

print(f"\nTotal gauge images collected: {len(gauge_images_to_copy)}")
