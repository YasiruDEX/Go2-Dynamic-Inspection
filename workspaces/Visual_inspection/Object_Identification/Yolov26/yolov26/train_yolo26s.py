"""
train_yolo26s.py
================
YOLO26s Local Training — Run directly from terminal.
Equivalent to running all cells in yolo26s_local_training.ipynb

Usage:
    python train_yolo26s.py
"""

import subprocess, sys, os
from pathlib import Path

# ─────────────────────────────────────────────────────────────
# SECTION 1 — Auto-install / upgrade ultralytics
# ─────────────────────────────────────────────────────────────
print("=" * 60)
print("  SECTION 1 — Environment Check")
print("=" * 60)

try:
    import ultralytics
    from packaging.version import Version
    if Version(ultralytics.__version__) < Version('8.4.0'):
        print('Upgrading ultralytics for YOLO26 support...')
        subprocess.run([sys.executable, '-m', 'pip', 'install', '-q',
                        'ultralytics>=8.4.0'], check=True)
        import importlib; importlib.reload(ultralytics)
except ImportError:
    subprocess.run([sys.executable, '-m', 'pip', 'install', '-q',
                    'ultralytics>=8.4.0'], check=True)

import torch
from ultralytics import YOLO
import ultralytics

print(f'  Python      : {sys.version.split()[0]}')
print(f'  PyTorch     : {torch.__version__}')
print(f'  Ultralytics : {ultralytics.__version__}')
print(f'  CUDA        : {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'  GPU         : {torch.cuda.get_device_name(0)}')
    vram = torch.cuda.get_device_properties(0).total_memory / 1e9
    print(f'  VRAM        : {vram:.1f} GB')
    DEVICE = -1   # idle GPU auto-selection
else:
    print('  WARNING: No GPU — training on CPU will be very slow!')
    DEVICE = 'cpu'
print("=" * 60)

# ─────────────────────────────────────────────────────────────
# SECTION 2 — Laptop Capability Check
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("  SECTION 2 — Laptop Capability Check")
print("=" * 60)

try:
    import psutil, platform
    cpu_count  = psutil.cpu_count(logical=True)
    cpu_phys   = psutil.cpu_count(logical=False)
    ram        = psutil.virtual_memory()
    disk       = psutil.disk_usage('C:\\')
    print(f'  OS          : {platform.system()} {platform.release()}')
    print(f'  CPU cores   : {cpu_phys} physical / {cpu_count} logical')
    print(f'  RAM total   : {ram.total/1e9:.1f} GB  |  Available: {ram.available/1e9:.1f} GB')
    print(f'  Disk free   : {disk.free/1e9:.1f} GB / {disk.total/1e9:.1f} GB')
except ImportError:
    print('  psutil not installed — skipping hardware check')

if torch.cuda.is_available():
    vram_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
    print(f'  GPU VRAM    : {vram_gb:.1f} GB')
    if vram_gb >= 8:
        print('  Status      : ✅ GOOD — batch=8 recommended, ~2-4 hours')
    elif vram_gb >= 4:
        print('  Status      : ⚠️  OK — use batch=4 to avoid OOM')
    else:
        print('  Status      : ⚠️  LOW VRAM — use batch=2 and imgsz=640')
else:
    print('  Status      : CPU-only — ~24-72 hours estimated')
print("=" * 60)

# ─────────────────────────────────────────────────────────────
# SECTION 3 — Path Configuration
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("  SECTION 3 — Path Configuration")
print("=" * 60)

WORK_DIR    = Path(r'C:\Users\Rebecca Fernando\Desktop\Dinethra\Yolov11s')
DATASET_DIR = Path(r'C:\Users\Rebecca Fernando\Desktop\Dinethra\YOLO_Final_Dataset_kaggle')
DATA_YAML   = WORK_DIR / 'data_local.yaml'
RUN_DIR     = WORK_DIR / 'runs'
EXP_NAME    = 'yolo26s_exp'
CLASS_NAMES = ['door', 'extinguisher', 'gauge', 'person']

WORK_DIR.mkdir(parents=True, exist_ok=True)
RUN_DIR.mkdir(parents=True, exist_ok=True)

# Verify dataset
splits = ['train', 'valid', 'test']
print('Dataset verification:')
print('-' * 40)
for split in splits:
    img_dir = DATASET_DIR / split / 'images'
    lbl_dir = DATASET_DIR / split / 'labels'
    n_imgs = len(list(img_dir.glob('*.*'))) if img_dir.exists() else 0
    n_lbls = len(list(lbl_dir.glob('*.txt'))) if lbl_dir.exists() else 0
    print(f'  {split:6s} → images: {n_imgs:5d}  labels: {n_lbls:5d}')
print(f'  data.yaml exists: {DATA_YAML.exists()}')
print(f'  Output dir: {RUN_DIR / EXP_NAME}')
print("=" * 60)

if not DATA_YAML.exists():
    print("ERROR: data_local.yaml not found! Aborting.")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────
# SECTION 4 — Train YOLO26s (100 epochs, idle GPU, MuSGD)
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("  SECTION 4 — Training YOLO26s")
print("  Model     : yolo26s.pt  (NMS-free, MuSGD)")
print(f"  Epochs    : 100")
print(f"  imgsz     : 960")
print(f"  batch     : 8  (reduce to 4 if OOM)")
print(f"  device    : -1 (idle GPU auto-selection)")
print(f"  optimizer : auto → MuSGD for long runs")
print("=" * 60)

if __name__ == '__main__':
    model = YOLO('yolo26s.pt')   # auto-downloads ~19 MB on first run

    results = model.train(
        data          = str(DATA_YAML),

        # Core
        epochs        = 100,
        imgsz         = 960,
        batch         = 8,        # reduce to 4 if GPU OOM
        device        = -1,       # idle GPU auto-selection
        workers       = 0,        # 0 = safe on Windows (no multiprocessing)
        cache         = False,

        # Optimizer — auto selects MuSGD for 100-epoch runs
        optimizer     = 'auto',
        lr0           = 0.01,
        lrf           = 0.01,
        momentum      = 0.937,
        weight_decay  = 0.0005,

        # Augmentation
        mosaic        = 1.0,
        scale         = 0.5,
        fliplr        = 0.5,
        translate     = 0.1,

        # Mixed precision
        amp           = True,

        # Saving
        project       = str(RUN_DIR),
        name          = EXP_NAME,
        save          = True,
        save_period   = 5,        # checkpoint every 5 epochs
        exist_ok      = True,

        # Stopping
        patience      = 40,
        plots         = True,
        verbose       = True,
    )

    print('\n' + '=' * 60)
    print('  TRAINING COMPLETE')
    print(f"  mAP50    : {results.results_dict.get('metrics/mAP50(B)', 'N/A')}")
    print(f"  mAP50-95 : {results.results_dict.get('metrics/mAP50-95(B)', 'N/A')}")
    print(f"  Weights  : {RUN_DIR / EXP_NAME / 'weights' / 'best.pt'}")
    print('=' * 60)
