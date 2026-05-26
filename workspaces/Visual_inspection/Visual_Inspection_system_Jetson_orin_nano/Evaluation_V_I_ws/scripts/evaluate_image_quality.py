#!/usr/bin/env python3
"""
evaluate_image_quality.py
Run on LAPTOP after copying images from Jetson.

Computes for each test image vs reference image:
  - SSIM   (structural similarity)
  - PSNR   (peak signal-to-noise ratio)
  - YCbCr luminance comparison
  - VIF    (visual information fidelity)
  - AlexNet cosine similarity

Usage:
  python3 scripts/evaluate_image_quality.py \
      --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
      --test eval_dataset/angle_eval/horizontal/30deg_L/ \
      --out  results/image_quality_30deg_L.csv
"""

import argparse, csv, sys, os
from pathlib import Path
import cv2
import numpy as np

def load_pair(ref_path, test_path, size=(640, 480)):
    ref  = cv2.imread(str(ref_path))
    test = cv2.imread(str(test_path))
    if ref is None:  raise FileNotFoundError(f'Cannot read ref: {ref_path}')
    if test is None: raise FileNotFoundError(f'Cannot read test: {test_path}')
    test_r = cv2.resize(test, (ref.shape[1], ref.shape[0]))
    return ref, test_r

def compute_ssim(ref, test):
    from skimage.metrics import structural_similarity as ssim_fn
    ref_g  = cv2.cvtColor(ref,  cv2.COLOR_BGR2GRAY)
    test_g = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
    score, _ = ssim_fn(ref_g, test_g, full=True)
    return round(float(score), 4)

def compute_psnr(ref, test):
    val = cv2.PSNR(ref, test)
    return round(float(val), 2)

def compute_ycbcr(ref, test):
    ref_y  = cv2.cvtColor(ref,  cv2.COLOR_BGR2YCrCb)[:,:,0]
    test_y = cv2.cvtColor(test, cv2.COLOR_BGR2YCrCb)[:,:,0]
    from skimage.metrics import structural_similarity as ssim_fn
    y_ssim, _ = ssim_fn(ref_y, test_y, full=True)
    y_diff = abs(float(ref_y.mean()) - float(test_y.mean()))
    return round(float(y_ssim), 4), round(y_diff, 2), round(float(test_y.mean()), 1)

def compute_vif(ref, test):
    try:
        import piq, torch
        def to_t(img):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype('float32') / 255.0
            return torch.from_numpy(img).permute(2,0,1).unsqueeze(0)
        score = piq.vif_p(to_t(ref), to_t(test), data_range=1.0)
        return round(float(score.item()), 4)
    except Exception as e:
        print(f'  [VIF] skipped: {e}')
        return None

def compute_alexnet(ref_path, test_path):
    try:
        import torch, torchvision.models as models, torchvision.transforms as T
        import torch.nn.functional as F
        from PIL import Image

        alexnet = models.alexnet(weights='IMAGENET1K_V1')
        extractor = torch.nn.Sequential(*list(alexnet.features.children()))
        extractor.eval()
        pre = T.Compose([T.Resize(256), T.CenterCrop(224), T.ToTensor(),
                         T.Normalize([0.485,0.456,0.406],[0.229,0.224,0.225])])
        def feat(p):
            img = Image.open(p).convert('RGB')
            with torch.no_grad():
                return extractor(pre(img).unsqueeze(0)).flatten()
        cos = F.cosine_similarity(feat(ref_path).unsqueeze(0),
                                  feat(test_path).unsqueeze(0))
        return round(float(cos.item()), 4)
    except Exception as e:
        print(f'  [AlexNet] skipped: {e}')
        return None

def eval_one(ref_path, test_path):
    ref, test = load_pair(ref_path, test_path)
    ssim  = compute_ssim(ref, test)
    psnr  = compute_psnr(ref, test)
    y_ssim, y_diff, y_mean = compute_ycbcr(ref, test)
    vif   = compute_vif(ref, test)
    alex  = compute_alexnet(ref_path, test_path)

    status = 'PASS'
    if ssim  is not None and ssim  < 0.70: status = 'FAIL'
    if psnr  is not None and psnr  < 30:   status = 'FAIL'
    if vif   is not None and vif   < 0.40: status = 'FAIL'
    if alex  is not None and alex  < 0.85: status = 'WARN'

    print(f'  {"✓" if status=="PASS" else "✗"} {Path(test_path).name}  '
          f'SSIM={ssim}  PSNR={psnr}dB  VIF={vif}  AlexNet={alex}  → {status}')
    return {
        'test_image': str(test_path),
        'ssim': ssim, 'psnr': psnr,
        'y_ssim': y_ssim, 'y_mean': y_mean, 'y_brightness_diff': y_diff,
        'vif': vif, 'alexnet_cos': alex, 'status': status
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ref',  required=True, help='Reference image path')
    ap.add_argument('--test', required=True, help='Test image or folder of images')
    ap.add_argument('--out',  default='results/image_quality.csv')
    args = ap.parse_args()

    ref_path  = Path(args.ref)
    test_path = Path(args.test)
    out_path  = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not ref_path.exists():
        print(f'ERROR: reference not found: {ref_path}'); sys.exit(1)

    if test_path.is_dir():
        test_files = sorted(test_path.glob('*.jpg')) + sorted(test_path.glob('*.png'))
    else:
        test_files = [test_path]

    if not test_files:
        print(f'No images found in {test_path}'); sys.exit(1)

    print(f'\nEvaluating {len(test_files)} image(s) vs reference: {ref_path.name}')
    results = []
    for f in test_files:
        try:
            results.append(eval_one(ref_path, f))
        except Exception as e:
            print(f'  ERROR {f.name}: {e}')

    if results:
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=results[0].keys())
            writer.writeheader()
            writer.writerows(results)

        # Summary
        good = [r for r in results if r['ssim'] is not None]
        print(f'\n=== SUMMARY ===')
        print(f'Images evaluated: {len(results)}')
        print(f'Avg SSIM:         {sum(r["ssim"] for r in good)/len(good):.4f}  (target > 0.70)')
        print(f'Avg PSNR:         {sum(r["psnr"] for r in good)/len(good):.2f} dB  (target > 30)')
        if any(r["vif"] for r in good):
            vifl = [r["vif"] for r in good if r["vif"]]
            print(f'Avg VIF:          {sum(vifl)/len(vifl):.4f}  (target > 0.40)')
        if any(r["alexnet_cos"] for r in good):
            all_ = [r["alexnet_cos"] for r in good if r["alexnet_cos"]]
            print(f'Avg AlexNet Sim:  {sum(all_)/len(all_):.4f}  (target > 0.85)')
        print(f'Results saved to: {out_path}')

if __name__ == '__main__':
    main()
