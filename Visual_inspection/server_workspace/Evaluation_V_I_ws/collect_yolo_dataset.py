#!/usr/bin/env python3
"""
collect_yolo_dataset.py  —  YOLO Training Data Collector
=========================================================
Run on Jetson (inside venv, no ROS needed).

  source ~/Documents/Visual_Inspection_ws/venv/bin/activate
  cd ~/Documents/Visual_Inspection_ws
  python3 collect_yolo_dataset.py

Controls (in the camera window):
  ← →      Pan left / right
  ↑ ↓      Tilt up / down
  SPACE    Capture both cameras simultaneously
  c        Change class
  h        Home (pan=90, tilt=90)
  +/-      Increase/decrease step size
  q        Quit

Output:
  yolo_dataset/
    fire_extinguisher/
      insta360/   img_0001.jpg  img_0002.jpg ...
      logitech/   img_0001.jpg  img_0002.jpg ...
    door/
      insta360/   ...
      logitech/   ...
    gauge/
      insta360/   ...
      logitech/   ...
"""

import cv2
import serial
import time
import os
import glob
from pathlib import Path
from datetime import datetime

# ── Paths ─────────────────────────────────────────────────────────────────────
BASE     = Path.home() / 'Documents/Visual_Inspection_ws'
OUT_DIR  = BASE / 'yolo_dataset'
ARDUINO  = '/dev/ttyACM0'
BAUD     = 9600

# ── Classes ───────────────────────────────────────────────────────────────────
CLASSES = ['fire_extinguisher', 'door', 'gauge']

# ── Servo ─────────────────────────────────────────────────────────────────────
PAN_MIN, PAN_MAX   = 0, 180
TILT_MIN, TILT_MAX = 20, 160
TILT_INVERT = True   # flip if tilt direction is wrong (matches ROS action server)

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN  = (0, 255, 80)
YELLOW = (0, 220, 255)
RED    = (0, 60, 255)
WHITE  = (240, 240, 240)
CYAN   = (255, 220, 0)
DARK   = (30, 30, 30)

# ─────────────────────────────────────────────────────────────────────────────
# Camera detection (3-layer from ibvs_pipeline.py)
# ─────────────────────────────────────────────────────────────────────────────
VENDOR_MAP = {
    'Insta360':      ('2e1a', '/dev/insta360'),
    'HD Pro Webcam': ('046d', '/dev/logitech'),
    'Logitech':      ('046d', '/dev/logitech'),
}

def find_camera(name_pattern):
    vendor_id, udev_path = None, None
    for key, (vid, udev) in VENDOR_MAP.items():
        if key in name_pattern or name_pattern in key:
            vendor_id, udev_path = vid, udev
            break

    # Layer 1: udev symlink (e.g. /dev/insta360, /dev/logitech)
    if udev_path and os.path.exists(udev_path):
        cap = cv2.VideoCapture(udev_path)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None and frame.size > 0:
                idx = int(os.path.realpath(udev_path).replace('/dev/video', ''))
                return idx

    # Layer 2: vendor ID via sysfs — open by PATH STRING (integer index fails on Jetson)
    if vendor_id:
        for path in sorted(glob.glob('/sys/class/video4linux/video*')):
            try:
                check = os.path.realpath(path)
                for _ in range(8):
                    vid_file = os.path.join(check, 'idVendor')
                    if os.path.exists(vid_file):
                        with open(vid_file) as f:
                            if f.read().strip() == vendor_id:
                                idx      = int(os.path.basename(path).replace('video', ''))
                                dev_path = f'/dev/video{idx}'
                                cap      = cv2.VideoCapture(dev_path)   # path string, not int
                                if cap.isOpened():
                                    ret, frame = cap.read()
                                    cap.release()
                                    if ret and frame is not None and frame.size > 0:
                                        return idx
                        break
                    check = os.path.dirname(check)
            except:
                pass

    # Layer 3: brute-force try /dev/video0 .. /dev/video9 matching device name
    for idx in range(10):
        dev_path = f'/dev/video{idx}'
        if not os.path.exists(dev_path):
            continue
        try:
            name_file = f'/sys/class/video4linux/video{idx}/name'
            if os.path.exists(name_file):
                with open(name_file) as f:
                    dev_name = f.read().strip()
                if name_pattern.split()[0].lower() in dev_name.lower():
                    cap = cv2.VideoCapture(dev_path)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        cap.release()
                        if ret and frame is not None and frame.size > 0:
                            return idx
        except:
            pass

    return -1

# ─────────────────────────────────────────────────────────────────────────────
# Arduino controller
# ─────────────────────────────────────────────────────────────────────────────
class Arduino:
    def __init__(self, port=ARDUINO, baud=BAUD):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(2.0)   # wait for Arduino reset
            print(f'  Arduino connected: {port}')
        except Exception as e:
            print(f'  Arduino NOT connected: {e}')
            print('  Servo control DISABLED — keyboard still works for capture')

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode())
                time.sleep(0.03)
            except:
                pass

    def pan(self, angle):
        angle = max(PAN_MIN, min(PAN_MAX, angle))
        self.send(f'PAN:{angle}')
        return angle

    def tilt(self, angle):
        angle = max(TILT_MIN, min(TILT_MAX, angle))
        # Match ROS action server — invert before sending
        hw = (180 - angle) if TILT_INVERT else angle
        self.send(f'TILT:{hw}')
        return angle

    def home(self):
        self.pan(90)
        self.tilt(90)

    def close(self):
        if self.ser:
            self.ser.close()

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def count_imgs(cls):
    d_i = OUT_DIR / cls / 'insta360'
    d_l = OUT_DIR / cls / 'logitech'
    n_i = len(list(d_i.glob('*.jpg'))) if d_i.exists() else 0
    n_l = len(list(d_l.glob('*.jpg'))) if d_l.exists() else 0
    return n_i, n_l

def next_idx(folder):
    existing = list(folder.glob('img_*.jpg'))
    if not existing:
        return 1
    nums = [int(f.stem.split('_')[1]) for f in existing if f.stem.split('_')[1].isdigit()]
    return max(nums) + 1 if nums else 1

def save_both(frame_insta, frame_logi, cls, n_captured):
    ts = datetime.now().strftime('%H%M%S')
    dir_i = OUT_DIR / cls / 'insta360'
    dir_l = OUT_DIR / cls / 'logitech'
    dir_i.mkdir(parents=True, exist_ok=True)
    dir_l.mkdir(parents=True, exist_ok=True)

    idx_i = next_idx(dir_i)
    idx_l = next_idx(dir_l)

    fname_i = dir_i / f'img_{idx_i:04d}.jpg'
    fname_l = dir_l / f'img_{idx_l:04d}.jpg'

    saved = []
    if frame_insta is not None:
        cv2.imwrite(str(fname_i), frame_insta)
        saved.append(f'insta360/{fname_i.name}')
    if frame_logi is not None:
        cv2.imwrite(str(fname_l), frame_logi)
        saved.append(f'logitech/{fname_l.name}')

    return saved

def draw_hud(canvas, cls, pan, tilt, step, n_i, n_l, flash=False):
    h, w = canvas.shape[:2]
    cv2.rectangle(canvas, (0, 0), (w, 44), DARK, -1)

    status_color = GREEN if not flash else (0, 200, 255)
    cv2.putText(canvas, f'CLASS: {cls}', (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)
    cv2.putText(canvas, f'PAN:{pan:3d}  TILT:{tilt:3d}  step:{step}',
                (310, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)
    cv2.putText(canvas, f'insta:{n_i}  logi:{n_l}',
                (680, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 1)

    cv2.rectangle(canvas, (0, h-30), (w, h), DARK, -1)
    help_txt = '[SPACE]=capture  [arrows]=pan/tilt  [c]=class  [h]=home  [+/-]=step  [q]=quit'
    cv2.putText(canvas, help_txt, (6, h-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)

def select_class():
    os.system('clear')
    print('')
    print('  ╔══════════════════════════════════════╗')
    print('  ║   YOLO DATASET COLLECTOR             ║')
    print('  ╚══════════════════════════════════════╝')
    print('')
    for i, c in enumerate(CLASSES, 1):
        n_i, n_l = count_imgs(c)
        print(f'  {i}: {c:<22}  insta={n_i}  logi={n_l}')
    print('  q: Quit')
    print('')
    ch = input('  Select class: ').strip().lower()
    if ch == 'q':
        return None
    if ch.isdigit() and 1 <= int(ch) <= len(CLASSES):
        return CLASSES[int(ch) - 1]
    return CLASSES[0]

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    # ── Find cameras ─────────────────────────────────────────────────────────
    print('\nDetecting cameras...')
    insta_idx = find_camera('Insta360')
    logi_idx  = find_camera('Logitech')

    # Fallback 1: sysfs name for Logitech C920 is "HD Pro Webcam C920" (no "logitech" word)
    # Try /dev/videoN directly, skip whichever index Insta360 already claimed
    if logi_idx < 0:
        for try_idx in [0, 1, 2, 3]:
            if try_idx == insta_idx:
                continue
            dev = f'/dev/video{try_idx}'
            if not os.path.exists(dev):
                continue
            cap_test = cv2.VideoCapture(dev)
            if cap_test.isOpened():
                ret, frm = cap_test.read()
                cap_test.release()
                if ret and frm is not None and frm.size > 0:
                    logi_idx = try_idx
                    print(f'  [fallback] Logitech found at /dev/video{try_idx}')
                    break

    # Fallback 2: hardcoded known-good positions for this Jetson
    # (Insta360 X3 = video2, Logitech C920 = video0 — from v4l2-ctl --list-devices)
    if insta_idx < 0:
        print('  [fallback] Using hardcoded insta_idx=2')
        insta_idx = 2
    if logi_idx < 0:
        print('  [fallback] Using hardcoded logi_idx=0')
        logi_idx = 0

    print(f'  Insta360 index: {insta_idx}  (/dev/video{insta_idx})')
    print(f'  Logitech index: {logi_idx}  (/dev/video{logi_idx})')

    cap_insta, cap_logi = None, None

    if insta_idx >= 0:
        cap_insta = cv2.VideoCapture(f'/dev/video{insta_idx}')
        if cap_insta.isOpened():
            cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
            cap_insta.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print(f'  Insta360  -> /dev/video{insta_idx}  OK')
        else:
            print(f'  Insta360  -> /dev/video{insta_idx}  OPEN FAILED (camera in use?)')
            cap_insta = None

    if logi_idx >= 0:
        cap_logi = cv2.VideoCapture(f'/dev/video{logi_idx}')
        if cap_logi.isOpened():
            cap_logi.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap_logi.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print(f'  Logitech  -> /dev/video{logi_idx}  OK')
        else:
            print(f'  Logitech  -> /dev/video{logi_idx}  OPEN FAILED (camera in use?)')
            cap_logi = None

    # ── Arduino ───────────────────────────────────────────────────────────────
    ard = Arduino()
    pan, tilt = 90, 90
    ard.home()
    step = 5

    # ── Class selection ───────────────────────────────────────────────────────
    cls = select_class()
    if cls is None:
        return

    # ── Window ────────────────────────────────────────────────────────────────
    WIN = 'YOLO Dataset Collector — press q to quit'
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 420)

    flash_timer = 0
    n_captured  = 0
    print(f'\n  Collecting: {cls}')
    print('  Window open — use keyboard controls')

    while True:
        # Read frames
        fi, fl = None, None
        if cap_insta and cap_insta.isOpened():
            ret, fi = cap_insta.read()
            if not ret: fi = None
        if cap_logi and cap_logi.isOpened():
            ret, fl = cap_logi.read()
            if not ret: fl = None

        # Build side-by-side canvas
        left  = fi.copy() if fi is not None else __import__('numpy').zeros((360,640,3), dtype='uint8')
        right = fl.copy() if fl is not None else __import__('numpy').zeros((360,640,3), dtype='uint8')
        left  = cv2.resize(left,  (640, 360))
        right = cv2.resize(right, (640, 360))

        cv2.putText(left,  'INSTA360', (8, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED,   2)
        cv2.putText(right, 'LOGITECH', (8, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.6, CYAN,  2)

        canvas = __import__('numpy').hstack([left, right])
        canvas = __import__('numpy').vstack([
            __import__('numpy').zeros((50, 1280, 3), dtype='uint8'),
            canvas,
            __import__('numpy').zeros((34, 1280, 3), dtype='uint8'),
        ])

        n_i, n_l = count_imgs(cls)
        draw_hud(canvas, cls, pan, tilt, step, n_i, n_l, flash=flash_timer > 0)
        if flash_timer > 0: flash_timer -= 1

        # Capture flash indicator
        if flash_timer > 3:
            h, w = canvas.shape[:2]
            cv2.rectangle(canvas, (0,0),(w,h), (0,200,200), 6)
            cv2.putText(canvas, 'CAPTURED!', (500, 220),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,100), 4)

        cv2.imshow(WIN, canvas)

        key = cv2.waitKey(30) & 0xFF

        # ── Quit ─────────────────────────────────────────────────────────────
        if key == ord('q'):
            break

        # ── Change class ─────────────────────────────────────────────────────
        elif key == ord('c'):
            cv2.destroyAllWindows()
            cls = select_class()
            if cls is None:
                break
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 1280, 420)
            print(f'\n  Collecting: {cls}')

        # ── Home ─────────────────────────────────────────────────────────────
        elif key == ord('h'):
            pan, tilt = 90, 90
            ard.home()

        # ── Step size ─────────────────────────────────────────────────────────
        elif key in (ord('+'), ord('=')):
            step = min(step + 1, 15)
        elif key == ord('-'):
            step = max(step - 1, 1)

        # ── Pan ───────────────────────────────────────────────────────────────
        elif key == 81 or key == 2424832:   # Left arrow
            pan = ard.pan(pan - step)
        elif key == 83 or key == 2555904:   # Right arrow
            pan = ard.pan(pan + step)

        # ── Tilt ──────────────────────────────────────────────────────────────
        elif key == 82 or key == 2490368:   # Up arrow
            tilt = ard.tilt(tilt - step)    # less = servo looks up
        elif key == 84 or key == 2621440:   # Down arrow
            tilt = ard.tilt(tilt + step)

        # ── CAPTURE ──────────────────────────────────────────────────────────
        elif key == 32:   # SPACE
            # Flush buffers — read a few frames first to get the latest
            for _ in range(3):
                if cap_insta: cap_insta.read()
                if cap_logi:  cap_logi.read()
            ret_i, fi_cap = (cap_insta.read() if cap_insta else (False, None))
            ret_l, fl_cap = (cap_logi.read()  if cap_logi  else (False, None))
            if not ret_i: fi_cap = None
            if not ret_l: fl_cap = None

            saved = save_both(fi_cap, fl_cap, cls, n_captured)
            n_captured += 1
            flash_timer = 10
            print(f'  [{n_captured}] Saved: {", ".join(saved)}  (class={cls})')

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if cap_insta: cap_insta.release()
    if cap_logi:  cap_logi.release()
    cv2.destroyAllWindows()
    ard.home()
    time.sleep(0.5)
    ard.close()

    # ── Final summary ─────────────────────────────────────────────────────────
    print('\n══════════════════════════════════════════')
    print('  COLLECTION COMPLETE')
    print('══════════════════════════════════════════')
    for c in CLASSES:
        n_i, n_l = count_imgs(c)
        print(f'  {c:<24}  insta={n_i:3d}  logi={n_l:3d}')
    print(f'\n  Output: {OUT_DIR}')
    print('\n  SCP to laptop:')
    print(f'  rsync -avz rgen@192.168.8.181:{OUT_DIR}/ \\')
    print(f'    /home/dinethra/Jetson_orin_nano/data/yolo_dataset/')
    print('══════════════════════════════════════════\n')

if __name__ == '__main__':
    main()
