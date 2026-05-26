import cv2
from ultralytics import YOLO
from pathlib import Path

# ── Paths & config ────────────────────────────────────────────
BEST_PT     = Path(r'C:\Users\Rebecca Fernando\Desktop\Dinethra\Yolov11s\runs\yolo26s_exp\weights\best.pt')
CLASS_NAMES = ['door', 'extinguisher', 'gauge', 'person']

# Colours per class (BGR)
CLASS_COLORS = {
    'door'         : (255, 140,   0),
    'extinguisher' : (  0,   0, 255),
    'gauge'        : (  0, 255,   0),
    'person'       : (255,   0, 255),
}

CONF_THRESHOLD = 0.30 # lower = more detections, higher = fewer false positives
WEBCAM_INDEX   = 0      # 0 = default webcam, change to 1/2 if needed

# ─────────────────────────────────────────────────────────────
def main():
    print("=" * 55)
    print("  YOLO26s Real-time Webcam Detection")
    print("=" * 55)
    print(f"  Weights : {BEST_PT}")
    print(f"  Classes : {CLASS_NAMES}")
    print(f"  Conf    : {CONF_THRESHOLD}")
    print("=" * 55)
    print("  Press  Q  to quit")
    print("  Press  S  to save screenshot")
    print("  Press  +  to increase confidence")
    print("  Press  -  to decrease confidence")
    print("=" * 55)

    if not BEST_PT.exists():
        print(f"\n❌ ERROR: best.pt not found at:\n   {BEST_PT}")
        return

    model = YOLO(str(BEST_PT))
    cap   = cv2.VideoCapture(WEBCAM_INDEX)

    if not cap.isOpened():
        print(f"❌ ERROR: Cannot open webcam (index={WEBCAM_INDEX})")
        return

    # Set webcam resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    conf     = CONF_THRESHOLD
    shot_num = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame from webcam.")
            break

        # ── Run inference ─────────────────────────────────────
        results = model(frame, imgsz=640, conf=conf, device=0, verbose=False)
        annotated = results[0].plot()

        # ── Count per-class detections ────────────────────────
        boxes = results[0].boxes
        counts = {name: 0 for name in CLASS_NAMES}
        if boxes is not None:
            for cls_id in boxes.cls.int().tolist():
                if cls_id < len(CLASS_NAMES):
                    counts[CLASS_NAMES[cls_id]] += 1

        # ── Overlay info panel ────────────────────────────────
        overlay_lines = [
            f"YOLO26s   conf={conf:.2f}",
            f"Total detections: {len(boxes) if boxes else 0}",
        ] + [f"  {k}: {v}" for k, v in counts.items() if v > 0]

        y0 = 30
        for line in overlay_lines:
            cv2.putText(annotated, line,
                        (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2, cv2.LINE_AA)
            y0 += 28

        cv2.putText(annotated, "Q=quit  S=screenshot  +/-=conf",
                    (10, annotated.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        cv2.imshow("YOLO26s — Real-time Detection", annotated)

        # ── Key handling ──────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:        # Q or ESC
            print("\nQuitting...")
            break
        elif key == ord('s'):                   # S = screenshot
            shot_num += 1
            fname = f"screenshot_{shot_num:03d}.jpg"
            cv2.imwrite(fname, annotated)
            print(f"  📸 Saved: {fname}")
        elif key == ord('+') or key == ord('='):
            conf = min(0.95, round(conf + 0.05, 2))
            print(f"  Conf → {conf:.2f}")
        elif key == ord('-'):
            conf = max(0.05, round(conf - 0.05, 2))
            print(f"  Conf → {conf:.2f}")

    cap.release()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == '__main__':
    main()
