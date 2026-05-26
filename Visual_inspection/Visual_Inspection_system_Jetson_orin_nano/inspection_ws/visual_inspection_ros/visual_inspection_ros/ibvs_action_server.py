#!/usr/bin/env python3
"""
ibvs_action_server.py -- ROS2 Action Server: Visual inspection pipeline.

Pipeline:
  Stage 1  DETECT (Insta360 only, up to 20s):
    - YOLO on Insta360 → detect object
    - degree-4 calibration formula → coarse servo angles
    - pan-tilt moves → stop Insta360 YOLO

  Stage 2  IBVS (Logitech only, time-based 40s total):
    - YOLO on Logitech → PID servo loop until err < 10px
    - Object can disappear for up to 3s (noise/occlusion) without aborting
    - After 40s total, abort with ibvs_timeout

  Capture
    - 4 images from Logitech once centred
    - Publish via MQTT

Debug (view in RViz2):
  /visual_inspection/debug   -- combined side-by-side view with arrow, mode, FPS
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from visual_inspection_interfaces.action import InspectObjects

import cv2
import numpy as np
import time
import os
import threading
import json
import yaml
from pathlib import Path
from datetime import datetime


# ---------------------------------------------------------------------------
# Calibration -- degree-4 polynomial (from calibration_config.py)
# ---------------------------------------------------------------------------

def calculate_pan(x, y):
    val = 155.6668858701
    val += 0.4560199871 * x + 0.7342655133 * y
    val += -0.0035976159 * (x**2) + -0.0043827793 * (x*y) + -0.0084661672 * (y**2)
    val += 0.0000057403 * (x**3) + 0.0000133400 * (x**2*y) + 0.0000101783 * (x*y**2) + 0.0000536484 * (y**3)
    val += -0.0000000029*(x**4) + -0.0000000106*(x**3*y) + -0.0000000180*(x**2*y**2) + 0.0000000082*(x*y**3) + -0.0000001569*(y**4)
    return float(np.clip(val, 0, 180))


def calculate_tilt(x, y):
    val = 113.6717028380
    val += 0.3335993613 * x + -0.6916488737 * y
    val += -0.0008407415 * (x**2) + -0.0009024326 * (x*y) + 0.0084145572 * (y**2)
    val += 0.0000005454 * (x**3) + 0.0000028613 * (x**2*y) + 0.0000022977 * (x*y**2) + -0.0000683027 * (y**3)
    val += 0.0000000001*(x**4) + -0.0000000038*(x**3*y) + 0.0000000088*(x**2*y**2) + -0.0000000351*(x*y**3) + 0.0000002091*(y**4)
    return float(np.clip(val, 20, 160))


def clamp(val, lo=0, hi=180):
    return int(max(lo, min(hi, round(val))))


# ---------------------------------------------------------------------------
# YOLO loader
# ---------------------------------------------------------------------------

def load_yolo(engine_path):
    try:
        from ultralytics import YOLO
        if os.path.exists(engine_path):
            model = YOLO(engine_path)
            print(f'[ibvs_action_server] YOLO TensorRT: {engine_path}')
        else:
            pt = engine_path.replace('.engine', '.pt')
            model = YOLO(pt)
            print(f'[ibvs_action_server] YOLO PT fallback: {pt}')
        return model
    except Exception as e:
        print(f'[ibvs_action_server] YOLO load failed: {e}')
        return None


# ---------------------------------------------------------------------------
# Action Server
# ---------------------------------------------------------------------------

class IBVSActionServer(Node):

    # Paths
    ENGINE_PATH     = os.path.expanduser('~/Documents/Visual_Inspection_ws/weights/yolov26s.engine')
    MQTT_CFG_PATH   = os.path.expanduser('~/Documents/Visual_Inspection_ws/config/mqtt_config.yaml')

    # Known object classes the BT can request (BT name → YOLO class name)
    # ACTUAL YOLO MODEL NAMES: {0: 'door', 1: 'extinguisher', 2: 'gauge', 3: 'person'}
    # BT sends human-readable names; we normalize them to match YOLO class names
    KNOWN_CLASSES = {
        'fire_extinguisher' : 'extinguisher',
        'fire extinguisher' : 'extinguisher',
        'extinguisher'      : 'extinguisher',
        'door'              : 'door',
        'person'            : 'person',
        'people'            : 'person',
        'gauge'             : 'gauge',
        'pressure_gauge'    : 'gauge',
        'main_cylinder'     : 'main_cylinder',
        'unknown'           : 'unknown',
        'any'               : '',
        ''                  : '',
    }

    # Classes that only need overview capture (no IBVS, no pan-tilt)
    # Servo goes home, captures BOTH Insta360 + Logitech overview
    OVERVIEW_ONLY_CLASSES = {'unknown', 'main_cylinder'}

    # YOLO-trained target classes (full IBVS pipeline)
    IBVS_CLASSES = {'extinguisher', 'door', 'person', 'gauge'}

    # Per-class confidence thresholds
    # gauge is weak — lower threshold so we don't miss it
    CLASS_CONF = {
        'extinguisher': 0.7,
        'door':         0.5,
        'person':       0.6,
        'gauge':        0.3,
    }
    CONF_DEFAULT    = 0.5   # fallback for unknown classes
    CONF_YOLO_MIN   = 0.3   # YOLO runs at lowest needed threshold, post-filter per-class
    CONF_IBVS       = 0.3   # IBVS detection (Logitech) — run low, post-filter per-class

    # Timeouts
    INSTA_SEARCH_TIMEOUT  = 20.0  # max wait for initial detection on Insta360
    GAUGE_INSTA_TIMEOUT   = 8.0   # shorter timeout for gauge before sweep scan
    LOGI_FIRST_DET_WAIT   = 5.0   # wait for first detection in Logitech after coarse
    IBVS_TOTAL_TIMEOUT    = 40.0  # total IBVS time budget (seconds)
    IBVS_LOST_PATIENCE    = 3.0   # how long object can vanish during IBVS

    # Timing
    COARSE_WAIT  = 2.0   # seconds for servo to reach coarse position

    # IBVS PID (KP/KI/KD from ibvs_pipeline.py)
    KP = 0.12;  KI = 0.002;  KD = 0.02
    MAX_STEP_DEG = 3.0
    SLOW_ZONE_PX = 15.0
    DEADBAND_DEG = 0.5
    IBVS_TOL_PX  = 10.0

    # Logitech frame info (640x480)
    CX_LOGI = 320.0;  CY_LOGI = 240.0
    FX_LOGI = 640.0;  FY_LOGI = 640.0

    # Capture
    IMAGES_PER_OBJ = 3
    CAPTURE_DELAY  = 0.5
    FOCUS_WAIT     = 10.0  # seconds to wait after IBVS for autofocus before capture
    KEEP_LOCAL     = True  # True = always keep images locally (dataset mode)
                           # False = delete after successful MQTT send

    # Tilt servo mounted in reverse -- flip to 180-tilt
    TILT_REVERSED  = True

    # Insta360 frame: top half = FRONT (mapped to pan-tilt), bottom half = BACK
    # Objects with cy < FRONT_Y_MAX are on the front side of the robot
    INSTA_H        = 360
    FRONT_Y_MAX    = 200   # pixels -- below this = front zone, above = back zone

    # Debug frame size
    DEBUG_W = 640;  DEBUG_H = 360  # each camera panel

    def __init__(self):
        super().__init__('ibvs_action_server')
        self.bridge   = CvBridge()
        self.cb_group = ReentrantCallbackGroup()

        # Latest frames
        self._lock_insta  = threading.Lock()
        self._lock_logi   = threading.Lock()
        self._frame_insta = None
        self._frame_logi  = None

        # Subscribers
        self.create_subscription(Image, '/visual_inspection/insta360/image_raw',
                                 self._cb_insta, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/visual_inspection/logitech/image_raw',
                                 self._cb_logi,  10, callback_group=self.cb_group)

        # Publishers
        self.servo_pub      = self.create_publisher(Int16MultiArray, '/servo/pan_tilt', 10)
        self.debug_pub      = self.create_publisher(Image,           '/visual_inspection/debug', 10)
        self.status_pub     = self.create_publisher(String,          '/visual_inspection/status', 10)
        self.ibvs_err_pub   = self.create_publisher(Point,           '/visual_inspection/ibvs_error', 10)
        self.detections_pub = self.create_publisher(String,          '/visual_inspection/detections', 10)

        # MQTT config
        self._mqtt_cfg = self._load_mqtt_cfg()

        # YOLO
        self.get_logger().info('Loading YOLO model...')
        self.model = load_yolo(self.ENGINE_PATH)
        if self.model:
            self.get_logger().info('YOLO model ready')
        else:
            self.get_logger().error('YOLO load failed')

        # State visible to debug timer
        self._yolo_lock    = threading.Lock()
        self._mode         = 'IDLE'
        self._target_class = ''          # '' = any class; set per goal from BT
        self._ibvs_ex      = 0.0
        self._ibvs_ey      = 0.0
        self._ibvs_iter    = 0
        self._ibvs_err     = 0.0
        self._ibvs_time_s  = 0.0
        self._ibvs_fps     = 0.0
        self._coarse_time_s= 0.0
        self._pipeline_start = 0.0
        self._initial_ibvs_err = 0.0   # coarse accuracy: IBVS pixel error at iteration 0
        self._fps_counter  = 0
        self._fps_time     = time.time()
        self._fps          = 0.0
        self._goal_active  = False

        # Action server
        self._action_server = ActionServer(
            self, InspectObjects, '/visual_inspection/inspect_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # Debug timer (4Hz)
        self.create_timer(0.25, self._debug_timer_cb)

        self.get_logger().info('Action server ready at /visual_inspection/inspect_objects')
        self.get_logger().info('Debug topic: /visual_inspection/debug  (add Image in RViz2)')
        self.get_logger().info('Status topics: /visual_inspection/status  /visual_inspection/ibvs_error  /visual_inspection/detections')
        broker = self._mqtt_cfg.get('broker', 'localhost')
        self.get_logger().info(f'MQTT broker: {broker}:{self._mqtt_cfg.get("port", 1883)}')

    # ---- Camera callbacks ---------------------------------------------------

    def _cb_insta(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock_insta:
            self._frame_insta = frame

    def _cb_logi(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock_logi:
            self._frame_logi = frame

    def _get_insta(self):
        with self._lock_insta:
            return self._frame_insta.copy() if self._frame_insta is not None else None

    def _get_logi(self):
        with self._lock_logi:
            return self._frame_logi.copy() if self._frame_logi is not None else None

    # ---- MQTT config loader -------------------------------------------------

    def _load_mqtt_cfg(self):
        try:
            with open(self.MQTT_CFG_PATH) as f:
                cfg = yaml.safe_load(f)
            self.get_logger().info(f'MQTT config loaded: {self.MQTT_CFG_PATH}')
            return cfg
        except Exception as e:
            self.get_logger().warn(f'MQTT config not found ({e}) -- using localhost defaults')
            return {
                'broker': 'localhost', 'port': 1883,
                'access_token': '', 'topic': 'visual_inspection/images',
                'timeout': 10, 'qos': 1, 'tls': False,
                'jpeg_quality': 85,
                'capture_dir': '~/Documents/Visual_Inspection_ws/captures'
            }

    # ---- Goal / cancel -------------------------------------------------------

    def goal_callback(self, goal_request):
        self.get_logger().info('Goal received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ---- Servo helpers -------------------------------------------------------

    def _servo(self, tilt, pan):
        """Send servo command. Tilt is flipped if TILT_REVERSED."""
        t = 180 - clamp(tilt) if self.TILT_REVERSED else clamp(tilt)
        msg = Int16MultiArray()
        msg.data = [t, clamp(pan)]
        self.servo_pub.publish(msg)

    def _home(self):
        self._servo(90, 90)
        self.get_logger().info('Servos home (90, 90)')

    # ---- YOLO detection -----------------------------------------------------

    # YOLO input resize — TRT engine compiled for 640x360 (same as original ibvs_pipeline.py)
    YOLO_INPUT_H = 360

    def _detect_raw(self, frame, conf):
        """Run YOLO. Returns (detections, annotated_frame). Call with lock held.
        det tuple: (cx, cy, x1, y1, x2, y2, conf, cls_name)
        
        FIX: Original ibvs_pipeline.py ran YOLO on cv2.resize(frame_logi, (640,360)).
        TensorRT engine was compiled for that input size. Running on raw 640x480 caused
        bad detections. We resize to YOLO_INPUT_H=360, run YOLO, then scale coords back.
        """
        if self.model is None or frame is None:
            return [], frame.copy() if frame is not None else None

        h_orig, w_orig = frame.shape[:2]

        # Resize to match TRT engine compiled size (640x360)
        if h_orig != self.YOLO_INPUT_H:
            yolo_frame = cv2.resize(frame, (w_orig, self.YOLO_INPUT_H))
            scale_y    = h_orig / self.YOLO_INPUT_H   # 480/360 = 1.333
            scale_x    = 1.0                           # width unchanged (both 640)
        else:
            yolo_frame = frame
            scale_y    = 1.0
            scale_x    = 1.0

        results = self.model(yolo_frame, verbose=False, conf=conf)[0]
        debug   = frame.copy()   # annotate on original resolution frame
        dets    = []

        for box in results.boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

            # Scale coords back to original frame size
            x1 = int(x1 * scale_x);  x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y);  y2 = int(y2 * scale_y)

            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            c  = float(box.conf[0])
            cls_id   = int(box.cls[0])
            cls_name = results.names.get(cls_id, str(cls_id)) if hasattr(results, 'names') else str(cls_id)
            dets.append((cx, cy, x1, y1, x2, y2, c, cls_name))
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug, f'{cls_name} {c:.2f}', (x1, max(y1-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        dets.sort(key=lambda d: d[6], reverse=True)

        # Post-filter: apply per-class confidence thresholds
        filtered = []
        for d in dets:
            cls = d[7].lower()
            min_conf = self.CLASS_CONF.get(cls, self.CONF_DEFAULT)
            if d[6] >= min_conf:
                filtered.append(d)
        dets = filtered

        # Filter to target class if specified
        if self._target_class:
            tc = self._target_class.lower()
            before = len(dets)
            dets = [d for d in dets if d[7].lower() == tc]
            if before > 0 and len(dets) == 0:
                self.get_logger().debug(
                    f'  [filter] {before} dets → 0 after class filter "{tc}" '
                    f'(classes seen: {[d[7] for d in filtered]})')

        h, w = frame.shape[:2]
        cv2.line(debug, (w//2, 0), (w//2, h), (80, 80, 200), 1)
        cv2.line(debug, (0, h//2), (w, h//2), (80, 80, 200), 1)

        if dets:
            cx, cy = dets[0][0], dets[0][1]
            cv2.circle(debug, (int(cx), int(cy)), 6, (0, 0, 255), -1)

        return dets, debug


    def _detect_insta(self, conf=None):
        """Detect on Insta360 using YOLO tracking (ByteTrack).
        Returns (front_dets, back_dets, raw_frame, annotated_frame).
        Detections sorted by tracker ID for consistent ordering."""
        frame = self._get_insta()
        if frame is None:
            return [], [], None, None
        c = conf or self.CONF_YOLO_MIN
        with self._yolo_lock:
            # Use track() for ByteTrack assignment
            try:
                results = self.model.track(frame, verbose=False, conf=c,
                                           persist=True, tracker='bytetrack.yaml')[0]
            except Exception:
                # Fallback to regular detect if tracker fails
                results = self.model(frame, verbose=False, conf=c)[0]

            debug = frame.copy()
            front = []  # objects in front half
            back  = []  # objects in back half
            h, w  = frame.shape[:2]

            for box in results.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                cx   = (x1 + x2) / 2.0
                cy_b = (y1 + y2) / 2.0
                c_   = float(box.conf[0])
                tid      = int(box.id[0]) if box.id is not None else -1
                cls_id   = int(box.cls[0])
                cls_name = results.names.get(cls_id, str(cls_id)) if hasattr(results, 'names') else str(cls_id)

                is_front = cy_b < self.FRONT_Y_MAX
                color    = (0, 255, 0) if is_front else (0, 100, 255)
                label    = f'[{cls_name}] ID{tid} {c_:.2f}' + ('' if is_front else ' [BACK]')

                cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
                cv2.putText(debug, label, (x1, max(y1-5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                cv2.circle(debug, (int(cx), int(cy_b)), 5, (0, 0, 255), -1)

                det = (cx, cy_b, x1, y1, x2, y2, c_, tid, cls_name)
                if is_front:
                    front.append(det)
                else:
                    back.append(det)

            # Draw front/back boundary line
            cv2.line(debug, (0, self.FRONT_Y_MAX), (w, self.FRONT_Y_MAX), (0, 200, 255), 1)
            cv2.putText(debug, 'FRONT', (w-65, self.FRONT_Y_MAX-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
            cv2.putText(debug, 'BACK', (w-55, self.FRONT_Y_MAX+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 1)

            # Crosshair
            cv2.line(debug, (w//2, 0), (w//2, h), (80, 80, 200), 1)
            cv2.line(debug, (0, h//2), (w, h//2), (80, 80, 200), 1)

            # Sort front by track ID for consistent ordering
            front.sort(key=lambda d: d[7] if d[7] >= 0 else d[6])
            back.sort( key=lambda d: d[7] if d[7] >= 0 else d[6])

            # Post-filter: per-class confidence thresholds
            for lst in (front, back):
                lst[:] = [d for d in lst
                          if d[6] >= self.CLASS_CONF.get(d[8].lower(), self.CONF_DEFAULT)]

            # Filter to target class if specified
            if self._target_class:
                tc = self._target_class.lower()
                before_f, before_b = len(front), len(back)
                front = [d for d in front if d[8].lower() == tc]
                back  = [d for d in back  if d[8].lower() == tc]
                if (before_f + before_b) > 0 and len(front) + len(back) == 0:
                    self.get_logger().debug(
                        f'  [insta filter] {before_f+before_b} dets → 0 after "{tc}"')

        return front, back, frame, debug

    def _detect_logi(self, conf=None):
        """Detect on Logitech. Returns (dets, raw_frame, annotated_frame)."""
        frame = self._get_logi()
        if frame is None:
            return [], None, None
        c = conf or self.CONF_IBVS
        with self._yolo_lock:
            dets, dbg = self._detect_raw(frame, c)
            # Draw IBVS arrow + class label if detection found
            if dets:
                h, w = frame.shape[:2]
                cx_d, cy_d = int(dets[0][0]), int(dets[0][1])
                cx_f, cy_f = w // 2, h // 2
                cv2.arrowedLine(dbg, (cx_d, cy_d), (cx_f, cy_f), (255, 0, 0), 2)
                cls_lbl = dets[0][7] if len(dets[0]) > 7 else ''
                cv2.putText(dbg, str(cls_lbl), (cx_d+5, cy_d-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        return dets, frame, dbg


    # ---- Combined debug publisher -------------------------------------------

    def _build_debug_frame(self, insta_dbg, logi_dbg):
        """Build combined side-by-side debug image (1280 x 360)."""
        W, H = self.DEBUG_W, self.DEBUG_H

        # Left: Insta360
        if insta_dbg is not None:
            left = cv2.resize(insta_dbg, (W, H))
        else:
            left = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.putText(left, 'INSTA360 (Coarse)', (10, H-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Right: Logitech
        if logi_dbg is not None:
            right = cv2.resize(logi_dbg, (W, H))
        else:
            right = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.putText(right, 'LOGITECH (IBVS)', (10, H-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # IBVS overlay on right
        if self._mode == 'IBVS':
            cv2.putText(right,
                        f'IBVS: err=({self._ibvs_ex:.1f},{self._ibvs_ey:.1f})  '
                        f'Iter={self._ibvs_iter}  px={self._ibvs_err:.1f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        combined = np.hstack([left, right])

        # Mode + FPS top-left
        cv2.putText(combined, f'Mode: {self._mode}   FPS: {self._fps:.1f}',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return combined

    def _publish_debug(self, insta_dbg, logi_dbg):
        frame = self._build_debug_frame(insta_dbg, logi_dbg)
        msg   = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_pub.publish(msg)

        # FPS counter
        self._fps_counter += 1
        now = time.time()
        if now - self._fps_time >= 2.0:
            self._fps      = self._fps_counter / (now - self._fps_time)
            self._fps_counter = 0
            self._fps_time    = now

    def _debug_timer_cb(self):
        """4Hz timer — publishes combined debug when no active goal.
        Applies per-class confidence thresholds so debug view only shows
        valid detections (e.g. person must be >=0.5, gauge >=0.3)."""
        if self._goal_active:
            return
        if not self._yolo_lock.acquire(blocking=False):
            return
        try:
            insta_f = self._get_insta()
            logi_f  = self._get_logi()
            # Run YOLO at lowest threshold, post-filter per-class
            raw_insta, insta_dbg = self._detect_raw(insta_f, self.CONF_YOLO_MIN) \
                if insta_f is not None else ([], None)
            raw_logi,  logi_dbg  = self._detect_raw(logi_f,  self.CONF_YOLO_MIN) \
                if logi_f  is not None else ([], None)
        finally:
            self._yolo_lock.release()

        # Rebuild debug frames with per-class filtered boxes only
        if insta_f is not None:
            insta_dbg = self._draw_filtered(insta_f, raw_insta)
        if logi_f is not None:
            logi_dbg = self._draw_filtered(logi_f, raw_logi)

        self._publish_debug(insta_dbg, logi_dbg)

    def _draw_filtered(self, frame, dets):
        """Draw only detections that pass per-class confidence threshold."""
        import cv2
        dbg = frame.copy()
        h, w = dbg.shape[:2]
        cv2.line(dbg, (w//2, 0), (w//2, h), (80, 80, 200), 1)
        cv2.line(dbg, (0, h//2), (w, h//2), (80, 80, 200), 1)
        for d in dets:
            cls  = d[7].lower()
            conf = d[6]
            min_conf = self.CLASS_CONF.get(cls, self.CONF_DEFAULT)
            if conf < min_conf:
                continue   # skip — below class threshold
            x1, y1, x2, y2 = int(d[2]), int(d[3]), int(d[4]), int(d[5])
            cx, cy = int(d[0]), int(d[1])
            cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(dbg, f'{cls} {conf:.2f}', (x1, max(y1-5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.circle(dbg, (cx, cy), 5, (0, 0, 255), -1)
        return dbg

    # ---- Gauge sweep scan (fallback when Insta360 misses gauge) --------------

    # Gauge sweep — smooth serpentine: tilt=[40,70,100], pan=[20↔160]
    SWEEP_TILTS      = [40, 70, 100]   # tilt row positions (degrees)
    SWEEP_PAN_RANGE  = (20, 160)       # pan extent
    SWEEP_INTERP_HZ  = 30              # servo update rate during sweep
    SWEEP_DEG_PER_S  = 40.0            # degrees per second (smooth speed)
    SWEEP_CHECK_HZ   = 10              # detection checks per second during sweep

    def _sweep_move(self, from_tilt, from_pan, to_tilt, to_pan,
                    check_fn, check_cancel_fn):
        """
        Smoothly interpolate servo from (from_tilt,from_pan) to (to_tilt,to_pan)
        at SWEEP_DEG_PER_S degrees/s. Calls check_fn() every 1/SWEEP_CHECK_HZ s.
        Returns det list immediately if check_fn() finds something, else [].
        """
        total_deg = max(abs(to_pan - from_pan), abs(to_tilt - from_tilt), 1)
        duration  = total_deg / self.SWEEP_DEG_PER_S          # seconds
        n_steps   = max(int(duration * self.SWEEP_INTERP_HZ), 1)
        check_every = max(1, self.SWEEP_INTERP_HZ // self.SWEEP_CHECK_HZ)

        for i in range(n_steps + 1):
            if check_cancel_fn():
                return None   # cancelled
            t = i / n_steps
            pan  = from_pan  + t * (to_pan  - from_pan)
            tilt = from_tilt + t * (to_tilt - from_tilt)
            self._servo(tilt, pan)

            # Check Logitech periodically during move
            if i % check_every == 0:
                dets, _, logi_dbg = self._detect_logi()
                insta_f = self._get_insta()
                self._publish_debug(insta_f, logi_dbg)
                if dets:
                    return dets

            time.sleep(1.0 / self.SWEEP_INTERP_HZ)

        return []   # reached target, nothing found

    def _gauge_sweep_scan(self, goal_handle, feedback, session_ts):
        """
        Smooth serpentine sweep for gauge:
          tilt=40 → pan 20→160
          tilt=70 → pan 160→20
          tilt=100 → pan 20→160
        Servo moves smoothly. Detection checked during motion.
        Returns Result on success, None if gauge not found.
        """
        self._mode = 'SWEEP'
        self.get_logger().info(
            '  Gauge sweep scan (smooth): tilt=[40,70,100] pan=[20↔160]')
        feedback.current_step = 'detecting'
        goal_handle.publish_feedback(feedback)

        def cancel(): return goal_handle.is_cancel_requested

        cur_tilt = 40
        cur_pan  = 20

        # Move to start position smoothly from home
        self._sweep_move(90, 90, cur_tilt, cur_pan, lambda: [], cancel)
        time.sleep(0.3)

        for sweep_idx, tilt in enumerate(self.SWEEP_TILTS):
            # Serpentine direction
            pan_end = self.SWEEP_PAN_RANGE[1] if sweep_idx % 2 == 0 \
                      else self.SWEEP_PAN_RANGE[0]

            # 1. Move tilt to row position first (current pan → same pan, new tilt)
            res = self._sweep_move(cur_tilt, cur_pan, tilt, cur_pan, lambda: [], cancel)
            if res is None:
                return None  # cancelled
            cur_tilt = tilt

            # 2. Pan sweep across row
            res = self._sweep_move(cur_tilt, cur_pan, cur_tilt, pan_end, lambda: [], cancel)
            if res is None:
                return None  # cancelled

            # Check if detection happened during sweep
            dets, _, logi_dbg = self._detect_logi()
            insta_f = self._get_insta()
            self._publish_debug(insta_f, logi_dbg)

            if res or dets:
                found_dets = res if res else dets
                self.get_logger().info(
                    f'  SWEEP: gauge found at approx tilt={tilt} '
                    f'pan={cur_pan}→{pan_end}! Starting IBVS')
                self._pub_status('IBVS')

                # Use current servo position as IBVS start
                centred = self._ibvs(goal_handle, cur_pan, cur_tilt,
                                     feedback, 1)
                if centred:
                    self._pub_status('CAPTURING')
                    self.get_logger().info(
                        f'  Waiting {self.FOCUS_WAIT}s for autofocus...')
                    time.sleep(self.FOCUS_WAIT)

                    conf = found_dets[0][6] if found_dets else 0.3
                    paths = self._capture(
                        self.IMAGES_PER_OBJ, obj_id=1,
                        session_ts=session_ts, cls_name='gauge',
                        conf_score=conf)
                    overview_paths = self._capture_insta_overview(
                        session_ts, 'sweep_gauge', count=1,
                        folder='inspection', obj_cls='gauge', obj_id=1)

                    all_paths = paths + overview_paths
                    self._mqtt(all_paths, 1, session_ts=session_ts,
                               cls_name='gauge')
                    self.get_logger().info(
                        f'  Sweep gauge done: {len(all_paths)} images')

                    result = InspectObjects.Result()
                    result.success = True
                    result.objects_inspected = 1
                    result.objects_found = 1
                    result.object_in_back = False
                    result.failed_reason = ''
                    self._pub_status('IDLE')
                    self._goal_active = False
                    self._home()
                    goal_handle.succeed()
                    return result
                else:
                    self.get_logger().warn('  Sweep IBVS did not converge — continuing')

            cur_pan = pan_end

        self.get_logger().warn('  Gauge sweep complete — gauge not found')
        return None

    # ---- Stage 1: search Insta360 -------------------------------------------

    def _search_insta(self, goal_handle):
        """Search Insta360 up to timeout.
        For gauge: uses shorter timeout (GAUGE_INSTA_TIMEOUT) before sweep scan.
        Returns (front_dets, back_dets) or (None, None) on timeout."""
        self._mode = 'DETECTING'
        # Gauge gets shorter timeout — we'll fall back to sweep scan
        timeout = (self.GAUGE_INSTA_TIMEOUT
                   if self._target_class == 'gauge'
                   else self.INSTA_SEARCH_TIMEOUT)
        self.get_logger().info(f'  Searching Insta360 (up to {timeout}s)...')
        deadline = time.time() + timeout

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return None, None

            front, back, _, insta_dbg = self._detect_insta()
            logi_f = self._get_logi()
            self._publish_debug(insta_dbg, logi_f)

            if front or back:
                n_front = len(front)
                n_back  = len(back)
                self.get_logger().info(
                    f'  Detected: {n_front} front object(s), {n_back} back object(s)')
                if back and not front:
                    self.get_logger().info('  All objects in BACK -- signalling BT to rotate')
                return front, back

            time.sleep(0.1)

        self.get_logger().warn(f'  No objects found on Insta360 within {timeout}s')
        return None, None

    # ---- Stage 2: IBVS on Logitech ------------------------------------------

    def _ibvs(self, goal_handle, start_pan, start_tilt, feedback, obj_idx):
        """IBVS on Logitech. Time-based 40s budget. Object can vanish 3s."""
        self._mode = 'IBVS'
        self.get_logger().info(f'  IBVS starting from pan={start_pan:.1f} tilt={start_tilt:.1f}')

        # PID state
        integral_pan = integral_tilt = 0.0
        prev_ep = prev_et = 0.0
        pan  = float(start_pan)
        tilt = float(start_tilt)
        dt   = 0.066  # ~15Hz

        last_seen_time = time.time()
        start_time     = time.time()
        ibvs_iter      = 0
        n_det_frames   = 0                 # count frames where object was detected
        insta_dbg_cache = None

        _, _, _, insta_dbg_cache = self._detect_insta()
        deadline = start_time + self.IBVS_TOTAL_TIMEOUT

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                self._ibvs_time_s = time.time() - start_time
                return False

            dets, _, logi_dbg = self._detect_logi()
            self._publish_debug(insta_dbg_cache, logi_dbg)

            now = time.time()

            if not dets:
                if now - last_seen_time > self.IBVS_LOST_PATIENCE:
                    self.get_logger().warn(f'  IBVS: object not seen for {now-last_seen_time:.1f}s -- aborting')
                    self._ibvs_time_s = now - start_time
                    return False
                self.get_logger().info(f'  IBVS: object not seen ({now-last_seen_time:.1f}s) -- waiting...')
                time.sleep(dt)
                continue

            last_seen_time = now
            n_det_frames  += 1

            if self._target_class:
                tc = self._target_class.lower()
                dets = [d for d in dets if d[7].lower() == tc]
                if not dets:
                    time.sleep(dt)
                    continue

            cx_d, cy_d = dets[0][0], dets[0][1]
            ex  = cx_d - self.CX_LOGI
            ey  = cy_d - self.CY_LOGI
            err = (ex**2 + ey**2) ** 0.5

            # Capture coarse accuracy — the error at the very first detection
            if ibvs_iter == 0:
                self._initial_ibvs_err = err

            self._ibvs_ex   = ex
            self._ibvs_ey   = ey
            self._ibvs_iter = ibvs_iter
            self._ibvs_err  = err

            feedback.current_step   = 'ibvs'
            feedback.current_object = obj_idx
            feedback.ibvs_error_px  = float(err)
            goal_handle.publish_feedback(feedback)

            if err < self.IBVS_TOL_PX:
                elapsed = time.time() - start_time
                self._ibvs_time_s = elapsed
                # ibvs_iter = number of PID loop steps completed
                # n_det_frames = frames where object was detected (may be 0 on instant converge)
                # Use ibvs_iter/elapsed as fps (more stable)
                if elapsed > 0:
                    self._ibvs_fps = round(
                        (n_det_frames if n_det_frames > 0 else ibvs_iter) / elapsed, 1)
                else:
                    self._ibvs_fps = 0.0
                self.get_logger().info(
                    f'  IBVS converged: err={err:.1f}px at iter {ibvs_iter} '
                    f'time={elapsed:.2f}s fps={self._ibvs_fps}')
                return True

            # Angular errors
            theta_x = np.degrees(np.arctan(ex / self.FX_LOGI))
            theta_y = np.degrees(np.arctan(ey / self.FY_LOGI))

            if abs(theta_x) < self.DEADBAND_DEG: theta_x = 0.0
            if abs(theta_y) < self.DEADBAND_DEG: theta_y = 0.0

            if abs(ex) > self.SLOW_ZONE_PX:
                theta_y = 0.0
                integral_tilt = 0.0

            integral_pan = np.clip(integral_pan + theta_x * dt, -50, 50)
            dp = -(self.KP*theta_x + self.KI*integral_pan + self.KD*(theta_x-prev_ep)/dt)
            prev_ep = theta_x

            integral_tilt = np.clip(integral_tilt + theta_y * dt, -50, 50)
            dt_ = -(self.KP*theta_y + self.KI*integral_tilt + self.KD*(theta_y-prev_et)/dt)
            prev_et = theta_y

            sp = min(1.0, abs(ex)/self.SLOW_ZONE_PX)
            st = min(1.0, abs(ey)/self.SLOW_ZONE_PX)
            dp  = np.clip(dp,  -self.MAX_STEP_DEG*sp,  self.MAX_STEP_DEG*sp)
            dt_ = np.clip(dt_, -self.MAX_STEP_DEG*st, self.MAX_STEP_DEG*st)

            pan  = float(np.clip(pan  + dp,  0,  180))
            tilt = float(np.clip(tilt + dt_, 20, 160))
            self._servo(tilt, pan)

            if ibvs_iter % 10 == 0:
                self.get_logger().info(f'  IBVS iter {ibvs_iter}: err={err:.1f}px pan={pan:.1f} tilt={tilt:.1f}')

            ibvs_iter += 1
            time.sleep(dt)

        elapsed = time.time() - start_time
        self._ibvs_time_s = elapsed
        self._ibvs_fps    = round(n_det_frames / elapsed, 1) if elapsed > 0 else 0.0
        self.get_logger().warn(f'  IBVS timeout after {elapsed:.1f}s')
        return False


    # ---- Image capture (local save) ----------------------------------------

    def _capture(self, n=4, obj_id=1, session_ts='', cls_name='object',
                 conf_score=0.0, ibvs_err=0.0, ibvs_iter=0, ibvs_converged=False,
                 location_label=''):
        """Capture n images from Logitech, save to inspection/ folder.
        Folder: captures/inspection/session_ts/cls_name/instance_N/
        Saves metadata.json with confidence + IBVS stats for evaluation CSV.
        Returns list of saved file paths."""
        base = Path(os.path.expanduser(
            self._mqtt_cfg.get('capture_dir', '~/Documents/Visual_Inspection_ws/captures')
        ))
        cap_dir = base / 'inspection' / session_ts / cls_name / f'instance_{obj_id}'
        cap_dir.mkdir(parents=True, exist_ok=True)

        paths = []
        for i in range(n):
            f = self._get_logi()
            if f is not None:
                fpath = cap_dir / f'img_{i+1:02d}_conf{conf_score:.2f}.jpg'
                q = self._mqtt_cfg.get('jpeg_quality', 85)
                cv2.imwrite(str(fpath), f, [cv2.IMWRITE_JPEG_QUALITY, q])
                paths.append(fpath)
                self.get_logger().info(f'  Saved {fpath.name}')
            time.sleep(self.CAPTURE_DELAY)

        # Real wall-clock ibvs_time_s (measured in _ibvs, not estimated)
        # Use getattr() fallbacks so stale __init__ on Jetson never causes AttributeError
        _ibvs_time     = round(float(getattr(self, '_ibvs_time_s',       0.0)), 3)
        _ibvs_fps      = round(float(getattr(self, '_ibvs_fps',           0.0)), 1)
        _coarse_time   = round(float(getattr(self, '_coarse_time_s',      0.0)), 3)
        _initial_err   = round(float(getattr(self, '_initial_ibvs_err',   0.0)), 2)
        _pipe_start    = getattr(self, '_pipeline_start', 0.0)
        _pipeline_time = round(time.time() - _pipe_start, 2) if _pipe_start > 0 else 0.0

        meta = {
            'class':            cls_name,
            'instance_id':      obj_id,
            'confidence':       round(conf_score, 4),
            'session':          session_ts,
            'location_label':   location_label,
            'num_images':       len(paths),
            'camera':           'logitech',
            'ibvs_converged':   ibvs_converged,
            'ibvs_error_px':    round(float(ibvs_err), 3),
            'ibvs_time_s':      _ibvs_time,
            'ibvs_iterations':  ibvs_iter,
            'ibvs_fps':         _ibvs_fps,
            'coarse_time_s':    _coarse_time,
            'initial_ibvs_error_px': _initial_err,
            'pipeline_time_s':  _pipeline_time,
        }

        meta_path = cap_dir / 'metadata.json'
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(
            f'  Metadata saved: conf={conf_score:.4f} '
            f'ibvs_converged={ibvs_converged} err={ibvs_err:.1f}px '
            f'time={self._ibvs_time_s}s')


        return paths


    def _capture_insta_overview(self, session_ts, location_label, count=1,
                                 folder='inspection', obj_cls='', obj_id=1):
        """Capture Insta360 RAW frames (no bounding boxes).
        For ROI inspection: folder='inspection', saved alongside ROI images.
        For BT overview request: folder='overview', standalone.
        Returns list of saved file paths."""
        base = Path(os.path.expanduser(
            self._mqtt_cfg.get('capture_dir', '~/Documents/Visual_Inspection_ws/captures')
        ))
        if folder == 'inspection':
            cap_dir = base / 'inspection' / session_ts / obj_cls / f'instance_{obj_id}'
        else:
            cap_dir = base / 'overview' / session_ts
        cap_dir.mkdir(parents=True, exist_ok=True)

        q = self._mqtt_cfg.get('jpeg_quality', 85)
        paths = []
        for i in range(count):
            # Save RAW frame — no bounding boxes
            raw = self._get_insta()
            if raw is not None:
                lbl = location_label.replace(' ', '_') or 'unknown'
                fname = f'overview_{lbl}_{i+1:02d}.jpg'
                fpath = cap_dir / fname
                cv2.imwrite(str(fpath), raw, [cv2.IMWRITE_JPEG_QUALITY, q])
                paths.append(fpath)
                self.get_logger().info(f'  Insta360 overview saved (raw): {fpath.name}')
            time.sleep(0.3)
        return paths

    # ---- MQTT (ThingsBoard token auth) + local cleanup ----------------------

    def _mqtt(self, image_paths, obj_id, session_ts='', cls_name='object'):
        """Upload images via MQTT. Keeps local files if KEEP_LOCAL=True.
        Uses ThingsBoard-style auth: username=access_token, password=empty."""
        if not image_paths:
            self.get_logger().warn('  MQTT: no images to send')
            return False
        try:
            import paho.mqtt.client as mqtt
            cfg      = self._mqtt_cfg
            broker   = cfg.get('broker', 'localhost')
            port     = cfg.get('port', 1883)
            token    = cfg.get('access_token', '')
            topic    = cfg.get('topic', 'v1/devices/me/telemetry')
            timeout  = cfg.get('timeout', 10)
            qos      = cfg.get('qos', 1)
            q_jpg    = cfg.get('jpeg_quality', 85)

            self.get_logger().info(f'  MQTT connecting to {broker}:{port}...')
            client = mqtt.Client()
            if token:
                client.username_pw_set(token, '')  # ThingsBoard: token as username
            client.connect(broker, port, timeout=timeout)
            client.loop_start()

            sent = 0
            for i, fpath in enumerate(image_paths):
                img = cv2.imread(str(fpath))
                if img is None:
                    self.get_logger().warn(f'  MQTT: cannot read {fpath}')
                    continue
                _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q_jpg])
                import base64
                payload = json.dumps({
                    'session':      session_ts,
                    'object_id':    obj_id,
                    'class_name':   cls_name,
                    'image_idx':    i + 1,
                    'total':        len(image_paths),
                    'timestamp':    time.time(),
                    'image_b64':    base64.b64encode(buf.tobytes()).decode()
                })
                result = client.publish(topic, payload, qos=qos)
                # wait_for_publish() timeout arg only in paho-mqtt >= 2.x
                try:
                    result.wait_for_publish(timeout=5)
                except TypeError:
                    result.wait_for_publish()  # paho-mqtt 1.x fallback
                sent += 1
                self.get_logger().info(f'  MQTT sent {i+1}/{len(image_paths)} (obj {obj_id})')

            client.loop_stop()
            client.disconnect()
            self.get_logger().info(
                f'  MQTT done: {sent}/{len(image_paths)} images to {broker}')

            # Delete local files only if KEEP_LOCAL is False
            if not self.KEEP_LOCAL and sent == len(image_paths):
                for fpath in image_paths:
                    try:
                        os.remove(fpath)
                    except Exception:
                        pass
                self.get_logger().info('  Local captures deleted (MQTT success, KEEP_LOCAL=False)')
                try:
                    image_paths[0].parent.rmdir()
                    image_paths[0].parent.parent.rmdir()
                except Exception:
                    pass
            elif self.KEEP_LOCAL:
                self.get_logger().info(
                    f'  Images kept locally (KEEP_LOCAL=True): {image_paths[0].parent}')
            return sent == len(image_paths)

        except Exception as e:
            self.get_logger().warn(f'  MQTT failed: {e} -- images kept locally')
            return False

    # ---- Status topic helpers -----------------------------------------------

    def _pub_status(self, mode: str):
        self._mode = mode
        self.status_pub.publish(String(data=mode))

    def _pub_detections(self, front, back):
        """Publish JSON list of detected objects to /visual_inspection/detections."""
        payload = json.dumps({
            'timestamp': time.time(),
            'front': [{'cx': d[0], 'cy': d[1], 'conf': d[6], 'track_id': d[7],
                       'class': d[8] if len(d) > 8 else 'unknown'}
                       for d in front],
            'back':  [{'cx': d[0], 'cy': d[1], 'conf': d[6], 'track_id': d[7],
                       'class': d[8] if len(d) > 8 else 'unknown'}
                       for d in back],
        })
        self.detections_pub.publish(String(data=payload))

    def _pub_ibvs_error(self, ex: float, ey: float):
        """Publish IBVS pixel error to /visual_inspection/ibvs_error."""
        p = Point()
        p.x = float(ex)
        p.y = float(ey)
        p.z = float((ex**2 + ey**2) ** 0.5)  # total error magnitude
        self.ibvs_err_pub.publish(p)


    # ---- Main execute --------------------------------------------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info('Inspection started')
        self._goal_active = True

        feedback = InspectObjects.Feedback()
        result   = InspectObjects.Result()
        max_obj        = goal_handle.request.max_objects
        ret_home       = goal_handle.request.return_home
        overview_only  = goal_handle.request.overview_only
        location_label = goal_handle.request.location_label or 'unknown'
        overview_count = goal_handle.request.overview_count or 2
        target_obj_raw = getattr(goal_handle.request, 'target_object', '') or ''

        # Normalise target_object → YOLO class name
        resolved_target = self.KNOWN_CLASSES.get(
            target_obj_raw.lower().strip(), target_obj_raw.lower().strip())

        # Decide mode: overview-only for unknown/main_cylinder, full IBVS for trained classes
        if resolved_target in self.OVERVIEW_ONLY_CLASSES:
            overview_only = True
            self._target_class = ''   # no YOLO filter — just capture overview
            self.get_logger().info(
                f'  Target "{target_obj_raw}" → OVERVIEW-ONLY mode (no pan-tilt/IBVS)')
        elif resolved_target:
            self._target_class = resolved_target
            self.get_logger().info(
                f'  Target object filter: "{target_obj_raw}" → YOLO class "{self._target_class}"')
        else:
            self._target_class = ''
            self.get_logger().info('  No target object filter — detecting all classes')

        def abort(reason, in_back=False, found=0):
            self._pub_status('IDLE')
            self._goal_active = False
            result.success           = False
            result.objects_inspected = 0
            result.objects_found     = found
            result.object_in_back    = in_back
            result.failed_reason     = reason
            if ret_home:
                self._home()
            goal_handle.abort()
            return result

        session_ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # ── OVERVIEW-ONLY MODE: for unknown/main_cylinder ──
        # Servo goes home (90,90), captures BOTH Insta360 + Logitech overview
        # No pan-tilt, no IBVS — we can't identify these objects with YOLO
        if overview_only:
            label = target_obj_raw or location_label
            self.get_logger().info(
                f'  Overview-only ({label}): servo home → capture both cameras')
            self._pub_status('OVERVIEW')
            feedback.current_step = 'overview'
            goal_handle.publish_feedback(feedback)

            # Move servo to home position
            self._home()
            time.sleep(2.0)  # wait for servo to settle

            # Capture Insta360 overview images
            insta_paths = self._capture_insta_overview(
                session_ts, label, count=overview_count, folder='overview')

            # Also capture 1 Logitech image from home position
            logi_paths = self._capture(
                n=1, obj_id=1,
                session_ts=session_ts, cls_name=label, conf_score=0.0)

            all_paths = insta_paths + logi_paths
            self.get_logger().info(
                f'  Overview done: {len(insta_paths)} Insta360 + '
                f'{len(logi_paths)} Logitech = {len(all_paths)} total')
            self._mqtt(all_paths, obj_id=0, session_ts=session_ts, cls_name=label)

            self._pub_status('IDLE')
            self._goal_active = False
            result.success           = len(all_paths) > 0
            result.objects_inspected = 0
            result.objects_found     = len(all_paths)
            result.object_in_back    = False
            result.failed_reason     = '' if all_paths else 'no_frames'
            goal_handle.succeed()
            return result

        # ── FULL INSPECTION MODE ──────────────────────────────────────────────
        front_dets, back_dets = self._search_insta(goal_handle)

        # ── GAUGE SWEEP SCAN: if gauge not found on Insta360, sweep with Logitech ──
        if (front_dets is None or (not front_dets and not back_dets)) and self._target_class == 'gauge':
            self.get_logger().info('  Gauge not found on Insta360 — starting sweep scan with Logitech')
            sweep_result = self._gauge_sweep_scan(goal_handle, feedback, session_ts)
            if sweep_result is not None:
                # sweep_result is the final Result
                return sweep_result
            # If sweep also failed, fall through to abort
            return abort('no_detection')

        if front_dets is None and back_dets is None:
            return abort('no_detection')

        self._pub_detections(front_dets or [], back_dets or [])
        total_found = len(front_dets or []) + len(back_dets or [])

        # If only back objects found, signal BT to rotate robot
        if not front_dets and back_dets:
            return abort('all_in_back', in_back=True, found=total_found)


        # Limit to max_objects if specified
        if max_obj > 0:
            front_dets = front_dets[:max_obj]

        # Number objects within each class by confidence (highest = 1)
        # e.g., 2 gauges: gauge #1 (conf=0.92), gauge #2 (conf=0.78)
        class_counters = {}  # {cls_name: count}
        numbered_dets = []
        for det in sorted(front_dets, key=lambda d: d[6], reverse=True):
            cls = det[8]   # cls_name field (index 8 in Insta360 det tuple)
            class_counters[cls] = class_counters.get(cls, 0) + 1
            numbered_dets.append((*det, class_counters[cls]))  # append instance_num
        # Re-sort by original order (front_dets was sorted by track_id)
        front_dets = numbered_dets

        self.get_logger().info(
            f'  Processing {len(front_dets)} front object(s) in order')
        for cls, cnt in class_counters.items():
            self.get_logger().info(f'    {cls}: {cnt} instance(s)')
        if back_dets:
            self.get_logger().info(
                f'  {len(back_dets)} back object(s) noted -- will signal BT after front done')

        n_inspected = 0
        for obj_idx, det in enumerate(front_dets, start=1):
            if goal_handle.is_cancel_requested:
                break

            cx_obj, cy_obj, *_, conf, tid, cls_name, instance_num = det
            self.get_logger().info(
                f'Object {obj_idx}/{len(front_dets)}: [{cls_name} #{instance_num}] '
                f'cx={cx_obj:.0f} cy={cy_obj:.0f} conf={conf:.2f} track_id={tid}')

            # ---- Coarse positioning ----
            self._pub_status('COARSE')
            pan_c  = calculate_pan(cx_obj, cy_obj)
            tilt_c = calculate_tilt(cx_obj, cy_obj)
            self._servo(tilt_c, pan_c)
            self.get_logger().info(
                f'  Coarse: pan={pan_c:.1f} tilt={tilt_c:.1f} -- waiting {self.COARSE_WAIT}s')

            t_wait = time.time()
            while time.time() - t_wait < self.COARSE_WAIT:
                _, _, _, insta_dbg = self._detect_insta()
                logi_f = self._get_logi()
                self._publish_debug(insta_dbg, logi_f)
                time.sleep(0.1)

            # ---- Wait for first detection in Logitech ----
            self.get_logger().info(
                f'  Waiting up to {self.LOGI_FIRST_DET_WAIT}s for object in Logitech...')
            self._pub_status('IBVS')
            deadline   = time.time() + self.LOGI_FIRST_DET_WAIT
            first_det  = False
            _, _, _, insta_dbg_frozen = self._detect_insta()

            while time.time() < deadline:
                if goal_handle.is_cancel_requested:
                    return abort('cancelled', found=total_found)
                logi_dets, _, logi_dbg = self._detect_logi()
                self._publish_debug(insta_dbg_frozen, logi_dbg)
                if logi_dets:
                    first_det = True
                    self.get_logger().info('  Object visible in Logitech -- starting IBVS')
                    break
                time.sleep(0.1)

            if not first_det:
                self.get_logger().warn(
                    f'  Object {obj_idx} not visible in Logitech -- skipping')
                continue  # try next object, don't abort entire run

            # ---- IBVS ----
            feedback.current_object = obj_idx
            centred = self._ibvs(goal_handle, pan_c, tilt_c, feedback, obj_idx)

            if not centred:
                self.get_logger().warn(f'  Object {obj_idx} IBVS did not converge -- skipping')
                continue  # try next object

            # ---- Capture (wait for autofocus first) ----
            self._pub_status('CAPTURING')
            self.get_logger().info(f'  Waiting {self.FOCUS_WAIT}s for autofocus...')
            time.sleep(self.FOCUS_WAIT)

            # Capture 4 Logitech ROI images (include confidence in filename)
            paths = self._capture(self.IMAGES_PER_OBJ, obj_id=instance_num,
                                  session_ts=session_ts, cls_name=cls_name,
                                  conf_score=conf,
                                  ibvs_err=self._ibvs_err,
                                  ibvs_iter=self._ibvs_iter,
                                  ibvs_converged=centred)

            # Also capture Insta360 overview with YOLO boxes
            overview_paths = self._capture_insta_overview(
                session_ts, location_label, count=1,
                folder='inspection', obj_cls=cls_name, obj_id=instance_num)

            all_paths = paths + overview_paths
            self.get_logger().info(
                f'  Captured {len(paths)} ROI + {len(overview_paths)} overview '
                f'= {len(all_paths)} total  [{cls_name} #{instance_num} conf={conf:.2f}]')
            self._mqtt(all_paths, instance_num, session_ts=session_ts, cls_name=cls_name)
            n_inspected += 1
            self.get_logger().info(
                f'  Object {obj_idx} ({cls_name} #{instance_num}) done ({n_inspected} total)')

        # ---- Done ----
        self._pub_status('IDLE')
        self._goal_active = False
        if ret_home:
            self._home()

        result.success           = n_inspected > 0
        result.objects_inspected = n_inspected
        result.objects_found     = total_found
        result.object_in_back    = len(back_dets) > 0
        result.failed_reason     = '' if n_inspected > 0 else 'ibvs_timeout'
        goal_handle.succeed()
        self.get_logger().info(
            f'Inspection complete: {n_inspected}/{len(front_dets)} front objects. '
            f'Back objects signalled: {len(back_dets)}')
        return result


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = IBVSActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
