#!/usr/bin/env python3
"""
inspection_service.py — ROS2 Service Server for visual inspection.

Subclasses IBVSActionServer to reuse all detection/IBVS/capture logic.
Exposes a single service endpoint:
  /visual_inspection/inspect   [visual_inspection_interfaces/srv/Inspect]

BT calls this like an API endpoint — no feedback, just request + response.
Monitoring topics (optional, for debugging):
  /visual_inspection/status    String
  /visual_inspection/debug     Image  (add in RViz2)
  /visual_inspection/detections String
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from datetime import datetime

from std_msgs.msg import Int16MultiArray, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from visual_inspection_interfaces.srv import Inspect

# Reuse all pipeline internals from the action server module
from visual_inspection_ros.ibvs_action_server import (
    IBVSActionServer, load_yolo, calculate_pan, calculate_tilt, clamp
)


class InspectionService(IBVSActionServer):
    """
    Wraps IBVSActionServer as a ROS2 Service.
    Inherits all detection, IBVS, capture, sweep-scan logic.
    Removes the ActionServer; adds a Service server instead.
    """

    def __init__(self):
        # Call Node.__init__ directly — skip IBVSActionServer.__init__
        # so we don't create an ActionServer
        Node.__init__(self, 'inspection_service')

        self.bridge   = CvBridge()
        self.cb_group = ReentrantCallbackGroup()

        import threading
        self._lock_insta  = threading.Lock()
        self._lock_logi   = threading.Lock()
        self._frame_insta = None
        self._frame_logi  = None

        self.create_subscription(Image, '/visual_inspection/insta360/image_raw',
                                 self._cb_insta, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/visual_inspection/logitech/image_raw',
                                 self._cb_logi,  10, callback_group=self.cb_group)

        self.servo_pub      = self.create_publisher(Int16MultiArray, '/servo/pan_tilt', 10)
        self.debug_pub      = self.create_publisher(Image,  '/visual_inspection/debug', 10)
        self.status_pub     = self.create_publisher(String, '/visual_inspection/status', 10)
        self.ibvs_err_pub   = self.create_publisher(Point,  '/visual_inspection/ibvs_error', 10)
        self.detections_pub = self.create_publisher(String, '/visual_inspection/detections', 10)

        self._mqtt_cfg   = self._load_mqtt_cfg()
        self._yolo_lock  = threading.Lock()
        self._mode       = 'IDLE'
        self._target_class = ''
        self._ibvs_ex = self._ibvs_ey = self._ibvs_err = 0.0
        self._ibvs_iter = 0
        self._fps_counter = 0
        self._fps_time = time.time()
        self._fps = 0.0
        self._goal_active = False

        # Service lock — only one inspection at a time
        self._svc_lock = threading.Lock()

        self.get_logger().info('Loading YOLO model...')
        self.model = load_yolo(self.ENGINE_PATH)
        if self.model:
            self.get_logger().info('YOLO model ready')
        else:
            self.get_logger().error('YOLO load failed')

        # ROS2 Service (replaces ActionServer)
        self.create_service(
            Inspect, '/visual_inspection/inspect',
            self._service_callback,
            callback_group=self.cb_group
        )

        # Debug timer (4Hz) — inherited method works as-is
        self.create_timer(0.25, self._debug_timer_cb)

        self.get_logger().info('Inspection service ready: /visual_inspection/inspect')
        self.get_logger().info('Monitor: /visual_inspection/status  /visual_inspection/debug')

    # ------------------------------------------------------------------
    # Service callback — runs the full pipeline, returns when done
    # ------------------------------------------------------------------

    def _service_callback(self, request, response):
        """Handles one inspection request. Blocks until complete."""

        if not self._svc_lock.acquire(blocking=False):
            self.get_logger().warn('Service busy — rejecting request')
            response.success  = False
            response.status   = 'busy'
            response.info     = 'Another inspection is already running'
            return response

        try:
            return self._run_inspection(request, response)
        finally:
            self._svc_lock.release()

    def _run_inspection(self, request, response):
        """Core pipeline — same logic as execute_callback but for services."""
        self._goal_active    = True
        self._pipeline_start = time.time()   # for pipeline_time_s in metadata.json
        session_ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        target_obj_raw = (request.target_object or '').strip()
        location_label = (request.location_label or 'unknown').strip()
        max_obj        = request.max_objects or 0
        ret_home       = request.return_home

        # Normalise → YOLO class name
        resolved = self.KNOWN_CLASSES.get(
            target_obj_raw.lower(), target_obj_raw.lower())

        if resolved in self.OVERVIEW_ONLY_CLASSES:
            self._target_class = ''
            self.get_logger().info(f'[SVC] Overview-only: {target_obj_raw}')
            return self._do_overview(request, response, session_ts,
                                     target_obj_raw or location_label, ret_home)

        self._target_class = resolved
        self._current_location_label = location_label   # used by sweep _capture()
        self.get_logger().info(
            f'[SVC] Inspect "{target_obj_raw}" → YOLO "{self._target_class}" '
            f'loc="{location_label}"')

        # --- Stage 1: Insta360 search ---
        front_dets, back_dets = self._search_insta_svc()

        # --- Gauge sweep fallback ---
        if (not front_dets and not back_dets) and self._target_class == 'gauge':
            self.get_logger().info('[SVC] Gauge sweep scan starting')
            swept = self._gauge_sweep_svc(session_ts)
            self._goal_active = False
            self._pub_status('IDLE')
            if ret_home:
                self._home()
            if swept:
                response.success          = True
                response.status           = 'ok'
                response.objects_found    = 1
                response.objects_inspected = 1
                response.object_in_back   = False
                response.image_paths      = [str(p) for p in swept]
                response.info             = f'gauge found via sweep — {len(swept)} images'
            else:
                response.success = False
                response.status  = 'no_detection'
                response.info    = 'gauge not found (Insta360 + sweep)'
            return response

        if not front_dets and not back_dets:
            self._goal_active = False
            self._pub_status('IDLE')
            if ret_home:
                self._home()
            response.success = False
            response.status  = 'no_detection'
            response.info    = 'no objects detected on Insta360'
            return response

        self._pub_detections(front_dets or [], back_dets or [])
        total_found = len(front_dets or []) + len(back_dets or [])

        if not front_dets and back_dets:
            self._goal_active = False
            self._pub_status('IDLE')
            response.success        = False
            response.status         = 'all_in_back'
            response.objects_found  = total_found
            response.object_in_back = True
            response.info           = f'{total_found} object(s) in back — rotate robot 180°'
            return response

        if max_obj > 0:
            front_dets = front_dets[:max_obj]

        # Number by confidence within class
        class_counters = {}
        numbered = []
        for det in sorted(front_dets, key=lambda d: d[6], reverse=True):
            cls = det[8]
            class_counters[cls] = class_counters.get(cls, 0) + 1
            numbered.append((*det, class_counters[cls]))
        front_dets = numbered

        all_paths   = []
        n_inspected = 0
        info_parts  = []

        for obj_idx, det in enumerate(front_dets, 1):
            cx_obj, cy_obj, *_, conf, tid, cls_name, instance_num = det

            # Coarse — timed
            self._pub_status('COARSE')
            _coarse_t0 = time.time()
            pan_c  = calculate_pan(cx_obj, cy_obj)
            tilt_c = calculate_tilt(cx_obj, cy_obj)
            self._servo(tilt_c, pan_c)
            self.get_logger().info(
                f'[SVC] Object {obj_idx}: [{cls_name}#{instance_num}] '
                f'coarse pan={pan_c:.1f} tilt={tilt_c:.1f}')
            time.sleep(self.COARSE_WAIT)
            self._coarse_time_s = round(time.time() - _coarse_t0, 3)

            # Wait for Logitech
            self._pub_status('IBVS')
            deadline  = time.time() + self.LOGI_FIRST_DET_WAIT
            first_det = False
            while time.time() < deadline:
                logi_dets, _, _ = self._detect_logi()
                if logi_dets:
                    first_det = True
                    break
                time.sleep(0.1)

            if not first_det:
                self.get_logger().warn(
                    f'[SVC] Object {obj_idx} not visible in Logitech — skipping')
                continue

            # IBVS — pass a dummy object with is_cancel_requested=False
            class _DummyHandle:
                is_cancel_requested = False
                def publish_feedback(self, _): pass

            class _DummyFeedback:
                current_step = ''
                current_object = 0
                ibvs_error_px = 0.0

            centred = self._ibvs(_DummyHandle(), pan_c, tilt_c,
                                 _DummyFeedback(), obj_idx)
            if not centred:
                self.get_logger().warn(
                    f'[SVC] Object {obj_idx} IBVS did not converge — skipping')
                continue

            # Capture
            self._pub_status('CAPTURING')
            time.sleep(self.FOCUS_WAIT)

            paths = self._capture(
                self.IMAGES_PER_OBJ, obj_id=instance_num,
                session_ts=session_ts, cls_name=cls_name,
                conf_score=conf,
                ibvs_err=self._ibvs_err,
                ibvs_iter=self._ibvs_iter,
                ibvs_converged=centred,
                location_label=location_label)

            ov_paths = self._capture_insta_overview(
                session_ts, location_label, count=1,
                folder='inspection', obj_cls=cls_name, obj_id=instance_num)

            obj_paths = paths + ov_paths
            all_paths.extend(obj_paths)
            self._mqtt(obj_paths, instance_num,
                       session_ts=session_ts, cls_name=cls_name)
            n_inspected += 1
            info_parts.append(
                f'{cls_name}#{instance_num}(conf={conf:.2f})')
            self.get_logger().info(
                f'[SVC] {cls_name}#{instance_num} done — {len(obj_paths)} images')

        self._goal_active = False
        self._pub_status('IDLE')
        if ret_home:
            self._home()

        response.success           = n_inspected > 0
        response.status            = 'ok' if n_inspected > 0 else 'ibvs_timeout'
        response.objects_found     = total_found
        response.objects_inspected = n_inspected
        response.object_in_back    = len(back_dets) > 0
        response.image_paths       = [str(p) for p in all_paths]
        response.info              = (
            ', '.join(info_parts) + f' — {len(all_paths)} images saved'
            if info_parts else 'no objects successfully inspected')
        self.get_logger().info(
            f'[SVC] Done: {n_inspected}/{len(front_dets)} inspected, '
            f'{len(all_paths)} images. Back={len(back_dets)}')
        return response

    # ------------------------------------------------------------------
    # Overview-only (unknown / main_cylinder)
    # ------------------------------------------------------------------

    def _do_overview(self, request, response, session_ts, label, ret_home):
        self._pub_status('OVERVIEW')
        self._home()
        time.sleep(2.0)

        overview_count = 2
        insta_paths = self._capture_insta_overview(
            session_ts, label, count=overview_count, folder='overview')
        logi_paths = self._capture(
            n=1, obj_id=1, session_ts=session_ts,
            cls_name=label, conf_score=0.0,
            ibvs_converged=False, ibvs_err=0.0, ibvs_iter=0,
            location_label=request.location_label or '')

        all_paths = insta_paths + logi_paths
        self._mqtt(all_paths, 0, session_ts=session_ts, cls_name=label)

        self._pub_status('IDLE')
        self._goal_active = False
        if ret_home:
            self._home()

        response.success           = len(all_paths) > 0
        response.status            = 'ok' if all_paths else 'no_frames'
        response.objects_found     = len(all_paths)
        response.objects_inspected = 0
        response.object_in_back    = False
        response.image_paths       = [str(p) for p in all_paths]
        response.info              = (
            f'overview-only ({label}): '
            f'{len(insta_paths)} Insta360 + {len(logi_paths)} Logitech')
        return response

    # ------------------------------------------------------------------
    # Service-compatible versions of search/sweep (no goal_handle)
    # ------------------------------------------------------------------

    def _search_insta_svc(self):
        """Like _search_insta but no goal_handle (service context)."""
        self._mode = 'DETECTING'
        timeout = (self.GAUGE_INSTA_TIMEOUT
                   if self._target_class == 'gauge'
                   else self.INSTA_SEARCH_TIMEOUT)
        self.get_logger().info(f'[SVC] Searching Insta360 up to {timeout}s')
        deadline = time.time() + timeout

        while time.time() < deadline:
            front, back, _, dbg = self._detect_insta()
            logi_f = self._get_logi()
            self._publish_debug(dbg, logi_f)
            if front or back:
                self.get_logger().info(
                    f'[SVC] Found: {len(front)} front, {len(back)} back')
                return front, back
            time.sleep(0.1)

        self.get_logger().warn('[SVC] Insta360 search timeout')
        return [], []

    def _gauge_sweep_svc(self, session_ts):
        """Smooth sweep for gauge using waypoints."""
        import numpy as np
        self._mode = 'SWEEP'
        self.get_logger().info('[SVC] Gauge sweep: T50(20→160), T80(160→20), T110(50→130)')

        SPEED    = 40.0   # degrees per second
        HZ       = 30     # servo update rate
        CHK      = 3      # check detection every N steps

        found_dets = []
        cur_tilt   = 90
        cur_pan    = 90

        def smooth_move(t0, p0, t1, p1):
            """Move servo smoothly from (t0,p0) to (t1,p1). Returns dets, end_t, end_p."""
            steps = max(int(max(abs(t1-t0), abs(p1-p0)) / SPEED * HZ), 1)
            for i, (t, p) in enumerate(zip(
                np.linspace(t0, t1, steps+1),
                np.linspace(p0, p1, steps+1)
            )):
                self._servo(float(t), float(p))
                if i % CHK == 0:
                    dets, _, dbg = self._detect_logi()
                    self._publish_debug(self._get_insta(), dbg)
                    if dets:
                        return dets, float(t), float(p)
                time.sleep(1.0 / HZ)
            return [], float(t1), float(p1)

        waypoints = [
            (50, 20),
            (50, 160),
            (80, 160),
            (80, 20),
            (110, 50),
            (110, 130)
        ]

        for wp_tilt, wp_pan in waypoints:
            res, cur_tilt, cur_pan = smooth_move(cur_tilt, cur_pan, wp_tilt, wp_pan)
            if res:
                found_dets = res
                break
            
            # Final check at waypoint
            dets, _, dbg = self._detect_logi()
            self._publish_debug(self._get_insta(), dbg)
            if dets:
                found_dets = dets
                break

        if not found_dets:
            self.get_logger().warn('[SVC] Gauge sweep: not found')
            return []

        self.get_logger().info(f'[SVC] Gauge found in sweep — starting IBVS')
        self._pub_status('IBVS')

        class _DH:
            is_cancel_requested = False
            def publish_feedback(self, _): pass

        class _FB:
            current_step = 'ibvs'
            current_object = 1
            ibvs_error_px = 0.0

        centred = self._ibvs(_DH(), cur_pan, cur_tilt, _FB(), 1)
        if not centred:
            self.get_logger().warn('[SVC] Sweep IBVS did not converge')
            return []

        self._pub_status('CAPTURING')
        time.sleep(self.FOCUS_WAIT)
        conf  = found_dets[0][6]
        paths = self._capture(self.IMAGES_PER_OBJ, obj_id=1,
                              session_ts=session_ts, cls_name='gauge',
                              conf_score=conf,
                              ibvs_err=self._ibvs_err,
                              ibvs_iter=self._ibvs_iter,
                              ibvs_converged=centred,
                              location_label=getattr(self, '_current_location_label', ''))
        ov    = self._capture_insta_overview(session_ts, 'sweep_gauge',
                                             count=1, folder='inspection',
                                             obj_cls='gauge', obj_id=1)
        all_p = paths + ov
        self._mqtt(all_p, 1, session_ts=session_ts, cls_name='gauge')
        return all_p



# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = InspectionService()
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
