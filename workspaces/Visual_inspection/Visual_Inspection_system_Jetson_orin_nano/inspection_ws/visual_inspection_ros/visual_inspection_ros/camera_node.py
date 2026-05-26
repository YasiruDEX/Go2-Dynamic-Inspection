#!/usr/bin/env python3
"""
camera_node.py — ROS2 Node: Publish Insta360 + Logitech camera feeds as topics.

Uses same 3-layer camera detection as ibvs_pipeline.py:
  Layer 1: udev symlink + frame verify (skips metadata devices)
  Layer 2: USB vendor ID scan (finds actual capture device)
  Layer 3: name-based fallback

Topics published:
  /visual_inspection/insta360/image_raw  (sensor_msgs/Image)
  /visual_inspection/logitech/image_raw  (sensor_msgs/Image)

Test:
  ros2 topic list | grep visual_inspection
  ros2 topic hz /visual_inspection/insta360/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob


# ── Camera Detection (ported from ibvs_pipeline.py) ──────────────────────────

VENDOR_MAP = {
    'Insta360':      ('2e1a', '/dev/insta360'),
    'HD Pro Webcam': ('046d', '/dev/logitech'),
    'Logitech':      ('046d', '/dev/logitech'),
}

def find_camera(name_pattern):
    """Find camera device index — 3 layers (same as ibvs_pipeline.py).
    Layer 1: udev symlink + frame read to verify capture (skips metadata devices)
    Layer 2: USB vendor ID via sysfs
    Layer 3: name-based fallback
    """
    vendor_id = None
    udev_path = None
    for key, (vid, udev) in VENDOR_MAP.items():
        if key in name_pattern or name_pattern in key:
            vendor_id = vid
            udev_path = udev
            break

    # Layer 1: udev symlink — verify by reading a frame (skips metadata device)
    if udev_path and os.path.exists(udev_path):
        cap = cv2.VideoCapture(udev_path)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None and frame.size > 0:
                idx = int(os.path.realpath(udev_path).replace('/dev/video', ''))
                print(f'   [udev] {udev_path} → /dev/video{idx}')
                return idx

    # Layer 2: USB vendor ID via sysfs
    if vendor_id:
        for path in sorted(glob.glob('/sys/class/video4linux/video*')):
            try:
                check = os.path.realpath(path)
                for _ in range(8):
                    vid_file = os.path.join(check, 'idVendor')
                    if os.path.exists(vid_file):
                        with open(vid_file) as f:
                            if f.read().strip() == vendor_id:
                                idx = int(os.path.basename(path).replace('video', ''))
                                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                                if cap.isOpened():
                                    ret, frame = cap.read()
                                    cap.release()
                                    if ret and frame is not None and frame.size > 0:
                                        print(f'   [vendor-id] /dev/video{idx}')
                                        return idx
                        break
                    check = os.path.dirname(check)
            except:
                pass

    # Layer 3: name fallback
    for path in sorted(glob.glob('/sys/class/video4linux/video*')):
        try:
            name_path = os.path.join(path, 'name')
            if not os.path.exists(name_path):
                continue
            with open(name_path) as f:
                name = f.read().strip()
            if name_pattern in name:
                idx = int(path.split('video')[-1])
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    if ret and frame is not None and frame.size > 0:
                        print(f'   [name] /dev/video{idx}')
                        return idx
        except:
            pass

    return -1


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        self.declare_parameter('fps', 30)
        fps = self.get_parameter('fps').value

        # --- Detect cameras (same 3-layer logic as ibvs_pipeline.py) ---
        self.get_logger().info('Detecting cameras...')

        insta_idx = find_camera('Insta360')
        logi_idx  = find_camera('Logitech')

        self.cap_insta = None
        self.cap_logi  = None

        if insta_idx >= 0:
            self.cap_insta = cv2.VideoCapture(insta_idx, cv2.CAP_V4L2)
            self.cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
            self.cap_insta.set(cv2.CAP_PROP_FPS, fps)
            self.cap_insta.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info(f'Insta360 at /dev/video{insta_idx}')
        else:
            self.get_logger().warn('Insta360 not found -- plug in and restart')

        if logi_idx >= 0:
            self.cap_logi = cv2.VideoCapture(logi_idx, cv2.CAP_V4L2)
            self.cap_logi.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            self.cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap_logi.set(cv2.CAP_PROP_FPS, fps)
            self.cap_logi.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info(f'Logitech at /dev/video{logi_idx}')
        else:
            self.get_logger().warn('Logitech not found -- plug in and restart')

        # --- Publishers ---
        self.pub_insta = self.create_publisher(
            Image, '/visual_inspection/insta360/image_raw', 10)
        self.pub_logi = self.create_publisher(
            Image, '/visual_inspection/logitech/image_raw', 10)

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'Camera publisher running at {fps} Hz')

    def timer_callback(self):
        if self.cap_insta is not None and self.cap_insta.isOpened():
            ret, frame = self.cap_insta.read()
            if ret and frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'insta360_frame'
                self.pub_insta.publish(msg)

        if self.cap_logi is not None and self.cap_logi.isOpened():
            ret, frame = self.cap_logi.read()
            if ret and frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'logitech_frame'
                self.pub_logi.publish(msg)

    def destroy_node(self):
        if self.cap_insta:
            self.cap_insta.release()
        if self.cap_logi:
            self.cap_logi.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
