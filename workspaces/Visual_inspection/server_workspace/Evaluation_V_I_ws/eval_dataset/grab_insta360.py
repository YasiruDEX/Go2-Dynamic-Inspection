#!/usr/bin/env python3
"""
grab_insta360.py
Grabs ONE frame from the Insta360 topic and saves it to the given path.
Usage: python3 grab_insta360.py /path/to/save.jpg
"""
import sys, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

TOPIC = '/visual_inspection/insta360/image_raw'

class FrameGrabber(Node):
    def __init__(self, save_path):
        super().__init__('insta360_frame_grabber')
        self.save_path = save_path
        self.done      = False
        self.sub = self.create_subscription(
            Image, TOPIC, self._cb, 1)

    def _cb(self, msg: Image):
        if self.done:
            return
        try:
            # Convert ROS Image → numpy → save with cv2
            dtype = 'uint8'
            img_data = bytes(msg.data)
            if msg.encoding in ('rgb8', 'bgr8'):
                arr = np.frombuffer(img_data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
                if msg.encoding == 'rgb8':
                    arr = arr[:, :, ::-1]  # RGB → BGR
            elif msg.encoding == 'mono8':
                arr = np.frombuffer(img_data, dtype=np.uint8).reshape(
                    (msg.height, msg.width))
            else:
                arr = np.frombuffer(img_data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, -1))

            if HAS_CV2:
                cv2.imwrite(self.save_path, arr)
            else:
                # Fallback: write raw bytes (uncommon path)
                with open(self.save_path, 'wb') as f:
                    f.write(img_data)

            self.done = True
        except Exception as e:
            print(f'[grab_insta360] Error: {e}', file=sys.stderr)
            self.done = True  # avoid spin-loop

def main():
    if len(sys.argv) < 2:
        print('Usage: grab_insta360.py <save_path.jpg>', file=sys.stderr)
        sys.exit(1)
    save_path = sys.argv[1]

    rclpy.init()
    node = FrameGrabber(save_path)
    import time
    t0 = time.time()
    while not node.done and (time.time() - t0) < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if node.done:
        print(f'[grab_insta360] Saved → {save_path}')
        sys.exit(0)
    else:
        print(f'[grab_insta360] Timeout — no frame received from {TOPIC}', file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
