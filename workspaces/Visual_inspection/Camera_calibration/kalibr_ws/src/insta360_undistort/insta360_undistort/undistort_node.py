'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml




class UndistortNode(Node):
    def __init__(self):
        super().__init__('undistort_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('camera_half', 'front')

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.camera_half = self.get_parameter('camera_half').get_parameter_value().string_value

        # Load calibration params
        with open(config_file, 'r') as f:
            calib = yaml.safe_load(f)

        cam0 = calib['cam0']
        self.K = np.array([[cam0['intrinsics'][0], 0, cam0['intrinsics'][2]],
                           [0, cam0['intrinsics'][1], cam0['intrinsics'][3]],
                           [0, 0, 1]])
        self.D = np.array(cam0['distortion_coeffs'])
        self.resolution = tuple(cam0['resolution'])

        # Precompute undistort maps
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, self.K, self.resolution, cv2.CV_16SC2
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_cropped', self.callback, 10)
        self.pub = self.create_publisher(Image, f'/image_undistorted/{self.camera_half}', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        msg_out = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
        self.pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = UndistortNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


'''
'''
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml


class UndistortNode(Node):
    def __init__(self):
        super().__init__('undistort_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('camera_half', 'front')

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.camera_half = self.get_parameter('camera_half').get_parameter_value().string_value

        # Load calibration params
        with open(config_file, 'r') as f:
            calib = yaml.safe_load(f)

        cam0 = calib['cam0']
        self.K = np.array([[cam0['intrinsics'][0], 0, cam0['intrinsics'][2]],
                           [0, cam0['intrinsics'][1], cam0['intrinsics'][3]],
                           [0, 0, 1]])
        self.D = np.array(cam0['distortion_coeffs'])
        self.resolution = tuple(cam0['resolution'])  # (width, height)

        #  Use OpenCV to get an optimal new camera matrix
        w, h = self.resolution
        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1, (w, h))

        # Build undistortion maps using the adjusted matrix
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, new_K, (w, h), cv2.CV_16SC2
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_cropped', self.callback, 10)
        self.pub = self.create_publisher(Image, f'/image_undistorted/{self.camera_half}', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        msg_out = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = UndistortNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 
        
    
'''



import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml


class UndistortNode(Node):
    def __init__(self):
        super().__init__('undistort_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('camera_half', 'front')

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.camera_half = self.get_parameter('camera_half').get_parameter_value().string_value

        # Load calibration params
        with open(config_file, 'r') as f:
            calib = yaml.safe_load(f)

        cam0 = calib['cam0']
        self.K = np.array([[cam0['intrinsics'][0], 0, cam0['intrinsics'][2]],
                           [0, cam0['intrinsics'][1], cam0['intrinsics'][3]],
                           [0, 0, 1]])
        self.D = np.array(cam0['distortion_coeffs'])
        self.resolution = tuple(cam0['resolution'])  # (width, height)

        #  Use OpenCV to get an optimal new camera matrix
        w, h = self.resolution
        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 0, (w, h))

        # Build undistortion maps using the adjusted matrix
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, new_K, (w, h), cv2.CV_16SC2
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_cropped', self.callback, 10)
        self.pub = self.create_publisher(Image, f'/image_undistorted/{self.camera_half}', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        msg_out = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = UndistortNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 
