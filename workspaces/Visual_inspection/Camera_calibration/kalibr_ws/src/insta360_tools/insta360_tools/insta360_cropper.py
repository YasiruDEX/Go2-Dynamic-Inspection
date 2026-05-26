'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Insta360Cropper(Node):
    def __init__(self):
        super().__init__('insta360_cropper')

        # Declare parameter (front or back)
        self.declare_parameter('camera_half', 'front')
        self.camera_half = self.get_parameter('camera_half').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Subscribe to raw feed (from gscam or cam2image)
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)

        # Publish cropped output
        self.publisher = self.create_publisher(Image, '/image_cropped', 10)

        self.get_logger().info(f'Insta360 Cropper started with camera_half={self.camera_half}')

    def listener_callback(self, msg):
        # Convert ROS2 image -> OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        h, w, _ = cv_image.shape
        if self.camera_half == 'front':
            cropped = cv_image[:h//2, :]  # Top half
        else:
            cropped = cv_image[h//2:, :]  # Bottom half

        # Publish cropped image
        ros_image = self.bridge.cv2_to_imgmsg(cropped, "bgr8")
        self.publisher.publish(ros_image)

        # Optional: show preview
        cv2.imshow("Cropped View", cropped)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Insta360Cropper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    '''
    
    
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Insta360Cropper(Node):
    def __init__(self):
        super().__init__('insta360_cropper')

        # Parameters
        self.declare_parameter('camera_half', 'front')
        self.declare_parameter('publish_rate', 5.0)  # default 5 Hz

        self.camera_half = self.get_parameter('camera_half').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Subscribe to raw feed (from cam2image or gscam)
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)

        # Publisher
        self.publisher = self.create_publisher(Image, '/image_cropped', 10)

        # Buffer for last frame
        self.last_frame = None

        # Timer for publishing at controlled rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(f'Insta360 Cropper started with camera_half={self.camera_half}, rate={self.publish_rate} Hz')

    def listener_callback(self, msg):
        # Store the latest frame
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def timer_callback(self):
        if self.last_frame is None:
            return

        h, w, _ = self.last_frame.shape
        if self.camera_half == 'front':
            cropped = self.last_frame[:h//2, :]  # Top half
        else:
            cropped = self.last_frame[h//2:, :]  # Bottom half

        ros_image = self.bridge.cv2_to_imgmsg(cropped, "bgr8")
        self.publisher.publish(ros_image)

        # Optional: show preview
        cv2.imshow("Cropped View", cropped)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Insta360Cropper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


