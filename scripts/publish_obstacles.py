import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/added_obstacles', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Change this to your global frame (e.g., 'map' or 'odom')
        msg.header.frame_id = 'map'

        # Specify your obstacle coordinates as [x, y, z]
        points = [
            [-1.0, -15.0, 0.0],
            [-0.8, -21.0, 0.0],
            [4.24, -18.0, 0.0],
            [-0.8, -21.0, 0.0]
        ]

        # Constructing PointCloud2 payload
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buffer = []
        for p in points:
             # The intensity value doesn't really matter here, as the C++ code forces it to 200.0
            buffer.append(struct.pack('ffff', p[0], p[1], p[2], 0.0))
        msg.data = b''.join(buffer)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
