import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

class GoalConvertor(Node):
    def __init__(self):
        super().__init__('goal_convertor')
        
        # Subscriber to goal_point_2d from Foxglove (PoseStamped)
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_point_2d',
            self.listener_callback,
            10
        )
        
        # Publisher to goal_point (PointStamped)
        self.publisher_ = self.create_publisher(PointStamped, 'goal_point', 10)
        self.get_logger().info('Goal Convertor node has started.')

    def listener_callback(self, msg: PoseStamped):
        convert_msg = PointStamped()
        convert_msg.header = msg.header
        convert_msg.point.x = msg.pose.position.x
        convert_msg.point.y = msg.pose.position.y
        convert_msg.point.z = msg.pose.position.z 
        
        self.publisher_.publish(convert_msg)
        self.get_logger().info(f'Converted Goal: [x: {convert_msg.point.x}, y: {convert_msg.point.y}]')

def main(args=None):
    rclpy.init(args=args)
    goal_convertor = GoalConvertor()
    try:
        rclpy.spin(goal_convertor)
    except KeyboardInterrupt:
        pass
    finally:
        goal_convertor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
