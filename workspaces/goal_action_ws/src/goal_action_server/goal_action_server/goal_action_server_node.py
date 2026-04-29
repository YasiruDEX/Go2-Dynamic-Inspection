import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from goal_action_interfaces.action import NavigateToGoal


class GoalActionServer(Node):
    def __init__(self):
        super().__init__('goal_action_server')
        
        # Publisher for the goal
        self.goal_publisher = self.create_publisher(
            PointStamped,
            '/goal_point',
            10
        )
        
        # Odometry subscriber
        self.current_pose = None
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/lidar_odometry/pose',
            self.odom_callback,
            10
        )
        
        # Action Server
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            '/navigate_to_goal',
            self.execute_callback
        )
        
        self.get_logger().info('Goal Action Server has been started.')

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received goal request.')
        
        # Extract the target point
        target_point_msg = goal_handle.request.goal_point
        target_x = target_point_msg.point.x
        target_y = target_point_msg.point.y
        target_z = target_point_msg.point.z
        
        # Publish the goal to /goal_point topic as requested
        # Ensure frame_id is 'map'
        if not target_point_msg.header.frame_id:
            target_point_msg.header.frame_id = 'map'
        
        self.goal_publisher.publish(target_point_msg)
        self.get_logger().info(f'Published goal to /goal_point: x={target_x}, y={target_y}, z={target_z}')
        
        feedback_msg = NavigateToGoal.Feedback()
        
        rate = self.create_rate(10) # 10 Hz loop
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result = NavigateToGoal.Result()
                result.success = False
                result.message = "Goal canceled"
                return result
            
            if self.current_pose is None:
                self.get_logger().info('Waiting for odometry...', throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            
            # Calculate vector distance
            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            curr_z = self.current_pose.position.z
            
            dx = curr_x - target_x
            dy = curr_y - target_y
            dz = curr_z - target_z
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Distance to goal: {distance:.2f}m', throttle_duration_sec=1.0)
            
            # Check success condition
            if distance < 0.8:
                self.get_logger().info('Goal reached (distance < 0.8m)!')
                goal_handle.succeed()
                
                result = NavigateToGoal.Result()
                result.success = True
                result.message = "Goal successfully reached."
                return result
            
            time.sleep(0.1) # sleep instead of rate.sleep() in action callbacks unless using multithreading
            # Note: in Python Action Server, execute_callback is run in a separate thread if using MultiThreadedExecutor,
            # but default is SingleThreadedExecutor where time.sleep() might block other callbacks.
            # However, rclpy action servers by default execute the callback in a separate thread, so time.sleep is fine.
            # Actually, starting in recent rclpy versions, action callbacks run in a thread pool.


def main(args=None):
    rclpy.init(args=args)
    
    node = GoalActionServer()
    
    # We use a MultiThreadedExecutor to allow the odom callback to execute while the action callback is in a while loop
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
