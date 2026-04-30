import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from goal_action_interfaces.action import NavigateToGoal

def get_yaw_from_quaternion(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


class GoalActionServer(Node):
    def __init__(self):
        super().__init__('goal_action_server')
        
        # Publisher for the goal
        self.goal_publisher = self.create_publisher(
            PointStamped,
            '/goal_point',
            10
        )
        
        # Publisher for the velocity
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Odometry subscriber
        self.current_pose = None
        self.pose_history = []
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
        
        # Service client to enable/disable local planner
        self.enable_planner_client = self.create_client(SetBool, '/enable_local_planner')
        
        self.get_logger().info('Goal Action Server has been started.')

    def set_local_planner_state(self, state: bool):
        if not self.enable_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/enable_local_planner service not available')
            return
        
        req = SetBool.Request()
        req.data = state
        self.get_logger().info(f'Calling /enable_local_planner with data={state}')
        
        future = self.enable_planner_client.call_async(req)
        def _service_done_cb(fut):
            try:
                res = fut.result()
                self.get_logger().info(f'Local planner service call returned: success={res.success}, message="{res.message}"')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                
        future.add_done_callback(_service_done_cb)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.pose_history.append((self.current_pose.position.x, self.current_pose.position.y))
        if len(self.pose_history) > 15: # 1.5 seconds at 10Hz
            self.pose_history.pop(0)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received goal request.')
        
        # Extract the target point
        target_point_msg = goal_handle.request.goal_point
        target_x = target_point_msg.point.x
        target_y = target_point_msg.point.y
        target_z = target_point_msg.point.z
        target_heading = goal_handle.request.goal_heading
        is_walking = goal_handle.request.is_walking
        
        # Publish the goal to /goal_point topic as requested
        if not target_point_msg.header.frame_id:
            target_point_msg.header.frame_id = 'map'
        
        self.goal_publisher.publish(target_point_msg)
        self.get_logger().info(f'Published goal to /goal_point: x={target_x}, y={target_y}, z={target_z}, heading={target_heading}, is_walking={is_walking}')
        
        feedback_msg = NavigateToGoal.Feedback()
        
        if is_walking:
            # --- Phase 1: Wait to reach the position ---
            self.set_local_planner_state(True)
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
                
                curr_x = self.current_pose.position.x
                curr_y = self.current_pose.position.y
                curr_z = self.current_pose.position.z
                
                dx = curr_x - target_x
                dy = curr_y - target_y
                dz = curr_z - target_z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                self.get_logger().info(f'Distance to goal: {distance:.2f} meters', throttle_duration_sec=1.0)
                
                feedback_msg.distance_remaining = distance
                goal_handle.publish_feedback(feedback_msg)
                
                if distance < 1.0:
                    self.get_logger().info('Goal position reached (distance < 1.0m).')
                    break
                
                time.sleep(0.1)

            # --- Phase 2: Wait until stationary ---
            self.get_logger().info('Waiting for robot to be stationary...')
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = NavigateToGoal.Result()
                    result.success = False
                    return result
                
                if len(self.pose_history) == 15:
                    xs = [p[0] for p in self.pose_history]
                    ys = [p[1] for p in self.pose_history]
                    max_dx =  max(xs) - min(xs)
                    max_dy = max(ys) - min(ys)
                    
                    if max_dx < 0.05 and max_dy < 0.05:
                        self.get_logger().info('Robot is stationary.')
                        self.set_local_planner_state(False)
                        break
                
                time.sleep(0.1)
        else:
            # --- Alternative Phase 1 & 2: Direct PID control ---
            self.get_logger().info('is_walking is false. Using direct PID control to reach position.')
            
            prev_dist_err = 0.0
            integral_dist = 0.0
            
            kp_dist = 0.5
            ki_dist = 0.0
            kd_dist = 0.1
            kp_heading = 0.8
            dt = 0.1
            twist_msg = Twist()
            
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist_msg)
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
                
                curr_x = self.current_pose.position.x
                curr_y = self.current_pose.position.y
                
                dx = target_x - curr_x
                dy = target_y - curr_y
                distance = math.sqrt(dx**2 + dy**2)
                
                target_angle = math.atan2(dy, dx)
                current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
                
                heading_err = target_angle - current_yaw
                # Normalize to [-pi, pi]
                while heading_err > math.pi: heading_err -= 2 * math.pi
                while heading_err < -math.pi: heading_err += 2 * math.pi
                
                self.get_logger().info(f'Distance to goal: {distance:.2f} meters, heading err: {math.degrees(heading_err):.2f} deg', throttle_duration_sec=1.0)
                
                feedback_msg.distance_remaining = distance
                goal_handle.publish_feedback(feedback_msg)
                
                if distance < 0.2:
                    self.get_logger().info('Goal position reached (distance < 0.2m) using PID.')
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist_msg)
                    break
                
                # PID for distance
                integral_dist += distance * dt
                derivative_dist = (distance - prev_dist_err) / dt
                prev_dist_err = distance
                
                linear_vel = kp_dist * distance + ki_dist * integral_dist + kd_dist * derivative_dist
                # clamp linear velocity
                max_lin_vel = 0.4
                linear_vel = max(min(linear_vel, max_lin_vel), -max_lin_vel)
                
                # P for heading
                angular_vel = kp_heading * heading_err
                max_ang_vel = 0.5
                angular_vel = max(min(angular_vel, max_ang_vel), -max_ang_vel)
                
                # Prevent driving forward if severely misaligned
                if abs(heading_err) > math.radians(45.0):
                    linear_vel = 0.0
                    
                twist_msg.linear.x = linear_vel
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = angular_vel
                self.cmd_vel_publisher.publish(twist_msg)

                self.get_logger().info(f'Velocity to goal: {linear_vel:.2f}', throttle_duration_sec=1.0)

                
                time.sleep(dt)
            
        # --- Phase 3: Heading Correction ---
        self.get_logger().info('Starting heading correction...')
        twist_msg = Twist()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                twist_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                goal_handle.canceled()
                result = NavigateToGoal.Result()
                result.success = False
                return result

            current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
            
            # Shortest angular distance
            heading_err = target_heading - current_yaw
            # Normalize to [-pi, pi]
            while heading_err > math.pi: heading_err -= 2 * math.pi
            while heading_err < -math.pi: heading_err += 2 * math.pi
            
            feedback_msg.heading_remaining = heading_err
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Heading angle correction: {math.degrees(heading_err):.2f} degrees', throttle_duration_sec=1.0)
            
            if abs(heading_err) < math.radians(10.0):
                self.get_logger().info('Heading aligned (error < 10 degrees).')
                twist_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                break
                
            # Proportional control for rotation
            kp = 0.5
            angular_z = kp * heading_err
            # Clamp angular velocity
            max_ang_vel = 0.5
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = max(min(angular_z, max_ang_vel), -max_ang_vel)
            self.cmd_vel_publisher.publish(twist_msg)
            
            time.sleep(0.1)
            
        # --- Success ---
        goal_handle.succeed()
        result = NavigateToGoal.Result()
        result.success = True
        result.message = "Goal successfully reached and aligned."
        return result



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
