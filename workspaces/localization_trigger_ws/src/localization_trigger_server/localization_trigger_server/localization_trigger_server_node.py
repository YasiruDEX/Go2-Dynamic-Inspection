import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from std_srvs.srv import SetBool
from mola_msgs.srv import MolaRuntimeParamSet
from localization_trigger_interfaces.action import ToggleSystem

class LocalizationTriggerServer(Node):
    def __init__(self):
        super().__init__('localization_trigger_server')
        
        self._action_server = ActionServer(
            self,
            ToggleSystem,
            '/toggle_system',
            self.execute_callback
        )
        
        self.mola_client = self.create_client(MolaRuntimeParamSet, '/mola_runtime_param_set')
        self.terrain_client = self.create_client(SetBool, '/enable_terrain_analysis')
        
        self.get_logger().info('Localization Trigger Server has been started.')

    def execute_callback(self, goal_handle: ServerGoalHandle):
        enable = goal_handle.request.enable
        self.get_logger().info(f'Received goal request: enable={enable}')
        
        feedback_msg = ToggleSystem.Feedback()
        
        if enable:
            # Enable Sequence
            feedback_msg.current_state = "Enabling Lidar Odometry..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            
            # 1. Enable Mola Lidar Odometry
            if not self.mola_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('/mola_runtime_param_set service not available')
            else:
                mola_req = MolaRuntimeParamSet.Request()
                mola_req.parameters = "mola::LidarOdometry:lidar_odom:\n active: true\n"
                # Call synchronous or asynchronous. We are in a callback, but action server callbacks run in thread pools usually.
                # However, it's safer to use call_async and wait if we use a MultiThreadedExecutor, or just use call_async.
                self.mola_client.call_async(mola_req)
                
            # 2. Wait 5 seconds
            feedback_msg.current_state = "Waiting 5 seconds..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            time.sleep(5.0)
            
            # 3. Enable Terrain Analysis
            feedback_msg.current_state = "Enabling Terrain Analysis..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            
            if not self.terrain_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('/enable_terrain_analysis service not available')
            else:
                terrain_req = SetBool.Request()
                terrain_req.data = True
                self.terrain_client.call_async(terrain_req)
                
            # 4. Wait 5 seconds
            feedback_msg.current_state = "Waiting 5 seconds..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            time.sleep(5.0)
            
        else:
            # Disable Sequence
            feedback_msg.current_state = "Disabling Terrain Analysis..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            
            # 1. Disable Terrain Analysis
            if not self.terrain_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('/enable_terrain_analysis service not available')
            else:
                terrain_req = SetBool.Request()
                terrain_req.data = False
                self.terrain_client.call_async(terrain_req)
                
            # 2. Wait 5 seconds
            feedback_msg.current_state = "Waiting 5 seconds..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            time.sleep(5.0)
            
            # 3. Disable Mola Lidar Odometry
            feedback_msg.current_state = "Disabling Lidar Odometry..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            
            if not self.mola_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('/mola_runtime_param_set service not available')
            else:
                mola_req = MolaRuntimeParamSet.Request()
                mola_req.parameters = "mola::LidarOdometry:lidar_odom:\n active: false\n"
                self.mola_client.call_async(mola_req)
                
            # 4. Wait 5 seconds
            feedback_msg.current_state = "Waiting 5 seconds..."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.current_state)
            time.sleep(5.0)

        goal_handle.succeed()
        result = ToggleSystem.Result()
        result.success = True
        result.message = "System toggled successfully"
        self.get_logger().info(result.message)
        return result

def main(args=None):
    rclpy.init(args=args)
    
    node = LocalizationTriggerServer()
    
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
