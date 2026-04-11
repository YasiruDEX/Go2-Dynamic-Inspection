import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosbag_recorder_msgs.action import Record

import subprocess
import signal
import time
import os
import shutil

class RecorderNode(Node):
    def __init__(self):
        super().__init__('rosbag_recorder_node')
        self._action_server = ActionServer(
            self,
            Record,
            'record_bag',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Recorder Action Server started (Action: record_bag).')
        self._record_proc = None

        self.bag_dir = "/home/yasiru/Documents/Far_planner_test/rosbags/recorded_mcap_bag"

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: record={goal_request.record}')
        if not goal_request.record:
            self.get_logger().warn('Goal requests record=False. Rejecting.')
            return GoalResponse.REJECT
        if self._record_proc is not None and self._record_proc.poll() is None:
            self.get_logger().warn('Recording is already in progress. Rejecting new goal.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        if os.path.exists(self.bag_dir):
            self.get_logger().info(f'Removing previous bag at {self.bag_dir}')
            shutil.rmtree(self.bag_dir, ignore_errors=True)
            
        parent_dir = os.path.dirname(self.bag_dir)
        os.makedirs(parent_dir, exist_ok=True)

        cmd = [
            "ros2", "bag", "record", 
            "/livox/lidar", "/livox/imu", 
            "-s", "mcap", 
            "-o", self.bag_dir
        ]
        
        self.get_logger().info(f'Starting recording process: {" ".join(cmd)}')
        self._record_proc = subprocess.Popen(cmd)

        feedback_msg = Record.Feedback()
        start_time = time.time()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled, stopping recording...')
                if self._record_proc and self._record_proc.poll() is None:
                    self._record_proc.send_signal(signal.SIGINT)
                    self._record_proc.wait()
                break

            if self._record_proc and self._record_proc.poll() is not None:
                self.get_logger().info('Recording process stopped independently.')
                break
                
            feedback_msg.duration = float(time.time() - start_time)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)
            
        result = Record.Result()
        if not goal_handle.is_cancel_requested:
            result.success = True
            result.message = "Recording stopped."
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded.')
            
        return result

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if node._record_proc and node._record_proc.poll() is None:
            node._record_proc.send_signal(signal.SIGINT)
            node._record_proc.wait()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
