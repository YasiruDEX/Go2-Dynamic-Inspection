import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from map_creator_msgs.action import CreateMap

import subprocess
import signal
import time
import os
import pty
import threading

class MapCreatorNode(Node):
    def __init__(self):
        super().__init__('map_creator_node')
        self._action_server = ActionServer(
            self,
            CreateMap,
            'create_map',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Map Creator Action Server started (Action: create_map).')
        self._mola_proc = None
        self._sm2mm_proc = None
        self.map_dir = "/home/yasiru/Documents/Far_planner_test/maps/"

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request for rosbag: {goal_request.rosbag_path}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal for rosbag: {goal_handle.request.rosbag_path}')
        rosbag_path = goal_handle.request.rosbag_path
        
        # Ensure map dir exists
        os.makedirs(self.map_dir, exist_ok=True)
        
        # 1. Run mola-lo-gui-rosbag2
        env = os.environ.copy()
        env['MOLA_GENERATE_SIMPLEMAP'] = 'true'
        env['MOLA_SIMPLEMAP_OUTPUT'] = 'myMap.simplemap'
        env['MOLA_SIMPLEMAP_MIN_XYZ'] = '0.2'
        env['MOLA_LO_INITIAL_LOCALIZATION_METHOD'] = 'InitLocalization::PitchAndRollFromIMU'
        env['MOLA_DESKEW_METHOD'] = 'MotionCompensationMethod::IMU'
        env['MOLA_IMU_TOPIC'] = '/livox/imu'
        env['MOLA_LIDAR_TOPIC'] = '/livox/lidar'
        env['MOLA_TF_BASE_LINK'] = 'livox_frame'
        
        cmd1 = ["mola-lo-gui-rosbag2", rosbag_path]
        
        self.get_logger().info('Running mola-lo-gui-rosbag2 pipeline...')
        
        master, slave = pty.openpty()
        self._mola_proc = subprocess.Popen(
            cmd1, env=env, cwd=self.map_dir,
            stdout=slave, stderr=subprocess.STDOUT,
            start_new_session=True  # give it its own process group
        )
        os.close(slave)

        dataset_end_event = threading.Event()

        def read_output(fd, event):
            with os.fdopen(fd, 'r', encoding='utf-8', errors='replace') as f:
                try:
                    for line in f:
                        print(line, end='', flush=True)
                        if "End of dataset reached!" in line:
                            event.set()
                except OSError:
                    pass

        stdout_thread = threading.Thread(
            target=read_output, 
            args=(master, dataset_end_event), 
            daemon=True
        )
        stdout_thread.start()

        feedback_msg = CreateMap.Feedback()
        feedback_msg.status = 'Running mola-lo-gui-rosbag2 pipeline'
        goal_handle.publish_feedback(feedback_msg)
        
        sigint_sent_for_completion = False
        
        while self._mola_proc.poll() is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled, killing mola pipeline...')
                try:
                    os.killpg(os.getpgid(self._mola_proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
                self._mola_proc.wait()
                try: os.close(master)
                except OSError: pass
                return CreateMap.Result()
                
            if dataset_end_event.is_set() and not sigint_sent_for_completion:
                self.get_logger().info('Dataset end reached! Sending SIGINT to process group to save map.')
                try:
                    os.killpg(os.getpgid(self._mola_proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
                sigint_sent_for_completion = True
                
            time.sleep(1.0)
            
        stdout_thread.join(timeout=2.0)
        try: os.close(master)
        except OSError: pass
            
        if self._mola_proc.returncode != 0 and not sigint_sent_for_completion:
            self.get_logger().error(f'mola-lo-gui-rosbag2 failed with code {self._mola_proc.returncode}')
            result = CreateMap.Result()
            result.success = False
            result.message = "mola-lo-gui-rosbag2 failed"
            goal_handle.abort()
            return result
            
        # 2. Run sm2mm
        simplemap_path = os.path.join(self.map_dir, 'myMap.simplemap')
        mm_path = os.path.join(self.map_dir, 'myMap.mm')
        yaml_path = "/home/yasiru/Documents/Far_planner_test/maps/sm2mm_no_decim_imu_mls_keyframe_map.yaml"
        
        cmd2 = ["sm2mm", "-i", simplemap_path, "-o", mm_path, "-p", yaml_path]
        
        self.get_logger().info('Running sm2mm conversion...')
        feedback_msg.status = 'Running sm2mm conversion'
        goal_handle.publish_feedback(feedback_msg)
        
        master2, slave2 = pty.openpty()
        self._sm2mm_proc = subprocess.Popen(cmd2, cwd=self.map_dir, stdout=slave2, stderr=subprocess.STDOUT)
        os.close(slave2)
        
        def read_output2(fd):
            with os.fdopen(fd, 'r', encoding='utf-8', errors='replace') as f:
                try:
                    for line in f:
                        print(line, end='', flush=True)
                except OSError:
                    pass

        stdout_thread2 = threading.Thread(target=read_output2, args=(master2,), daemon=True)
        stdout_thread2.start()
        
        while self._sm2mm_proc.poll() is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled, killing sm2mm...')
                self._sm2mm_proc.send_signal(signal.SIGINT)
                self._sm2mm_proc.wait()
                try: os.close(master2)
                except OSError: pass
                return CreateMap.Result()
            time.sleep(1.0)
            
        stdout_thread2.join(timeout=2.0)
        try: os.close(master2)
        except OSError: pass
            
        result = CreateMap.Result()
        if self._sm2mm_proc.returncode == 0:
            result.success = True
            result.message = "Map created successfully"
            result.map_path = mm_path
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded.')
        else:
            result.success = False
            result.message = "sm2mm failed"
            goal_handle.abort()
            self.get_logger().error('sm2mm failed.')
            
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MapCreatorNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if node._mola_proc and node._mola_proc.poll() is None:
            node._mola_proc.send_signal(signal.SIGINT)
            node._mola_proc.wait()
        if node._sm2mm_proc and node._sm2mm_proc.poll() is None:
            node._sm2mm_proc.send_signal(signal.SIGINT)
            node._sm2mm_proc.wait()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
