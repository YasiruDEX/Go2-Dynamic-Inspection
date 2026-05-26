#!/usr/bin/env python3
"""
test_full_pipeline.py
=====================
Stand-alone test script that simulates the Behavior Tree.

Runs the full inspection pipeline without a BT:
  1. Calls /visual_inspection/inspect_objects action
  2. Monitors feedback in real-time
  3. Handles all result cases (success / back-side / multi-object / timeout)
  4. Optionally loops (simulating 'return and retry after back-side detected')

Usage:
  source /opt/ros/humble/setup.bash
  source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
  python3 test_scripts/test_full_pipeline.py

  Options (edit constants below):
    MAX_OBJECTS   - 0 = inspect all, N = first N only
    RETURN_HOME   - servo returns to 90,90 after each run
    AUTO_RETRY    - if object_in_back=True, wait RETRY_WAIT then resend goal
    RETRY_WAIT    - seconds to wait before retry (simulates robot rotate)
    RUN_ONCE      - True = one run then exit, False = loop forever
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visual_inspection_interfaces.action import InspectObjects

import sys
import time

# ---- Config (edit here) -----------------------------------------------
MAX_OBJECTS  = 0      # 0 = inspect all detected front objects
RETURN_HOME  = True   # servo returns to 90,90 after done
AUTO_RETRY   = True   # auto-retry if object_in_back detected
RETRY_WAIT   = 3.0    # seconds before retry (simulates robot 180° rotation)
RUN_ONCE     = False  # True = one run; False = loop until Ctrl+C
# -----------------------------------------------------------------------


class PipelineTester(Node):

    def __init__(self):
        super().__init__('pipeline_tester')
        self._client = ActionClient(
            self, InspectObjects, '/visual_inspection/inspect_objects')
        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()
        self.get_logger().info('Action server found -- ready to test')

    def run_one(self, run_id=1):
        """Send one inspection goal and wait for result."""
        print(f'\n{"="*60}')
        print(f'  RUN {run_id}  (max_objects={MAX_OBJECTS}  return_home={RETURN_HOME})')
        print(f'{"="*60}')

        goal = InspectObjects.Goal()
        goal.max_objects = MAX_OBJECTS
        goal.return_home = RETURN_HOME

        future        = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        self._result  = None
        self._done    = False

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('[ERROR] Goal rejected by server')
            return None

        print('[OK] Goal accepted')
        result_future = goal_handle.get_result_async()

        # Spin until result received
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result().result
        status = result_future.result().status

        self._print_result(result, status)
        return result

    def _feedback_cb(self, msg):
        fb = msg.feedback
        bar = '.' * int(max(0, 60 - fb.ibvs_error_px))
        print(f'  [{fb.current_step.upper():10s}] obj={fb.current_object}  '
              f'IBVS err={fb.ibvs_error_px:6.1f}px  {bar}', end='\r')
        sys.stdout.flush()

    def _print_result(self, result, status):
        print()  # newline after feedback line
        print(f'\n{"─"*50}')
        print(f'  RESULT (status={status})')
        print(f'  success          : {result.success}')
        print(f'  objects_found    : {result.objects_found}')
        print(f'  objects_inspected: {result.objects_inspected}')
        print(f'  object_in_back   : {result.object_in_back}')
        print(f'  failed_reason    : "{result.failed_reason}"')
        print(f'{"─"*50}')

        # Interpret like a BT would
        if result.success and not result.object_in_back:
            print('  [BT] -> SUCCESS: All front objects inspected. Return to base.')
        elif result.success and result.object_in_back:
            print('  [BT] -> SUCCESS (front done) + BACK DETECTED -> rotate 180° then retry')
        elif result.object_in_back and result.failed_reason == 'all_in_back':
            print('  [BT] -> All objects in BACK -> rotate 180° then retry')
        elif result.failed_reason == 'no_detection':
            print('  [BT] -> No objects found. Navigate to next position or stop.')
        elif result.failed_reason == 'ibvs_timeout':
            print('  [BT] -> IBVS did not converge. Check calibration. Move closer?')
        elif result.failed_reason == 'logi_no_detection':
            print('  [BT] -> Object not visible in Logitech after coarse. Recalibrate?')
        else:
            print(f'  [BT] -> Unknown state: {result.failed_reason}')

    def run_loop(self):
        run_id    = 1
        max_runs  = 1 if RUN_ONCE else 999

        while run_id <= max_runs:
            result = self.run_one(run_id)
            if result is None:
                print('[ERROR] No result received — is the action server running?')
                break

            # Simulate BT decision
            if result.object_in_back or result.failed_reason in ('all_in_back',):
                if AUTO_RETRY:
                    print(f'\n[TEST] Back-side detected — waiting {RETRY_WAIT}s '
                          f'(simulating 180° rotation)...')
                    time.sleep(RETRY_WAIT)
                    run_id += 1
                    continue
                else:
                    print('[TEST] Back-side detected. Set AUTO_RETRY=True to auto-retry.')
                    break

            if result.success:
                if RUN_ONCE:
                    print('\n[TEST] Done. Exiting.')
                    break
                else:
                    print(f'\n[TEST] Run {run_id} complete. Press Ctrl+C to stop. '
                          f'Next run in 3s...')
                    time.sleep(3.0)
                    run_id += 1
            else:
                print(f'\n[TEST] Failed: {result.failed_reason}. '
                      'Check nodes and try again.')
                if RUN_ONCE:
                    break
                time.sleep(3.0)
                run_id += 1


def main():
    print('Visual Inspection Pipeline Test')
    print('  Make sure these are running:')
    print('  T1: ros2 run visual_inspection_ros camera_node')
    print('  T2: ros2 run visual_inspection_ros servo_node')
    print('  T3: ros2 run visual_inspection_ros ibvs_action_server')
    print('  RViz2: subscribe to /visual_inspection/debug')
    print()
    print('  Monitor topics:')
    print('    ros2 topic echo /visual_inspection/status')
    print('    ros2 topic echo /visual_inspection/ibvs_error')
    print('    ros2 topic echo /visual_inspection/detections')
    print()

    rclpy.init()
    node = PipelineTester()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        print('\n[TEST] Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
