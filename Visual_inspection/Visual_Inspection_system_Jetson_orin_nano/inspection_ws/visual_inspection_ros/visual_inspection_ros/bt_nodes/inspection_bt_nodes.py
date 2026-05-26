#!/usr/bin/env python3
"""
inspection_bt_nodes.py — py_trees_ros Behavior Tree leaf nodes
for the Visual Inspection System.

The BT person imports these into the main tree:

    from visual_inspection_ros.bt_nodes.inspection_bt_nodes import (
        InspectObjectsAction,
        CaptureOverviewAction,
        CheckObjectInBack,
    )

Run the demo standalone tree with:
    ros2 run visual_inspection_ros run_inspection_bt
"""

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time


def _wait_server(node, client, timeout=10.0):
    deadline = time.time() + timeout
    while not client.wait_for_server(timeout_sec=0.5):
        if time.time() > deadline:
            raise RuntimeError('Action server not available')


# ---------------------------------------------------------------------------
# Leaf 1: InspectObjectsAction
# ---------------------------------------------------------------------------

class InspectObjectsAction(py_trees.behaviour.Behaviour):
    """
    Action leaf: calls /visual_inspection/inspect_objects.

    Blackboard reads:
        max_objects   (int,  default 0  = all)
        return_home   (bool, default True)
        location_label (str, default "unknown")

    Blackboard writes (on completion):
        inspection_success    (bool)
        objects_found         (int)
        objects_inspected     (int)
        object_in_back        (bool)
        failed_reason         (str)
        ibvs_error_px         (float)  -- latest feedback value
    """

    def __init__(self, name='InspectObjects', node: Node = None,
                 max_objects=0, return_home=True, location_label='unknown'):
        super().__init__(name)
        self._ros_node      = node
        self._max_objects   = max_objects
        self._return_home   = return_home
        self._location_label = location_label
        self._client        = None
        self._goal_handle   = None
        self._result_future = None
        self._done          = False
        self._result        = None

        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key('inspection_success',  access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('objects_found',       access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('objects_inspected',   access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('object_in_back',      access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('failed_reason',       access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('ibvs_error_px',       access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        from visual_inspection_interfaces.action import InspectObjects as IA
        self._client = ActionClient(self._ros_node, IA,
                                    '/visual_inspection/inspect_objects')
        _wait_server(self._ros_node, self._client)
        self.logger.info(f'{self.name}: action server connected')

    def initialise(self):
        from visual_inspection_interfaces.action import InspectObjects as IA
        self._done = False
        self._result = None

        goal = IA.Goal()
        goal.max_objects   = self._max_objects
        goal.return_home   = self._return_home
        goal.location_label = self._location_label
        goal.overview_only = False
        goal.overview_count = 2

        self._send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        self._send_future.add_done_callback(self._goal_accepted_cb)

    def _feedback_cb(self, msg):
        fb = msg.feedback
        self.blackboard.ibvs_error_px = float(fb.ibvs_error_px)

    def _goal_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._done = True
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._result = future.result().result
        self._done   = True

    def update(self):
        if not self._done:
            return py_trees.common.Status.RUNNING

        r = self._result
        if r is None:
            return py_trees.common.Status.FAILURE

        self.blackboard.inspection_success  = r.success
        self.blackboard.objects_found       = r.objects_found
        self.blackboard.objects_inspected   = r.objects_inspected
        self.blackboard.object_in_back      = r.object_in_back
        self.blackboard.failed_reason       = r.failed_reason

        return py_trees.common.Status.SUCCESS if r.success else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if not self._done and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


# ---------------------------------------------------------------------------
# Leaf 2: CaptureOverviewAction  (for VLM — 360 snapshot only)
# ---------------------------------------------------------------------------

class CaptureOverviewAction(py_trees.behaviour.Behaviour):
    """
    Action leaf: sends overview_only=True goal to capture Insta360 snapshots.
    Use when BT arrives at a known location and wants a 360 image for VLM.

    Blackboard reads:
        location_label  (str) — "gauge_room_A" / "unknown"
        overview_count  (int) — how many images (default 2)

    Blackboard writes:
        overview_success (bool)
    """

    def __init__(self, name='CaptureOverview', node: Node = None,
                 location_label='unknown', overview_count=2):
        super().__init__(name)
        self._ros_node       = node
        self._location_label = location_label
        self._overview_count = overview_count
        self._done = False
        self._result = None

        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key('overview_success', access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        from visual_inspection_interfaces.action import InspectObjects as IA
        self._client = ActionClient(self._ros_node, IA,
                                    '/visual_inspection/inspect_objects')
        _wait_server(self._ros_node, self._client)

    def initialise(self):
        from visual_inspection_interfaces.action import InspectObjects as IA
        self._done = False
        self._result = None

        goal = IA.Goal()
        goal.overview_only  = True
        goal.location_label = self._location_label
        goal.overview_count = self._overview_count
        goal.max_objects    = 0
        goal.return_home    = False

        self._send_future = self._client.send_goal_async(goal)
        self._send_future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._done = True
            return
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._result = future.result().result
        self._done   = True

    def update(self):
        if not self._done:
            return py_trees.common.Status.RUNNING
        r = self._result
        ok = r is not None and r.success
        self.blackboard.overview_success = ok
        return py_trees.common.Status.SUCCESS if ok else py_trees.common.Status.FAILURE


# ---------------------------------------------------------------------------
# Leaf 3: CheckObjectInBack  (Condition)
# ---------------------------------------------------------------------------

class CheckObjectInBack(py_trees.behaviour.Behaviour):
    """
    Condition: reads 'object_in_back' from blackboard.
    SUCCESS if object was detected in back half → BT should rotate 180°.
    FAILURE if no back objects.
    """

    def __init__(self, name='CheckObjectInBack'):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key('object_in_back', access=py_trees.common.Access.READ)

    def update(self):
        in_back = getattr(self.blackboard, 'object_in_back', False)
        return (py_trees.common.Status.SUCCESS if in_back
                else py_trees.common.Status.FAILURE)


# ---------------------------------------------------------------------------
# Demo standalone tree (run with: ros2 run visual_inspection_ros run_inspection_bt)
# ---------------------------------------------------------------------------

def build_tree(node):
    """
    Example BT structure:
    Selector
    ├── Sequence (front inspection)
    │   ├── InspectObjectsAction
    │   └── (SUCCESS = all front objects done)
    └── Sequence (back-side handling)
        ├── CheckObjectInBack
        └── (BT person adds: RotateRobotAction here)
    """
    inspect = InspectObjectsAction(name='Inspect', node=node,
                                    max_objects=0, return_home=True,
                                    location_label='unknown')
    check_back = CheckObjectInBack()

    front_seq = py_trees.composites.Sequence(name='FrontInspection',
                                              memory=True,
                                              children=[inspect])

    back_seq = py_trees.composites.Sequence(name='BackHandling',
                                             memory=True,
                                             children=[check_back])

    root = py_trees.composites.Selector(name='InspectionSelector',
                                         memory=True,
                                         children=[front_seq, back_seq])
    return root


def main():
    rclpy.init()
    node = rclpy.create_node('inspection_bt_runner')

    tree_root = build_tree(node)
    bt = py_trees.trees.BehaviourTree(tree_root)
    bt.setup(timeout=15.0)

    print('[BT] Inspection Behavior Tree running...')
    print('[BT] Topic: /visual_inspection/debug for visualization')
    print('[BT] Ctrl+C to stop\n')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            bt.tick()
            status = tree_root.status
            print(f'[BT] tick  root={status.name}', end='\r')
            if status in (py_trees.common.Status.SUCCESS,
                          py_trees.common.Status.FAILURE):
                print(f'\n[BT] Tree finished: {status.name}')
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\n[BT] Stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
