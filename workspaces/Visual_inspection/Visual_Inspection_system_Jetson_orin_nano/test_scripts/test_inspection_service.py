#!/usr/bin/env python3
"""
test_inspection_service.py — Test client for the inspection ROS2 service.

Simulates what the Behaviour Tree sends.

Usage:
  python3 test_inspection_service.py --object fire_extinguisher
  python3 test_inspection_service.py --object gauge
  python3 test_inspection_service.py --object unknown
  python3 test_inspection_service.py --object main_cylinder
  python3 test_inspection_service.py --object door
  python3 test_inspection_service.py --object person
  python3 test_inspection_service.py               # detect all classes
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from visual_inspection_interfaces.srv import Inspect


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object',   default='',        help='Target object class')
    parser.add_argument('--location', default='test_loc', help='Location label')
    parser.add_argument('--max',      default=0, type=int, help='Max objects (0=all)')
    parser.add_argument('--no-home',  action='store_true', help='Skip return home')
    args = parser.parse_args()

    rclpy.init()
    node = Node('inspection_test_client')
    cli  = node.create_client(Inspect, '/visual_inspection/inspect')

    print(f'\nWaiting for /visual_inspection/inspect service...')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('[ERROR] Service not available')
        sys.exit(1)

    req = Inspect.Request()
    req.target_object  = args.object
    req.location_label = args.location
    req.max_objects    = args.max
    req.return_home    = not args.no_home

    obj_label = args.object or 'any'
    print(f'\n{"="*55}')
    print(f'  Sending request: object="{obj_label}"  loc="{args.location}"')
    print(f'{"="*55}')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        print('[ERROR] Service call failed')
        sys.exit(1)

    res = future.result()
    print(f'\n--- RESPONSE ---')
    print(f'  success          : {res.success}')
    print(f'  status           : {res.status}')
    print(f'  objects_found    : {res.objects_found}')
    print(f'  objects_inspected: {res.objects_inspected}')
    print(f'  object_in_back   : {res.object_in_back}')
    print(f'  info             : {res.info}')
    print(f'  images saved ({len(res.image_paths)}):')
    for p in res.image_paths:
        print(f'    {p}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
