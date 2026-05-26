#!/usr/bin/env python3
"""
test_bt_full_flow.py — Simulates the full Behaviour Tree inspection flow
=========================================================================
This script acts exactly like a BT node would:

  Step 1: Call /visual_inspection/inspect        → get image_paths
  Step 2: Call /visual_inspection/upload_images  → send images+metadata to laptop

Run this INSTEAD of test_inspection_service.py when you want to test the
full end-to-end pipeline including the HTTP upload to laptop.

REQUIREMENTS (all must be running first):
  Jetson Terminal 1: ros2 run visual_inspection_ros camera_node
  Jetson Terminal 2: ros2 run visual_inspection_ros servo_node
  Jetson Terminal 3: ros2 run visual_inspection_ros inspection_service
  Jetson Terminal 4: ros2 run visual_inspection_ros image_uploader
  Laptop:            python3 Evaluation_V_I_ws/laptop_receiver.py

USAGE:
  python3 test_bt_full_flow.py --object gauge --location engine_room_A
  python3 test_bt_full_flow.py --object fire_extinguisher --location corridor_B
  python3 test_bt_full_flow.py --object door --location engine_room_A
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from visual_inspection_interfaces.srv import Inspect, UploadImages


# ── ANSI colours ──────────────────────────────────────────────────────────────
G = '\033[92m'   # green
R = '\033[91m'   # red
Y = '\033[93m'   # yellow
B = '\033[94m'   # blue
W = '\033[97m'   # white
X = '\033[0m'    # reset


class BTFlowTester(Node):
    """Mimics a BT leaf node — inspect then upload."""

    def __init__(self, target_object: str, location_label: str):
        super().__init__('bt_flow_tester')
        self.target_object  = target_object
        self.location_label = location_label

        self.inspect_client = self.create_client(Inspect, '/visual_inspection/inspect')
        self.upload_client  = self.create_client(UploadImages, '/visual_inspection/upload_images')

    # ──────────────────────────────────────────────────────────────────────────
    def wait_for_services(self, timeout_sec=10.0):
        print(f'\n{B}[BT] Waiting for services...{X}')
        ok_i = self.inspect_client.wait_for_service(timeout_sec=timeout_sec)
        ok_u = self.upload_client.wait_for_service(timeout_sec=timeout_sec)

        if not ok_i:
            print(f'{R}[BT] ✗ /visual_inspection/inspect not available{X}')
            print(f'     Is inspection_service running? (Terminal 3)')
            return False
        if not ok_u:
            print(f'{R}[BT] ✗ /visual_inspection/upload_images not available{X}')
            print(f'     Is image_uploader running? (Terminal 4)')
            return False

        print(f'{G}[BT] ✓ Both services ready{X}')
        return True

    # ──────────────────────────────────────────────────────────────────────────
    def step1_inspect(self):
        """Step 1: Call inspection service (same as BT Inspect leaf node)."""
        print(f'\n{W}━━━ STEP 1: Visual Inspection ━━━━━━━━━━━━━━━━━━━━━━━━━━━━{X}')
        print(f'{B}[BT] Requesting inspection of "{self.target_object}" at "{self.location_label}"{X}')

        req = Inspect.Request()
        req.target_object  = self.target_object
        req.location_label = self.location_label
        req.max_objects    = 0      # inspect all found
        req.return_home    = True

        future = self.inspect_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

        if future.result() is None:
            print(f'{R}[BT] ✗ Inspect service call failed (timeout or crash){X}')
            return None

        res = future.result()
        print(f'\n{B}[BT] Inspect response:{X}')
        print(f'     success          : {G if res.success else R}{res.success}{X}')
        print(f'     status           : {res.status}')
        print(f'     objects_found    : {res.objects_found}')
        print(f'     objects_inspected: {res.objects_inspected}')
        print(f'     object_in_back   : {res.object_in_back}')
        print(f'     info             : {res.info}')
        print(f'     image_paths ({len(res.image_paths)}):')
        for p in res.image_paths:
            print(f'       {Y}{p}{X}')

        if res.object_in_back:
            print(f'\n{Y}[BT] ⚠ Object in back — BT should rotate robot 180° and retry{X}')
            return None

        if not res.success or not res.image_paths:
            print(f'\n{R}[BT] ✗ Inspection failed or no images — status: {res.status}{X}')
            return None

        print(f'\n{G}[BT] ✓ Inspection complete — proceeding to upload{X}')
        return list(res.image_paths)

    # ──────────────────────────────────────────────────────────────────────────
    def step2_upload(self, image_paths: list):
        """Step 2: Call upload service (same as BT Upload leaf node)."""
        print(f'\n{W}━━━ STEP 2: Upload Images to Laptop ━━━━━━━━━━━━━━━━━━━━━━{X}')
        print(f'{B}[BT] Uploading {len(image_paths)} paths with label="{self.location_label}"{X}')

        req = UploadImages.Request()
        req.image_paths   = image_paths
        req.session_label = self.location_label  # ← this becomes the top folder on laptop

        future = self.upload_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            print(f'{R}[BT] ✗ Upload service call failed (timeout or crash){X}')
            return False

        res = future.result()
        print(f'\n{B}[BT] Upload response:{X}')
        print(f'     success        : {G if res.success else R}{res.success}{X}')
        print(f'     uploaded_count : {res.uploaded_count}')
        print(f'     info           : {res.info}')

        if res.success:
            print(f'\n{G}[BT] ✓ Upload complete — files saved on laptop at:{X}')
            print(f'     {Y}received_captures/{self.location_label}/...{X}')
        else:
            print(f'\n{R}[BT] ✗ Upload failed — is laptop_receiver.py running on the laptop?{X}')
            print(f'     Run on laptop: python3 Evaluation_V_I_ws/laptop_receiver.py')

        return res.success

    # ──────────────────────────────────────────────────────────────────────────
    def run(self):
        if not self.wait_for_services():
            return False

        # Step 1 — Inspect
        image_paths = self.step1_inspect()
        if image_paths is None:
            return False

        # Step 2 — Upload
        upload_ok = self.step2_upload(image_paths)

        # ── Final BT result ────────────────────────────────────────────────
        print(f'\n{W}━━━ BT RESULT ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{X}')
        if upload_ok:
            print(f'{G}✓ BT node → SUCCESS{X}')
            print(f'  Inspection done + images on laptop at: received_captures/{self.location_label}/')
        else:
            print(f'{R}✗ BT node → FAILURE (upload failed — BT can retry){X}')
        return upload_ok


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Simulate the full BT flow: inspect → upload to laptop')
    parser.add_argument('--object',   '-o',
        default='gauge',
        choices=['gauge', 'fire_extinguisher', 'extinguisher',
                 'door', 'person', 'unknown', 'main_cylinder'],
        help='Object to inspect (default: gauge)')
    parser.add_argument('--location', '-l',
        default='test_location',
        help='Location label from BT map (e.g. engine_room_A). Used as folder on laptop.')
    args = parser.parse_args()

    print(f'\n{W}Visual Inspection — Full BT Flow Test{X}')
    print(f'{W}Target: {G}{args.object}{X}  Location: {G}{args.location}{X}\n')

    rclpy.init()
    node = BTFlowTester(
        target_object=args.object,
        location_label=args.location
    )

    success = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
