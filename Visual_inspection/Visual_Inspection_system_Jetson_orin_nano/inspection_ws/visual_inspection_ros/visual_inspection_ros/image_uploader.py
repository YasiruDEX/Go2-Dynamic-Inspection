"""
image_uploader.py — ROS2 Service Node
======================================
Service: /visual_inspection/upload_images
Type:    visual_inspection_interfaces/srv/UploadImages

BT calls this AFTER /visual_inspection/inspect returns image_paths.
This node reads each image file + its metadata.json from disk, then
HTTP-POSTs them to the laptop receiver server running on the same network.

Laptop server URL is read from the ROS2 param 'laptop_url'.
Set it when launching:
    ros2 run visual_inspection_ros image_uploader \
        --ros-args -p laptop_url:=http://192.168.1.XXX:8888/upload
"""

import os
import json
import pathlib
import rclpy
from rclpy.node import Node
import requests
from visual_inspection_interfaces.srv import UploadImages


class ImageUploaderNode(Node):

    def __init__(self):
        super().__init__('image_uploader')

        # ── Laptop HTTP server URL ─────────────────────────────────────────
        # Default port 8888 — must match laptop_receiver.py
        # Override with:  --ros-args -p laptop_url:=http://192.168.X.X:8888/upload
        self.declare_parameter('laptop_url', 'http://192.168.8.62:8888/upload')
        self.laptop_url = self.get_parameter('laptop_url').get_parameter_value().string_value

        # ── ROS2 Service ───────────────────────────────────────────────────
        self.srv = self.create_service(
            UploadImages,
            '/visual_inspection/upload_images',
            self._handle_upload
        )

        self.get_logger().info(f'Image uploader ready — target: {self.laptop_url}')
        self.get_logger().info('Service: /visual_inspection/upload_images')

    # ──────────────────────────────────────────────────────────────────────
    def _handle_upload(self, request, response):
        """
        Called by BT. Reads each file path from request.image_paths,
        finds metadata.json in the same folder, and POSTs everything to laptop.
        """
        image_paths  = list(request.image_paths)
        session_label = request.session_label or 'inspection'

        if not image_paths:
            response.success        = False
            response.info           = 'No image paths provided'
            response.uploaded_count = 0
            return response

        self.get_logger().info(
            f'[UPLOAD] {len(image_paths)} paths → {self.laptop_url}  label={session_label}')

        uploaded = 0
        errors   = []

        # Collect unique parent folders (each instance folder has one metadata.json)
        instance_folders = {str(pathlib.Path(p).parent) for p in image_paths}

        for folder in instance_folders:
            folder_path = pathlib.Path(folder)

            # ── Gather files to send ───────────────────────────────────────
            images_in_folder = sorted(folder_path.glob('img_*.jpg'))
            meta_file        = folder_path / 'metadata.json'

            files_to_send = []

            for img in images_in_folder:
                if img.exists():
                    files_to_send.append(
                        ('files', (img.name, open(img, 'rb'), 'image/jpeg'))
                    )

            if meta_file.exists():
                files_to_send.append(
                    ('files', ('metadata.json', open(meta_file, 'rb'), 'application/json'))
                )

            if not files_to_send:
                errors.append(f'No files found in {folder}')
                continue

            # ── HTTP POST ─────────────────────────────────────────────────
            data = {
                'session_label': session_label,
                'subfolder':     folder_path.name,   # e.g. "instance_1"
                'object_class':  folder_path.parent.name,  # e.g. "gauge"
            }

            try:
                resp = requests.post(
                    self.laptop_url,
                    files=files_to_send,
                    data=data,
                    timeout=30
                )
                if resp.status_code == 200:
                    uploaded += len(images_in_folder) + (1 if meta_file.exists() else 0)
                    self.get_logger().info(f'[UPLOAD] ✓ {folder_path.name} — {resp.json()}')
                else:
                    errors.append(f'{folder_path.name}: HTTP {resp.status_code}')
                    self.get_logger().warn(f'[UPLOAD] ✗ {folder_path.name}: {resp.status_code}')
            except requests.exceptions.ConnectionError:
                errors.append('Cannot reach laptop — is laptop_receiver.py running?')
                self.get_logger().error(
                    f'[UPLOAD] Connection failed — is laptop running? URL={self.laptop_url}')
                break
            except requests.exceptions.Timeout:
                errors.append(f'{folder_path.name}: timeout (>30s)')
                self.get_logger().error(f'[UPLOAD] Timeout uploading {folder_path.name}')
            finally:
                # Close all file handles
                for _, (_, fh, _) in files_to_send:
                    if hasattr(fh, 'close'):
                        fh.close()

        # ── Build response ────────────────────────────────────────────────
        if errors:
            response.success = False
            response.info    = f'Errors: {"; ".join(errors)}'
        else:
            response.success = True
            response.info    = f'Uploaded {uploaded} files ({len(instance_folders)} folders) to laptop'

        response.uploaded_count = uploaded
        self.get_logger().info(f'[UPLOAD] Done — success={response.success} | {response.info}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageUploaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
