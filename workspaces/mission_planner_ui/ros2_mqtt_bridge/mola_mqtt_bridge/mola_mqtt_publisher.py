#!/usr/bin/env python3
"""
mola_mqtt_publisher.py
======================
ROS2 node that bridges MOLA LiDAR-Odometry topics to the Go2 Mission Planner
backend via HiveMQ MQTT.

Subscribed ROS2 Topics
-----------------------
  /lidar_odometry/pose          geometry_msgs/PoseStamped
  /lidar_odometry/localmap_points  sensor_msgs/PointCloud2

Published MQTT Topics (HiveMQ: broker.hivemq.com:1883)
------------------------------------------------------
  robot/{robot_id}/status/pose
      JSON: {
        "x": float,    # position in map frame
        "y": float,
        "z": float,
        "qx": float,   # orientation quaternion
        "qy": float,
        "qz": float,
        "qw": float,
        "frame_id": str,
        "stamp": float  # ROS time as seconds
      }

  robo_gen_labs/go2_robot_1/telemetry/points
      Binary: raw PointCloud2 data bytes (float32 XYZ packed, compatible with
      the existing Go backend WsPoints handler and Three.js frontend renderer)

Usage
-----
  ros2 run mola_mqtt_bridge mola_mqtt_publisher

Environment Variables (optional overrides)
------------------------------------------
  MQTT_BROKER    default: broker.hivemq.com
  MQTT_PORT      default: 1883
  ROBOT_ID       default: robot_01
"""

import os
import json
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

import paho.mqtt.client as mqtt_client


# ---------------------------------------------------------------------------
# Helper: pack a PointCloud2 into the binary format the Go/Three.js frontend
# expects — a flat array of float32 triplets [x0,y0,z0, x1,y1,z1, ...].
# The existing backend just stores PointsMsg as raw bytes and forwards it
# verbatim over the WebSocket; Three.js unpacks it as Float32Array.
# ---------------------------------------------------------------------------
def pointcloud2_to_flat_float32_bytes(msg: PointCloud2) -> bytes:
    """Extract XYZ floats from a PointCloud2 and return as raw float32 bytes."""
    # Locate field offsets in the point step
    field_offsets = {}
    for field in msg.fields:
        field_offsets[field.name] = field.offset

    if 'x' not in field_offsets or 'y' not in field_offsets or 'z' not in field_offsets:
        return b''

    ox = field_offsets['x']
    oy = field_offsets['y']
    oz = field_offsets['z']
    step = msg.point_step
    data = bytes(msg.data)
    num_points = msg.width * msg.height

    floats = []
    for i in range(num_points):
        base = i * step
        x = struct.unpack_from('<f', data, base + ox)[0]
        y = struct.unpack_from('<f', data, base + oy)[0]
        z = struct.unpack_from('<f', data, base + oz)[0]
        floats.extend([x, y, z])

    return struct.pack(f'<{len(floats)}f', *floats)


class MolaMqttPublisher(Node):
    """ROS2 ↔ MQTT bridge for MOLA LiDAR-Odometry."""

    def __init__(self):
        super().__init__('mola_mqtt_publisher')

        # --- Parameters (prefer env vars, fall back to defaults) -----------
        self.robot_id = os.getenv('ROBOT_ID', 'robot_01')
        broker = os.getenv('MQTT_BROKER', 'broker.hivemq.com')
        port = int(os.getenv('MQTT_PORT', '1883'))

        # --- MQTT Topics ---------------------------------------------------
        self.pose_topic = f'robot/{self.robot_id}/status/pose'
        self.points_topic = 'robo_gen_labs/go2_robot_1/telemetry/points'

        # --- MQTT Client Setup --------------------------------------------
        client_id = f'mola_mqtt_bridge_{self.robot_id}_{int(time.time())}'
        self._mqtt = mqtt_client.Client(
            client_id=client_id,
            protocol=mqtt_client.MQTTv311,
        )
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect

        self.get_logger().info(
            f'Connecting to MQTT broker {broker}:{port} '
            f'(client_id={client_id}) ...'
        )
        try:
            self._mqtt.connect(broker, port, keepalive=60)
            self._mqtt.loop_start()  # Background thread for MQTT I/O
        except Exception as exc:
            self.get_logger().error(f'MQTT connect failed: {exc}')

        # --- QoS: best-effort, keep last 1 (matches sensor data patterns) -
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- ROS2 Subscriptions -------------------------------------------
        self._pose_sub = self.create_subscription(
            PoseStamped,
            '/lidar_odometry/pose',
            self._pose_callback,
            sensor_qos,
        )
        self.get_logger().info('Subscribed to /lidar_odometry/pose')

        self._map_sub = self.create_subscription(
            PointCloud2,
            '/lidar_odometry/localmap_points',
            self._localmap_callback,
            sensor_qos,
        )
        self.get_logger().info('Subscribed to /lidar_odometry/localmap_points')

        # Rate-limit stats
        self._pose_count = 0
        self._map_count = 0
        self._stat_timer = self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f'MOLA MQTT Publisher ready  →  '
            f'pose: {self.pose_topic}  |  '
            f'points: {self.points_topic}'
        )

    # -----------------------------------------------------------------------
    # MQTT Callbacks
    # -----------------------------------------------------------------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected successfully.')
        else:
            self.get_logger().error(f'MQTT connection refused (rc={rc}).')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected unexpectedly (rc={rc}), auto-reconnecting …')

    # -----------------------------------------------------------------------
    # ROS2 → MQTT: Pose
    # -----------------------------------------------------------------------

    def _pose_callback(self, msg: PoseStamped):
        """Serialize PoseStamped → JSON → MQTT."""
        payload = {
            'x':  msg.pose.position.x,
            'y':  msg.pose.position.y,
            'z':  msg.pose.position.z,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
            'qw': msg.pose.orientation.w,
            'frame_id': msg.header.frame_id,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        try:
            self._mqtt.publish(
                self.pose_topic,
                json.dumps(payload),
                qos=0,
                retain=False,
            )
            self._pose_count += 1
        except Exception as exc:
            self.get_logger().error(f'MQTT pose publish error: {exc}')

    # -----------------------------------------------------------------------
    # ROS2 → MQTT: LocalMap PointCloud2
    # -----------------------------------------------------------------------

    def _localmap_callback(self, msg: PointCloud2):
        """Pack PointCloud2 XYZ → raw float32 bytes → MQTT binary."""
        raw = pointcloud2_to_flat_float32_bytes(msg)
        if not raw:
            self.get_logger().warn('LocalMap PointCloud2 has no XYZ fields — skipping.')
            return
        try:
            self._mqtt.publish(
                self.points_topic,
                raw,
                qos=0,
                retain=False,
            )
            self._map_count += 1
        except Exception as exc:
            self.get_logger().error(f'MQTT localmap publish error: {exc}')

    # -----------------------------------------------------------------------
    # Periodic Stats
    # -----------------------------------------------------------------------

    def _log_stats(self):
        self.get_logger().info(
            f'[5s stats]  pose msgs published: {self._pose_count}  |  '
            f'localmap msgs published: {self._map_count}'
        )
        self._pose_count = 0
        self._map_count = 0

    def destroy_node(self):
        self.get_logger().info('Shutting down MOLA MQTT Publisher …')
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MolaMqttPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
