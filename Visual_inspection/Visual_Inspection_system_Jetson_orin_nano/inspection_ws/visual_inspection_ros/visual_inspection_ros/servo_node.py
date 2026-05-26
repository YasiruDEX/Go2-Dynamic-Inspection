#!/usr/bin/env python3
"""
servo_node.py -- ROS2 Node: Arduino servo control via serial.

Subscribes to /servo/pan_tilt (std_msgs/Int16MultiArray: [tilt, pan])
Writes "tilt,pan\n" to Arduino via /dev/arduino serial port.

Uses same find_arduino() as ibvs_pipeline.py.

Test:
  ros2 topic pub /servo/pan_tilt std_msgs/msg/Int16MultiArray \
    "{layout: {dim: [], data_offset: 0}, data: [90, 90]}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

import serial
import glob
import os


# -- Arduino Detection (same as ibvs_pipeline.py) -----------------------------

def find_arduino():
    """Find Arduino by USB vendor ID -- works regardless of enumeration order."""
    ARDUINO_VIDS = {'2341', '1a86', '0403'}

    if os.path.exists('/dev/arduino'):
        print('   [arduino] Found via udev symlink: /dev/arduino')
        return '/dev/arduino'

    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for port in ports:
        try:
            dev_name = os.path.basename(port)
            check = os.path.realpath(f'/sys/class/tty/{dev_name}')
            for _ in range(8):
                vid_file = os.path.join(check, 'idVendor')
                if os.path.exists(vid_file):
                    with open(vid_file) as f:
                        vid = f.read().strip()
                    if vid in ARDUINO_VIDS:
                        print(f'   [arduino] Found at {port} (VID={vid})')
                        return port
                    break
                check = os.path.dirname(check)
        except:
            pass

    print('   [arduino] Not found')
    return None


# -- ROS2 Node ----------------------------------------------------------------

class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_controller')

        self.declare_parameter('baud_rate', 9600)
        baud = self.get_parameter('baud_rate').value

        self.get_logger().info('Finding Arduino...')
        port = find_arduino()

        self.arduino = None
        if port:
            try:
                self.arduino = serial.Serial(port, baud, timeout=1)
                import time; time.sleep(2)
                self.get_logger().info(f'Arduino connected at {port} ({baud} baud)')
            except serial.SerialException as e:
                self.get_logger().error(f'Arduino serial error: {e}')
                self.get_logger().error('Try: sudo chmod 666 /dev/ttyACM0')
        else:
            self.get_logger().warn('Arduino not found -- servo commands will be ignored')

        self.sub = self.create_subscription(
            Int16MultiArray,
            '/servo/pan_tilt',
            self.servo_callback,
            10
        )

        self.get_logger().info('Servo node ready -- listening on /servo/pan_tilt')
        self.get_logger().info('Format: data=[tilt, pan]  Range: 0-180  Home: [90, 90]')

    def servo_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Expected [tilt, pan] -- got fewer values')
            return

        tilt = max(0, min(180, int(msg.data[0])))
        pan  = max(0, min(180, int(msg.data[1])))
        cmd  = f'{tilt},{pan}\n'

        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(cmd.encode())
                self.get_logger().info(f'Servo command: tilt={tilt} pan={pan}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
        else:
            self.get_logger().warn(f'Arduino not connected -- would send: {cmd.strip()}')

    def destroy_node(self):
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(b'90,90\n')
                self.get_logger().info('Servos returned to home position (90,90)')
            except:
                pass
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
