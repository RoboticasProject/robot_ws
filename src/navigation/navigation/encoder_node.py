#!/usr/bin/env python3
"""
encoder_node.py — Publishes wheel encoder counts over ROS2.

Publishes /wheel_encoders (std_msgs/Int64MultiArray) at 50 Hz.
Layout:
  data[0] : left  raw count — always increasing, never reset
  data[1] : right raw count — always increasing, never reset
  data[2] : left  odometry  — signed cumulative, positive = forward
  data[3] : right odometry  — signed cumulative, positive = forward

This node owns the GPIO lines (libgpiod). No other node may import
encoder_reader directly — subscribe to /wheel_encoders instead.
"""

import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

from .encoder_reader import enc_G, enc_D


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')
        self._pub = self.create_publisher(Int64MultiArray, '/wheel_encoders', 10)
        self.create_timer(0.02, self._publish)   # 50 Hz
        self.get_logger().info('EncoderNode ready — publishing /wheel_encoders at 50 Hz')

    def _publish(self):
        msg      = Int64MultiArray()
        msg.data = [
            enc_G.get_raw(),
            enc_D.get_raw(),
            enc_G.get_odo(),
            enc_D.get_odo(),
        ]
        self._pub.publish(msg)

    def destroy_node(self):
        enc_G.stop()
        enc_D.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

    def _handle_sigterm(signum, frame):
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
