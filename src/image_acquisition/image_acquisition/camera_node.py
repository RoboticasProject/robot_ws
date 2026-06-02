#!/usr/bin/env python3
"""
camera_node.py
==============
Module 1 — Image Acquisition
Captures frames from a USB camera and publishes them as ROS2 Image messages.

Publishes:
  /camera/image_raw  (sensor_msgs/Image)

Parameters:
  device  (string)  — camera device path  (default: '/dev/video0')
  width   (int)     — capture width        (default: 640)
  height  (int)     — capture height       (default: 480)
  fps     (int)     — frames per second    (default: 30)
"""

import glob
import signal
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Max attempts to open the device if the V4L2 driver is still releasing it
_OPEN_RETRIES     = 5
_OPEN_RETRY_DELAY = 1.2   # seconds between retries


def _find_video_capture_device() -> str:
    """
    Scan /dev/video* in order and return the first node that:
      - opens with cv2.VideoCapture
      - successfully returns a frame (confirms it is a VIDEO_CAPTURE node,
        not a METADATA_CAPTURE-only node which opens but returns no frames)
    Returns None if nothing found.
    """
    for path in sorted(glob.glob('/dev/video*')):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            cap.release()
            continue
        # Warm-up: some cameras need a couple of reads before the first frame
        ok = False
        for _ in range(5):
            ret, _ = cap.read()
            if ret:
                ok = True
                break
            time.sleep(0.05)
        cap.release()
        if ok:
            return path
    return None


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',    30)

        device = self.get_parameter('device').value
        width  = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps    = self.get_parameter('fps').value

        self.bridge = CvBridge()
        self.cap    = None

        # ── Auto-detect if caller passed 'auto' ───────────────────────────────
        if device.lower() == 'auto':
            found = _find_video_capture_device()
            if found is None:
                self.get_logger().fatal("Auto-detect: no working video capture device found.")
                raise RuntimeError("No video capture device available.")
            self.get_logger().info(f"Auto-detected camera: {found}")
            device = found

        # ── Open camera — retry loop (handles stale V4L2 lock after restart) ─
        for attempt in range(1, _OPEN_RETRIES + 1):
            cap = cv2.VideoCapture(device)
            if cap.isOpened():
                self.cap = cap
                break
            cap.release()
            if attempt < _OPEN_RETRIES:
                self.get_logger().warn(
                    f"Cannot open {device} (attempt {attempt}/{_OPEN_RETRIES}), "
                    f"retrying in {_OPEN_RETRY_DELAY:.1f} s..."
                )
                time.sleep(_OPEN_RETRY_DELAY)

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().fatal(f"Cannot open camera after {_OPEN_RETRIES} attempts: {device}")
            raise RuntimeError(f"Camera device {device} not available.")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,   width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  height)
        self.cap.set(cv2.CAP_PROP_FPS,           fps)

        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera opened: {device} | {actual_w}×{actual_h} @ {actual_fps:.1f} fps"
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ── Timer ────────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    # ─────────────────────────────────────────────────────────────────────────

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame — skipping.")
            return

        msg                 = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released.")
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = None

    def _handle_sigterm(signum, frame):
        """SIGTERM handler — ros2 launch sends SIGTERM on shutdown."""
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
