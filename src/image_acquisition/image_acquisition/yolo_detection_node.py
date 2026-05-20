#!/usr/bin/env python3
"""
yolo_detection_node.py
======================
Module 1 — Image Acquisition / Detection
Subscribes to raw camera frames, runs YOLOv8s inference, and publishes
detection results with super-class labels remapped from original TACO 60-class
output to 5 super-classes (Plastic, Metal, Glass, Paper, Other).

Subscribes:
  /camera/image_raw         (sensor_msgs/Image)

Publishes:
  /detections               (vision_msgs/Detection2DArray)  — all detections
  /best_detection           (vision_msgs/Detection2D)       — highest-confidence detection
  /camera/image_annotated   (sensor_msgs/Image)             — debug view with bboxes

Parameters:
  model_path            (string) — path to best.pt weights
  confidence_threshold  (float)  — minimum confidence to publish  (default: 0.6)
  device                (string) — 'cuda' for Jetson GPU, 'cpu' fallback
  img_size              (int)    — YOLO input size (default: 640)
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

# ── Shared frame buffer for MJPEG stream ─────────────────────────────────────
_frame_lock   = threading.Lock()
_latest_frame = None


class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # silence HTTP access logs

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    frame = _latest_frame
                if frame is None:
                    time.sleep(0.05)
                    continue
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                data = jpeg.tobytes()
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(data)
                self.wfile.write(b'\r\n')
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError):
            pass


# ── Super-class definitions (from infer.py) ───────────────────────────────────

SUPER_CLASSES = ['Plastic', 'Metal', 'Glass', 'Paper', 'Other']

# BGR colors per super-class — used for annotated image
COLORS = {
    0: (0,   165, 255),   # Plastic → Orange
    1: (0,   0,   255),   # Metal   → Red
    2: (255, 255,   0),   # Glass   → Cyan
    3: (0,   255,   0),   # Paper   → Green
    4: (128,   0, 128),   # Other   → Purple
}

# Maps original TACO 60-class index → super-class index (0-4)
CLASS_MAP = {
    # Plastic → 0
    3: 0, 4: 0, 5: 0, 7: 0, 21: 0, 22: 0, 24: 0, 27: 0,
    29: 0, 36: 0, 37: 0, 38: 0, 39: 0, 40: 0, 41: 0, 42: 0,
    43: 0, 44: 0, 45: 0, 46: 0, 47: 0, 48: 0, 49: 0, 54: 0,
    55: 0, 57: 0,
    # Metal → 1
    0: 1, 1: 1, 2: 1, 8: 1, 10: 1, 11: 1, 12: 1, 28: 1, 50: 1, 52: 1,
    # Glass → 2
    6: 2, 9: 2, 23: 2, 26: 2,
    # Paper / Cardboard → 3
    13: 3, 14: 3, 15: 3, 16: 3, 17: 3, 18: 3, 19: 3, 20: 3,
    30: 3, 31: 3, 32: 3, 33: 3, 34: 3, 35: 3, 56: 3,
    # Other → 4
    25: 4, 51: 4, 53: 4, 58: 4, 59: 4,
}


# ── Node ──────────────────────────────────────────────────────────────────────

class YoloDetectionNode(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('model_path',           '/home/afro-robotics/robot_ws/models/best.engine')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('device',               'cuda')
        self.declare_parameter('img_size',             640)

        model_path  = self.get_parameter('model_path').value
        self.conf   = self.get_parameter('confidence_threshold').value
        device      = self.get_parameter('device').value
        self.imgsz  = self.get_parameter('img_size').value

        # ── Load YOLO model ───────────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            self.model = YOLO(model_path, task='detect')
            if not model_path.endswith('.engine'):
                self.model.to(device)
            self.device = device
            self.get_logger().info(f"YOLO model loaded on [{device}] — conf threshold: {self.conf}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # ── cv_bridge ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Subscriber ────────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.det_pub  = self.create_publisher(Detection2DArray, '/detections',             10)
        self.best_pub = self.create_publisher(Detection2D,      '/best_detection',         10)
        self.ann_pub  = self.create_publisher(Image,            '/camera/image_annotated', 10)

        # ── FPS tracking ──────────────────────────────────────────────────────
        self._frame_count = 0
        self._t0          = time.time()

        # ── MJPEG stream server on port 8080 ──────────────────────────────────
        server = HTTPServer(('0.0.0.0', 8080), _MJPEGHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        import socket as _socket
        try:
            _s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            _s.connect(('8.8.8.8', 80))
            _local_ip = _s.getsockname()[0]
            _s.close()
        except Exception:
            _local_ip = '0.0.0.0'
        self.get_logger().info(f"MJPEG stream: http://{_local_ip}:8080")

        self.get_logger().info("YoloDetectionNode ready — waiting for frames on /camera/image_raw")

    # ─────────────────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        """Called for every frame received from the camera node."""

        # Convert ROS2 Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ── Run YOLO inference ────────────────────────────────────────────────
        results = self.model(
            frame,
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False
        )[0]

        # ── Build Detection2DArray ────────────────────────────────────────────
        det_array        = Detection2DArray()
        det_array.header = msg.header

        best_det   = None
        best_score = -1.0

        annotated_frame = frame.copy()

        for box in results.boxes:
            original_cls = int(box.cls[0])
            super_cls    = CLASS_MAP.get(original_cls, 4)   # default → Other
            conf_score   = float(box.conf[0])
            class_name   = SUPER_CLASSES[super_cls]
            color        = COLORS[super_cls]

            # ── Bounding box pixel coordinates ────────────────────────────────
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = x2 - x1
            bh = y2 - y1

            # ── Fill Detection2D ──────────────────────────────────────────────
            det               = Detection2D()
            det.header        = msg.header
            det.id            = f"{class_name} {conf_score:.2f}"

            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = bw
            det.bbox.size_y = bh

            # Class hypothesis
            hyp                     = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name    # e.g. "Plastic"
            hyp.hypothesis.score    = conf_score
            det.results.append(hyp)

            det_array.detections.append(det)

            # Track best (highest confidence) detection
            if conf_score > best_score:
                best_score = conf_score
                best_det   = det

            # ── Draw on annotated frame ───────────────────────────────────────
            ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
            label = f"{class_name} {conf_score:.2f}"

            cv2.rectangle(annotated_frame, (ix1, iy1), (ix2, iy2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(annotated_frame, (ix1, iy1 - th - 8), (ix1 + tw + 4, iy1), color, -1)
            cv2.putText(annotated_frame, label, (ix1 + 2, iy1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # ── Publish detections ────────────────────────────────────────────────
        self.det_pub.publish(det_array)

        if best_det is not None:
            self.best_pub.publish(best_det)

        # ── Publish annotated image ───────────────────────────────────────────
        ann_msg              = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        ann_msg.header       = msg.header
        self.ann_pub.publish(ann_msg)

        # ── Update MJPEG stream buffer ────────────────────────────────────────
        global _latest_frame
        with _frame_lock:
            _latest_frame = annotated_frame.copy()

        # ── FPS log every 30 frames ───────────────────────────────────────────
        self._frame_count += 1
        if self._frame_count % 30 == 0:
            elapsed = time.time() - self._t0
            fps     = self._frame_count / elapsed
            n_det   = len(det_array.detections)
            self.get_logger().info(f"FPS: {fps:.1f} | Detections this frame: {n_det}")


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
