#!/usr/bin/env python3
"""
line_follower_node.py — Détection de double-bordure par vision (OpenCV).

Subscribes:
  /camera/image_raw       (sensor_msgs/Image)

Publishes:
  /line_error             (std_msgs/Float32MultiArray)
    data[0] : erreur latérale normalisée [-1, +1]
              négatif = robot dévie à gauche (corriger à gauche)
              positif = robot dévie à droite (corriger à droite)
    data[1] : état flottant  0.0=OK  1.0=PERDU  2.0=VIRAGE
    data[2] : position X de la bordure GAUCHE en pixels  (-1 si non détectée)
    data[3] : position X de la bordure DROITE en pixels  (-1 si non détectée)
    data[4] : largeur couloir en pixels (right_cx - left_cx, 0 si non détectée)

  /camera/image_line      (sensor_msgs/Image) — frame annotée pour debug

MJPEG debug stream sur port 8081 (YOLO est sur 8080).

Paramètres :
  line_dark                (bool)  True  → murs noirs sur sol clair
  threshold                (int)   80    → seuil binarisation (0–255)
  roi_top_fraction         (float) 0.60  → début de la ROI depuis le haut
  min_line_area            (int)   800   → aire minimum du blob de bordure (px²)
  turn_line_ratio          (float) 0.60  → largeur blob / largeur frame > seuil = ligne de virage
  expected_left_ratio      (float) 0.25  → position X supposée de la bordure G si non vue
  expected_right_ratio     (float) 0.75  → position X supposée de la bordure D si non vue
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# ── États de ligne (publiés dans data[1]) ────────────────────────────────────
LINE_OK   = 0.0
LINE_LOST = 1.0
LINE_TURN = 2.0   # ligne de virage horizontale détectée → déclencher demi-tour

# ── Buffer MJPEG partagé (port 8081) ─────────────────────────────────────────
_frame_lock   = threading.Lock()
_latest_frame = None


class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

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
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError):
            pass


# ═══════════════════════════════════════════════════════════════════════════════
#  NŒUD
# ═══════════════════════════════════════════════════════════════════════════════

class LineFollowerNode(Node):

    def __init__(self):
        super().__init__('line_follower_node')

        # ── Paramètres ────────────────────────────────────────────────────────
        self.declare_parameter('line_dark',           True)
        self.declare_parameter('threshold',           80)
        self.declare_parameter('roi_top_fraction',    0.45)
        self.declare_parameter('min_line_area',       800)
        self.declare_parameter('turn_line_ratio',     0.60)
        self.declare_parameter('expected_left_ratio',  0.25)
        self.declare_parameter('expected_right_ratio', 0.75)

        self._dark      = self.get_parameter('line_dark').value
        self._thresh    = self.get_parameter('threshold').value
        self._roi_top   = self.get_parameter('roi_top_fraction').value
        self._min_area  = self.get_parameter('min_line_area').value
        self._turn_r    = self.get_parameter('turn_line_ratio').value
        self._exp_left  = self.get_parameter('expected_left_ratio').value
        self._exp_right = self.get_parameter('expected_right_ratio').value

        self._bridge = CvBridge()

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Image, '/camera/image_raw', self._cb_image, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_err   = self.create_publisher(Float32MultiArray, '/line_error',        10)
        self._pub_debug = self.create_publisher(Image,             '/camera/image_line', 10)

        # ── MJPEG debug stream sur port 8081 ──────────────────────────────────
        server = HTTPServer(('0.0.0.0', 8081), _MJPEGHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        import socket as _sock
        try:
            s = _sock.socket(_sock.AF_INET, _sock.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            ip = '0.0.0.0'

        self.get_logger().info(
            f"LineFollowerNode prêt\n"
            f"  Mode       : {'DARK (murs noirs sur sol clair)' if self._dark else 'LIGHT (blanc sur sombre)'}\n"
            f"  Seuil      : {self._thresh}\n"
            f"  ROI        : bas {int((1.0 - self._roi_top) * 100)} % du frame\n"
            f"  Aire min   : {self._min_area} px²\n"
            f"  Ligne virage: blob ≥ {int(self._turn_r * 100)} % de largeur\n"
            f"  Bordures attendues : G={int(self._exp_left*100)}%  D={int(self._exp_right*100)}%\n"
            f"  Debug MJPEG: http://{ip}:8081"
        )

    # ─────────────────────────────────────────────────────────────────────────

    def _cb_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        # ── ROI : bas du frame ────────────────────────────────────────────────
        roi_y = int(h * self._roi_top)
        roi   = frame[roi_y:h, 0:w]
        rh    = h - roi_y

        # ── Traitement image ──────────────────────────────────────────────────
        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        if self._dark:
            _, binary = cv2.threshold(blurred, self._thresh, 255, cv2.THRESH_BINARY_INV)
        else:
            _, binary = cv2.threshold(blurred, self._thresh, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── Frame de debug ────────────────────────────────────────────────────
        debug = frame.copy()
        cv2.line(debug, (0, roi_y), (w, roi_y), (0, 255, 255), 1)
        cv2.line(debug, (w // 2, roi_y), (w // 2, h), (200, 200, 200), 1)

        # ── Valeurs par défaut ────────────────────────────────────────────────
        exp_l      = float(int(w * self._exp_left))
        exp_r      = float(int(w * self._exp_right))
        left_cx    = exp_l
        right_cx   = exp_r
        error      = 0.0
        corridor_w = right_cx - left_cx
        line_state = LINE_LOST
        left_found = False
        right_found = False

        # ── 1. Chercher d'abord la ligne de virage (blob horizontal large) ────
        #    Critère : bounding-rect width ≥ turn_ratio * w  ET  width ≥ 2.5 * height
        turn_detected = False
        for c in sorted(contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(c) < self._min_area:
                break
            x, y, bw, bh = cv2.boundingRect(c)
            if bw >= self._turn_r * w and bh > 0 and bw >= 2.5 * bh:
                turn_detected = True
                cv2.rectangle(debug, (x, y + roi_y), (x + bw, y + bh + roi_y), (0, 0, 255), 2)
                cv2.putText(debug, f"TURN  w={bw}px",
                            (5, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                break

        if turn_detected:
            line_state = LINE_TURN

        else:
            # ── 2. Détection double-bordure ───────────────────────────────────
            #    Séparer les contours en moitié G / moitié D selon leur centroïde
            left_cands  = []   # (area, contour, cx, cy)
            right_cands = []

            for c in contours:
                area = cv2.contourArea(c)
                if area < self._min_area:
                    continue
                M = cv2.moments(c)
                if M['m00'] <= 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if cx < w // 2:
                    left_cands.append((area, c, cx, cy))
                else:
                    right_cands.append((area, c, cx, cy))

            if left_cands:
                _, lc, lcx, lcy = max(left_cands, key=lambda t: t[0])
                left_cx    = float(lcx)
                left_found = True
                shifted = [lc + np.array([[0, roi_y]])]
                cv2.drawContours(debug, shifted, -1, (255, 100, 0), 2)
                cv2.circle(debug, (lcx, lcy + roi_y), 6, (255, 100, 0), -1)

            if right_cands:
                _, rc, rcx, rcy = max(right_cands, key=lambda t: t[0])
                right_cx    = float(rcx)
                right_found = True
                shifted = [rc + np.array([[0, roi_y]])]
                cv2.drawContours(debug, shifted, -1, (0, 100, 255), 2)
                cv2.circle(debug, (rcx, rcy + roi_y), 6, (0, 100, 255), -1)

            if left_found or right_found:
                mid_cx     = (left_cx + right_cx) / 2.0
                corridor_w = (right_cx - left_cx) if (left_found and right_found) else corridor_w
                error      = (mid_cx - w / 2.0) / (w / 2.0)
                error      = max(-1.0, min(1.0, error))
                line_state = LINE_OK

                mid_px = int(mid_cx)
                mid_y  = roi_y + rh // 2
                cv2.circle(debug, (mid_px, mid_y), 8, (0, 255, 0), -1)
                cv2.arrowedLine(debug, (w // 2, mid_y), (mid_px, mid_y),
                                (0, 165, 255), 2, tipLength=0.2)

                both_tag = "LR" if (left_found and right_found) else ("L" if left_found else "R")
                cv2.putText(debug, f"LINE_OK[{both_tag}]  err={error:+.2f}",
                            (5, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2)

        if line_state == LINE_LOST:
            cv2.putText(debug, "LINE_LOST",
                        (5, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # ── Publication /line_error  [error, state, left_cx, right_cx, corridor_w] ──
        msg_out      = Float32MultiArray()
        msg_out.data = [error, line_state, left_cx, right_cx, corridor_w]
        self._pub_err.publish(msg_out)

        # ── Publication image annotée ─────────────────────────────────────────
        dbg_msg        = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        dbg_msg.header = msg.header
        self._pub_debug.publish(dbg_msg)

        # ── Buffer MJPEG ──────────────────────────────────────────────────────
        global _latest_frame
        with _frame_lock:
            _latest_frame = debug.copy()


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
