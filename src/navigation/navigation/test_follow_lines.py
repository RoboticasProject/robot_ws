#!/usr/bin/env python3
"""
test_follow_lines.py — Go straight only when BOTH parallel lines are visible.
Stop immediately as soon as one line disappears.
Requires camera_node + line_follower_node running (robot_line.launch.py).
Run:  python3 src/navigation/navigation/test_follow_lines.py
"""
import time, signal, sys, smbus2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ── motors ─────────────────────────────────────────────────────────────────────
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)
CANAUX_D     = (6, 7, 8)
SPEED_FWD    = 35
SPEED_TRIM_D = 0.842

# ── line validation ────────────────────────────────────────────────────────────
FRAME_W           = 640
CENTER            = FRAME_W / 2          # 320 px
LEFT_MAX          = CENTER * 0.75        # left line must be left of 240 px
RIGHT_MIN         = CENTER * 1.25        # right line must be right of 400 px
MIN_CORRIDOR_PX   = 150                  # minimum gap between the two lines

def _find_i2c():
    for n in range(10):
        try:
            b = smbus2.SMBus(n); b.read_byte(PCA9685_ADDR); b.close(); return n
        except Exception: pass
    return None

def _pca_w(bus, reg, val):
    bus.write_byte_data(PCA9685_ADDR, reg, val, force=True)

def _pca_init(bus, freq=1000):
    _pca_w(bus, 0x00, 0x10)
    _pca_w(bus, 0xFE, int(25_000_000 / (4096 * freq) - 1))
    _pca_w(bus, 0x00, 0x00)
    time.sleep(0.005)
    _pca_w(bus, 0x00, 0xA0)

def _canal(bus, ch, v):
    reg = 0x06 + 4 * ch
    if v >= 4095:   d = [0x00, 0x10, 0x00, 0x00]
    elif v <= 0:    d = [0x00, 0x00, 0x00, 0x10]
    else:           d = [0x00, 0x00, v & 0xFF, v >> 8]
    bus.write_i2c_block_data(PCA9685_ADDR, reg, d, force=True)

def _moteur(bus, cote, vitesse):
    if cote == 'D' and vitesse != 0:
        vitesse *= SPEED_TRIM_D
    pwm = int(abs(vitesse) / 100 * 4095)
    ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
    if vitesse > 0:   _canal(bus, ch_a, 4095); _canal(bus, ch_b, 0)
    elif vitesse < 0: _canal(bus, ch_a, 0);    _canal(bus, ch_b, 4095)
    else:             _canal(bus, ch_a, 0);    _canal(bus, ch_b, 0); pwm = 0
    _canal(bus, ch_en, pwm)

def _stop(bus):
    _moteur(bus, 'G', 0)
    _moteur(bus, 'D', 0)

def _go(bus):
    _moteur(bus, 'G', SPEED_FWD)
    _moteur(bus, 'D', SPEED_FWD)

# ── ROS node ───────────────────────────────────────────────────────────────────
class FollowLinesNode(Node):
    def __init__(self, bus):
        super().__init__('follow_lines_node')
        self.bus       = bus
        self._moving   = False
        self._log_t    = None

        _stop(bus)
        self.create_subscription(
            Float32MultiArray, '/line_error', self._cb_line, 10)
        self.get_logger().info("Waiting for both parallel lines …")

    def _cb_line(self, msg):
        if len(msg.data) < 4:
            return

        left_cx  = msg.data[2]
        right_cx = msg.data[3]

        # both lines detected, on correct sides, with enough gap between them
        both = (left_cx  >= 0        and
                right_cx >= 0        and
                left_cx  < LEFT_MAX  and
                right_cx > RIGHT_MIN and
                right_cx - left_cx > MIN_CORRIDOR_PX)

        if both and not self._moving:
            self._moving = True
            _go(self.bus)
            self.get_logger().info(
                f"▶ GO — left={left_cx:.0f}px  right={right_cx:.0f}px"
                f"  corridor={right_cx - left_cx:.0f}px"
            )

        elif not both and self._moving:
            self._moving = False
            _stop(self.bus)
            l = f"{left_cx:.0f}px" if left_cx >= 0 else "MISSING"
            r = f"{right_cx:.0f}px" if right_cx >= 0 else "MISSING"
            self.get_logger().info(f"■ STOP — left={l}  right={r}")

        # periodic status while stopped
        elif not both:
            now = time.monotonic()
            if self._log_t is None or now - self._log_t >= 1.0:
                self._log_t = now
                l = f"{left_cx:.0f}px (ok)" if (left_cx >= 0 and left_cx < LEFT_MAX) \
                    else (f"{left_cx:.0f}px (wrong side)" if left_cx >= 0 else "MISSING")
                r = f"{right_cx:.0f}px (ok)" if (right_cx >= 0 and right_cx > RIGHT_MIN) \
                    else (f"{right_cx:.0f}px (wrong side)" if right_cx >= 0 else "MISSING")
                self.get_logger().info(f"  waiting — left={l}  right={r}")

# ── main ───────────────────────────────────────────────────────────────────────
def main():
    n = _find_i2c()
    if n is None:
        print("ERROR: PCA9685 not found"); return
    bus = smbus2.SMBus(n)
    _pca_init(bus)

    rclpy.init()
    node = FollowLinesNode(bus)

    def _shutdown(sig, frame):
        _stop(bus)
        bus.close()
        print("\nStopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _stop(bus)
        bus.close()

if __name__ == '__main__':
    main()
