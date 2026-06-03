#!/usr/bin/env python3
"""
test_wait_lines.py — Robot stays frozen until both parallel lines are detected.
Once both left AND right lines are seen, robot starts moving straight.
Requires: camera_node + line_follower_node running.
Run:  ros2 run navigation test_wait_lines  (or python3 test_wait_lines.py)
"""
import time, signal, sys, smbus2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)
CANAUX_D     = (6, 7, 8)
SPEED_FWD    = 35
SPEED_TRIM_D = 0.842

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


class WaitLinesNode(Node):
    def __init__(self, bus):
        super().__init__('wait_lines_node')
        self.bus     = bus
        self._ready  = False
        self._log_t  = None

        self.create_subscription(
            Float32MultiArray, '/line_error', self._cb_line, 10)

        _stop(bus)
        self.get_logger().info(
            "Robot frozen — waiting for BOTH parallel lines (left + right)…"
        )

    def _cb_line(self, msg):
        if self._ready:
            return

        if len(msg.data) < 4:
            return

        left_cx  = msg.data[2]
        right_cx = msg.data[3]
        both_seen = left_cx >= 0 and right_cx >= 0

        # Print status every second while waiting
        now = time.monotonic()
        if self._log_t is None or now - self._log_t >= 1.0:
            self._log_t = now
            l_str = f"{left_cx:.0f}px" if left_cx >= 0 else "NOT SEEN"
            r_str = f"{right_cx:.0f}px" if right_cx >= 0 else "NOT SEEN"
            self.get_logger().info(
                f"  Left line: {l_str}   Right line: {r_str}"
            )

        if both_seen:
            self._ready = True
            self.get_logger().info(
                f"✓ Both lines detected !  "
                f"left={left_cx:.0f}px  right={right_cx:.0f}px  "
                f"corridor={right_cx - left_cx:.0f}px"
            )
            self.get_logger().info("→ Starting straight movement.")
            _moteur(self.bus, 'G', SPEED_FWD)
            _moteur(self.bus, 'D', SPEED_FWD)


def main():
    n = _find_i2c()
    if n is None:
        print("ERROR: PCA9685 not found"); return
    bus = smbus2.SMBus(n)
    _pca_init(bus)

    rclpy.init()
    node = WaitLinesNode(bus)

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
