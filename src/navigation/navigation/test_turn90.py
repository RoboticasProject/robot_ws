#!/usr/bin/env python3
"""
Right-wheel-only 90° pivot test.
Left wheel stays still; right wheel turns forward until the theoretical
90° arc is reached, then stops and reports actual encoder pulses.
Run:  python3 test_turn90.py
"""
import math, time, smbus2, rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

# ── constants (keep in sync with navigation_node.py) ──────────────────────────
PPR           = 1475   # PPR_EFFECTIF from encoder_reader.py
WHEEL_DIAM_MM = 65.0
WHEEL_CIRC    = math.pi * WHEEL_DIAM_MM
MM_PER_PULSE  = WHEEL_CIRC / PPR
WHEELBASE_MM  = 135.0   # measured from test result (was wrong at 300)

# single-wheel pivot: right wheel travels π * WHEELBASE / 2 for 90°
PUL_TARGET    = 1100   # fixed test value

SPEED_TURN    = 35
SPEED_TRIM_D  = 0.861

PCA9685_ADDR  = 0x40
CANAUX_G      = (3, 5, 4)
CANAUX_D      = (6, 7, 8)

LOOP_HZ       = 50

# ── PCA9685 helpers ────────────────────────────────────────────────────────────
def _find_i2c(addr=PCA9685_ADDR):
    for n in range(10):
        try:
            b = smbus2.SMBus(n); b.read_byte(addr); b.close(); return n
        except Exception:
            pass
    return None

def pca_w(bus, reg, val):
    bus.write_byte_data(PCA9685_ADDR, reg, val)

def pca_init(bus, freq=1000):
    pca_w(bus, 0x00, 0x10)
    time.sleep(0.005)
    prescale = round(25_000_000 / (4096 * freq)) - 1
    pca_w(bus, 0xFE, prescale)
    pca_w(bus, 0x00, 0x00)
    time.sleep(0.005)
    pca_w(bus, 0x00, 0xA1)

def pca_canal(bus, ch, val):
    val = max(0, min(4095, val))
    base = 0x06 + 4 * ch
    if val == 4095:
        bus.write_i2c_block_data(PCA9685_ADDR, base, [0x00, 0x10, 0x00, 0x00])
    elif val == 0:
        bus.write_i2c_block_data(PCA9685_ADDR, base, [0x00, 0x00, 0x00, 0x10])
    else:
        bus.write_i2c_block_data(PCA9685_ADDR, base, [0x00, 0x00, val & 0xFF, val >> 8])

def moteur(bus, cote, vitesse):
    if cote == 'D' and vitesse != 0:
        vitesse_eff = vitesse * SPEED_TRIM_D
    else:
        vitesse_eff = vitesse
    pwm = int(abs(vitesse_eff) / 100 * 4095)
    ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
    if vitesse > 0:
        pca_canal(bus, ch_a, 4095); pca_canal(bus, ch_b, 0)
    elif vitesse < 0:
        pca_canal(bus, ch_a, 0);    pca_canal(bus, ch_b, 4095)
    else:
        pca_canal(bus, ch_a, 0);    pca_canal(bus, ch_b, 0); pwm = 0
    pca_canal(bus, ch_en, pwm)

def stop_all(bus):
    moteur(bus, 'G', 0)
    moteur(bus, 'D', 0)

# ── ROS node ───────────────────────────────────────────────────────────────────
class TurnTest(Node):
    def __init__(self, bus):
        super().__init__('turn_test')
        self.bus = bus
        self._enc_left = 0
        self._start_left = None
        self._done = False

        self.create_subscription(
            Int64MultiArray, '/wheel_encoders', self._cb_enc, 10)
        self.create_timer(1.0 / LOOP_HZ, self._loop)

        print(f"Target : {PUL_TARGET} pulses  ({PUL_TARGET * MM_PER_PULSE:.1f} mm arc → 90°)")
        print("Left wheel turning … right wheel stopped.")
        moteur(bus, 'G', SPEED_TURN)
        moteur(bus, 'D', 0)

    def _cb_enc(self, msg):
        # msg.data = [raw_l, raw_r, odo_l, odo_r]
        self._enc_left = msg.data[2]
        if self._start_left is None:
            self._start_left = self._enc_left
            print(f"  encoder zero: L={self._start_left}")

    def _loop(self):
        if self._done or self._start_left is None:
            return

        done = abs(self._enc_left - self._start_left)

        if done >= PUL_TARGET:
            stop_all(self.bus)
            self._done = True
            deg = done * MM_PER_PULSE / (math.pi * WHEELBASE_MM) * 180
            print(f"\n=== RESULT ===")
            print(f"  Target pulses : {PUL_TARGET}")
            print(f"  Actual pulses : {done}")
            print(f"  Angle reached : {deg:.1f}°  (expected 90.0°)")
            rclpy.shutdown()

# ── main ───────────────────────────────────────────────────────────────────────
def main():
    n = _find_i2c()
    if n is None:
        print("ERROR: PCA9685 not found on I²C"); return
    bus = smbus2.SMBus(n)
    pca_init(bus)
    print(f"PCA9685 on I²C bus {n}")

    rclpy.init()
    node = TurnTest(bus)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_all(bus)
        bus.close()
        print("Motors stopped.")

if __name__ == '__main__':
    main()
