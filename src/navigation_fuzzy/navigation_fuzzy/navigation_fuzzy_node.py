#!/usr/bin/env python3
"""
navigation_fuzzy_node.py — Ligne droite avec synchronisation et contrôle de vitesse fuzzy.

Comportement
────────────
Le robot avance en ligne droite indéfiniment.

Layer 1 — Speed fuzzy (Mamdani)
  Entrées : confiance YOLO × aire bbox
  Sortie  : _cruise_speed  (SPEED_FWD → 0)
  Si speed ≤ STOP_THRESH → moteurs stop, état STOPPED
  Si no detection pendant RESUME_DELAY s → reprise à SPEED_FWD

Layer 2 — Sync fuzzy 2 entrées (conception corrigée)
  Entrée 1 : vel_error  = vel_G − vel_D  (pulses/fenêtre de 50 ms)
             → erreur de vitesse instantanée
  Entrée 2 : pos_drift  = pos_G − pos_D  (pulses cumulés depuis dernier reset)
             → dérive accumulée (terme intégral)
  Sortie   : correction PWM DIRECTE sur D  (pas d'accumulateur)
             new_d = base_pwm_d + correction  ← sans windup possible
  Table de règles 3×3 : NEG/ZE/POS × NEG/ZE/POS → NB/NS/ZE/PS/PB

Pourquoi la version précédente dérivait
────────────────────────────────────────
  1. Accumulateur _cur_pwm_d += delta → windup incontrôlé
  2. pos_diff // SYNC_POS_WEIGHT=30  → dérive ignorée jusqu'à 30+ pulses
  3. Un seul input combiné → perte d'information vel vs. position

États : GOING ↔ STOPPED
Encodeurs reçus depuis encoder_node via /wheel_encoders (50 Hz).
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from vision_msgs.msg import Detection2DArray
import smbus2


# ═══════════════════════════════════════════════════════════════════════════════
#  ENCODEUR
# ═══════════════════════════════════════════════════════════════════════════════
PPR_EFFECTIF  = 633
WHEEL_DIAM_MM = 65.0
MM_PER_PULSE  = math.pi * WHEEL_DIAM_MM / PPR_EFFECTIF   # ~0.323 mm/pulse
WHEELBASE_MM  = 300.0

# ═══════════════════════════════════════════════════════════════════════════════
#  MOTEURS
# ═══════════════════════════════════════════════════════════════════════════════
SPEED_FWD    = 35
SPEED_TRIM_D = 0.861

# ── Layer 2 — Fuzzy sync (2 entrées, sortie directe) ─────────────────────────
#
# Fenêtre de 50 ms à 35 % → ~27 pulses/fenêtre par roue.
# Erreur de 1 pulse/fenêtre ≈ 3.7 % de différence de vitesse.
#
# Fonctions d'appartenance — vel_error (pulses/fenêtre)
SYNC_VEL_ZE   = 1     # demi-largeur zone zéro  (±1 pulse → ignoré)
SYNC_VEL_S    = 4     # pic   zone petite        (3–6 pulses = 10–22%)
SYNC_VEL_B    = 9     # début zone grande        (>7 pulses = >26%)

# Fonctions d'appartenance — pos_drift (pulses cumulés)
SYNC_POS_ZE   = 3     # demi-largeur zone zéro  (±3 pulses ≈ ±1 mm)
SYNC_POS_S    = 15    # pic   zone petite        (15 pulses ≈ 5 mm)
SYNC_POS_B    = 50    # début zone grande        (>40 pulses ≈ >13 mm)

# Singletons de sortie (correction PWM ajoutée à base_pwm_d)
_CORR_MAX = int(4095 * 0.18)   # ±18 % de la plage PWM = ±737
_CORR_S   = int(_CORR_MAX * 0.35)   # ±258

SYNC_OUT_NB = -_CORR_MAX
SYNC_OUT_NS = -_CORR_S
SYNC_OUT_ZE =  0
SYNC_OUT_PS = +_CORR_S
SYNC_OUT_PB = +_CORR_MAX

# Table de règles [vel_idx][pos_idx]
# vel  : 0=NEG  1=ZE  2=POS
# pos  : 0=NEG  1=ZE  2=POS
# Sémantique : vel_error>0 → G plus rapide → augmenter D (sortie positive)
SYNC_RULES = [
    [SYNC_OUT_NB, SYNC_OUT_NS, SYNC_OUT_ZE],   # vel=NEG
    [SYNC_OUT_NS, SYNC_OUT_ZE, SYNC_OUT_PS],   # vel=ZE
    [SYNC_OUT_ZE, SYNC_OUT_PS, SYNC_OUT_PB],   # vel=POS
]

# ── Layer 1 — Fuzzy speed ─────────────────────────────────────────────────────
_SPD_STOP    = 0
_SPD_SLOW    = 15
_SPD_MEDIUM  = 25
_AREA_SMALL  = 20_000
_AREA_MEDIUM = 60_000
_AREA_FRAME  = 640 * 480
STOP_THRESH  = 5.0
RESUME_DELAY = 1.5

# ═══════════════════════════════════════════════════════════════════════════════
#  MATÉRIEL
# ═══════════════════════════════════════════════════════════════════════════════
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)
CANAUX_D     = (6, 7, 8)


# ── Fonctions d'appartenance ──────────────────────────────────────────────────

def _trap(x, a, b, c, d):
    if x <= a or x >= d:
        return 0.0
    if x <= b:
        return (x - a) / (b - a)
    if x <= c:
        return 1.0
    return (d - x) / (d - c)


def _tri(x, a, b, c):
    return _trap(x, a, b, b, c)


def _find_i2c(addr=PCA9685_ADDR):
    for n in range(10):
        try:
            b = smbus2.SMBus(n); b.read_byte(addr); b.close(); return n
        except (FileNotFoundError, PermissionError):
            pass
        except OSError:
            try: b.close()
            except Exception: pass
    return None


# ═══════════════════════════════════════════════════════════════════════════════
#  NŒUD
# ═══════════════════════════════════════════════════════════════════════════════

class NavigationFuzzyNode(Node):

    ST_GOING   = 'GOING'
    ST_STOPPED = 'STOPPED'

    def __init__(self):
        super().__init__('navigation_fuzzy_node')

        # ── PCA9685 ───────────────────────────────────────────────────────────
        n = _find_i2c()
        if n is None:
            self.get_logger().fatal("PCA9685 introuvable sur I²C 0-9")
            raise RuntimeError("PCA9685 not found")
        self.bus = smbus2.SMBus(n)
        self._pca_init()
        self.get_logger().info(f"PCA9685 sur bus I²C {n}")

        # ── Encoder data ──────────────────────────────────────────────────────
        self._enc_ready     = False
        self._enc_left_raw  = 0
        self._enc_right_raw = 0
        self._seg_base_l    = 0
        self._seg_base_r    = 0

        # ── Layer 2 sync state ────────────────────────────────────────────────
        self._base_pwm_g = 0
        self._base_pwm_d = 0
        self._sync_ref_l = 0    # snapshot for velocity measurement
        self._sync_ref_r = 0

        # ── Layer 1 speed state ───────────────────────────────────────────────
        self._cruise_speed  = SPEED_FWD
        self._last_det_time = None
        self._camera_ready  = False

        # ── State ─────────────────────────────────────────────────────────────
        self._state = self.ST_GOING

        # ── Logging ───────────────────────────────────────────────────────────
        self._t_start    = None
        self._last_log_t = 0.0

        # ── ROS ───────────────────────────────────────────────────────────────
        self.create_subscription(
            Int64MultiArray, '/wheel_encoders', self._cb_encoders, 10
        )
        self.create_subscription(
            Detection2DArray, '/detections', self._cb_detection, 10
        )
        self.create_timer(0.05, self._loop)
        self.create_timer(0.05, self._sync_loop)

        self.get_logger().info(
            f"NavigationFuzzyNode prêt\n"
            f"  SPEED_FWD={SPEED_FWD}%  TRIM_D={SPEED_TRIM_D}\n"
            f"  Sync: VEL_ZE=±{SYNC_VEL_ZE}p  POS_ZE=±{SYNC_POS_ZE}p"
            f"  CORR_MAX=±{_CORR_MAX}PWM\n"
            f"  En attente caméra + encodeurs..."
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  ENCODEURS
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_encoders(self, msg: Int64MultiArray):
        self._enc_left_raw  = msg.data[0]
        self._enc_right_raw = msg.data[1]
        if not self._enc_ready:
            self._enc_ready  = True
            self._seg_base_l = self._enc_left_raw
            self._seg_base_r = self._enc_right_raw
            self._sync_ref_l = 0
            self._sync_ref_r = 0
            self.get_logger().info("Encodeurs reçus.")

    def _seg_l(self) -> int:
        return self._enc_left_raw - self._seg_base_l

    def _seg_r(self) -> int:
        return self._enc_right_raw - self._seg_base_r

    def _reset_seg(self):
        self._seg_base_l = self._enc_left_raw
        self._seg_base_r = self._enc_right_raw

    # ─────────────────────────────────────────────────────────────────────────
    #  LAYER 1 — FUZZY SPEED
    # ─────────────────────────────────────────────────────────────────────────

    def _fuzzy_speed(self, conf: float, area: float) -> float:
        low_c  = _trap(conf, 0.0, 0.0, 0.50, 0.70)
        med_c  = _tri( conf, 0.50, 0.70, 0.90)
        high_c = _trap(conf, 0.70, 0.90, 1.0,  1.0)

        small_a = _trap(area, 0,           0,            _AREA_SMALL,  _AREA_MEDIUM)
        med_a   = _tri( area, _AREA_SMALL,  _AREA_MEDIUM, _AREA_FRAME // 2)
        large_a = _trap(area, _AREA_MEDIUM, _AREA_FRAME // 2, _AREA_FRAME, _AREA_FRAME)

        w_stop   = min(high_c, large_a)
        w_slow   = max(min(high_c, med_a),   min(med_c, large_a))
        w_medium = max(min(high_c, small_a), min(med_c, med_a))
        w_fast   = max(min(med_c, small_a),  low_c)

        total = w_stop + w_slow + w_medium + w_fast
        if total < 1e-6:
            return float(SPEED_FWD)

        return (
            w_stop   * _SPD_STOP   +
            w_slow   * _SPD_SLOW   +
            w_medium * _SPD_MEDIUM +
            w_fast   * SPEED_FWD
        ) / total

    # ─────────────────────────────────────────────────────────────────────────
    #  DÉTECTION YOLO
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_detection(self, msg: Detection2DArray):
        if not self._camera_ready:
            self._camera_ready = True
            self.get_logger().info("Caméra prête — en attente encodeurs...")
            # Actual startup deferred to _loop() which also waits for _enc_ready
            return

        if not msg.detections:
            return

        self._last_det_time = self.get_clock().now()

        best  = max(msg.detections,
                    key=lambda d: d.results[0].hypothesis.score if d.results else 0.0)
        conf  = best.results[0].hypothesis.score if best.results else 0.0
        area  = best.bbox.size_x * best.bbox.size_y
        speed = self._fuzzy_speed(conf, area)
        label = best.results[0].hypothesis.class_id if best.results else 'Other'

        if speed <= STOP_THRESH:
            if self._state != self.ST_STOPPED:
                self._stop_motors()
                self._state = self.ST_STOPPED
                self.get_logger().info(
                    f"[SPEED] {label}  conf={conf:.2f}  area={int(area)}px²"
                    f"  speed={speed:.1f}% → STOP"
                )
        else:
            if self._state == self.ST_STOPPED:
                self._state = self.ST_GOING
                self._start_going()
            self._set_cruise(speed)
            self.get_logger().info(
                f"[SPEED] {label}  conf={conf:.2f}  area={int(area)}px²"
                f"  → {speed:.1f}%"
            )

    # ─────────────────────────────────────────────────────────────────────────
    #  BOUCLE PRINCIPALE  20 Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _loop(self):
        if not self._camera_ready or not self._enc_ready:
            return

        now = self.get_clock().now()

        # First-time startup — both camera and encoders confirmed ready
        if self._t_start is None and self._state == self.ST_GOING:
            self.get_logger().info("Caméra + encodeurs prêts — démarrage !")
            self._start_going()
            return

        # Auto-resume after RESUME_DELAY s with no detection
        if self._state == self.ST_STOPPED and self._last_det_time is not None:
            if (now - self._last_det_time).nanoseconds / 1e9 >= RESUME_DELAY:
                self._state = self.ST_GOING
                self._cruise_speed = SPEED_FWD
                self._start_going()
                self.get_logger().info(
                    f"Aucune détection depuis {RESUME_DELAY}s — reprise."
                )

        if self._t_start is None:
            return

        t = (now - self._t_start).nanoseconds / 1e9
        if t - self._last_log_t >= 1.0:
            self._last_log_t = t
            dist = ((self._seg_l() + self._seg_r()) / 2) * MM_PER_PULSE
            self.get_logger().info(
                f"[LINE]  t={t:.0f}s  dist={dist/1000:.2f}m"
                f"  G={self._seg_l()}pul  D={self._seg_r()}pul"
                f"  drift={self._seg_l()-self._seg_r():+d}pul"
                f"  speed={self._cruise_speed:.1f}%  state={self._state}"
            )

    # ─────────────────────────────────────────────────────────────────────────
    #  LAYER 2 — FUZZY SYNC  20 Hz  (2 entrées, sortie directe sans windup)
    # ─────────────────────────────────────────────────────────────────────────

    def _sync_loop(self):
        if not self._enc_ready or self._state != self.ST_GOING or self._base_pwm_d == 0:
            return

        cur_l = self._seg_l()
        cur_r = self._seg_r()

        # Velocity error : G − D this 50 ms window
        vel_err = (cur_l - self._sync_ref_l) - (cur_r - self._sync_ref_r)
        self._sync_ref_l = cur_l
        self._sync_ref_r = cur_r

        # Position drift : G − D since last segment reset
        pos_err = cur_l - cur_r

        # Both zero → nothing to do
        if vel_err == 0 and pos_err == 0:
            return

        # ── Fuzzify vel_error ─────────────────────────────────────────────────
        # NEG : G slower than D (need to decrease D)
        # ZE  : wheels in sync
        # POS : G faster than D (need to increase D)
        vel_neg = _trap(vel_err, -30, -SYNC_VEL_B, -SYNC_VEL_ZE, 0)
        vel_ze  = _tri( vel_err, -SYNC_VEL_ZE,      0,             SYNC_VEL_ZE)
        vel_pos = _trap(vel_err,  0,             SYNC_VEL_ZE,  SYNC_VEL_B, 30)

        # ── Fuzzify pos_drift ─────────────────────────────────────────────────
        pos_neg = _trap(pos_err, -200, -SYNC_POS_B, -SYNC_POS_ZE, 0)
        pos_ze  = _tri( pos_err, -SYNC_POS_ZE,      0,             SYNC_POS_ZE)
        pos_pos = _trap(pos_err,  0,             SYNC_POS_ZE,  SYNC_POS_B, 200)

        mf_vel = [vel_neg, vel_ze, vel_pos]   # index 0=NEG 1=ZE 2=POS
        mf_pos = [pos_neg, pos_ze, pos_pos]

        # ── Inference + aggregation ───────────────────────────────────────────
        # For each output singleton, keep the maximum firing strength
        out_weights: dict[int, float] = {}
        for i, mv in enumerate(mf_vel):
            if mv < 1e-6:
                continue
            for j, mp in enumerate(mf_pos):
                if mp < 1e-6:
                    continue
                singleton = SYNC_RULES[i][j]
                w = min(mv, mp)
                if w > out_weights.get(singleton, 0.0):
                    out_weights[singleton] = w

        if not out_weights:
            return

        # ── Defuzzification — weighted average of singletons ──────────────────
        total      = sum(out_weights.values())
        correction = int(sum(v * w for v, w in out_weights.items()) / total)

        # ── Apply DIRECTLY — no accumulator, no windup ────────────────────────
        new_d = self._base_pwm_d + correction
        new_d = max(0, min(4095, new_d))
        self._pca_canal(CANAUX_D[0], new_d)

        self.get_logger().debug(
            f"[SYNC] vel={vel_err:+d} pos={pos_err:+d}"
            f"  corr={correction:+d}  D={new_d}  base={self._base_pwm_d}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  COMMANDES MOTEURS
    # ─────────────────────────────────────────────────────────────────────────

    def _start_going(self):
        if self._t_start is None:
            self._t_start = self.get_clock().now()
        self._reset_seg()
        self._moteur('G', self._cruise_speed)
        self._moteur('D', self._cruise_speed)
        self._sync_ref_l = 0
        self._sync_ref_r = 0

    def _set_cruise(self, speed: float):
        """Update speed mid-motion; sync auto-adapts since it uses base_pwm_d."""
        self._cruise_speed = speed
        if self._state == self.ST_GOING and self._base_pwm_d != 0:
            self._moteur('G', speed)
            self._moteur('D', speed)

    def _stop_motors(self):
        self._moteur('G', 0)
        self._moteur('D', 0)

    def _moteur(self, cote, vitesse):
        if cote == 'D' and vitesse != 0:
            vitesse *= SPEED_TRIM_D
        pwm = int(abs(vitesse) / 100 * 4095)
        ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
        if vitesse > 0:
            self._pca_canal(ch_a, 4095); self._pca_canal(ch_b, 0)
        elif vitesse < 0:
            self._pca_canal(ch_a, 0);    self._pca_canal(ch_b, 4095)
        else:
            self._pca_canal(ch_a, 0);    self._pca_canal(ch_b, 0); pwm = 0
        self._pca_canal(ch_en, pwm)
        if cote == 'G':
            self._base_pwm_g = pwm
        else:
            self._base_pwm_d = pwm

    # ─────────────────────────────────────────────────────────────────────────
    #  PCA9685 bas niveau
    # ─────────────────────────────────────────────────────────────────────────

    def _pca_init(self, freq=1000):
        self._pca_w(0x00, 0x10)
        self._pca_w(0xFE, int(25_000_000 / (4096 * freq) - 1))
        self._pca_w(0x00, 0x00)
        time.sleep(0.005)
        self._pca_w(0x00, 0xA0)

    def _pca_w(self, reg, val):
        self.bus.write_byte_data(PCA9685_ADDR, reg, val, force=True)

    def _pca_canal(self, canal, v):
        reg = 0x06 + 4 * canal
        if v >= 4095:
            d = [0x00, 0x10, 0x00, 0x00]
        elif v <= 0:
            d = [0x00, 0x00, 0x00, 0x10]
        else:
            d = [0x00, 0x00, v & 0xFF, v >> 8]
        self.bus.write_i2c_block_data(PCA9685_ADDR, reg, d, force=True)

    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_motors()
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationFuzzyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
