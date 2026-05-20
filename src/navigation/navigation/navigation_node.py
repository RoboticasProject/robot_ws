#!/usr/bin/env python3
"""
navigation_node.py — Serpentine 2×2 m + navigation vers 4 bins par type de déchet

Machine à états
───────────────
Serpentine : FORWARD → TURN1 → SHIFT → TURN2 → (boucle) → DONE
Bin trip   : TURN_TO_BIN → GO_TO_BIN → AT_BIN
           → TURN_TO_HOME → GO_HOME → REORIENT → reprise serpentine

Bins (coins de la salle 2×2 m, robot démarre en (0,0) face +X)
  Plastic → (0.2, 0.2) m   bas-gauche
  Metal   → (1.8, 0.2) m   bas-droit
  Glass   → (1.8, 1.8) m   haut-droit
  Paper   → (0.2, 1.8) m   haut-gauche

Fuzzy logics (cascade, aucun override)
────────────────────────────────────────
Layer 1 — Speed fuzzy : YOLO confidence × bbox area → _cruise_speed (0 → SPEED_FWD)
           Si speed ≤ BIN_TRIP_SPEED → déclenche bin trip (objet proche/certain)
           Sinon → ralentit le serpentin, garde le cap

Layer 2 — Sync fuzzy  : drift encodeur G/D → correction PWM moteur D
           Fonctionne à toute vitesse ; s'ancre sur la base PWM courante.

Encodeurs reçus via /wheel_encoders (encoder_node). Ce nœud ne touche
pas aux GPIO — encoder_node est l'unique propriétaire des lignes libgpiod.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from vision_msgs.msg import Detection2DArray
import smbus2


# ═══════════════════════════════════════════════════════════════════════════════
#  ENCODEUR  (keep in sync with encoder_reader.PPR_EFFECTIF)
# ═══════════════════════════════════════════════════════════════════════════════
PPR_EFFECTIF  = 633
PPR           = PPR_EFFECTIF
WHEEL_DIAM_MM = 65.0
WHEEL_CIRC    = math.pi * WHEEL_DIAM_MM
MM_PER_PULSE  = WHEEL_CIRC / PPR
WHEELBASE_MM  = 300.0

# ═══════════════════════════════════════════════════════════════════════════════
#  GÉOMÉTRIE SERPENTINE
# ═══════════════════════════════════════════════════════════════════════════════
ROOM_MM     = 2000.0
CORRIDOR_MM = 400.0
NUM_PASSES  = int(ROOM_MM / CORRIDOR_MM)

PUL_STRAIGHT = int(ROOM_MM     / MM_PER_PULSE)
PUL_SHIFT    = int(CORRIDOR_MM / MM_PER_PULSE)
PUL_TURN90   = int((math.pi * WHEELBASE_MM / 4) / MM_PER_PULSE)

# ═══════════════════════════════════════════════════════════════════════════════
#  MOTEURS
# ═══════════════════════════════════════════════════════════════════════════════
SPEED_FWD    = 35
SPEED_TURN   = 28
SPEED_TRIM_D = 0.861   # right wheel is physically faster; calibrated by test_sync.py

# ── Layer 2 — Fuzzy wheel-sync ────────────────────────────────────────────────
SYNC_ZERO_THRESH  = 1                  # pulse/window noise floor
SYNC_SMALL_THRESH = 4                  # boundary small/large error zone
SYNC_STEP_MAX     = int(4095 * 0.008)  # 32 PWM per 50 ms window
SYNC_DRIFT_MAX    = int(4095 * 0.10)   # ±409 PWM max correction from base
SYNC_POS_WEIGHT   = 30                 # position drift weight vs velocity

# ── Layer 1 — Fuzzy speed ─────────────────────────────────────────────────────
_SPD_STOP        = 0
_SPD_SLOW        = 15
_SPD_MEDIUM      = 25
# FAST singleton = SPEED_FWD (35 %)
_AREA_SMALL      = 20_000   # px²  — object far
_AREA_MEDIUM     = 60_000   # px²  — object mid-range
_AREA_FRAME      = 640 * 480
BIN_TRIP_SPEED   = 5.0      # fuzzy output ≤ this % → trigger bin trip

# ═══════════════════════════════════════════════════════════════════════════════
#  BINS
# ═══════════════════════════════════════════════════════════════════════════════
BIN_POSITIONS = {
    'Plastic': ( 200.0,  200.0),
    'Metal':   (1800.0,  200.0),
    'Glass':   (1800.0, 1800.0),
    'Paper':   ( 200.0, 1800.0),
    'Other':   ( 200.0, 1800.0),
}

# ═══════════════════════════════════════════════════════════════════════════════
#  MATÉRIEL
# ═══════════════════════════════════════════════════════════════════════════════
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)   # ENA, IN2, IN1 — gauche (monté en miroir)
CANAUX_D     = (6, 7, 8)   # ENB, IN3, IN4 — droit


# ── Fuzzy membership functions ────────────────────────────────────────────────

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

class NavigationNode(Node):

    # États serpentine
    ST_FORWARD = 'FORWARD'
    ST_TURN1   = 'TURN1'
    ST_SHIFT   = 'SHIFT'
    ST_TURN2   = 'TURN2'
    ST_DONE    = 'DONE'
    # États bin trip
    ST_TURN_TO_BIN  = 'TURN_TO_BIN'
    ST_GO_TO_BIN    = 'GO_TO_BIN'
    ST_AT_BIN       = 'AT_BIN'
    ST_TURN_TO_HOME = 'TURN_TO_HOME'
    ST_GO_HOME      = 'GO_HOME'
    ST_REORIENT     = 'REORIENT'

    SERP_STATES          = {ST_FORWARD, ST_TURN1, ST_SHIFT, ST_TURN2}
    STRAIGHT_SYNC_STATES = {ST_FORWARD, ST_SHIFT, ST_GO_TO_BIN, ST_GO_HOME}

    def __init__(self):
        super().__init__('navigation_node')

        # ── PCA9685 ───────────────────────────────────────────────────────────
        n = _find_i2c()
        if n is None:
            self.get_logger().fatal("PCA9685 introuvable sur I²C 0-9")
            raise RuntimeError("PCA9685 not found")
        self.bus = smbus2.SMBus(n)
        self._pca_init()
        self.get_logger().info(f"PCA9685 sur bus I²C {n}")

        # ── Encoder data (received from /wheel_encoders) ──────────────────────
        self._enc_ready     = False
        self._enc_left_raw  = 0   # always-increasing raw count from encoder_node
        self._enc_right_raw = 0
        self._enc_left_odo  = 0   # signed cumulative (for odometry)
        self._enc_right_odo = 0
        self._seg_base_l    = 0   # snapshot at last _reset_sm()
        self._seg_base_r    = 0

        # ── Odométrie ─────────────────────────────────────────────────────────
        self._last_og = 0
        self._last_od = 0

        # ── Layer 2 — Fuzzy wheel-sync state ──────────────────────────────────
        self._base_pwm_g     = 0
        self._base_pwm_d     = 0
        self._cur_pwm_d      = 0   # D's corrected PWM — integral accumulator
        self._sync_ref_abs_g = 0
        self._sync_ref_abs_d = 0

        # ── Layer 1 — Fuzzy speed state ───────────────────────────────────────
        self._cruise_speed = SPEED_FWD   # updated by _cb_detection via _fuzzy_speed

        # ── Pose (mm, rad) ────────────────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._θ = 0.0

        # ── Serpentine ────────────────────────────────────────────────────────
        self._state    = self.ST_FORWARD
        self._pass_num = 0
        self._turn_dir = 'L'

        # ── Bin trip ──────────────────────────────────────────────────────────
        self._saved_x       = 0.0
        self._saved_y       = 0.0
        self._saved_θ       = 0.0
        self._saved_serp_st = self.ST_FORWARD
        self._saved_sm_g    = 0
        self._saved_sm_d    = 0
        self._bin_tx        = 0.0
        self._bin_ty        = 0.0
        self._bin_turn_pul  = 0
        self._bin_turn_left = True
        self._bin_drive_pul = 0
        self._at_bin_time   = None

        # ── ROS ───────────────────────────────────────────────────────────────
        self._camera_ready = False
        self.create_subscription(
            Int64MultiArray, '/wheel_encoders', self._cb_encoders, 10
        )
        self.create_subscription(
            Detection2DArray, '/detections', self._cb_detection, 10
        )
        self.create_timer(0.05, self._loop)
        self.create_timer(0.05, self._sync_loop)

        self.get_logger().info(
            f"Navigation 2×2 m — {NUM_PASSES} couloirs × {ROOM_MM/1000:.0f} m\n"
            f"  STRAIGHT={PUL_STRAIGHT} pul  SHIFT={PUL_SHIFT} pul  TURN90={PUL_TURN90} pul\n"
            f"  En attente caméra + encodeurs..."
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  ENCODEURS  (données reçues depuis encoder_node via /wheel_encoders)
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_encoders(self, msg: Int64MultiArray):
        self._enc_left_raw  = msg.data[0]
        self._enc_right_raw = msg.data[1]
        self._enc_left_odo  = msg.data[2]
        self._enc_right_odo = msg.data[3]
        if not self._enc_ready:
            self._enc_ready      = True
            self._seg_base_l     = self._enc_left_raw
            self._seg_base_r     = self._enc_right_raw
            self._sync_ref_abs_g = 0
            self._sync_ref_abs_d = 0

    def _seg_l(self) -> int:
        """Pulses since last _reset_sm() — left wheel."""
        return self._enc_left_raw - self._seg_base_l

    def _seg_r(self) -> int:
        """Pulses since last _reset_sm() — right wheel."""
        return self._enc_right_raw - self._seg_base_r

    def _get_sm(self):
        return self._seg_l(), self._seg_r()

    def _reset_sm(self):
        self._seg_base_l = self._enc_left_raw
        self._seg_base_r = self._enc_right_raw

    def _set_sm(self, val_l: int, val_r: int):
        """Restore segment counters to previously saved values (bin trip resume)."""
        self._seg_base_l = self._enc_left_raw - val_l
        self._seg_base_r = self._enc_right_raw - val_r

    # ─────────────────────────────────────────────────────────────────────────
    #  ODOMÉTRIE DIFFÉRENTIELLE
    # ─────────────────────────────────────────────────────────────────────────

    def _update_pose(self):
        og = self._enc_left_odo
        od = self._enc_right_odo

        dg = (og - self._last_og) * MM_PER_PULSE
        dd = (od - self._last_od) * MM_PER_PULSE
        self._last_og = og
        self._last_od = od

        d  = (dg + dd) / 2.0
        dθ = (dd - dg) / WHEELBASE_MM

        self._θ += dθ / 2.0
        self._x += d * math.cos(self._θ)
        self._y += d * math.sin(self._θ)
        self._θ += dθ / 2.0
        self._θ  = math.atan2(math.sin(self._θ), math.cos(self._θ))

    # ─────────────────────────────────────────────────────────────────────────
    #  LAYER 1 — FUZZY SPEED
    # ─────────────────────────────────────────────────────────────────────────

    def _fuzzy_speed(self, conf: float, area: float) -> float:
        """
        Mamdani fuzzy controller — confidence × bbox area → speed %.
        Output in [0, SPEED_FWD]. Values ≤ BIN_TRIP_SPEED trigger a bin trip.
        """
        # Confidence membership
        low_c  = _trap(conf, 0.0, 0.0, 0.50, 0.70)
        med_c  = _tri( conf, 0.50, 0.70, 0.90)
        high_c = _trap(conf, 0.70, 0.90, 1.0,  1.0)

        # Area membership
        small_a = _trap(area, 0,           0,            _AREA_SMALL,  _AREA_MEDIUM)
        med_a   = _tri( area, _AREA_SMALL,  _AREA_MEDIUM, _AREA_FRAME // 2)
        large_a = _trap(area, _AREA_MEDIUM, _AREA_FRAME // 2, _AREA_FRAME, _AREA_FRAME)

        # Rules (Mamdani — min operator)
        w_stop   = min(high_c, large_a)
        w_slow   = max(min(high_c, med_a),   min(med_c, large_a))
        w_medium = max(min(high_c, small_a), min(med_c, med_a))
        w_fast   = max(min(med_c, small_a),  low_c)

        # Defuzzification — weighted average of singletons
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
            self.get_logger().info("Caméra prête — démarrage de la navigation !")
            if not msg.detections:
                self._avancer()

        if self._state not in self.SERP_STATES:
            return

        if not msg.detections:
            if self._cruise_speed != SPEED_FWD:
                self._set_cruise(SPEED_FWD)
            return

        best = max(
            msg.detections,
            key=lambda d: d.results[0].hypothesis.score if d.results else 0.0
        )
        conf  = best.results[0].hypothesis.score if best.results else 0.0
        area  = best.bbox.size_x * best.bbox.size_y
        speed = self._fuzzy_speed(conf, area)
        label = best.results[0].hypothesis.class_id if best.results else 'Other'

        if speed <= BIN_TRIP_SPEED:
            self.get_logger().info(
                f"[SPEED] {label}  conf={conf:.2f}  area={int(area)}px²"
                f"  speed={speed:.1f}% → BIN TRIP"
            )
            self._start_bin_trip(label)
        elif self._state in (self.ST_FORWARD, self.ST_SHIFT):
            self._set_cruise(speed)
            self.get_logger().info(
                f"[SPEED] {label}  conf={conf:.2f}  area={int(area)}px²"
                f"  → {speed:.1f}%"
            )

    # ─────────────────────────────────────────────────────────────────────────
    #  BIN TRIP — déclenchement
    # ─────────────────────────────────────────────────────────────────────────

    def _start_bin_trip(self, waste_class):
        self._saved_x       = self._x
        self._saved_y       = self._y
        self._saved_θ       = self._θ
        self._saved_serp_st = self._state
        bx, by = BIN_POSITIONS.get(waste_class, BIN_POSITIONS['Other'])
        self._bin_tx = bx
        self._bin_ty = by

        self._stop()
        self._cruise_speed = SPEED_FWD   # full speed during bin trip navigation

        self._saved_sm_g = self._seg_l()
        self._saved_sm_d = self._seg_r()
        self._reset_sm()
        self._begin_turn_to(bx, by)
        self._state = self.ST_TURN_TO_BIN

        self.get_logger().info(
            f"DÉCHET [{waste_class}] pos=({self._x:.0f},{self._y:.0f}) mm"
            f"  →  BIN ({bx:.0f},{by:.0f}) mm"
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  BIN TRIP — helpers géométriques
    # ─────────────────────────────────────────────────────────────────────────

    def _begin_turn_to(self, tx, ty):
        dθ = math.atan2(ty - self._y, tx - self._x) - self._θ
        dθ = math.atan2(math.sin(dθ), math.cos(dθ))

        arc = abs(dθ) * WHEELBASE_MM / 2.0
        self._bin_turn_pul  = int(arc / MM_PER_PULSE)
        self._bin_turn_left = dθ > 0

        if self._bin_turn_pul > 10:
            self._pivoter('L' if self._bin_turn_left else 'R')
        else:
            self._bin_turn_pul = 0

    def _begin_drive_to(self, tx, ty):
        dist = math.sqrt((tx - self._x) ** 2 + (ty - self._y) ** 2)
        self._bin_drive_pul = int(dist / MM_PER_PULSE)
        if self._bin_drive_pul > 10:
            self._avancer()
        else:
            self._bin_drive_pul = 0

    def _finish_bin_trip(self):
        self._state = self._saved_serp_st
        self._set_sm(self._saved_sm_g, self._saved_sm_d)

        if self._state in (self.ST_FORWARD, self.ST_SHIFT):
            self._avancer()
        elif self._state in (self.ST_TURN1, self.ST_TURN2):
            self._pivoter(self._turn_dir)

        self.get_logger().info(f"← Retour au serpentin : {self._state}")

    # ─────────────────────────────────────────────────────────────────────────
    #  BOUCLE DE CONTRÔLE  20 Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _loop(self):
        if not self._camera_ready or not self._enc_ready:
            return

        self._update_pose()

        if self._state == self.ST_DONE:
            return

        pg, pd = self._get_sm()
        avg  = (pg + pd) // 2
        mini = min(pg, pd)

        # ════════════════════════════════════════════════
        #  SERPENTINE
        # ════════════════════════════════════════════════

        if self._state == self.ST_FORWARD:
            if avg >= PUL_STRAIGHT:
                self._pass_num += 1
                if self._pass_num >= NUM_PASSES:
                    self._stop()
                    self._state = self.ST_DONE
                    self.get_logger().info("✓ Salle entièrement couverte !")
                else:
                    self._reset_sm()
                    self._pivoter(self._turn_dir)
                    self._state = self.ST_TURN1

        elif self._state == self.ST_TURN1:
            if mini >= PUL_TURN90:
                self._reset_sm()
                self._avancer()
                self._state = self.ST_SHIFT

        elif self._state == self.ST_SHIFT:
            if avg >= PUL_SHIFT:
                self._reset_sm()
                self._pivoter(self._turn_dir)
                self._state = self.ST_TURN2

        elif self._state == self.ST_TURN2:
            if mini >= PUL_TURN90:
                self._turn_dir = 'R' if self._turn_dir == 'L' else 'L'
                self._reset_sm()
                self._avancer()
                self._state = self.ST_FORWARD
                self.get_logger().info(
                    f"→ Couloir {self._pass_num + 1}/{NUM_PASSES}"
                    f"  pos=({self._x:.0f},{self._y:.0f}) mm  θ={math.degrees(self._θ):.0f}°"
                )

        # ════════════════════════════════════════════════
        #  BIN TRIP
        # ════════════════════════════════════════════════

        elif self._state == self.ST_TURN_TO_BIN:
            if self._bin_turn_pul == 0 or mini >= self._bin_turn_pul:
                self._stop()
                self._reset_sm()
                self._begin_drive_to(self._bin_tx, self._bin_ty)
                self._state = self.ST_GO_TO_BIN

        elif self._state == self.ST_GO_TO_BIN:
            if self._bin_drive_pul == 0 or avg >= self._bin_drive_pul:
                self._stop()
                self._at_bin_time = self.get_clock().now()
                self._state = self.ST_AT_BIN
                self.get_logger().info("Arrivé au bin — pause 2 s [futur: pick & place]")

        elif self._state == self.ST_AT_BIN:
            elapsed = (self.get_clock().now() - self._at_bin_time).nanoseconds / 1e9
            if elapsed >= 2.0:
                self._reset_sm()
                self._begin_turn_to(self._saved_x, self._saved_y)
                self._state = self.ST_TURN_TO_HOME

        elif self._state == self.ST_TURN_TO_HOME:
            if self._bin_turn_pul == 0 or mini >= self._bin_turn_pul:
                self._stop()
                self._reset_sm()
                self._begin_drive_to(self._saved_x, self._saved_y)
                self._state = self.ST_GO_HOME

        elif self._state == self.ST_GO_HOME:
            if self._bin_drive_pul == 0 or avg >= self._bin_drive_pul:
                self._stop()
                self._reset_sm()
                dθ = self._saved_θ - self._θ
                dθ = math.atan2(math.sin(dθ), math.cos(dθ))
                arc = abs(dθ) * WHEELBASE_MM / 2.0
                self._bin_turn_pul  = int(arc / MM_PER_PULSE)
                self._bin_turn_left = dθ > 0
                if self._bin_turn_pul > 10:
                    self._pivoter('L' if self._bin_turn_left else 'R')
                    self._state = self.ST_REORIENT
                else:
                    self._finish_bin_trip()

        elif self._state == self.ST_REORIENT:
            if mini >= self._bin_turn_pul:
                self._stop()
                self._finish_bin_trip()

    # ─────────────────────────────────────────────────────────────────────────
    #  COMMANDES VITESSE
    # ─────────────────────────────────────────────────────────────────────────

    def _set_cruise(self, speed: float):
        """Apply a new cruise speed mid-motion; re-anchors sync integral to new base."""
        self._cruise_speed = speed
        if self._state in self.STRAIGHT_SYNC_STATES and self._base_pwm_d != 0:
            self._moteur('G', speed)
            self._moteur('D', speed)
            self._cur_pwm_d = self._base_pwm_d   # re-anchor so sync works from new base

    def _avancer(self):
        self._moteur('G', self._cruise_speed)
        self._moteur('D', self._cruise_speed)
        self._cur_pwm_d      = self._base_pwm_d
        self._sync_ref_abs_g = self._seg_l()
        self._sync_ref_abs_d = self._seg_r()

    def _pivoter(self, direction):
        if direction == 'L':
            self._moteur('G', -SPEED_TURN)
            self._moteur('D',  SPEED_TURN)
        else:
            self._moteur('G',  SPEED_TURN)
            self._moteur('D', -SPEED_TURN)

    def _stop(self):
        self._moteur('G', 0)
        self._moteur('D', 0)

    def _moteur(self, cote, vitesse):
        if cote == 'D' and vitesse != 0:
            vitesse *= SPEED_TRIM_D
        pwm = int(abs(vitesse) / 100 * 4095)
        ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
        if vitesse > 0:
            self._pca_canal(ch_a, 4095)
            self._pca_canal(ch_b, 0)
        elif vitesse < 0:
            self._pca_canal(ch_a, 0)
            self._pca_canal(ch_b, 4095)
        else:
            self._pca_canal(ch_a, 0)
            self._pca_canal(ch_b, 0)
            pwm = 0
        self._pca_canal(ch_en, pwm)
        if cote == 'G':
            self._base_pwm_g = pwm
        else:
            self._base_pwm_d = pwm

    # ─────────────────────────────────────────────────────────────────────────
    #  LAYER 2 — FUZZY WHEEL SYNC  (20 Hz)
    # ─────────────────────────────────────────────────────────────────────────

    def _fuzzy_sync_delta(self, diff: int) -> int:
        """
        diff = (vel_G - vel_D) + position_drift_weight
        Positive → G faster → increase D.
        Returns signed PWM step to add to D's current value.
        """
        abs_diff = abs(diff)
        if abs_diff <= SYNC_ZERO_THRESH:
            return 0
        sign = 1 if diff > 0 else -1
        if abs_diff <= SYNC_SMALL_THRESH:
            t = (abs_diff - SYNC_ZERO_THRESH) / (SYNC_SMALL_THRESH - SYNC_ZERO_THRESH)
            delta = int(t * SYNC_STEP_MAX * 0.5)
        else:
            t = min(1.0, (abs_diff - SYNC_SMALL_THRESH) / SYNC_SMALL_THRESH)
            delta = int(SYNC_STEP_MAX * 0.5 + t * SYNC_STEP_MAX * 0.5)
        return sign * delta

    def _sync_loop(self):
        if not self._enc_ready:
            return

        cur_abs_g = self._seg_l()
        cur_abs_d = self._seg_r()

        stalled  = (cur_abs_g == self._sync_ref_abs_g and
                    cur_abs_d == self._sync_ref_abs_d)
        vel_diff = ((cur_abs_g - self._sync_ref_abs_g) -
                    (cur_abs_d - self._sync_ref_abs_d))
        self._sync_ref_abs_g = cur_abs_g
        self._sync_ref_abs_d = cur_abs_d

        if stalled or self._state not in self.STRAIGHT_SYNC_STATES or self._base_pwm_d == 0:
            return

        pos_diff = cur_abs_g - cur_abs_d
        diff     = vel_diff + pos_diff // SYNC_POS_WEIGHT
        delta    = self._fuzzy_sync_delta(diff)
        if delta == 0:
            return

        new_d = self._cur_pwm_d + delta
        new_d = max(self._base_pwm_d - SYNC_DRIFT_MAX,
                    min(self._base_pwm_d + SYNC_DRIFT_MAX, new_d))
        new_d = max(0, min(4095, new_d))
        self._cur_pwm_d = new_d
        self._pca_canal(CANAUX_D[0], new_d)
        self.get_logger().debug(
            f"[SYNC] vel={vel_diff:+d} pos={pos_diff:+d} comb={diff:+d} "
            f"delta={delta:+d} D={new_d} base={self._base_pwm_d}"
        )

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
        self._stop()
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
