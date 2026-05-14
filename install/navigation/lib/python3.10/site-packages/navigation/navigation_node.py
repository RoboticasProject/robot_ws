#!/usr/bin/env python3
"""
navigation_node.py — Serpentin 2×2 m + navigation vers 4 bins par type de déchet

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
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import smbus2
import Jetson.GPIO as GPIO


# ═══════════════════════════════════════════════════════════════════════════════
#  GÉOMÉTRIE ROBOT
# ═══════════════════════════════════════════════════════════════════════════════
PPR           = 663
WHEEL_DIAM_MM = 65.0
WHEEL_CIRC    = math.pi * WHEEL_DIAM_MM        # 204.2 mm
MM_PER_PULSE  = WHEEL_CIRC / PPR               # 0.308 mm/pulse
WHEELBASE_MM  = 300.0

ROOM_MM     = 2000.0
CORRIDOR_MM = 400.0
NUM_PASSES  = int(ROOM_MM / CORRIDOR_MM)       # 5 couloirs

PUL_STRAIGHT = int(ROOM_MM     / MM_PER_PULSE) # ~6 497
PUL_SHIFT    = int(CORRIDOR_MM / MM_PER_PULSE) # ~1 299
PUL_TURN90   = int((math.pi * WHEELBASE_MM / 4) / MM_PER_PULSE)  # ~765

SPEED_FWD  = 35  # % ligne droite
SPEED_TURN = 28   # % pivot

# ═══════════════════════════════════════════════════════════════════════════════
#  POSITIONS BINS  (en mm, origine = départ robot)
# ═══════════════════════════════════════════════════════════════════════════════
BIN_POSITIONS = {
    'Plastic': ( 200.0,  200.0),   # coin bas-gauche
    'Metal':   (1800.0,  200.0),   # coin bas-droit
    'Glass':   (1800.0, 1800.0),   # coin haut-droit
    'Paper':   ( 200.0, 1800.0),   # coin haut-gauche
    'Other':   ( 200.0, 1800.0),   # même que Paper
}

# ═══════════════════════════════════════════════════════════════════════════════
#  MATÉRIEL
# ═══════════════════════════════════════════════════════════════════════════════
ENC_G_A, ENC_G_B = 11, 13     # GPIO BOARD — encodeur gauche
ENC_D_A, ENC_D_B = 15, 16     # GPIO BOARD — encodeur droit

PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)       # ENA, IN2, IN1  (gauche monté en miroir)
CANAUX_D     = (6, 7, 8)       # ENB, IN3, IN4


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

    SERP_STATES = {ST_FORWARD, ST_TURN1, ST_SHIFT, ST_TURN2}

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

        # ── GPIO encodeurs ────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BOARD)
        for p in [ENC_G_A, ENC_G_B, ENC_D_A, ENC_D_B]:
            GPIO.setup(p, GPIO.IN)
        GPIO.add_event_detect(ENC_G_A, GPIO.RISING, callback=self._cb_enc_g)
        GPIO.add_event_detect(ENC_D_A, GPIO.RISING, callback=self._cb_enc_d)

        # ── Compteurs encodeurs ───────────────────────────────────────────────
        self._lock   = threading.Lock()
        self._sm_g   = 0    # state-machine : absolu, reset par état
        self._sm_d   = 0
        self._odo_g  = 0    # odométrie    : signé, jamais reset
        self._odo_d  = 0
        self._last_og = 0
        self._last_od = 0

        # ── Pose (mm, rad) ────────────────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._θ = 0.0   # 0 = face +X

        # ── Serpentine ────────────────────────────────────────────────────────
        self._state    = self.ST_FORWARD
        self._pass_num = 0
        self._turn_dir = 'L'   # alterne L / R à chaque couloir

        # ── Variables bin trip ────────────────────────────────────────────────
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
            Detection2DArray, '/detections', self._cb_detection, 10
        )
        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f"Navigation 2×2 m démarrée — {NUM_PASSES} couloirs × {ROOM_MM/1000:.0f} m\n"
            f"  STRAIGHT={PUL_STRAIGHT} pul  SHIFT={PUL_SHIFT} pul  TURN90={PUL_TURN90} pul\n"
            f"  En attente de la caméra..."
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  ENCODEURS
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_enc_g(self, _):
        B = GPIO.input(ENC_G_B)
        with self._lock:
            self._sm_g  += 1
            self._odo_g += 1 if B == GPIO.LOW else -1

    def _cb_enc_d(self, _):
        B = GPIO.input(ENC_D_B)
        with self._lock:
            self._sm_d  += 1
            self._odo_d += 1 if B == GPIO.HIGH else -1   # miroir

    def _get_sm(self):
        with self._lock:
            return self._sm_g, self._sm_d

    def _reset_sm(self):
        with self._lock:
            self._sm_g = 0
            self._sm_d = 0

    # ─────────────────────────────────────────────────────────────────────────
    #  ODOMÉTRIE DIFFÉRENTIELLE
    # ─────────────────────────────────────────────────────────────────────────

    def _update_pose(self):
        with self._lock:
            og, od = self._odo_g, self._odo_d

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
        self._θ  = math.atan2(math.sin(self._θ), math.cos(self._θ))   # normalise

    # ─────────────────────────────────────────────────────────────────────────
    #  DÉTECTION YOLO
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_detection(self, msg: Detection2DArray):
        if not self._camera_ready:
            self._camera_ready = True
            self.get_logger().info("Caméra prête — démarrage de la navigation !")
            if not msg.detections:
                self._avancer()

        # Déclenchement uniquement pendant la phase serpentine
        if not msg.detections or self._state not in self.SERP_STATES:
            return

        best = max(
            msg.detections,
            key=lambda d: d.results[0].hypothesis.score if d.results else 0.0
        )
        waste_class = best.results[0].hypothesis.class_id if best.results else 'Other'
        self._start_bin_trip(waste_class)

    # ─────────────────────────────────────────────────────────────────────────
    #  BIN TRIP — déclenchement
    # ─────────────────────────────────────────────────────────────────────────

    def _start_bin_trip(self, waste_class):
        # Sauvegarde de l'état serpentine courant
        self._saved_x       = self._x
        self._saved_y       = self._y
        self._saved_θ       = self._θ
        self._saved_serp_st = self._state
        with self._lock:
            self._saved_sm_g = self._sm_g
            self._saved_sm_d = self._sm_d

        bx, by = BIN_POSITIONS.get(waste_class, BIN_POSITIONS['Other'])
        self._bin_tx = bx
        self._bin_ty = by

        self._stop()
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
        """Calcule le pivot vers (tx,ty) et démarre les moteurs."""
        dθ = math.atan2(ty - self._y, tx - self._x) - self._θ
        dθ = math.atan2(math.sin(dθ), math.cos(dθ))   # normalise [-π, π]

        arc = abs(dθ) * WHEELBASE_MM / 2.0
        self._bin_turn_pul  = int(arc / MM_PER_PULSE)
        self._bin_turn_left = dθ > 0

        if self._bin_turn_pul > 10:
            self._pivoter('L' if self._bin_turn_left else 'R')
        else:
            self._bin_turn_pul = 0   # déjà aligné, pas de pivot

    def _begin_drive_to(self, tx, ty):
        """Calcule la distance vers (tx,ty) et démarre les moteurs."""
        dist = math.sqrt((tx - self._x) ** 2 + (ty - self._y) ** 2)
        self._bin_drive_pul = int(dist / MM_PER_PULSE)
        if self._bin_drive_pul > 10:
            self._avancer()
        else:
            self._bin_drive_pul = 0   # déjà sur place

    def _finish_bin_trip(self):
        """Reprend le serpentin exactement là où il en était."""
        self._state = self._saved_serp_st
        with self._lock:
            self._sm_g = self._saved_sm_g
            self._sm_d = self._saved_sm_d

        if self._state in (self.ST_FORWARD, self.ST_SHIFT):
            self._avancer()
        elif self._state in (self.ST_TURN1, self.ST_TURN2):
            self._pivoter(self._turn_dir)

        self.get_logger().info(f"← Retour au serpentin : {self._state}")

    # ─────────────────────────────────────────────────────────────────────────
    #  BOUCLE DE CONTRÔLE  20 Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _loop(self):
        if not self._camera_ready:
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
                # Pivot pour retrouver l'orientation sauvegardée
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
    #  COMMANDES MOTEURS
    # ─────────────────────────────────────────────────────────────────────────

    def _avancer(self):
        self._moteur('G',  SPEED_FWD)
        self._moteur('D',  SPEED_FWD)

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
        GPIO.cleanup()
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
