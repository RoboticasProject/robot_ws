#!/usr/bin/env python3
"""
navigation_line_node.py — Suivi de ligne caméra + détection de déchets → 4 bins.

Machine à états
───────────────
Suivi ligne  : LINE_FOLLOWING   (braquage fuzzy caméra, vitesse modulée YOLO)
Virage 90°   : UTURN_1 (90°) → LINE_FOLLOWING
Ramassage    : STOPPED_TRASH → TURN_TO_BIN → GO_TO_BIN → AT_BIN
               → TURN_TO_LINE → GO_TO_LINE → REORIENT → REACQUIRE
                                                              ↓
                                                        LINE_FOLLOWING (reprise)
Perte ligne  : REACQUIRE (pivot lent, retrouve la ligne)
Fin          : DONE

Pourquoi ne pas utiliser les encodeurs pour le droit ?
  Dérive mécanique sur 2 m avec moteurs non parfaitement calibrés.
  La caméra corrige la dérive latérale en continu.
  Encodeurs réservés aux courtes manœuvres mesurées :
    virages 90°, décalages 400 mm, aller/retour au bin ≤ 2 m.

Layer 1 — Vitesse fuzzy (YOLO)
  conf × aire → _cruise_speed  →  si ≤ BIN_TRIP_SPEED : bin trip

Layer 2 — Braquage fuzzy (caméra)
  line_error → commande différentielle G/D

Layer 3 — Sync encodeur (ligne droite encodeur uniquement)
  vel_error + pos_drift → correction PWM D  (identique navigation_fuzzy_node)

Subscribes :
  /line_error         (std_msgs/Float32MultiArray)   ligne caméra
  /detections         (vision_msgs/Detection2DArray) YOLO
  /wheel_encoders     (std_msgs/Int64MultiArray)     encodeurs

Contrôle :
  PCA9685 I²C → L298N → moteurs DC
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int64MultiArray
from vision_msgs.msg import Detection2DArray
import smbus2


# ═══════════════════════════════════════════════════════════════════════════════
#  ENCODEUR
# ═══════════════════════════════════════════════════════════════════════════════
PPR_EFFECTIF  = 1475
WHEEL_DIAM_MM = 65.0
WHEEL_CIRC    = math.pi * WHEEL_DIAM_MM
MM_PER_PULSE  = WHEEL_CIRC / PPR_EFFECTIF   # ~0.323 mm/pulse
WHEELBASE_MM  = 300.0

# ═══════════════════════════════════════════════════════════════════════════════
#  GÉOMÉTRIE SERPENTINE
# ═══════════════════════════════════════════════════════════════════════════════
ROOM_MM      = 2000.0
CORRIDOR_MM  = 400.0
NUM_PASSES   = int(ROOM_MM / CORRIDOR_MM)            # 5

PUL_STRAIGHT = int(ROOM_MM     / MM_PER_PULSE)       # ~6186 pul
PUL_TURN90   = int((math.pi * WHEELBASE_MM / 4) / MM_PER_PULSE)  # ~731 pul  (90°)
PUL_UTURN_STRAIGHT = int(200.0 / MM_PER_PULSE)   # 20 cm droit entre les deux 90°
UTURN_TURN_TIMEOUT = 2.5   # s — fallback par pivot 90°
UTURN_STR_TIMEOUT  = 3.0   # s — fallback segment droit 20 cm

# Temps minimum dans le couloir avant d'accepter "fin de couloir".
# Remplace le check encodeur (MIN_PASS_PULSES) — robuste si encodeurs défaillants.
MIN_PASS_TIME = 2.5   # secondes de suivi de ligne avant d'autoriser un virage

# ═══════════════════════════════════════════════════════════════════════════════
#  MOTEURS
# ═══════════════════════════════════════════════════════════════════════════════
SPEED_FWD      = 35      # % vitesse encodeur (virages, bins)
SPEED_FWD_LINE = 30      # % vitesse suivi de ligne caméra
SPEED_TURN     = 28      # % vitesse de pivot (virages serpentine / bin)
SPEED_TRIM_D   = 0.842   # roue D physiquement plus rapide — calibré test_sync 20s

# ── Layer 2 — Braquage différentiel ──────────────────────────────────────────
# speed_L = base × (1 + STEER_GAIN × steer)
# speed_R = base × (1 − STEER_GAIN × steer)  puis × TRIM_D
STEER_GAIN     = 0.55    # [0–1] ; 0.55 → ±55 % de vitesse de base
SPEED_MAX_LINE = 50      # % plafond par roue pendant le suivi de ligne

# ── Timeouts suivi de ligne ───────────────────────────────────────────────────
LINE_LOST_TIMEOUT  = 0.5   # s avant de réagir à la perte de ligne
REACQUIRE_SP       = 20    # % vitesse pivot REACQUIRE (lent) — min > SPEED_MIN
REACQUIRE_T1       = 2.0   # s → changer direction pivot
REACQUIRE_T2       = 4.5   # s → abandon total

# ── Anti-blocage moteur ───────────────────────────────────────────────────────
SPEED_MIN        = 18    # % PWM minimum — en dessous le moteur cale sur friction statique
STALL_TIMEOUT_S  = 0.50  # s encodeur figé malgré commande PWM → roue calée
KICK_SPEED       = 65    # % impulsion kickstart pour briser la friction statique
KICK_DURATION    = 0.12  # s durée de l'impulsion

# ── Layer 3 — Sync encodeur (algorithme identique à test_sync.py) ────────────
SYNC_ZERO_THRESH  = 1
SYNC_SMALL_THRESH = 4
SYNC_STEP_MAX     = int(4095 * 0.008)   # 32 PWM / fenêtre 50ms
SYNC_DRIFT_MAX    = int(4095 * 0.10)    # ±409 PWM max déviation par rapport à la base
SYNC_POS_WEIGHT   = 30

# ── Layer 1 — Speed fuzzy ─────────────────────────────────────────────────────
_SPD_STOP   = 0;  _SPD_SLOW = 15;  _SPD_MEDIUM = 25
_AREA_SMALL = 20_000;  _AREA_MEDIUM = 60_000;  _AREA_FRAME = 640 * 480
BIN_TRIP_SPEED = 5.0   # % seuil déclenchement bin trip
TRASH_PAUSE    = 2.0   # s pause au point de ramassage
BIN_PAUSE      = 2.0   # s pause au bin (futur : gripper)

# ── États ligne (doit correspondre à line_follower_node) ─────────────────────
_LINE_OK  = 0.0;  _LINE_LOST = 1.0;  _LINE_TURN = 2.0

# ═══════════════════════════════════════════════════════════════════════════════
#  BINS (coins salle 2×2 m, robot démarre en (0,0))
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
CANAUX_G     = (3, 5, 4)   # ENA, IN2, IN1
CANAUX_D     = (6, 7, 8)   # ENB, IN3, IN4


# ── Fonctions d'appartenance fuzzy ───────────────────────────────────────────

def _trap(x, a, b, c, d):
    if x <= a or x >= d: return 0.0
    if x <= b: return (x - a) / (b - a)
    if x <= c: return 1.0
    return (d - x) / (d - c)


def _tri(x, a, b, c):
    return _trap(x, a, b, b, c)


def _sync_scalar(diff: int) -> int:
    """Fuzzy scalaire — même algorithme que test_sync.py, validé sur matériel."""
    a = abs(diff)
    if a <= SYNC_ZERO_THRESH: return 0
    sign = 1 if diff > 0 else -1
    if a <= SYNC_SMALL_THRESH:
        t = (a - SYNC_ZERO_THRESH) / (SYNC_SMALL_THRESH - SYNC_ZERO_THRESH)
        return sign * int(t * SYNC_STEP_MAX * 0.5)
    t = min(1.0, (a - SYNC_SMALL_THRESH) / SYNC_SMALL_THRESH)
    return sign * int(SYNC_STEP_MAX * 0.5 + t * SYNC_STEP_MAX * 0.5)


def _find_i2c(addr=PCA9685_ADDR):
    for n in range(10):
        try:
            b = smbus2.SMBus(n); b.read_byte(addr); b.close(); return n
        except (FileNotFoundError, PermissionError): pass
        except OSError:
            try: b.close()
            except Exception: pass
    return None


# ═══════════════════════════════════════════════════════════════════════════════
#  NŒUD
# ═══════════════════════════════════════════════════════════════════════════════

class NavigationLineNode(Node):

    # ── États ────────────────────────────────────────────────────────────────
    ST_LINE_FOLLOWING = 'LINE_FOLLOWING'
    ST_UTURN_1        = 'UTURN_1'         # 1er pivot 90°
    ST_UTURN_STRAIGHT = 'UTURN_STRAIGHT'  # avance 20 cm
    ST_UTURN_2        = 'UTURN_2'         # 2e pivot 90°
    ST_STOPPED_TRASH  = 'STOPPED_TRASH'
    ST_TURN_TO_BIN    = 'TURN_TO_BIN'
    ST_GO_TO_BIN      = 'GO_TO_BIN'
    ST_AT_BIN         = 'AT_BIN'
    ST_TURN_TO_LINE   = 'TURN_TO_LINE'   # retour vers position sauvegardée
    ST_GO_TO_LINE     = 'GO_TO_LINE'     # avance vers position sauvegardée
    ST_REORIENT       = 'REORIENT'       # pivot pour retrouver le cap sauvegardé
    ST_REACQUIRE      = 'REACQUIRE'      # pivot lent pour retrouver la ligne
    ST_DONE           = 'DONE'

    # YOLO actif seulement pendant le suivi de ligne
    FOLLOW_STATES = {ST_LINE_FOLLOWING}

    # Sync Layer 3 actif pendant les droites ET les pivots encodeur
    SYNC_STATES = {ST_GO_TO_BIN, ST_GO_TO_LINE, ST_UTURN_1, ST_UTURN_STRAIGHT, ST_UTURN_2}

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('navigation_line_node')

        # ── PCA9685 ───────────────────────────────────────────────────────────
        n = _find_i2c()
        if n is None:
            self.get_logger().fatal("PCA9685 introuvable sur I²C 0-9")
            raise RuntimeError("PCA9685 not found")
        self.bus = smbus2.SMBus(n)
        self._pca_init()
        self.get_logger().info(f"PCA9685 sur bus I²C {n}")

        # ── Encodeurs ─────────────────────────────────────────────────────────
        self._enc_ready     = False
        self._enc_left_raw  = 0;  self._enc_right_raw = 0
        self._enc_left_odo  = 0;  self._enc_right_odo = 0
        self._seg_base_l    = 0;  self._seg_base_r    = 0

        # ── Odométrie (mm, rad) ───────────────────────────────────────────────
        self._x = 0.0;  self._y = 0.0;  self._θ = 0.0
        self._last_og = 0;  self._last_od = 0

        # ── Layer 3 — sync state ──────────────────────────────────────────────
        self._base_pwm_g = 0;  self._base_pwm_d = 0
        self._sync_ref_l = 0;  self._sync_ref_r = 0
        self._sync_pwm_d = 0   # accumulateur intégral D (comme test_sync._cur_pwm_d)

        # ── Layer 1 — vitesse fuzzy ───────────────────────────────────────────
        self._cruise_speed = SPEED_FWD_LINE

        # ── Données ligne ─────────────────────────────────────────────────────
        self._line_ready  = False
        self._line_data   = [0.0, _LINE_LOST, 160.0, 480.0, 320.0]  # error, state, left_cx, right_cx, corridor_w
        self._line_lost_t = None    # timestamp première perte ligne
        self._last_error  = 0.0    # dernière erreur connue

        # ── Serpentine ────────────────────────────────────────────────────────
        self._state          = self.ST_LINE_FOLLOWING
        self._pass_num       = 0
        self._turn_dir       = 'L'
        self._t_start        = None
        self._follow_start_t = None   # timestamp début du couloir courant
        self._uturn_start_t  = None   # timestamp entrée ST_UTURN

        # ── Bin trip ──────────────────────────────────────────────────────────
        self._saved_x       = 0.0;  self._saved_y    = 0.0;  self._saved_θ = 0.0
        self._saved_sm_g    = 0;    self._saved_sm_d = 0
        self._bin_tx        = 0.0;  self._bin_ty     = 0.0
        self._bin_turn_pul  = 0;    self._bin_turn_left = True
        self._bin_drive_pul = 0
        self._pause_time    = None   # timer STOPPED_TRASH / AT_BIN

        # ── REACQUIRE ─────────────────────────────────────────────────────────
        self._reacquire_t   = None
        self._reacquire_dir = 'L'
        self._after_bin     = False

        # ── Anti-blocage (stall detection + kickstart) ────────────────────────
        self._stall_ref_l  = 0;    self._stall_ref_r  = 0
        self._stall_t_l    = None; self._stall_t_r    = None
        self._kick_active  = False; self._kick_t      = None
        self._kick_pwm_g   = 0;    self._kick_pwm_d   = 0

        # ── ROS ───────────────────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, '/line_error',
                                 self._cb_line_error, 10)
        self.create_subscription(Int64MultiArray,   '/wheel_encoders',
                                 self._cb_encoders,  10)
        self.create_subscription(Detection2DArray,  '/detections',
                                 self._cb_detection, 10)
        self.create_timer(0.05, self._loop)
        self.create_timer(0.05, self._sync_loop)
        self.create_timer(0.10, self._stall_loop)

        self.get_logger().info(
            f"NavigationLineNode prêt\n"
            f"  Serpentine : {NUM_PASSES} couloirs × {ROOM_MM/1000:.0f} m\n"
            f"  PUL_STRAIGHT={PUL_STRAIGHT}  PUL_TURN90={PUL_TURN90}  PUL_UTURN_STRAIGHT={PUL_UTURN_STRAIGHT}\n"
            f"  STEER_GAIN={STEER_GAIN}  SPEED_LINE={SPEED_FWD_LINE}%\n"
            f"  En attente /line_error + /wheel_encoders..."
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_encoders(self, msg: Int64MultiArray):
        self._enc_left_raw  = msg.data[0];  self._enc_right_raw = msg.data[1]
        self._enc_left_odo  = msg.data[2];  self._enc_right_odo = msg.data[3]
        if not self._enc_ready:
            self._enc_ready  = True
            self._seg_base_l = self._enc_left_raw
            self._seg_base_r = self._enc_right_raw
            self._sync_ref_l = 0;  self._sync_ref_r = 0
            self.get_logger().info("Encodeurs prêts.")

    def _cb_line_error(self, msg: Float32MultiArray):
        self._line_data = list(msg.data)
        if not self._line_ready:
            self._line_ready = True
            self.get_logger().info("Détection ligne prête.")

    def _cb_detection(self, msg: Detection2DArray):
        """Layer 1 — Vitesse YOLO ; déclenche bin trip si objet très proche."""
        if self._state not in self.FOLLOW_STATES:
            return

        if not msg.detections:
            if self._cruise_speed != SPEED_FWD_LINE:
                self._cruise_speed = SPEED_FWD_LINE
            return

        best  = max(msg.detections,
                    key=lambda d: d.results[0].hypothesis.score if d.results else 0.0)
        conf  = best.results[0].hypothesis.score  if best.results else 0.0
        area  = best.bbox.size_x * best.bbox.size_y
        speed = self._fuzzy_speed(conf, area)
        label = best.results[0].hypothesis.class_id if best.results else 'Other'

        if speed <= BIN_TRIP_SPEED:
            self.get_logger().info(
                f"[YOLO] {label}  conf={conf:.2f}  area={int(area)}px²"
                f"  speed={speed:.1f}% → BIN TRIP"
            )
            self._start_bin_trip(label)
        else:
            self._cruise_speed = speed

    # ─────────────────────────────────────────────────────────────────────────
    #  COMPTEURS SEGMENT ENCODEUR
    # ─────────────────────────────────────────────────────────────────────────

    def _seg_l(self) -> int:
        return self._enc_left_raw - self._seg_base_l

    def _seg_r(self) -> int:
        return self._enc_right_raw - self._seg_base_r

    def _reset_sm(self):
        self._seg_base_l = self._enc_left_raw
        self._seg_base_r = self._enc_right_raw

    def _set_sm(self, val_l: int, val_r: int):
        """Restaure les compteurs de segment après un bin trip."""
        self._seg_base_l = self._enc_left_raw  - val_l
        self._seg_base_r = self._enc_right_raw - val_r

    # ─────────────────────────────────────────────────────────────────────────
    #  ODOMÉTRIE
    # ─────────────────────────────────────────────────────────────────────────

    def _update_pose(self):
        og = self._enc_left_odo;  od = self._enc_right_odo
        dg = (og - self._last_og) * MM_PER_PULSE
        dd = (od - self._last_od) * MM_PER_PULSE
        self._last_og = og;  self._last_od = od

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
        low_c  = _trap(conf, 0.0, 0.0, 0.50, 0.70)
        med_c  = _tri( conf, 0.50, 0.70, 0.90)
        high_c = _trap(conf, 0.70, 0.90, 1.0,  1.0)

        small_a = _trap(area, 0,            0,            _AREA_SMALL,       _AREA_MEDIUM)
        med_a   = _tri( area, _AREA_SMALL,  _AREA_MEDIUM, _AREA_FRAME // 2)
        large_a = _trap(area, _AREA_MEDIUM, _AREA_FRAME // 2, _AREA_FRAME,   _AREA_FRAME)

        w_stop   = min(high_c, large_a)
        w_slow   = max(min(high_c, med_a),   min(med_c, large_a))
        w_medium = max(min(high_c, small_a), min(med_c, med_a))
        w_fast   = max(min(med_c, small_a),  low_c)

        total = w_stop + w_slow + w_medium + w_fast
        if total < 1e-6:
            return float(SPEED_FWD_LINE)
        return (w_stop * _SPD_STOP + w_slow * _SPD_SLOW +
                w_medium * _SPD_MEDIUM + w_fast * SPEED_FWD_LINE) / total

    # ─────────────────────────────────────────────────────────────────────────
    #  LAYER 2 — FUZZY STEERING
    # ─────────────────────────────────────────────────────────────────────────

    def _fuzzy_steer(self, error: float) -> float:
        """
        error [-1,+1] → steer [-1,+1].
        Positif = ligne à droite du centre → robot dévie à gauche → corriger à droite.
        Réponse non-linéaire : petites erreurs → faible correction (zone morte ±0.08).
        """
        nl = _trap(error, -1.0, -1.0, -0.40, -0.08)
        ns = _tri( error, -0.40, -0.08,  0.00)
        ze = _tri( error, -0.08,  0.00,  0.08)
        ps = _tri( error,  0.00,  0.08,  0.40)
        pl = _trap(error,  0.08,  0.40,  1.0,  1.0)

        total = nl + ns + ze + ps + pl
        if total < 1e-6:
            return 0.0
        return (-1.0 * nl - 0.40 * ns + 0.0 * ze + 0.40 * ps + 1.0 * pl) / total

    # ─────────────────────────────────────────────────────────────────────────
    #  BOUCLE PRINCIPALE  20 Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _loop(self):
        if not self._line_ready:
            return

        self._update_pose()

        now = self.get_clock().now()

        # ── Démarrage initial ─────────────────────────────────────────────────
        if self._t_start is None:
            self.get_logger().info(
                "Caméra + encodeurs prêts — démarrage suivi de ligne !"
            )
            self._t_start        = now
            self._follow_start_t = now
            self._reset_sm()
            self._state = self.ST_LINE_FOLLOWING

        if self._state == self.ST_DONE:
            return

        pg_raw, pd_raw = self._seg_l(), self._seg_r()
        # Si l'encodeur D est mort (toujours 0), utiliser G comme référence unique
        right_ok = self._enc_right_raw > 0
        pg   = pg_raw
        pd   = pd_raw if right_ok else pg_raw
        avg  = (pg + pd) // 2
        mini = min(pg, pd)

        # ══════════════════════════════════════════════════════════════════════
        #  SUIVI DE LIGNE (braquage fuzzy caméra)
        # ══════════════════════════════════════════════════════════════════════

        if self._state == self.ST_LINE_FOLLOWING:
            line_state = float(self._line_data[1]) if len(self._line_data) > 1 else _LINE_LOST
            raw_error  = float(self._line_data[0]) if self._line_data else 0.0

            # Temps écoulé depuis le début de ce couloir
            follow_dt = (now - self._follow_start_t).nanoseconds / 1e9 \
                        if self._follow_start_t else 0.0

            # ── Ligne de virage → demi-tour ───────────────────────────────────
            if line_state == _LINE_TURN and follow_dt >= MIN_PASS_TIME:
                self.get_logger().info(
                    f"LIGNE VIRAGE @{follow_dt:.1f}s — fin couloir {self._pass_num}"
                )
                self._trigger_u_turn(now)
                return

            if line_state != _LINE_LOST:
                # Ligne visible : braquage normal
                self._last_error  = raw_error
                self._line_lost_t = None
                steer = self._fuzzy_steer(self._last_error)
                self._moteur_follow(self._cruise_speed, steer)

            else:
                # Ligne perdue : grace period (maintenir dernier braquage)
                if self._line_lost_t is None:
                    self._line_lost_t = now

                lost_dt = (now - self._line_lost_t).nanoseconds / 1e9
                if lost_dt >= LINE_LOST_TIMEOUT:
                    if follow_dt >= MIN_PASS_TIME:
                        # Fin de couloir confirmée (ligne perdue après MIN_PASS_TIME)
                        self._trigger_u_turn(now)
                    else:
                        # Perte inattendue en début de couloir
                        self.get_logger().warn(
                            f"Ligne perdue trop tôt ({follow_dt:.1f}/{MIN_PASS_TIME}s) — REACQUIRE"
                        )
                        self._stop()
                        self._saved_sm_g = pg;  self._saved_sm_d = pd
                        self._enter_reacquire(now, after_bin=False)
                return

        # ══════════════════════════════════════════════════════════════════════
        #  DEMI-TOUR SERPENTINE — 90° + 20 cm droit + 90°
        # ══════════════════════════════════════════════════════════════════════

        elif self._state == self.ST_UTURN_1:
            if self._uturn_start_t is None:
                self._uturn_start_t = now
            t_uturn = (now - self._uturn_start_t).nanoseconds / 1e9
            if mini >= PUL_TURN90 or t_uturn >= UTURN_TURN_TIMEOUT:
                how = f"{mini}pul" if mini >= PUL_TURN90 else f"{t_uturn:.1f}s timeout"
                self._stop()
                self._uturn_start_t  = None
                self._reset_sm()
                self._line_lost_t    = None
                self._last_error     = 0.0
                self._cruise_speed   = SPEED_FWD_LINE
                self._follow_start_t = now
                self._state          = self.ST_LINE_FOLLOWING
                self.get_logger().info(
                    f"UTURN_1 90° ({how}) → LINE_FOLLOWING  couloir {self._pass_num}/{NUM_PASSES}"
                    f"  θ={math.degrees(self._θ):.0f}°"
                )

        # ══════════════════════════════════════════════════════════════════════
        #  PAUSE AU DÉCHET
        # ══════════════════════════════════════════════════════════════════════

        elif self._state == self.ST_STOPPED_TRASH:
            elapsed = (now - self._pause_time).nanoseconds / 1e9
            if elapsed >= TRASH_PAUSE:
                self._reset_sm()
                self._begin_turn_to(self._bin_tx, self._bin_ty)
                self._state = self.ST_TURN_TO_BIN
                self.get_logger().info(
                    f"STOPPED_TRASH → TURN_TO_BIN "
                    f"({self._bin_tx:.0f},{self._bin_ty:.0f}) mm"
                )

        # ══════════════════════════════════════════════════════════════════════
        #  BIN TRIP (encodeur)
        # ══════════════════════════════════════════════════════════════════════

        elif self._state == self.ST_TURN_TO_BIN:
            if self._bin_turn_pul == 0 or mini >= self._bin_turn_pul:
                self._stop()
                self._reset_sm()
                self._begin_drive_to(self._bin_tx, self._bin_ty)
                self._state = self.ST_GO_TO_BIN

        elif self._state == self.ST_GO_TO_BIN:
            if self._bin_drive_pul == 0 or avg >= self._bin_drive_pul:
                self._stop()
                self._pause_time = now
                self._state = self.ST_AT_BIN
                self.get_logger().info("Arrivé au bin — pause 2 s  [futur : pick & place]")

        elif self._state == self.ST_AT_BIN:
            elapsed = (now - self._pause_time).nanoseconds / 1e9
            if elapsed >= BIN_PAUSE:
                self._reset_sm()
                self._begin_turn_to(self._saved_x, self._saved_y)
                self._state = self.ST_TURN_TO_LINE
                self.get_logger().info("AT_BIN → TURN_TO_LINE")

        elif self._state == self.ST_TURN_TO_LINE:
            if self._bin_turn_pul == 0 or mini >= self._bin_turn_pul:
                self._stop()
                self._reset_sm()
                self._begin_drive_to(self._saved_x, self._saved_y)
                self._state = self.ST_GO_TO_LINE

        elif self._state == self.ST_GO_TO_LINE:
            if self._bin_drive_pul == 0 or avg >= self._bin_drive_pul:
                self._stop()
                self._reset_sm()
                # Recalculer le pivot pour retrouver le cap sauvegardé
                dθ = self._saved_θ - self._θ
                dθ = math.atan2(math.sin(dθ), math.cos(dθ))
                arc = abs(dθ) * WHEELBASE_MM / 2.0
                self._bin_turn_pul  = int(arc / MM_PER_PULSE)
                self._bin_turn_left = dθ > 0
                if self._bin_turn_pul > 10:
                    self._pivoter('L' if self._bin_turn_left else 'R')
                    self._state = self.ST_REORIENT
                    self.get_logger().info(
                        f"GO_TO_LINE → REORIENT  Δθ={math.degrees(dθ):.0f}°"
                    )
                else:
                    # Cap déjà correct → chercher la ligne directement
                    self._enter_reacquire(now, after_bin=True)

        elif self._state == self.ST_REORIENT:
            if mini >= self._bin_turn_pul:
                self._stop()
                self._enter_reacquire(now, after_bin=True)

        # ══════════════════════════════════════════════════════════════════════
        #  REACQUIRE — pivot lent pour retrouver la ligne
        # ══════════════════════════════════════════════════════════════════════

        elif self._state == self.ST_REACQUIRE:
            line_state = float(self._line_data[1]) if len(self._line_data) > 1 else _LINE_LOST

            if line_state == _LINE_OK:
                self._stop()
                if self._after_bin:
                    self._set_sm(self._saved_sm_g, self._saved_sm_d)
                self._after_bin      = False
                self._line_lost_t    = None
                self._last_error     = float(self._line_data[0])
                self._cruise_speed   = SPEED_FWD_LINE
                self._follow_start_t = now   # ne pas déclencher UTURN immédiatement
                self._state = self.ST_LINE_FOLLOWING
                self.get_logger().info("REACQUIRE → LINE_FOLLOWING (ligne retrouvée)")
                return

            if self._reacquire_t is None:
                return   # garde-fou

            elapsed = (now - self._reacquire_t).nanoseconds / 1e9

            if REACQUIRE_T1 <= elapsed < REACQUIRE_T1 + 0.06:
                # Changer de direction une seule fois (fenêtre 60 ms)
                new_dir = 'R' if self._reacquire_dir == 'L' else 'L'
                self._reacquire_dir = new_dir
                self._pivoter_slow(new_dir)
                self.get_logger().warn(
                    f"REACQUIRE : pivot {new_dir} (2e tentative)"
                )

            elif elapsed >= REACQUIRE_T2:
                self._stop()
                self._state = self.ST_DONE
                self.get_logger().error(
                    "REACQUIRE : ligne introuvable après 4.5 s — ARRÊT."
                )

    # ─────────────────────────────────────────────────────────────────────────
    #  HELPERS SERPENTINE
    # ─────────────────────────────────────────────────────────────────────────

    def _trigger_u_turn(self, now):
        """Incrémente le compteur de couloirs et démarre le virage en U (90°+20cm+90°)."""
        self._pass_num += 1
        if self._pass_num >= NUM_PASSES:
            self._stop()
            self._state = self.ST_DONE
            self.get_logger().info(
                f"✓ Couverture complète — {NUM_PASSES} couloirs parcourus !"
            )
            return

        self._stop()
        self._reset_sm()
        self._uturn_start_t = None
        self._pivoter(self._turn_dir)
        self._state = self.ST_UTURN_1
        self.get_logger().info(
            f"Couloir {self._pass_num - 1} terminé → UTURN_1 90° ({self._turn_dir})"
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  HELPERS BIN TRIP
    # ─────────────────────────────────────────────────────────────────────────

    def _start_bin_trip(self, waste_class: str):
        self._saved_x    = self._x;  self._saved_y = self._y;  self._saved_θ = self._θ
        self._saved_sm_g = self._seg_l();  self._saved_sm_d = self._seg_r()

        bx, by = BIN_POSITIONS.get(waste_class, BIN_POSITIONS['Other'])
        self._bin_tx = bx;  self._bin_ty = by

        self._stop()
        self._cruise_speed = SPEED_FWD
        self._pause_time   = self.get_clock().now()
        self._state        = self.ST_STOPPED_TRASH
        self.get_logger().info(
            f"DÉCHET [{waste_class}]  pos=({self._x:.0f},{self._y:.0f}) mm"
            f"  → BIN ({bx:.0f},{by:.0f}) mm"
        )

    def _begin_turn_to(self, tx: float, ty: float):
        """Calcule et démarre le pivot vers (tx, ty)."""
        dθ = math.atan2(ty - self._y, tx - self._x) - self._θ
        dθ = math.atan2(math.sin(dθ), math.cos(dθ))
        arc = abs(dθ) * WHEELBASE_MM / 2.0
        self._bin_turn_pul  = int(arc / MM_PER_PULSE)
        self._bin_turn_left = dθ > 0
        if self._bin_turn_pul > 10:
            self._pivoter('L' if self._bin_turn_left else 'R')
        else:
            self._bin_turn_pul = 0

    def _begin_drive_to(self, tx: float, ty: float):
        """Calcule et démarre l'avancée vers (tx, ty)."""
        dist = math.sqrt((tx - self._x) ** 2 + (ty - self._y) ** 2)
        self._bin_drive_pul = int(dist / MM_PER_PULSE)
        if self._bin_drive_pul > 10:
            self._avancer(SPEED_FWD)
        else:
            self._bin_drive_pul = 0

    def _enter_reacquire(self, now, after_bin: bool):
        """Démarre le pivot lent REACQUIRE dans la direction de l'erreur courante."""
        self._after_bin     = after_bin
        self._reacquire_t   = now
        # Tourner vers l'endroit où la ligne devrait être
        self._reacquire_dir = 'R' if self._last_error >= 0 else 'L'
        self._pivoter_slow(self._reacquire_dir)
        self._state = self.ST_REACQUIRE
        self.get_logger().info(
            f"REACQUIRE — pivot lent {self._reacquire_dir}"
            f"  last_err={self._last_error:+.2f}  after_bin={after_bin}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  LAYER 3 — FUZZY WHEEL SYNC  20 Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _sync_loop(self):
        if not self._enc_ready or self._kick_active:
            return
        if self._state not in self.SYNC_STATES or self._base_pwm_d == 0:
            return

        cur_l = self._seg_l();  cur_r = self._seg_r()

        # Stall guard : aucun mouvement → pas de correction (évite le windup)
        if cur_l == self._sync_ref_l and cur_r == self._sync_ref_r:
            return

        vel_diff = (cur_l - self._sync_ref_l) - (cur_r - self._sync_ref_r)
        pos_diff = cur_l - cur_r
        self._sync_ref_l = cur_l;  self._sync_ref_r = cur_r

        diff  = vel_diff + pos_diff // SYNC_POS_WEIGHT
        delta = _sync_scalar(diff)

        if delta != 0:
            new_d = self._sync_pwm_d + delta
            new_d = max(self._base_pwm_d - SYNC_DRIFT_MAX,
                        min(self._base_pwm_d + SYNC_DRIFT_MAX, new_d))
            new_d = max(0, min(4095, new_d))
            self._sync_pwm_d = new_d
            self._pca_canal(CANAUX_D[0], new_d)
            self.get_logger().debug(
                f"[SYNC] vel={vel_diff:+d} pos={pos_diff:+d}"
                f"  delta={delta:+d}  D={new_d}  base={self._base_pwm_d}"
            )

    # ─────────────────────────────────────────────────────────────────────────
    #  COMMANDES MOTEURS
    # ─────────────────────────────────────────────────────────────────────────

    def _moteur_follow(self, speed: float, steer: float):
        """
        Braquage différentiel pendant le suivi de ligne.
        steer > 0 → tourner à droite (roue G plus rapide que D).
        """
        spd_L = max(0.0, min(float(SPEED_MAX_LINE), speed * (1.0 + STEER_GAIN * steer)))
        spd_R = max(0.0, min(float(SPEED_MAX_LINE), speed * (1.0 - STEER_GAIN * steer)))
        self._moteur('G', spd_L)
        self._moteur('D', spd_R)

    def _avancer(self, speed: float = SPEED_FWD):
        self._moteur('G', speed)
        self._moteur('D', speed)
        self._sync_ref_l = self._seg_l()
        self._sync_ref_r = self._seg_r()
        self._sync_pwm_d = self._base_pwm_d  # reset accumulateur intégral

    def _pivoter(self, direction: str):
        if direction == 'L':
            self._moteur('G', -SPEED_TURN);  self._moteur('D',  SPEED_TURN)
        else:
            self._moteur('G',  SPEED_TURN);  self._moteur('D', -SPEED_TURN)
        self._sync_ref_l = self._seg_l()
        self._sync_ref_r = self._seg_r()
        self._sync_pwm_d = self._base_pwm_d  # reset accumulateur pour le pivot

    def _pivoter_slow(self, direction: str):
        if direction == 'L':
            self._moteur('G', -REACQUIRE_SP);  self._moteur('D',  REACQUIRE_SP)
        else:
            self._moteur('G',  REACQUIRE_SP);  self._moteur('D', -REACQUIRE_SP)

    def _stop(self):
        self._moteur('G', 0);  self._moteur('D', 0)

    def _moteur(self, cote: str, vitesse: float):
        if cote == 'D' and vitesse != 0:
            vitesse *= SPEED_TRIM_D
        # Ne jamais commander entre 0 et SPEED_MIN — zone de calage garantie
        if 0 < abs(vitesse) < SPEED_MIN:
            vitesse = math.copysign(SPEED_MIN, vitesse)
        pwm = int(abs(vitesse) / 100.0 * 4095)
        ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
        if vitesse > 0:
            self._pca_canal(ch_a, 4095);  self._pca_canal(ch_b, 0)
        elif vitesse < 0:
            self._pca_canal(ch_a, 0);     self._pca_canal(ch_b, 4095)
        else:
            self._pca_canal(ch_a, 0);     self._pca_canal(ch_b, 0);  pwm = 0
        self._pca_canal(ch_en, pwm)
        if cote == 'G': self._base_pwm_g = pwm
        else:           self._base_pwm_d = pwm

    # ─────────────────────────────────────────────────────────────────────────
    #  ANTI-BLOCAGE  10 Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _stall_loop(self):
        """Détecte un calage de roue et applique une impulsion kickstart."""
        if not self._enc_ready:
            return

        now = self.get_clock().now()

        # Fin de kickstart → restaurer PWM sauvegardé
        if self._kick_active:
            elapsed = (now - self._kick_t).nanoseconds / 1e9
            if elapsed >= KICK_DURATION:
                self._kick_active = False
                self._pca_canal(CANAUX_G[0], self._kick_pwm_g)
                self._pca_canal(CANAUX_D[0], self._kick_pwm_d)
                self._base_pwm_g = self._kick_pwm_g
                self._base_pwm_d = self._kick_pwm_d
            return

        # Pas de détection si moteurs à l'arrêt ou état stationnaire
        if self._base_pwm_g == 0 and self._base_pwm_d == 0:
            self._stall_t_l = None;  self._stall_t_r = None
            return
        if self._state in {self.ST_DONE, self.ST_STOPPED_TRASH, self.ST_AT_BIN}:
            return

        cur_l = self._enc_left_raw
        cur_r = self._enc_right_raw

        # Roue G calée ?
        if self._base_pwm_g > 0 and cur_l == self._stall_ref_l:
            if self._stall_t_l is None:
                self._stall_t_l = now
            elif (now - self._stall_t_l).nanoseconds / 1e9 >= STALL_TIMEOUT_S:
                self.get_logger().warn(
                    f"[STALL] roue G calée {STALL_TIMEOUT_S:.2f} s → kickstart {KICK_SPEED}%"
                )
                self._start_kickstart()
                return
        else:
            self._stall_t_l = None

        # Roue D calée ?
        if self._base_pwm_d > 0 and cur_r == self._stall_ref_r:
            if self._stall_t_r is None:
                self._stall_t_r = now
            elif (now - self._stall_t_r).nanoseconds / 1e9 >= STALL_TIMEOUT_S:
                self.get_logger().warn(
                    f"[STALL] roue D calée {STALL_TIMEOUT_S:.2f} s → kickstart {KICK_SPEED}%"
                )
                self._start_kickstart()
                return
        else:
            self._stall_t_r = None

        self._stall_ref_l = cur_l
        self._stall_ref_r = cur_r

    def _start_kickstart(self):
        """Impulsion haute puissance brève pour briser la friction statique."""
        self._kick_active = True
        self._kick_t      = self.get_clock().now()
        self._kick_pwm_g  = self._base_pwm_g
        self._kick_pwm_d  = self._base_pwm_d
        self._stall_t_l   = None;  self._stall_t_r = None
        kick_pwm = int(KICK_SPEED / 100.0 * 4095)
        if self._base_pwm_g > 0:
            self._pca_canal(CANAUX_G[0], kick_pwm)
        if self._base_pwm_d > 0:
            self._pca_canal(CANAUX_D[0], kick_pwm)

    # ─────────────────────────────────────────────────────────────────────────
    #  PCA9685 bas niveau
    # ─────────────────────────────────────────────────────────────────────────

    def _pca_init(self, freq: int = 1000):
        self._pca_w(0x00, 0x10)
        self._pca_w(0xFE, int(25_000_000 / (4096 * freq) - 1))
        self._pca_w(0x00, 0x00)
        time.sleep(0.005)
        self._pca_w(0x00, 0xA0)

    def _pca_w(self, reg: int, val: int):
        self.bus.write_byte_data(PCA9685_ADDR, reg, val, force=True)

    def _pca_canal(self, canal: int, v: int):
        reg = 0x06 + 4 * canal
        if v >= 4095:   d = [0x00, 0x10, 0x00, 0x00]
        elif v <= 0:    d = [0x00, 0x00, 0x00, 0x10]
        else:           d = [0x00, 0x00, v & 0xFF, v >> 8]
        self.bus.write_i2c_block_data(PCA9685_ADDR, reg, d, force=True)

    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop()
        self.bus.close()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NavigationLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
