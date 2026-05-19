#!/usr/bin/env python3
"""
motor_node.py
=============
Nœud ROS2 — Contrôle moteurs avec correcteur fuzzy de vitesse

Logique :
  - Démarre en avant à SPEED_CRUISE (60 %)
  - Si /detections contient un objet → vitesse calculée par le contrôleur fuzzy
      Entrées  : confiance YOLO (0–1)  ×  aire bbox (px²)
      Sortie   : vitesse moteur 0–60 %  (lisse, pas de saut binaire)
  - Si /detections est vide → retour à SPEED_CRUISE
  - Watchdog : reprend à SPEED_CRUISE si aucun message depuis 1 s

Chemin matériel : Jetson → I²C → PCA9685 → L298N → Moteurs
Canaux validés avec TEST_JESTON_Moteur_AVANT.py
"""

import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import smbus2


# ── Assignation canaux PCA9685 → L298N ───────────────────────────────────────
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)   # ENA, IN2, IN1  — moteur gauche (monté en miroir)
CANAUX_D     = (6, 7, 8)   # ENB, IN3, IN4  — moteur droit

# ── Vitesses de sortie fuzzy (singletons, en %) ───────────────────────────────
SPEED_CRUISE = 60    # vitesse de croisière — aucune détection
SPEED_MEDIUM = 35    # approche prudente
SPEED_SLOW   = 15    # objet assez proche
SPEED_STOP   = 0     # objet très proche / haute confiance

# ── Seuils aire bbox (px²) — à calibrer sur le terrain ───────────────────────
# Cadre caméra : 640×480 = 307 200 px²
# Petit  (<20 000 px²) ≈ objet loin  (> ~1 m)
# Moyen  (20 000–60 000 px²) ≈ distance intermédiaire
# Grand  (> 60 000 px²) ≈ objet proche (< ~0.5 m)
AREA_SMALL_THRESH  = 20_000
AREA_MEDIUM_THRESH = 60_000
AREA_FRAME         = 640 * 480   # 307 200 px²


# ── Fonctions d'appartenance fuzzy ───────────────────────────────────────────

def _trap(x: float, a: float, b: float, c: float, d: float) -> float:
    """Fonction d'appartenance trapézoïdale.  0 hors [a,d], 1 entre [b,c]."""
    if x <= a or x >= d:
        return 0.0
    if x <= b:
        return (x - a) / (b - a)
    if x <= c:
        return 1.0
    return (d - x) / (d - c)


def _tri(x: float, a: float, b: float, c: float) -> float:
    """Fonction d'appartenance triangulaire."""
    return _trap(x, a, b, b, c)


# ── Détection bus I²C ─────────────────────────────────────────────────────────

def _detect_i2c_bus(addr=PCA9685_ADDR, candidates=range(10)):
    for n in candidates:
        try:
            b = smbus2.SMBus(n)
            b.read_byte(addr)
            b.close()
            return n
        except (FileNotFoundError, PermissionError):
            pass
        except OSError:
            try:
                b.close()
            except Exception:
                pass
    return None


# ── Nœud ─────────────────────────────────────────────────────────────────────

class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        self.declare_parameter('speed', SPEED_CRUISE)
        self.speed = self.get_parameter('speed').value   # vitesse croisière

        bus_num = _detect_i2c_bus()
        if bus_num is None:
            self.get_logger().fatal(
                "PCA9685 introuvable sur les bus I²C 0-9. "
                "Vérifiez le câblage SDA/SCL et l'alimentation."
            )
            raise RuntimeError("PCA9685 not found")

        self.get_logger().info(f"PCA9685 détecté sur le bus I²C {bus_num}")
        self.bus = smbus2.SMBus(bus_num)
        self._pca_init(freq=1000)

        self._current_speed  = 0.0
        self._detection_active = False
        self._last_msg_time  = self.get_clock().now()

        self._set_speed(self.speed)
        self.get_logger().info(f"Moteurs démarrés — vitesse croisière {self.speed} %")

        self.create_subscription(
            Detection2DArray, '/detections', self._detection_callback, 10
        )
        self.create_timer(0.5, self._watchdog_callback)

        self.get_logger().info(
            f"MotorNode fuzzy prêt — écoute /detections\n"
            f"  Seuils aire : petit<{AREA_SMALL_THRESH}  moyen<{AREA_MEDIUM_THRESH}  grand≥{AREA_MEDIUM_THRESH} px²\n"
            f"  Singletons  : Stop={SPEED_STOP}%  Slow={SPEED_SLOW}%  "
            f"Medium={SPEED_MEDIUM}%  Fast={self.speed}%"
        )

    # ── Contrôleur fuzzy ──────────────────────────────────────────────────────

    def _fuzzy_speed(self, conf: float, area: float) -> float:
        """
        Contrôleur fuzzy manuel (Mamdani + défuzzification par moyenne pondérée).

        Entrées
          conf : confiance YOLO     0.0 – 1.0
          area : aire bbox en px²   0 – 307 200

        Sortie
          vitesse moteur en %       0 – SPEED_CRUISE
        """
        # ── Fuzzification — confiance ─────────────────────────────────────────
        low_c  = _trap(conf, 0.0, 0.0, 0.50, 0.70)
        med_c  = _tri( conf, 0.50, 0.70, 0.90)
        high_c = _trap(conf, 0.70, 0.90, 1.0, 1.0)

        # ── Fuzzification — aire bbox ─────────────────────────────────────────
        small_a = _trap(area,
                        0,                  0,
                        AREA_SMALL_THRESH,  AREA_MEDIUM_THRESH)
        med_a   = _tri( area,
                        AREA_SMALL_THRESH,  AREA_MEDIUM_THRESH,
                        AREA_FRAME // 2)
        large_a = _trap(area,
                        AREA_MEDIUM_THRESH, AREA_FRAME // 2,
                        AREA_FRAME,         AREA_FRAME)

        # ── Règles fuzzy (inférence Mamdani — opérateur min) ──────────────────
        #   Confiance  ×  Aire      →  Vitesse
        #   High       ×  Large     →  Stop
        #   High       ×  Medium    →  Slow
        #   High       ×  Small     →  Medium
        #   Medium     ×  Large     →  Slow
        #   Medium     ×  Medium    →  Medium
        #   Medium     ×  Small     →  Fast
        #   Low        ×  Any       →  Fast
        w_stop   = min(high_c, large_a)
        w_slow   = max(min(high_c, med_a),   min(med_c, large_a))
        w_medium = max(min(high_c, small_a), min(med_c, med_a))
        w_fast   = max(min(med_c, small_a),  low_c)

        # ── Défuzzification — moyenne pondérée des singletons ─────────────────
        total = w_stop + w_slow + w_medium + w_fast
        if total < 1e-6:
            return float(self.speed)   # aucune règle active → croisière

        speed = (
            w_stop   * SPEED_STOP   +
            w_slow   * SPEED_SLOW   +
            w_medium * SPEED_MEDIUM +
            w_fast   * self.speed
        ) / total
        return speed

    # ── Callback détection ────────────────────────────────────────────────────

    def _detection_callback(self, msg: Detection2DArray):
        self._last_msg_time = self.get_clock().now()

        if msg.detections:
            # Meilleure détection (score le plus élevé)
            best = max(
                msg.detections,
                key=lambda d: d.results[0].hypothesis.score if d.results else 0.0
            )
            conf  = best.results[0].hypothesis.score if best.results else 0.0
            area  = best.bbox.size_x * best.bbox.size_y
            speed = self._fuzzy_speed(conf, area)

            self._set_speed(speed)
            self._detection_active = True

            label = best.results[0].hypothesis.class_id if best.results else '?'
            self.get_logger().info(
                f"[FUZZY] {label}  conf={conf:.2f}  "
                f"aire={int(area)} px²  →  vitesse={speed:.1f} %"
            )

        else:
            if self._detection_active:
                self._set_speed(self.speed)
                self._detection_active = False
                self.get_logger().info(
                    f"Aucun déchet → vitesse croisière {self.speed} %"
                )

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog_callback(self):
        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if elapsed > 1.0 and self._detection_active:
            self._set_speed(self.speed)
            self._detection_active = False
            self.get_logger().warn(
                f"Watchdog : aucune détection depuis 1 s — "
                f"reprise à {self.speed} %"
            )

    # ── Contrôle moteurs ──────────────────────────────────────────────────────

    def _set_speed(self, speed: float):
        self._moteur('G', speed)
        self._moteur('D', speed)
        self._current_speed = speed

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

    # ── Bas niveau PCA9685 ────────────────────────────────────────────────────

    def _pca_init(self, freq=1000):
        self._pca_write(0x00, 0x10)
        self._pca_write(0xFE, int(25_000_000 / (4096 * freq) - 1))
        self._pca_write(0x00, 0x00)
        time.sleep(0.005)
        self._pca_write(0x00, 0xA0)

    def _pca_write(self, reg, val):
        self.bus.write_byte_data(PCA9685_ADDR, reg, val, force=True)

    def _pca_canal(self, canal, valeur):
        reg = 0x06 + 4 * canal
        if valeur >= 4095:
            self.bus.write_i2c_block_data(
                PCA9685_ADDR, reg, [0x00, 0x10, 0x00, 0x00], force=True)
        elif valeur <= 0:
            self.bus.write_i2c_block_data(
                PCA9685_ADDR, reg, [0x00, 0x00, 0x00, 0x10], force=True)
        else:
            self.bus.write_i2c_block_data(
                PCA9685_ADDR, reg,
                [0x00, 0x00, valeur & 0xFF, valeur >> 8], force=True)

    # ── Nettoyage ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._set_speed(0)
        self.bus.close()
        self.get_logger().info("Moteurs arrêtés — nœud détruit.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
