#!/usr/bin/env python3
"""
motor_node.py
=============
Nœud ROS2 — Contrôle moteurs (navigation)

Logique :
  - Démarre en avant à 60 %
  - Si /detections contient au moins un objet → ARRÊT
  - Si /detections est vide → reprend en avant
  - Watchdog : si aucun message /detections depuis 1 s → reprend
    (protection contre perte caméra)

Chemin matériel : Jetson → I²C → PCA9685 → L298N → Moteurs
Canaux validés avec TEST_JESTON_Moteur_AVANT.py
"""

import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import smbus2


# ── Assignation canaux PCA9685 → L298N (validés sur le robot physique) ────────
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)   # ENA, IN2, IN1  — moteur gauche (monté en miroir)
CANAUX_D     = (6, 7, 8)   # ENB, IN3, IN4  — moteur droit


def _detect_i2c_bus(addr=PCA9685_ADDR, candidates=range(10)):
    """Retourne le premier numéro de bus I²C où addr répond, ou None."""
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


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── Paramètre vitesse (modifiable au lancement) ───────────────────────
        self.declare_parameter('speed', 60)
        self.speed = self.get_parameter('speed').value

        # ── Init PCA9685 via I²C ──────────────────────────────────────────────
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

        # ── État interne ──────────────────────────────────────────────────────
        self._stopped = False
        self._last_msg_time = self.get_clock().now()

        # ── Démarrage immédiat en avant ───────────────────────────────────────
        self._avancer(self.speed)
        self.get_logger().info(f"Moteurs démarrés — vitesse {self.speed} %")

        # ── Abonnement aux détections YOLO ────────────────────────────────────
        self.create_subscription(
            Detection2DArray,
            '/detections',
            self._detection_callback,
            10
        )

        # ── Watchdog : reprend si /detections silencieux depuis 1 s ──────────
        self.create_timer(0.5, self._watchdog_callback)

        self.get_logger().info("MotorNode prêt — écoute /detections")

    # ── Callback détection ────────────────────────────────────────────────────

    def _detection_callback(self, msg: Detection2DArray):
        self._last_msg_time = self.get_clock().now()

        if len(msg.detections) > 0:
            if not self._stopped:
                self._stop()
                labels = [
                    d.results[0].hypothesis.class_id
                    for d in msg.detections if d.results
                ]
                self.get_logger().info(f"Déchet détecté → {labels} — ARRÊT")
        else:
            if self._stopped:
                self._avancer(self.speed)
                self.get_logger().info("Aucun déchet → reprise en avant")

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog_callback(self):
        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if elapsed > 1.0 and self._stopped:
            self._avancer(self.speed)
            self.get_logger().warn(
                "Watchdog : aucune détection depuis 1 s — reprise automatique"
            )

    # ── Contrôle moteurs ──────────────────────────────────────────────────────

    def _avancer(self, vitesse=60):
        self._moteur('G', vitesse)
        self._moteur('D', vitesse)
        self._stopped = False

    def _stop(self):
        self._moteur('G', 0)
        self._moteur('D', 0)
        self._stopped = True

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
        prescaler = int(25_000_000 / (4096 * freq) - 1)
        self._pca_write(0xFE, prescaler)
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
        self._stop()
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
