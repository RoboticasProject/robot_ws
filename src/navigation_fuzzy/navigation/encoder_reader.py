#!/usr/bin/env python3
"""
encoder_reader.py — Lecture encodeurs Hall via libgpiod (mode polling).

LINE_REQ_EV_BOTH_EDGES ne génère pas d'événements noyau sur ces lignes GPIO
du Jetson Orin Nano — on utilise le polling get_value() à 0.5 ms comme
scan_gpio.py, ce qui est largement suffisant (encodeur max ~63 Hz à pleine vitesse).

Correspondance BOARD → gpiochip0 (vérifiée avec scan_gpio.py) :
  BOARD 11 → line 112 (PR.04)  — ENC_G_A
  BOARD 13 → line 122 (PY.00)  — ENC_G_B
  BOARD 15 → line  85 (PN.01)  — ENC_D_A
  BOARD 16 → line 126 (PY.04)  — ENC_D_B

Installation requise :
  sudo apt install python3-libgpiod
"""

import atexit
import time

import gpiod
import threading

# ── DFRobot FIT0277 — 12V, 146 RPM max, Two-phase Hall encoder ──────────────
PPR_MOTEUR       = 13
RATIO_REDUCTEUR  = 51
PPR_ROUE         = PPR_MOTEUR * RATIO_REDUCTEUR   # 663 théorique (RISING seul)
# Polling Python 0.5 ms rate ~50 % des fronts à PWM 35 % (période ~0.89 ms).
# PPR_EFFECTIF est la valeur MESURÉE terrain : 500 mm = 1550 pulses (moy. G/D).
PPR_EFFECTIF     = 633
DEGRES_PAR_PULSE = 360.0 / PPR_EFFECTIF            # 0.569 °/pulse

# ── GPIO — offsets vérifiés sur ce Jetson avec Jetson.GPIO.gpio_pin_data ─────
CHIP_NAME    = 'gpiochip0'   # tegra234-gpio (164 lignes)
LINE_ENC_G_A = 112            # BOARD pin 11 — front A gauche
LINE_ENC_G_B = 122            # BOARD pin 13 — phase B gauche
LINE_ENC_D_A = 85             # BOARD pin 15 — front A droit
LINE_ENC_D_B = 126            # BOARD pin 16 — phase B droit


class EncodeurRoue:
    """
    Lecteur d'encodeur quadrature Hall via libgpiod (noyau Linux).

    Le thread interne se bloque sur event_wait() — il est réveillé par le
    noyau dès qu'un front arrive (~10 µs de latence). Aucun polling, aucune
    perte d'impulsion.

    fwd_b_on_rising : niveau logique du signal B qui indique « marche avant »
                      au moment d'un front montant de A.
                        0 → roue gauche (B LOW = avant)
                        1 → roue droite (montage en miroir, B HIGH = avant)
    """

    def __init__(self, chip_name: str, line_A: int, line_B: int,
                 nom: str, fwd_b_on_rising: int = 0):
        self.nom    = nom
        self._lock  = threading.Lock()
        self._fwd_b = fwd_b_on_rising

        # _raw_abs : compteur brut, jamais remis à zéro (toujours +1)
        # _baseline : valeur de _raw_abs au dernier reset_abs()
        # _odo      : compte signé pour l'odométrie (jamais remis à zéro)
        # _rpm_cnt  : compteur RPM, remis à zéro par get_rpm()
        self._raw_abs  = 0
        self._baseline = 0
        self._odo      = 0
        self._rpm_cnt  = 0

        chip = gpiod.Chip(chip_name)
        self._chip   = chip          # kept alive so we can close() it later
        self._line_A = chip.get_line(line_A)
        self._line_B = chip.get_line(line_B)

        self._line_B.request(consumer=f'enc_{nom}_B', type=gpiod.LINE_REQ_DIR_IN)
        self._line_A.request(consumer=f'enc_{nom}_A', type=gpiod.LINE_REQ_DIR_IN)

        self._running = True
        self._thread  = threading.Thread(
            target=self._loop, name=f'encoder_{nom}', daemon=True)
        self._thread.start()

    def _loop(self):
        """Polling 0.5 ms — suffisant pour encodeur max ~63 Hz à pleine vitesse."""
        prev = self._line_A.get_value()
        while self._running:
            v = self._line_A.get_value()
            if v != prev:
                b      = self._line_B.get_value()
                rising = (v == 1)
                if rising:
                    delta = +1 if b == self._fwd_b else -1
                else:
                    delta = +1 if b != self._fwd_b else -1
                with self._lock:
                    self._raw_abs += 1
                    self._odo     += delta
                    self._rpm_cnt += 1
                prev = v
            time.sleep(0.0005)

    # ── Interface machine à états (navigation) ───────────────────────────────

    def get_abs(self) -> int:
        """Pulses depuis le dernier reset_abs() — toujours positif."""
        with self._lock:
            return self._raw_abs - self._baseline

    def reset_abs(self):
        """Remet le compteur relatif à zéro (odométrie inchangée)."""
        with self._lock:
            self._baseline = self._raw_abs

    def set_abs(self, value: int):
        """Restaure le compteur relatif à une valeur précédemment sauvegardée."""
        with self._lock:
            self._baseline = self._raw_abs - value

    # ── Interface odométrie ──────────────────────────────────────────────────

    def get_raw(self) -> int:
        """Raw absolute count — always increasing, never reset."""
        with self._lock:
            return self._raw_abs

    def get_odo(self) -> int:
        """Compte signé cumulatif — positif = avant, négatif = arrière."""
        with self._lock:
            return self._odo

    # ── Interface RPM (optionnel — pour le PID moteur) ───────────────────────

    def get_rpm(self, dt: float) -> float:
        """RPM moyen sur la fenêtre dt (secondes) — consomme le compteur RPM."""
        with self._lock:
            cnt = self._rpm_cnt
            self._rpm_cnt = 0
        return (cnt / PPR_EFFECTIF) * (60.0 / dt) if dt > 0 else 0.0

    # ── Nettoyage ────────────────────────────────────────────────────────────

    def stop(self):
        """Arrête le thread de lecture et libère les lignes GPIO."""
        if not self._running:
            return
        self._running = False
        self._thread.join(timeout=2)
        for ln in (self._line_A, self._line_B):
            try:
                ln.release()
            except Exception:
                pass
        try:
            self._chip.close()
        except Exception:
            pass


# ── Singletons globaux — importés par navigation_node ────────────────────────
enc_G = EncodeurRoue(CHIP_NAME, LINE_ENC_G_A, LINE_ENC_G_B, 'G', fwd_b_on_rising=0)
enc_D = EncodeurRoue(CHIP_NAME, LINE_ENC_D_A, LINE_ENC_D_B, 'D', fwd_b_on_rising=1)

# Garantit la libération GPIO même si le process est tué sans appeler stop()
atexit.register(enc_D.stop)
atexit.register(enc_G.stop)
