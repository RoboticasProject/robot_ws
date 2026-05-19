#!/usr/bin/env python3
"""
test_motors.py — Test moteurs + calibration encodeur/trim.

Fait avancer le robot en ligne droite ~500 mm, puis :
  1. Vérifie que les deux moteurs tournent
  2. Mesure l'écart G/D → suggère SPEED_TRIM_D correct
  3. Calcule le PPR réel si vous mesurez la distance parcourue

Prérequis :
  - Moteurs alimentés (12 V)
  - 60 cm d'espace dégagé devant le robot
  - Marquer la position de départ sur le sol (repère ruban adhésif)

Lancement :
  cd ~/robot_ws
  python3 src/navigation/navigation/test_motors.py
"""

import math
import signal
import sys
import time

import smbus2

try:
    from navigation.encoder_reader import enc_G, enc_D, PPR_EFFECTIF
except ImportError:
    from encoder_reader import enc_G, enc_D, PPR_EFFECTIF

# ── Paramètres — doivent correspondre à navigation_node.py ───────────────────
WHEEL_DIAM_MM = 65.0
WHEEL_CIRC    = math.pi * WHEEL_DIAM_MM          # 204.2 mm
MM_PER_PULSE  = WHEEL_CIRC / PPR_EFFECTIF        # ~0.154 mm/pulse
TARGET_MM     = 500.0
PUL_TARGET    = int(TARGET_MM / MM_PER_PULSE)    # ~3247
SPEED_FWD     = 35                               # %
SPEED_TRIM_D  = 0.85                             # applied to motor D (physically right/faster)
MAX_SECS      = 8.0                              # failsafe

# ── PCA9685 ───────────────────────────────────────────────────────────────────
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)    # ENA, IN2, IN1
CANAUX_D     = (6, 7, 8)    # ENB, IN3, IN4

bus = None
_done = False


def _find_i2c():
    for n in range(10):
        try:
            b = smbus2.SMBus(n)
            b.read_byte(PCA9685_ADDR)
            b.close()
            return n
        except (FileNotFoundError, PermissionError):
            pass
        except OSError:
            try: b.close()
            except Exception: pass
    return None


def _pca_w(reg, val):
    bus.write_byte_data(PCA9685_ADDR, reg, val, force=True)


def _pca_init(freq=1000):
    _pca_w(0x00, 0x10)
    _pca_w(0xFE, int(25_000_000 / (4096 * freq) - 1))
    _pca_w(0x00, 0x00)
    time.sleep(0.005)
    _pca_w(0x00, 0xA0)


def _canal(canal, v):
    reg = 0x06 + 4 * canal
    if v >= 4095:
        d = [0x00, 0x10, 0x00, 0x00]
    elif v <= 0:
        d = [0x00, 0x00, 0x00, 0x10]
    else:
        d = [0x00, 0x00, v & 0xFF, v >> 8]
    bus.write_i2c_block_data(PCA9685_ADDR, reg, d, force=True)


def _moteur(cote, vitesse):
    if cote == 'D' and vitesse != 0:   # D motor = physically right (faster) wheel
        vitesse *= SPEED_TRIM_D
    pwm = int(abs(vitesse) / 100 * 4095)
    ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
    if vitesse > 0:
        _canal(ch_a, 4095); _canal(ch_b, 0)
    elif vitesse < 0:
        _canal(ch_a, 0); _canal(ch_b, 4095)
    else:
        _canal(ch_a, 0); _canal(ch_b, 0); pwm = 0
    _canal(ch_en, pwm)


def _stop():
    if bus:
        _moteur('G', 0)
        _moteur('D', 0)


def _cleanup(sig=None, frame=None):
    global _done
    if _done:
        return
    _done = True
    _stop()
    enc_G.stop()
    enc_D.stop()
    if bus:
        bus.close()
    if sig is not None:
        print("\nArrêt d'urgence.")
        sys.exit(0)


signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

# ── Programme principal ───────────────────────────────────────────────────────
try:
    n = _find_i2c()
    if n is None:
        print("ERREUR : PCA9685 introuvable (I²C 0-9) — vérifier alimentation 5 V pont-H.")
        sys.exit(1)
    bus = smbus2.SMBus(n)
    _pca_init()
    print(f"PCA9685 OK (bus I²C {n})\n")

    print(f"Cible    : {TARGET_MM:.0f} mm  ({PUL_TARGET} pulses, PPR={PPR_EFFECTIF})")
    print(f"Vitesse  : G={SPEED_FWD}%  D={int(SPEED_FWD * SPEED_TRIM_D)}%  (SPEED_TRIM_D={SPEED_TRIM_D} applied to D/right)")
    print("\nMarquez la position de départ sur le sol.")
    input("Dégagez 60 cm devant le robot, puis Entrée pour démarrer...")

    enc_G.reset_abs()
    enc_D.reset_abs()
    t0 = time.monotonic()

    _moteur('G', SPEED_FWD)
    _moteur('D', SPEED_FWD)

    while True:
        pg  = enc_G.get_abs()
        pd  = enc_D.get_abs()
        avg = (pg + pd) // 2
        t   = time.monotonic() - t0
        print(f"\r  G:{pg:5d}  D:{pd:5d}  moy:{avg:5d}/{PUL_TARGET}  {t:.1f}s",
              end='', flush=True)
        if avg >= PUL_TARGET:
            break
        if t >= MAX_SECS:
            print("\n  [failsafe : temps max atteint]")
            break
        time.sleep(0.01)

    _stop()
    elapsed = time.monotonic() - t0
    pg = enc_G.get_abs()
    pd = enc_D.get_abs()

    # ── Rapport ──────────────────────────────────────────────────────────────
    print(f"\n\n{'═' * 56}")
    print(f"RÉSULTATS — {elapsed:.2f} s")
    print(f"  Roue G : {pg:5d} pulses  (~{pg * MM_PER_PULSE:.0f} mm)")
    print(f"  Roue D : {pd:5d} pulses  (~{pd * MM_PER_PULSE:.0f} mm)")
    diff_pct = (pg - pd) / max(pd, 1) * 100
    print(f"  Écart  : {pg - pd:+d} pulses ({diff_pct:+.1f} %)")

    if pg < 50 and pd < 50:
        print("\nERREUR  Quasi-aucun pulse — moteurs non alimentés ou encodeurs débranchés.")
    else:
        # ── Équilibre ─────────────────────────────────────────────────────
        print(f"\n── Équilibre (SPEED_TRIM_D={SPEED_TRIM_D}) ──────────────────────")
        if abs(diff_pct) < 3.0:
            print("  OK  Roues équilibrées (écart < 3 %)")
        else:
            suggested = round(SPEED_TRIM_D * pg / pd, 3) if pd > 0 else SPEED_TRIM_D
            suggested = max(0.5, min(1.0, suggested))   # garde dans [0.5, 1.0]
            if pd > pg:
                print(f"  Roue D trop rapide → SPEED_TRIM_D : {SPEED_TRIM_D} → {suggested}")
            else:
                print(f"  Roue D trop lente  → SPEED_TRIM_D : {SPEED_TRIM_D} → {suggested}")
            print(f"  Modifier dans navigation_node.py ET test_motors.py")

        # ── Calibration PPR ───────────────────────────────────────────────
        print(f"\n── Calibration PPR ──────────────────────────────────────────")
        print(f"  PPR actuel {PPR_EFFECTIF} → distance estimée {(pg+pd)//2 * MM_PER_PULSE:.0f} mm")
        raw = input("  Mesurez la distance parcourue (mm) [Entrée pour ignorer] : ").strip()
        if raw:
            try:
                actual = float(raw)
                ppr_g   = pg * WHEEL_CIRC / actual
                ppr_d   = pd * WHEEL_CIRC / actual
                ppr_avg = (ppr_g + ppr_d) / 2
                print(f"\n  Distance mesurée : {actual:.0f} mm")
                print(f"  PPR roue G       : {ppr_g:.0f}")
                print(f"  PPR roue D       : {ppr_d:.0f}")
                print(f"  PPR moyen        : {ppr_avg:.0f}")
                if abs(ppr_avg - PPR_EFFECTIF) > PPR_EFFECTIF * 0.04:
                    ppr_int = round(ppr_avg)
                    ratio   = round(ppr_int / 2 / 13)   # PPR_MOTEUR=13
                    print(f"\n  → Mettre à jour encoder_reader.py :")
                    print(f"    PPR_MOTEUR      = 13")
                    print(f"    RATIO_REDUCTEUR = {ratio}   # {13*ratio*2} pulses/tour")
                    print(f"    (ou directement PPR_EFFECTIF = {ppr_int})")
                else:
                    print(f"\n  PPR actuel ({PPR_EFFECTIF}) correct — pas de mise à jour nécessaire.")
            except ValueError:
                print("  Valeur non reconnue, ignorée.")

finally:
    _cleanup()
