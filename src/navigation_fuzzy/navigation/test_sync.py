#!/usr/bin/env python3
"""
test_sync.py — Validation synchronisation roues : ligne droite 30 secondes.

Fait avancer le robot en ligne droite pendant 30 s avec le correcteur
fuzzy actif. Affiche l'écart G/D en temps réel chaque seconde.
Un écart final < 2 % valide la synchronisation.

Prérequis :
  - Moteurs alimentés (12 V)
  - ~10 m d'espace dégagé devant le robot (ou Ctrl+C pour arrêter avant)

Lancement :
  cd ~/robot_ws
  python3 src/navigation/navigation/test_sync.py
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

# ── Paramètres — identiques à navigation_node.py ─────────────────────────────
WHEEL_DIAM_MM = 65.0
WHEEL_CIRC    = math.pi * WHEEL_DIAM_MM
MM_PER_PULSE  = WHEEL_CIRC / PPR_EFFECTIF
SPEED_FWD     = 35
SPEED_TRIM_D  = 0.861
DURATION_SECS = 10.0

# ── Fuzzy sync — identiques à navigation_node.py ─────────────────────────────
SYNC_ZERO_THRESH  = 1                  # 1 pulse/50ms noise floor  (was 2/100ms)
SYNC_SMALL_THRESH = 4                  # boundary small/large zone (was 8/100ms)
SYNC_STEP_MAX     = int(4095 * 0.008)  # 32 PWM/50ms — same correction rate as 61/100ms
SYNC_DRIFT_MAX    = int(4095 * 0.10)   # 409 PWM — tighter limit, prevents runaway (was 20%)
SYNC_INTERVAL     = 0.05               # 50 ms — 2× faster response (was 100 ms)
SYNC_POS_WEIGHT   = 30                 # position drift scaled down more to reduce windup

# ── PCA9685 ───────────────────────────────────────────────────────────────────
PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)
CANAUX_D     = (6, 7, 8)

bus         = None
_done       = False
_base_pwm_g = 0
_base_pwm_d = 0
_cur_pwm_d  = 0   # D's current corrected PWM — integral accumulator (G is never touched)


# ── PCA9685 bas niveau ────────────────────────────────────────────────────────

def _find_i2c():
    for n in range(10):
        try:
            b = smbus2.SMBus(n); b.read_byte(PCA9685_ADDR); b.close(); return n
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
    global _base_pwm_g, _base_pwm_d
    if cote == 'D' and vitesse != 0:
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
    if cote == 'G':
        _base_pwm_g = pwm
    else:
        _base_pwm_d = pwm


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
        print("\n\nArrêt d'urgence — moteurs coupés.")
        sys.exit(0)


signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)


# ── Logique fuzzy — identique à navigation_node.py ───────────────────────────

def _fuzzy_sync_delta(diff: int) -> int:
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


def _apply_sync(ref_abs_g: int, ref_abs_d: int):
    global _cur_pwm_d
    cur_abs_g = enc_G.get_abs()
    cur_abs_d = enc_D.get_abs()

    # Stall guard: if neither wheel moved this window, skip correction entirely.
    # Without this, the integral winds up to maximum when the robot stops.
    if cur_abs_g == ref_abs_g and cur_abs_d == ref_abs_d:
        return cur_abs_g, cur_abs_d, 0

    vel_diff = (cur_abs_g - ref_abs_g) - (cur_abs_d - ref_abs_d)  # speed gap this window
    pos_diff = cur_abs_g - cur_abs_d                               # total drift since start
    diff     = vel_diff + pos_diff // SYNC_POS_WEIGHT
    delta    = _fuzzy_sync_delta(diff)

    if delta != 0:
        new_d      = _cur_pwm_d + delta
        new_d      = max(_base_pwm_d - SYNC_DRIFT_MAX, min(_base_pwm_d + SYNC_DRIFT_MAX, new_d))
        new_d      = max(0, min(4095, new_d))
        _cur_pwm_d = new_d
        _canal(CANAUX_D[0], new_d)

    return cur_abs_g, cur_abs_d, delta


# ── Programme principal ───────────────────────────────────────────────────────
try:
    n = _find_i2c()
    if n is None:
        print("ERREUR : PCA9685 introuvable — vérifier alimentation.")
        sys.exit(1)
    bus = smbus2.SMBus(n)
    _pca_init()
    print(f"PCA9685 OK (bus I²C {n})\n")

    est_dist_m = SPEED_FWD / 100 * 0.35 * DURATION_SECS   # rough estimate ~3-4 m
    print(f"Durée      : {DURATION_SECS:.0f} secondes")
    print(f"Vitesse    : G={SPEED_FWD}%  D={int(SPEED_FWD*SPEED_TRIM_D)}%  (SPEED_TRIM_D={SPEED_TRIM_D})")
    print(f"Base PWM   : G={int(SPEED_FWD/100*4095)}  D={int(SPEED_FWD*SPEED_TRIM_D/100*4095)}  (sur 4095)")
    print(f"Fuzzy      : zero≤{SYNC_ZERO_THRESH}p  small≤{SYNC_SMALL_THRESH}p  step±{SYNC_STEP_MAX}PWM/100ms  drift±{SYNC_DRIFT_MAX}PWM — D only")
    print(f"\nATTENTION  : Dégagez ~10 m devant le robot (Ctrl+C pour arrêter avant).")
    print()
    input("Entrée pour démarrer...")

    enc_G.reset_abs()
    enc_D.reset_abs()

    _moteur('G', SPEED_FWD)
    _moteur('D', SPEED_FWD)

    _cur_pwm_d = _base_pwm_d   # reset D integral at start

    t0          = time.monotonic()
    t_next_sync = t0 + SYNC_INTERVAL
    t_next_log  = t0 + 1.0
    ref_abs_g   = enc_G.get_abs()
    ref_abs_d   = enc_D.get_abs()

    total_corrections = 0
    total_windows     = 0
    gap_samples       = []

    # ── En-tête tableau ───────────────────────────────────────────────────────
    print(f"\n{'Temps':>6}  {'G (pulses)':>10}  {'D (pulses)':>10}  "
          f"{'Écart':>8}  {'%':>6}  {'D_PWM':>6}  {'Corr.'}")
    print("─" * 72)

    while True:
        now = time.monotonic()
        t   = now - t0

        if now >= t_next_sync:
            ref_abs_g, ref_abs_d, delta = _apply_sync(ref_abs_g, ref_abs_d)
            total_windows += 1
            if delta != 0:
                total_corrections += 1
            t_next_sync = now + SYNC_INTERVAL

        if now >= t_next_log:
            pg       = enc_G.get_abs()
            pd       = enc_D.get_abs()
            gap      = pg - pd
            gap_pct  = gap / max(pd, 1) * 100
            gap_samples.append(abs(gap_pct))

            bar_len  = int(abs(gap_pct) / 0.5)          # 1 char per 0.5%
            bar_len  = min(bar_len, 20)
            bar      = ('>' if gap > 0 else '<') * bar_len

            status = '✓' if abs(gap_pct) < 2.0 else ('~' if abs(gap_pct) < 4.0 else '!')
            print(f"{t:5.0f}s  {pg:>10d}  {pd:>10d}  "
                  f"{gap:>+8d}  {gap_pct:>+5.1f}%  "
                  f"{_cur_pwm_d:6d}  "
                  f"{total_corrections:3d}/{total_windows:3d}  {status} {bar}")
            t_next_log = now + 1.0

        if t >= DURATION_SECS:
            break

        time.sleep(0.005)

    _stop()
    elapsed = time.monotonic() - t0

    # ── Rapport final ─────────────────────────────────────────────────────────
    pg      = enc_G.get_abs()
    pd      = enc_D.get_abs()
    gap_pct = (pg - pd) / max(pd, 1) * 100
    avg_gap = sum(gap_samples) / len(gap_samples) if gap_samples else 0

    print(f"\n{'═' * 68}")
    print(f"RÉSULTATS FINAUX — {elapsed:.1f} s")
    print(f"  Roue G  : {pg:6d} pulses  ({pg * MM_PER_PULSE / 1000:.2f} m)")
    print(f"  Roue D  : {pd:6d} pulses  ({pd * MM_PER_PULSE / 1000:.2f} m)")
    print(f"  Écart   : {pg-pd:+d} pulses  ({gap_pct:+.1f} %)")
    print(f"  Écart moyen sur le run : {avg_gap:.1f} %")

    print(f"\n── Correcteur fuzzy ─────────────────────────────────────────────")
    print(f"  Fenêtres évaluées   : {total_windows}")
    print(f"  Corrections actives : {total_corrections}  "
          f"({total_corrections/max(total_windows,1)*100:.0f} % des fenêtres)")

    print(f"\n── Verdict ──────────────────────────────────────────────────────")
    if abs(gap_pct) < 2.0 and avg_gap < 3.0:
        print("  VALIDÉ   Écart final < 2% et moyenne < 3% — ligne droite OK.")
    elif abs(gap_pct) < 4.0:
        print("  PROCHE   Écart < 4% — acceptable, affiner SPEED_TRIM_D.")
    else:
        print(f"  À REVOIR  Écart {gap_pct:+.1f}% — voir suggestions :")
        if total_corrections == total_windows:
            print(f"    → Correcteur toujours saturé : augmenter SYNC_STEP_MAX.")
        print(f"    → Ajuster SPEED_TRIM_D (actuel {SPEED_TRIM_D}).")

    # Suggest a new SPEED_TRIM_D based on where the fuzzy settled
    base_pwm_no_trim = int(SPEED_FWD / 100 * 4095)   # what D would be without trim
    suggested_trim   = _cur_pwm_d / base_pwm_no_trim
    print(f"\n── Calibration suggérée ─────────────────────────────────────────")
    print(f"  D_PWM convergé  : {_cur_pwm_d}  (base sans trim = {base_pwm_no_trim})")
    print(f"  SPEED_TRIM_D    : {SPEED_TRIM_D:.3f}  →  suggéré {suggested_trim:.3f}")
    if abs(suggested_trim - SPEED_TRIM_D) > 0.02:
        print(f"  → Mettre à jour SPEED_TRIM_D={suggested_trim:.3f} dans test_sync.py")
        print(f"     et navigation_node.py pour réduire la charge sur le correcteur fuzzy.")

finally:
    _cleanup()
