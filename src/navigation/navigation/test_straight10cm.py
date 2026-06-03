#!/usr/bin/env python3
"""
test_straight10cm.py — Go straight exactly 10 cm then stop.
Reports actual pulses and distance for each wheel.
Run:  python3 src/navigation/navigation/test_straight10cm.py
"""
import math, signal, sys, time, smbus2

try:
    from navigation.encoder_reader import enc_G, enc_D, PPR_EFFECTIF
except ImportError:
    from encoder_reader import enc_G, enc_D, PPR_EFFECTIF

# ── constants ──────────────────────────────────────────────────────────────────
WHEEL_DIAM_MM = 65.0
MM_PER_PULSE  = math.pi * WHEEL_DIAM_MM / PPR_EFFECTIF

TARGET_MM     = 100.0
PUL_TARGET    = 289   # calibrated: 722 pulses = 25 cm measured → 289 for 10 cm

SPEED_FWD     = 35
SPEED_TRIM_D  = 0.842

# fuzzy sync (same as test_sync.py)
SYNC_ZERO_THRESH  = 1
SYNC_SMALL_THRESH = 4
SYNC_STEP_MAX     = int(4095 * 0.008)
SYNC_DRIFT_MAX    = int(4095 * 0.10)
SYNC_INTERVAL     = 0.05
SYNC_POS_WEIGHT   = 30

PCA9685_ADDR = 0x40
CANAUX_G     = (3, 5, 4)
CANAUX_D     = (6, 7, 8)

bus         = None
_done       = False
_base_pwm_d = 0
_cur_pwm_d  = 0

# ── PCA9685 ────────────────────────────────────────────────────────────────────
def _find_i2c():
    for n in range(10):
        try:
            b = smbus2.SMBus(n); b.read_byte(PCA9685_ADDR); b.close(); return n
        except (FileNotFoundError, PermissionError): pass
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
    if v >= 4095:   d = [0x00, 0x10, 0x00, 0x00]
    elif v <= 0:    d = [0x00, 0x00, 0x00, 0x10]
    else:           d = [0x00, 0x00, v & 0xFF, v >> 8]
    bus.write_i2c_block_data(PCA9685_ADDR, reg, d, force=True)

def _moteur(cote, vitesse):
    global _base_pwm_d
    if cote == 'D' and vitesse != 0:
        vitesse *= SPEED_TRIM_D
    pwm = int(abs(vitesse) / 100 * 4095)
    ch_en, ch_a, ch_b = CANAUX_G if cote == 'G' else CANAUX_D
    if vitesse > 0:   _canal(ch_a, 4095); _canal(ch_b, 0)
    elif vitesse < 0: _canal(ch_a, 0);    _canal(ch_b, 4095)
    else:             _canal(ch_a, 0);    _canal(ch_b, 0); pwm = 0
    _canal(ch_en, pwm)
    if cote == 'D':
        _base_pwm_d = pwm

def _stop():
    if bus:
        _moteur('G', 0)
        _moteur('D', 0)

def _cleanup(sig=None, frame=None):
    global _done
    if _done: return
    _done = True
    _stop()
    enc_G.stop(); enc_D.stop()
    if bus: bus.close()
    if sig is not None:
        print("\nArrêt — moteurs coupés.")
        sys.exit(0)

signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

# ── fuzzy sync ─────────────────────────────────────────────────────────────────
def _fuzzy_sync_delta(diff):
    abs_diff = abs(diff)
    if abs_diff <= SYNC_ZERO_THRESH: return 0
    sign = 1 if diff > 0 else -1
    if abs_diff <= SYNC_SMALL_THRESH:
        t = (abs_diff - SYNC_ZERO_THRESH) / (SYNC_SMALL_THRESH - SYNC_ZERO_THRESH)
        return sign * int(t * SYNC_STEP_MAX * 0.5)
    t = min(1.0, (abs_diff - SYNC_SMALL_THRESH) / SYNC_SMALL_THRESH)
    return sign * int(SYNC_STEP_MAX * 0.5 + t * SYNC_STEP_MAX * 0.5)

def _apply_sync(ref_g, ref_d):
    global _cur_pwm_d
    cur_g = enc_G.get_abs(); cur_d = enc_D.get_abs()
    if cur_g == ref_g and cur_d == ref_d:
        return cur_g, cur_d
    vel_diff = (cur_g - ref_g) - (cur_d - ref_d)
    pos_diff = cur_g - cur_d
    delta    = _fuzzy_sync_delta(vel_diff + pos_diff // SYNC_POS_WEIGHT)
    if delta != 0:
        new_d      = _cur_pwm_d + delta
        new_d      = max(_base_pwm_d - SYNC_DRIFT_MAX, min(_base_pwm_d + SYNC_DRIFT_MAX, new_d))
        _cur_pwm_d = max(0, min(4095, new_d))
        _canal(CANAUX_D[0], _cur_pwm_d)
    return cur_g, cur_d

# ── main ───────────────────────────────────────────────────────────────────────
try:
    n = _find_i2c()
    if n is None:
        print("ERREUR : PCA9685 introuvable"); sys.exit(1)
    bus = smbus2.SMBus(n)
    _pca_init()
    print(f"PCA9685 OK (bus I²C {n})")
    print(f"Target : {PUL_TARGET} pulses → {TARGET_MM:.0f} mm")
    print()
    input("Enter pour démarrer …")

    enc_G.reset_abs(); enc_D.reset_abs()
    _moteur('G', SPEED_FWD)
    _moteur('D', SPEED_FWD)
    _cur_pwm_d = _base_pwm_d

    t_next_sync = time.monotonic() + SYNC_INTERVAL
    ref_g = enc_G.get_abs(); ref_d = enc_D.get_abs()

    while True:
        now = time.monotonic()
        if now >= t_next_sync:
            ref_g, ref_d = _apply_sync(ref_g, ref_d)
            t_next_sync = now + SYNC_INTERVAL

        pg = enc_G.get_abs(); pd = enc_D.get_abs()
        if min(pg, pd) >= PUL_TARGET:
            _stop()
            break
        time.sleep(0.002)

    dist_g = pg * MM_PER_PULSE
    dist_d = pd * MM_PER_PULSE
    print(f"\n=== RESULT ===")
    print(f"  Target : {PUL_TARGET} pulses = {TARGET_MM:.0f} mm")
    print(f"  Left   : {pg} pulses → {dist_g:.1f} mm")
    print(f"  Right  : {pd} pulses → {dist_d:.1f} mm")
    print(f"  Average: {(dist_g+dist_d)/2:.1f} mm")

finally:
    _cleanup()
