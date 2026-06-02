#!/usr/bin/env python3
"""
scan_gpio.py — Trouve les lignes GPIO des encodeurs (2 roues).

Lancement :
  python3 src/navigation/navigation/scan_gpio.py

Tournez chaque roue d'UN TOUR COMPLET quand demandé.
Lignes 0-79 ignorées (GPIO système Jetson — ne pas toucher).
"""

import gpiod
import signal
import sys
import time

CHIP_NAME = 'gpiochip0'
# Expansion header only — system lines < 80 skipped to prevent hardware lockup
LINE_MIN = 80
LINE_MAX = 163
SKIP = {114, 115, 131}   # lines held by kernel driver

chip  = None
lines = {}


def _release_all():
    for ln in list(lines.values()):
        try:
            ln.release()
        except Exception:
            pass
    lines.clear()
    if chip is not None:
        try:
            chip.close()
        except Exception:
            pass


def _sig_handler(sig, frame):
    print("\nInterruption — libération des GPIO...", flush=True)
    _release_all()
    sys.exit(0)


signal.signal(signal.SIGINT,  _sig_handler)
signal.signal(signal.SIGTERM, _sig_handler)


def _open_lines():
    global chip
    chip = gpiod.Chip(CHIP_NAME)
    count = 0
    for i in range(LINE_MIN, LINE_MAX + 1):
        if i in SKIP:
            continue
        try:
            ln = chip.get_line(i)
            ln.request(consumer='enc_scan', type=gpiod.LINE_REQ_DIR_IN)
            lines[i] = ln
            count += 1
        except Exception:
            pass
    return count


def _scan_wheel(label: str, duration: int = 8) -> dict:
    baseline = {i: ln.get_value() for i, ln in lines.items()}
    changed: dict = {}
    t_end = time.monotonic() + duration
    print(f"  Tournez la roue {label} d'UN TOUR COMPLET ({duration} s)...", flush=True)
    while time.monotonic() < t_end:
        for i, ln in lines.items():
            v = ln.get_value()
            if v != baseline[i]:
                changed.setdefault(i, []).append(
                    (round(time.monotonic(), 3), baseline[i], v)
                )
                baseline[i] = v
        time.sleep(0.0005)
    return changed


def _print_wheel(label: str, changed: dict):
    if not changed:
        print(f"  !! Aucune transition pour {label} — vérifier câblage.\n")
        return
    top = sorted(changed.items(), key=lambda x: -len(x[1]))
    print(f"  Roue {label} — lignes actives :")
    print(f"  {'Ligne':<8} {'Transitions':>12}  Détail")
    for line_no, events in top[:4]:
        print(f"  {line_no:<8} {len(events):>12}  {events[:3]}")
    print()


try:
    n = _open_lines()
    print(f"Scanner actif sur {n} lignes (range {LINE_MIN}-{LINE_MAX}).\n")

    input("Prêt pour la roue GAUCHE ? Appuyez sur Entrée puis tournez...")
    g_res = _scan_wheel('GAUCHE')
    _print_wheel('GAUCHE', g_res)

    input("Prêt pour la roue DROITE ? Appuyez sur Entrée puis tournez...")
    d_res = _scan_wheel('DROITE')
    _print_wheel('DROITE', d_res)

    print("═" * 60)
    print("RÉSUMÉ — copier dans encoder_reader.py :")
    for label, res in [('GAUCHE', g_res), ('DROITE', d_res)]:
        top2 = sorted(res.items(), key=lambda x: -len(x[1]))[:2] if res else []
        s = ', '.join(str(l) for l, _ in top2) if top2 else 'NON TROUVÉ'
        print(f"  Roue {label}: lignes {s}")

finally:
    _release_all()
