#!/usr/bin/env python3
"""
test_calibration.py — Vérification mécanique des encodeurs via libgpiod.

Tournez chaque roue d'exactement 1 tour à la main et vérifiez le compte.

Résultats attendus :
  ~1326 pulses  → encodeur OK, câblage correct (BOTH_EDGES)
  ~663 pulses   → un seul front détecté — vérifier branchement phase B
  Valeur < 100  → problème électrique ou mécanique à investiguer

Lancement :
  cd /home/afro-robotics/robot_ws/src/navigation/navigation
  python3 test_calibration.py
"""

import threading
import time

try:
    from navigation.encoder_reader import enc_G, enc_D, PPR_EFFECTIF
except ImportError:
    from encoder_reader import enc_G, enc_D, PPR_EFFECTIF

print(f"Configuration libgpiod — {PPR_EFFECTIF} pulses/tour attendus\n")


def _live(enc, stop_evt, label):
    """Affiche le compteur en temps réel jusqu'à ce que stop_evt soit levé."""
    while not stop_evt.is_set():
        print(f"\r  {label}: {enc.get_abs():5d} pulses (tournez puis Entrée)...",
              end='', flush=True)
        time.sleep(0.2)


# ── Roue gauche ──────────────────────────────────────────────────────────────
enc_G.reset_abs()
print("Roue GAUCHE — tournez d'exactement 1 tour complet, puis appuyez sur Entrée.")
_stop_g = threading.Event()
threading.Thread(target=_live, args=(enc_G, _stop_g, 'G'), daemon=True).start()
input()
_stop_g.set()
time.sleep(0.25)   # laisse le thread finir son dernier affichage
count_g = enc_G.get_abs()
ecart_g = abs(count_g - PPR_EFFECTIF)
print(f"\nRoue G : {count_g} pulses  (attendu : {PPR_EFFECTIF})  — écart : {ecart_g}")

print()

# ── Roue droite ──────────────────────────────────────────────────────────────
enc_D.reset_abs()
print("Roue DROITE — tournez d'exactement 1 tour complet, puis appuyez sur Entrée.")
_stop_d = threading.Event()
threading.Thread(target=_live, args=(enc_D, _stop_d, 'D'), daemon=True).start()
input()
_stop_d.set()
time.sleep(0.25)
count_d = enc_D.get_abs()
ecart_d = abs(count_d - PPR_EFFECTIF)
print(f"\nRoue D : {count_d} pulses  (attendu : {PPR_EFFECTIF})  — écart : {ecart_d}")

print()
print("─" * 55)

# ── Diagnostic ───────────────────────────────────────────────────────────────
half      = PPR_EFFECTIF // 2          # 663 — phase B manquante
tol_ok    = int(PPR_EFFECTIF * 0.20)  # ±20 % — imprecision tour manuel acceptée
tol_half  = int(PPR_EFFECTIF * 0.08)  # ±8 %  — autour de 663

near_half_g = abs(count_g - half) < tol_half
near_half_d = abs(count_d - half) < tol_half
ok_g        = ecart_g < tol_ok and not near_half_g
ok_d        = ecart_d < tol_ok and not near_half_d

if ok_g and ok_d:
    print("OK  Les deux encodeurs sont opérationnels.")
    print(f"    Ecarts G={ecart_g}, D={ecart_d} pulses — normale pour un tour manuel.")
elif near_half_g:
    print("ATTENTION  Roue G compte ~663 au lieu de ~1326 — phase B non détectée.")
    print("    Vérifier BOARD pin 13 → gpiochip0 line 122.")
elif near_half_d:
    print("ATTENTION  Roue D compte ~663 au lieu de ~1326 — phase B non détectée.")
    print("    Vérifier BOARD pin 16 → gpiochip0 line 126.")
elif count_g < 50 or count_d < 50:
    print("ERREUR  Quasi-aucune impulsion — vérifier Vcc encodeur et câblage.")
else:
    ppr_moy = (count_g + count_d) // 2
    print(f"INFO  Comptes hors tolérance ±20 % : G={count_g}, D={count_d}.")
    print(f"    Si vous avez tourné exactement 1 tour, PPR_EFFECTIF réel ≈ {ppr_moy}.")
    print(f"    Sinon, recommencez en marquant 1 tour exact sur la roue.")

# ── Nettoyage ────────────────────────────────────────────────────────────────
enc_G.stop()
enc_D.stop()
