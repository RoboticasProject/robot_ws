#!/usr/bin/env python3
"""
test_sync.py — Diagnostic de synchronisation encodeurs en temps réel.

Affiche le RPM mesuré sur chaque roue toutes les 100 ms.
Un écart < 5 % indique une bonne synchronisation.

Prérequis : moteurs déjà en marche (lancer navigation_node ou motor_node).
Lancement :
  cd /home/afro-robotics/robot_ws/src/navigation/navigation
  python3 test_sync.py
"""

import time

try:
    from navigation.encoder_reader import enc_G, enc_D
except ImportError:
    from encoder_reader import enc_G, enc_D

DT = 0.1  # fenêtre de mesure 100 ms

print(f"{'Temps':>7}  {'RPM_G':>8}  {'RPM_D':>8}  {'Écart':>8}  Statut")
print("─" * 55)

t_start = time.time()

try:
    while True:
        time.sleep(DT)

        rpm_G = enc_G.get_rpm(DT)
        rpm_D = enc_D.get_rpm(DT)
        ecart = abs(rpm_G - rpm_D)
        pct   = (ecart / rpm_G * 100) if rpm_G > 0 else 0.0
        statut = "OK" if pct < 5 else "DESYNC"

        elapsed = time.time() - t_start
        print(f"{elapsed:7.1f}s  {rpm_G:8.1f}  {rpm_D:8.1f}  {ecart:7.1f}  {statut}")

except KeyboardInterrupt:
    print("\nTest terminé.")
finally:
    enc_G.stop()
    enc_D.stop()
