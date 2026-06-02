#!/usr/bin/env python3
"""
test_fuzzy_speed.py — Validates the fuzzy speed controller logic.

No ROS2, no hardware required.  Run directly:
  python3 src/navigation/navigation/test_fuzzy_speed.py
"""

# ── Copy constants and math from motor_node (no hardware import needed) ───────
SPEED_CRUISE       = 60
SPEED_MEDIUM       = 35
SPEED_SLOW         = 15
SPEED_STOP         = 0
AREA_SMALL_THRESH  = 20_000
AREA_MEDIUM_THRESH = 60_000
AREA_FRAME         = 640 * 480   # 307 200 px²


def _trap(x, a, b, c, d):
    if x <= a or x >= d:
        return 0.0
    if x <= b:
        return (x - a) / (b - a)
    if x <= c:
        return 1.0
    return (d - x) / (d - c)


def _tri(x, a, b, c):
    return _trap(x, a, b, b, c)


def fuzzy_speed(conf: float, area: float) -> float:
    low_c  = _trap(conf, 0.0, 0.0, 0.50, 0.70)
    med_c  = _tri( conf, 0.50, 0.70, 0.90)
    high_c = _trap(conf, 0.70, 0.90, 1.0, 1.0)

    small_a = _trap(area, 0, 0, AREA_SMALL_THRESH, AREA_MEDIUM_THRESH)
    med_a   = _tri( area, AREA_SMALL_THRESH, AREA_MEDIUM_THRESH, AREA_FRAME // 2)
    large_a = _trap(area, AREA_MEDIUM_THRESH, AREA_FRAME // 2, AREA_FRAME, AREA_FRAME)

    w_stop   = min(high_c, large_a)
    w_slow   = max(min(high_c, med_a),   min(med_c, large_a))
    w_medium = max(min(high_c, small_a), min(med_c, med_a))
    w_fast   = max(min(med_c, small_a),  low_c)

    total = w_stop + w_slow + w_medium + w_fast
    if total < 1e-6:
        return float(SPEED_CRUISE)

    return (w_stop * SPEED_STOP + w_slow * SPEED_SLOW +
            w_medium * SPEED_MEDIUM + w_fast * SPEED_CRUISE) / total


# ── Test grid ─────────────────────────────────────────────────────────────────

CONF_CASES = [
    (0.40, "Low  (0.40)"),
    (0.62, "Low→Med (0.62)"),
    (0.72, "Med  (0.72)"),
    (0.85, "High (0.85)"),
    (0.98, "High (0.98)"),
]

# (area px², label, approx distance)
AREA_CASES = [
    (2_000,   "  2k px²  (very far  ~2m)"),
    (10_000,  " 10k px²  (far       ~1m)"),
    (20_000,  " 20k px²  (small→med ~0.8m)"),
    (40_000,  " 40k px²  (medium    ~0.5m)"),
    (60_000,  " 60k px²  (med→large ~0.4m)"),
    (100_000, "100k px²  (large     ~0.3m)"),
    (200_000, "200k px²  (very close~0.1m)"),
]


def speed_bar(s):
    filled = int(s / SPEED_CRUISE * 20)
    return "█" * filled + "░" * (20 - filled)


print("\n" + "═" * 72)
print("  FUZZY SPEED CONTROLLER — validation table")
print("  Columns: confidence level    Rows: bounding box area (distance proxy)")
print("═" * 72)

header = f"{'Aire / Distance':<26}" + "".join(f"{c[1]:>12}" for c in CONF_CASES)
print(header)
print("─" * 72)

for area, area_label in AREA_CASES:
    row = f"{area_label:<26}"
    for conf, _ in CONF_CASES:
        s = fuzzy_speed(conf, area)
        row += f"  {s:5.1f} %   "
    print(row)

print("─" * 72)

print("\n── Scénarios clés ───────────────────────────────────────────────────")
scenarios = [
    (0.40, 5_000,   "Faible confiance, objet loin   → croisière attendu"),
    (0.72, 40_000,  "Confiance moy, distance moy    → ralentissement attendu"),
    (0.85, 80_000,  "Haute confiance, objet proche  → lent attendu"),
    (0.95, 150_000, "Très haute conf, très proche   → arrêt attendu"),
    (0.95, 5_000,   "Haute conf mais objet minuscule→ vitesse moy attendue"),
]

all_ok = True
for conf, area, desc in scenarios:
    s = fuzzy_speed(conf, area)
    bar = speed_bar(s)
    print(f"  conf={conf:.2f}  aire={area:>7d}  → {s:5.1f}%  [{bar}]  {desc}")

print("\n── Courbe de décélération (conf=0.90, objet qui s'approche) ─────────")
print(f"  {'Distance proxy':<20}  {'Aire':>8}  {'Vitesse':>8}  Barre")
for area in [3_000, 10_000, 20_000, 35_000, 60_000, 100_000, 180_000]:
    s = fuzzy_speed(0.90, area)
    bar = speed_bar(s)
    label = f"~{area//1000}k px²"
    print(f"  {label:<20}  {area:>8d}  {s:>7.1f}%  [{bar}]")

print()
