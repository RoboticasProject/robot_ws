# Autonomous Waste-Sorting Robot — Robot Workspace

Autonomous differential-drive robot built on NVIDIA Jetson Orin Nano.  
Detects and classifies waste (5 classes) using YOLOv8, navigates a 2×2 m arena,
and delivers trash to the correct bin.

---

## Hardware

| Component | Details |
|---|---|
| Computer | NVIDIA Jetson Orin Nano |
| Motors | DFRobot FIT0277 — 12V, 146 RPM max, 51:1 gearbox |
| Motor driver | L298N |
| PWM controller | PCA9685 (I²C address 0x40, bus 7) |
| Encoders | Two-phase Hall — 13 PPR motor × 51 = 663 PPR wheel (effective: 633) |
| Camera | USB camera — /dev/video0, 640×480 @ 30 fps |
| Wheel diameter | 65 mm |
| Wheelbase | 300 mm |
| Arena | 2×2 m |

### Motor channel assignments (PCA9685)

| Side | Channels (EN, A, B) | Physical wheel |
|---|---|---|
| G (Left) | 3, 5, 4 | Left wheel |
| D (Right) | 6, 7, 8 | Right wheel — physically faster, trimmed with SPEED_TRIM_D |

### Encoder GPIO lines (gpiochip0)

| Signal | Line | Board pin |
|---|---|---|
| ENC_G_A | 112 | BOARD 11 |
| ENC_G_B | 122 | BOARD 13 |
| ENC_D_A | 85 | BOARD 15 |
| ENC_D_B | 126 | BOARD 16 |

---

## Software Architecture

```
src/
├── image_acquisition/
│   ├── camera_node.py          — captures USB camera frames → /camera/image_raw
│   └── yolo_detection_node.py  — YOLOv8 inference → /detections + /best_detection
│
└── navigation/
    ├── motor_node.py           — fuzzy speed control (reacts to /detections)
    ├── navigation_node.py      — full serpentine navigation + bin trips
    ├── encoder_reader.py       — Hall encoder reader via libgpiod
    ├── test_sync.py            — wheel sync validation script
    ├── test_fuzzy_speed.py     — fuzzy logic unit test (no hardware needed)
    └── launch/
        └── robot.launch.py     — single launch: camera + YOLO + motor_node
```

### ROS2 Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node | yolo_detection_node |
| `/detections` | `vision_msgs/Detection2DArray` | yolo_detection_node | motor_node, navigation_node |
| `/best_detection` | `vision_msgs/Detection2D` | yolo_detection_node | — |
| `/camera/image_annotated` | `sensor_msgs/Image` | yolo_detection_node | — |

---

## Building

```bash
cd ~/robot_ws
colcon build --packages-select navigation
source install/setup.bash
```

Build everything:
```bash
colcon build
source install/setup.bash
```

> Always `source install/setup.bash` after building before running anything.

---

## 1 — Wheel Synchronization

The right wheel (D) is physically faster than the left (G). A fuzzy corrector
runs every 50 ms and adjusts D's PWM to keep both wheels synchronized.

### How it works

- Measures per-window velocity error (`get_abs()` delta) + cumulative position drift
- Applies a signed PWM correction to D only — G is never touched
- Stall detection: stops correcting if neither wheel moves (prevents runaway)

### Key parameters (`navigation_node.py` and `test_sync.py`)

| Parameter | Value | Meaning |
|---|---|---|
| `SPEED_TRIM_D` | 0.861 | Base trim for right wheel (hardware bias) |
| `SYNC_INTERVAL` | 50 ms | How often the corrector fires |
| `SYNC_STEP_MAX` | 32 PWM | Max correction per step |
| `SYNC_DRIFT_MAX` | 409 PWM (10%) | Max total PWM offset from base |
| `SYNC_ZERO_THRESH` | 1 pulse | Noise floor — no correction below this |
| `SYNC_SMALL_THRESH` | 4 pulses | Boundary between small and large correction |
| `SYNC_POS_WEIGHT` | 30 | How much cumulative drift contributes |

### Run the wheel sync validation test

```bash
cd ~/robot_ws
python3 src/navigation/navigation/test_sync.py
```

**What you see:**

```
 Temps  G (pulses)  D (pulses)     Écart       %   D_PWM  Corr.
    1s        1133        1147       -14   -1.2%    1161   ✓
    2s        2455        2462        -7   -0.2%    1130   ✓
    ...
VALIDÉ   Écart final < 2% et moyenne < 3% — ligne droite OK.
```

**Verdict meanings:**

| Symbol | Meaning |
|---|---|
| `✓` | Gap < 2% — good |
| `~` | Gap 2–4% — acceptable |
| `!` | Gap > 4% — needs correction |

**After the run**, check "Calibration suggérée":
- If suggested `SPEED_TRIM_D` differs from current by more than 0.02, average
  2–3 runs and update in both `test_sync.py` and `navigation_node.py`.

---

## 2 — Fuzzy Detection Speed Control

Replaces the old binary stop/go with a smooth fuzzy controller.

### How it works

**Inputs** (from `/detections`):
- YOLO confidence score (0.0 – 1.0)
- Bounding box area in pixels² (proxy for distance)

**Output:** motor speed 0 – 25% (cruise)

### Fuzzy rules

| Confidence | Box size | → Speed |
|---|---|---|
| High (> 0.7) | Large (> 60k px²) — close | **Stop 0%** |
| High | Medium (20k–60k px²) | **Slow 15%** |
| High | Small (< 20k px²) — far | **Medium 35%** (capped at cruise) |
| Medium (0.5–0.9) | Large | **Slow 15%** |
| Medium | Medium | **Medium 35%** (capped at cruise) |
| Medium | Small | **Cruise 25%** |
| Low (< 0.7) | Any | **Cruise 25%** |

> Output is a weighted average — speed changes smoothly, not in steps.

### Key parameters (`motor_node.py`)

| Parameter | Value | Meaning |
|---|---|---|
| `SPEED_CRUISE` | 25% | Normal forward speed |
| `SPEED_MEDIUM` | 35% | Cautious approach (capped to cruise if lower) |
| `SPEED_SLOW` | 15% | Near stall — object close |
| `SPEED_STOP` | 0% | Full stop |
| `AREA_SMALL_THRESH` | 20 000 px² | Far boundary (~1 m) |
| `AREA_MEDIUM_THRESH` | 60 000 px² | Close boundary (~0.4 m) |

### Test fuzzy logic only (no hardware, no ROS2)

```bash
python3 src/navigation/navigation/test_fuzzy_speed.py
```

Prints a table of speed outputs for all confidence × distance combinations,
and a deceleration curve as the robot approaches an object.

---

## 3 — Running the Full System

### Single command — starts everything

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch navigation robot.launch.py
```

This starts in order:
1. `camera_node` — opens /dev/video0
2. `yolo_detection_node` — loads best.engine, runs inference
3. `motor_node` — fuzzy speed control, listens to /detections

### Launch arguments

```bash
# Change cruise speed
ros2 launch navigation robot.launch.py speed:=30

# Lower confidence threshold (react to less certain detections)
ros2 launch navigation robot.launch.py confidence_threshold:=0.5

# Use CPU instead of GPU (fallback if CUDA issue)
ros2 launch navigation robot.launch.py device:=cpu

# Different camera device
ros2 launch navigation robot.launch.py camera_device:=/dev/video1

# Combine multiple arguments
ros2 launch navigation robot.launch.py speed:=25 confidence_threshold:=0.55
```

### Watch the fuzzy log live

```bash
# In a second terminal
source ~/robot_ws/install/setup.bash
ros2 topic echo /detections --no-arr
```

You will see lines like:
```
[FUZZY] Plastic  conf=0.87  aire=45231 px²  →  vitesse=18.3 %
```

### Live annotated video stream (MJPEG)

Open in any browser:
```
http://10.12.44.113:8080
```

---

## 4 — Tuning Guide

### Robot drifts left or right (straight line)

Run `test_sync.py` 2–3 times and average the suggested `SPEED_TRIM_D`.
Update in both `test_sync.py` and `navigation_node.py`.

```python
# navigation_node.py and test_sync.py
SPEED_TRIM_D = 0.861   # ← update this value
```

Then rebuild:
```bash
colcon build --packages-select navigation && source install/setup.bash
```

### Robot stops too early (trash still far away)

Increase `AREA_MEDIUM_THRESH` in `motor_node.py`:
```python
AREA_MEDIUM_THRESH = 80_000   # was 60_000 — robot now stops closer
```

### Robot gets too close before slowing

Decrease `AREA_SMALL_THRESH`:
```python
AREA_SMALL_THRESH = 10_000   # was 20_000 — starts slowing earlier
```

### Robot reacts to false detections

Increase confidence threshold:
```bash
ros2 launch navigation robot.launch.py confidence_threshold:=0.7
```

Or edit the default in `motor_node.py` / launch file.

### Motors don't move (stall)

Minimum safe speed is ~20% under load. Do not go below `speed:=20`.

---

## 5 — Git History

| Commit | Description |
|---|---|
| `9f30fbb` | Initial commit |
| `536360a` | Encoder reader + libgpiod wheel sync baseline |
| `83ab677` | Fuzzy wheel sync calibration — validated on ground |
| `b4d0f08` | Fuzzy detection speed control — 1st version |
| `835180b` | Cruise speed set to 25% for 2×2 m arena |
