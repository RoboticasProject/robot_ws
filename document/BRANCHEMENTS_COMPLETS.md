# 🤖 ROBOT MOBILE — Liste complète des branchements

---

## 📋 PHASE ÉCRITURE (Jetson → Moteurs)

### 1. Jetson Nano → PCA9685 (I²C)

| Jetson Nano | PCA9685 | Signal | Fonction |
|-----------|---------|--------|----------|
| Pin 3 (SDA) | SDA | I²C DATA | Communication données |
| Pin 5 (SCL) | SCL | I²C CLOCK | Horloge synchronisation |
| GND | GND | MASSE | Référence 0V |
| +5V (pin 2/4) | VCC | +5V | Alimentation logique |

**Protocole**: I²C @ 100-400 kHz, Adresse: 0x40

---

### 2. PCA9685 → L298N (PWM)

| PCA9685 | L298N | Canal | Fonction |
|---------|-------|-------|----------|
| OUT0 | ENA | Ch.0 | **Vitesse moteur GAUCHE** (PWM 0-4095) |
| OUT1 | IN1 | Ch.1 | Direction GAUCHE bit A |
| OUT2 | IN2 | Ch.2 | Direction GAUCHE bit B |
| OUT3 | ENB | Ch.3 | **Vitesse moteur DROIT** (PWM 0-4095) |
| OUT4 | IN3 | Ch.4 | Direction DROIT bit A |
| OUT5 | IN4 | Ch.5 | Direction DROIT bit B |
| GND | GND | - | MASSE commune |

**Assignation L298N** :
- ENA/IN1/IN2 → Moteur gauche
- ENB/IN3/IN4 → Moteur droit

---

### 3. L298N → Moteurs DC 12V

| L298N | Moteur | Tension | Fonction |
|-------|--------|---------|----------|
| OUT1 | M_Gauche (+) | 12V | Rotation avant/arrière |
| OUT2 | M_Gauche (-) | GND | Retour masse |
| OUT3 | M_Droit (+) | 12V | Rotation avant/arrière |
| OUT4 | M_Droit (-) | GND | Retour masse |
| +12V (VSS) | - | 12V batterie | Alimentation moteurs |
| GND | - | GND batterie | Masse batterie |

---

### 4. Alimentation externe

| Source | Destination | Tension | Courant |
|--------|------------|---------|---------|
| **Batterie 12V** | L298N VSS | 12V | 2-4A (dépend moteurs) |
| **Batterie 12V → Régulateur** | Régulateur IN | 12V | 0.5A |
| **Régulateur OUT** | PCA9685 VCC + Jetson 5V | 5V | 0.5A |
| **Jetson 5V** | PCA9685 VCC | 5V | 0.2A |

---

## 📖 PHASE LECTURE (Moteurs → Jetson)

### 1. Moteurs → Encodeurs (Couplage mécanique)

| Moteur | Encodeur | Signal | Fonction |
|--------|----------|--------|----------|
| Axe rotation G | Enc. Gauche | Rotation mécanique | 663 pulses/tour |
| Axe rotation D | Enc. Droit | Rotation mécanique | 663 pulses/tour |

---

### 2. Encodeurs → Module Encodeur (Connecteur)

| Encodeur Gauche | Module Enc. G | Signal | Tension |
|-----------------|---------------|--------|---------|
| Phase A | OUT_A | Impulsion | 5V TTL |
| Phase B | OUT_B | Quadrature | 5V TTL |
| +5V | +5V | Alimentation | 5V |
| GND | GND | Masse | 0V |

| Encodeur Droit | Module Enc. D | Signal | Tension |
|-----------------|---------------|--------|---------|
| Phase A | OUT_A | Impulsion | 5V TTL |
| Phase B | OUT_B | Quadrature | 5V TTL |
| +5V | +5V | Alimentation | 5V |
| GND | GND | Masse | 0V |

---

### 3. Module Encodeur → Diviseur de tension (5V → 3.3V)

#### **Diviseur Gauche** (10kΩ/20kΩ)

| Entrée (5V) | Diviseur | Sortie (3.3V) | Fonction |
|------------|----------|---------------|----------|
| Enc. G Phase A | R1=10kΩ, R2=20kΩ | Div_G_A | Conversion 5V → 3.3V |
| Enc. G Phase B | R1=10kΩ, R2=20kΩ | Div_G_B | Conversion 5V → 3.3V |
| +5V | Alimentation | +5V | Source tension |
| GND | GND | GND | Masse |

**Formule diviseur** : Vout = Vin × R2/(R1+R2) = 5V × 20/(10+20) = **3.33V** ✓

#### **Diviseur Droit** (10kΩ/20kΩ)

| Entrée (5V) | Diviseur | Sortie (3.3V) | Fonction |
|------------|----------|---------------|----------|
| Enc. D Phase A | R1=10kΩ, R2=20kΩ | Div_D_A | Conversion 5V → 3.3V |
| Enc. D Phase B | R1=10kΩ, R2=20kΩ | Div_D_B | Conversion 5V → 3.3V |
| +5V | Alimentation | +5V | Source tension |
| GND | GND | GND | Masse |

---

### 4. Diviseur → Jetson GPIO (Lecture)

| Diviseur | Jetson GPIO | Pin | Fonction |
|----------|-------------|-----|----------|
| Div_G_A | GPIO input | Pin 11 | Enc. Gauche Phase A (interrupt RISING) |
| Div_G_B | GPIO input | Pin 13 | Enc. Gauche Phase B (lecture) |
| Div_D_A | GPIO input | Pin 15 | Enc. Droit Phase A (interrupt RISING) |
| Div_D_B | GPIO input | Pin 16 | Enc. Droit Phase B (lecture) |
| GND | GND | - | MASSE commune |

---

## 🔴 PROBLÈME IDENTIFIÉ : Roue droite désynchronisée

### **Symptômes observés** :
1. ✅ Roue gauche tourne régulièrement
2. ❌ Roue droite se bloque périodiquement
3. ❌ Après déblocage manuel → fonctionne
4. ❌ Robot dévie à droite (moteur droit plus lent)

### **Hypothèses principales** :

#### **Hypothèse 1 : Encodeur droit bloqué mécaniquement** (PROBABLE)
- Disque encodeur coincé, capteurs mal alignés
- **Solution** : Vérifier mécanique, aligner capteurs

#### **Hypothèse 2 : Signal encodeur droit trop faible** (PROBABLE)
- Diviseur de tension mal calibré pour droit
- Capacité PCB trop élevée → atténuation signal
- **Vérifier** : 
  ```bash
  # Tension à la sortie du diviseur droit
  sudo cat /sys/class/gpio/gpio{15,16}/value
  ```

#### **Hypothèse 3 : Perte d'impulsions = détection manquée** (TRÈS PROBABLE ❗)
- Jetson trop lent pour capturer TOUS les fronts montants
- Fréquence interruptions trop haute (100+ kHz possible à haute vitesse)
- **Cause** : À 100 RPM, encodeur génère ~1100 impulsions/sec
  - Si Jetson perd 5% → compte faux → roue droite semble plus lente
  - Avec déblocage manuel → compteur réinitialise, synchrone reprend

#### **Hypothèse 4 : Différence d'alimentation L298N** (POSSIBLE)
- Moteur droit reçoit moins de courant
- **Vérifier** : Contrôler tensions OUT3/OUT4 vs OUT1/OUT2 du L298N

---

## 🛠️ DIAGNOSTICS À EFFECTUER

### **Test 1 : Vérifier les signaux encodeurs bruts (avant diviseur)**

```bash
# Brancher oscilloscope sur sorties Module Encodeur (5V)
# Vérifier Phase A et B pour gauche ET droit

# Gauche : devrait voir ~1100 Hz à 100 RPM
# Droit : devrait voir ~1100 Hz à 100 RPM aussi

# Si droit est plus faible → problème diviseur ou encodeur
```

### **Test 2 : Vérifier tensions diviseur**

```bash
# Multimètre en continu sur sorties diviseur
# Laisser moteurs tourner

# Div_G_A/B : devrait osciller 0-3.3V
# Div_D_A/B : devrait osciller 0-3.3V

# Si Div_D oscillations < 1.5V ou > 4V → diviseur mauvais
```

### **Test 3 : Vérifier réception GPIO Jetson**

```python
# Dans le code, ajouter loggeur pour chaque callback

import time

pulse_count_G = 0
pulse_count_D = 0
last_log_G = time.time()
last_log_D = time.time()

def callback_gauche(channel):
    global pulse_count_G, last_log_G
    pulse_count_G += 1
    now = time.time()
    if now - last_log_G > 1.0:  # Log chaque seconde
        print(f"[G] {pulse_count_G} pulses/sec")
        pulse_count_G = 0
        last_log_G = now

def callback_droit(channel):
    global pulse_count_D, last_log_D
    pulse_count_D += 1
    now = time.time()
    if now - last_log_D > 1.0:
        print(f"[D] {pulse_count_D} pulses/sec")
        pulse_count_D = 0
        last_log_D = now

# Résultat attendu à vitesse 60% (moyenne) :
# [G] 600 pulses/sec
# [D] 600 pulses/sec
#
# Si [D] << [G] ou absent → perte d'impulsions
```

### **Test 4 : Vérifier alimentation L298N**

```bash
# Multimètre pendant que moteurs tournent
# Mesurer tension entre OUT1-OUT2 (gauche)
# Mesurer tension entre OUT3-OUT4 (droit)

# Vérifier courant L298N avec pince ampèremétrique
# Si courant droit << gauche → moteur bloqué ou sous-alimenté
```

---

## 💡 SOLUTIONS PROPOSÉES

### **Solution immédiate 1 : Anti-débounce dans le code**

```python
# Ajouter délai minimal entre pulses détectées

last_pulse_time_G = 0
last_pulse_time_D = 0
MIN_PULSE_INTERVAL = 0.0005  # 500 microsecondes

def callback_gauche(channel):
    global last_pulse_time_G
    now = time.time()
    if now - last_pulse_time_G < MIN_PULSE_INTERVAL:
        return  # Ignorer pulse parasite
    last_pulse_time_G = now
    
    # ... reste du code ...
```

### **Solution 2 : Augmenter la réactivité GPIO**

```python
# Utiliser edge_detect plus agressif

# AVANT
GPIO.add_event_detect(ENC_D_A, GPIO.RISING, callback=callback_droit)

# APRÈS : ajout de bounce=0 pour éviter débounce logiciel
GPIO.add_event_detect(ENC_D_A, GPIO.RISING, callback=callback_droit, bouncetime=0)
```

### **Solution 3 : Réduire la fréquence PWM**

```python
# Si fréquence PWM trop élevée → encodeur surchargé

# Réduire de 1000 Hz à 500 Hz
pca.init(freq=500)  # au lieu de 1000

# Cela donne plus de temps à Jetson pour traiter interruptions
```

### **Solution 4 : Corriger manuellement le déséquilibre**

```python
# Ajouter correction de vitesse basée sur encodeurs

def avancer_equilibre(vitesse_cible):
    vitesse_G = vitesse_cible
    vitesse_D = vitesse_cible
    
    # Tous les 100ms, mesurer et corriger
    rpm_G_actual = get_vitesse_rpm('G')
    rpm_D_actual = get_vitesse_rpm('D')
    
    # Si droit plus lent, boost légèrement
    if rpm_D_actual < rpm_G_actual * 0.95:  # Plus de 5% plus lent
        vitesse_D = min(100, vitesse_D + 2)  # Augmenter de 2%
    
    moteur('G', vitesse_G)
    moteur('D', vitesse_D)
```

### **Solution 5 : Vérifier le câblage diviseur**

```
MAUVAIS BRANCHEMENT (haute capacité parasite):
┌─────────────────────────────────┐
│ Enc.D Phase A (5V)  ────┬────── Jetson Pin 15
│                        R1 (10kΩ)
│                        │
│                      ┌─┴─┐
│                      │   │ R2 (20kΩ)
│                      │   │
│                      └─┬─┘
│                        ├──── Jetson Pin 15
│                        │
│                       GND

PROBLÈME: Trop de capacité parasite sur câble long
→ Signal atténué, temps montée lent → pulses perdues

SOLUTION:
- Réduire longueur câble Phase A/B
- Ajouter condensateur de découplage 100nF près Jetson
- Utiliser câble blindé si câble long (>50cm)
```

---

## 📊 TABLEAU DE COMPARAISON

| Aspect | Gauche | Droit | Problème ? |
|--------|--------|-------|-----------|
| Branchement moteur | L298N OUT1/OUT2 | L298N OUT3/OUT4 | Vérifier |
| PWM (Ch0 vs Ch3) | PCA9685 Ch.0 | PCA9685 Ch.3 | Tester indépendance |
| Encodeur | Module enc G | Module enc D | **À diagnostiquer** |
| Diviseur | 10kΩ/20kΩ | 10kΩ/20kΩ | Mesurer tensions |
| GPIO Jetson | Pin 11, 13 | Pin 15, 16 | Vérifier réception |
| Vitesse @ 60% | ~100 RPM | ~95 RPM (plus lent) | **OUI** |

---

## 🎯 ORDRE DE DIAGNOSTIC RECOMMANDÉ

1. ✅ **D'abord** : Tester mécaniquement
   - Tourner roues à la main
   - Vérifier encodeur droit ne bloque pas
   - Vérifier alignement capteurs

2. ✅ **Ensuite** : Mesurer signaux
   - Oscilloscope sorties encodeurs
   - Multimètre tensions diviseurs
   - Vérifier GPIO reçoit bien pulses

3. ✅ **Puis** : Tester électronique
   - Vérifier L298N alimente bien moteur D
   - Tester PCA9685 produit PWM différent

4. ✅ **Finalement** : Corriger logiciel
   - Implémenter correction dynamique
   - Réduire fréquence si nécessaire

