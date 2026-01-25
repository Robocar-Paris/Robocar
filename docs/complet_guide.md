# Guide Complet du Projet Robocar

## Pour qui est ce guide ?

Ce guide est écrit pour quelqu'un qui :
- Connaît les bases de la programmation (variables, fonctions, boucles)
- N'a jamais fait de robotique ou de hardware
- Veut comprendre TOUT ce qui a été fait

---

## Table des matières

1. [Le but du projet](#1-le-but-du-projet)
2. [Comment fonctionne un robot autonome](#2-comment-fonctionne-un-robot-autonome)
3. [Le hardware (matériel)](#3-le-hardware-matériel)
4. [La structure du projet](#4-la-structure-du-projet)
5. [Explication de chaque fichier](#5-explication-de-chaque-fichier)
6. [Pourquoi ça marche sur Jetson Nano](#6-pourquoi-ça-marche-sur-jetson-nano)
7. [Pourquoi c'est scalable](#7-pourquoi-cest-scalable)
8. [Les fichiers "vides" (__init__.py)](#8-les-fichiers-vides-__init__py)
9. [Comment tout s'assemble](#9-comment-tout-sassemble)

---

## 1. Le but du projet

### Objectif simple
Faire une voiture RC qui va **toute seule** d'un point A à un point B, comme les robots livreurs dans les hôtels.

### Ce que ça implique
```
Point A ──────────────────────────────────────────► Point B
         │                                      │
         │  Le robot doit :                     │
         │  1. Savoir où il est                 │
         │  2. Savoir où sont les obstacles     │
         │  3. Calculer un chemin               │
         │  4. Suivre ce chemin                 │
         │  5. Éviter les obstacles en route    │
         └──────────────────────────────────────┘
```

---

## 2. Comment fonctionne un robot autonome

### L'analogie avec un humain

Imagine que tu dois aller de ta chambre à la cuisine les yeux bandés :

| Humain | Robot | Notre solution |
|--------|-------|----------------|
| Yeux | Caméra, LiDAR | LiDAR LD19 |
| GPS interne (sens de l'orientation) | GPS | GPS RTK Point One |
| Cerveau | Ordinateur | Jetson Nano |
| Jambes | Moteurs | VESC + Servo |
| Mémoire (carte mentale) | Carte numérique | Grille d'occupation |

### Les 3 grandes questions d'un robot

```
┌─────────────────────────────────────────────────────────────┐
│                     ROBOT AUTONOME                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. OÙ SUIS-JE ?          → Localisation (GPS + SLAM)       │
│                                                              │
│  2. OÙ SONT LES DANGERS ? → Perception (LiDAR)              │
│                                                              │
│  3. OÙ DOIS-JE ALLER ?    → Navigation (Planification)      │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Le cycle de contrôle (10 fois par seconde)

```
    ┌──────────────────────────────────────────────────────┐
    │                                                       │
    ▼                                                       │
┌─────────┐    ┌─────────────┐    ┌──────────┐    ┌───────┴───────┐
│ CAPTEURS │───►│ TRAITEMENT  │───►│ DÉCISION │───►│    ACTION     │
│ (LiDAR,  │    │ (Perception,│    │ (Naviga- │    │ (Moteur,      │
│  GPS)    │    │  SLAM)      │    │  tion)   │    │  Direction)   │
└─────────┘    └─────────────┘    └──────────┘    └───────────────┘
    │                                                       │
    └───────────────── RÉPÉTER 10x/seconde ─────────────────┘
```

---

## 3. Le hardware (matériel)

### Jetson Nano 4GB - Le cerveau

```
┌─────────────────────────────────────────┐
│           JETSON NANO 4GB               │
├─────────────────────────────────────────┤
│  CPU: ARM Cortex-A57 (4 coeurs)         │
│  GPU: 128 coeurs CUDA (NVIDIA)          │
│  RAM: 4 GB                              │
│  OS: Linux Ubuntu                       │
├─────────────────────────────────────────┤
│  POURQUOI C'EST SUFFISANT ?             │
│                                          │
│  • Notre code utilise ~200-500 MB RAM   │
│  • Le CPU suffit pour 10 Hz             │
│  • Le GPU peut accélérer si besoin      │
│  • C'est ce qu'utilise F1Tenth          │
└─────────────────────────────────────────┘
```

### LiDAR LD19 - Les "yeux" du robot

```
                    360°
                     │
            ╱────────┼────────╲
          ╱          │          ╲
        ╱            │            ╲
       │      ┌──────┴──────┐      │
       │      │   LiDAR     │      │    Le LiDAR envoie un faisceau laser
  270°─┼──────│   LD19      │──────┼─90°  qui tourne à 360° et mesure
       │      │             │      │    la distance des obstacles
       │      └──────┬──────┘      │
        ╲            │            ╱    = 360 mesures de distance
          ╲          │          ╱       par tour (1 par degré)
            ╲────────┼────────╱
                     │
                    180°

Données reçues : [(0°, 2.5m), (1°, 2.5m), (2°, 1.2m), ...]
                  angle  distance
```

**Pourquoi le LiDAR et pas une caméra ?**
- Fonctionne dans le noir
- Donne directement les distances (pas besoin de calcul)
- Plus simple à programmer
- Très utilisé en robotique

### GPS RTK Point One - La position globale

```
GPS Normal (téléphone)          GPS RTK (Point One)
        │                               │
        ▼                               ▼
   Précision: 3-5 mètres          Précision: 1-2 cm !

   ┌─────────────────┐            ┌─────────────────┐
   │   ╔═══╗         │            │        •        │
   │   ║ • ║  Tu es  │            │     Tu es       │
   │   ║   ║ quelque │            │   exactement    │
   │   ╚═══╝ part ici│            │       ici       │
   └─────────────────┘            └─────────────────┘

RTK = Real Time Kinematic
Il utilise une station de base pour corriger les erreurs
```

### VESC - Le contrôleur moteur

```
┌─────────────────────────────────────────────────────────┐
│                        VESC                              │
│         (Vedder Electronic Speed Controller)             │
├─────────────────────────────────────────────────────────┤
│                                                          │
│   Jetson ──USB──► VESC ──► Moteur                       │
│                     │                                    │
│                     └──► Servo direction                 │
│                                                          │
│   On envoie: "duty_cycle = 0.1" (10% de puissance)      │
│   On reçoit: RPM, courant, température, distance        │
│                                                          │
│   L'ODOMÉTRIE: Le VESC compte les tours du moteur       │
│   → On peut calculer la distance parcourue !            │
└─────────────────────────────────────────────────────────┘
```

---

## 4. La structure du projet

### Vue d'ensemble

```
Robocar/
│
├── src/                    # CODE SOURCE (le cerveau)
│   ├── drivers/            # Communication avec le hardware
│   ├── perception/         # "Voir" l'environnement
│   ├── slam/               # Cartographie + Localisation
│   ├── navigation/         # Planification de chemin (Phase 3)
│   ├── control/            # Contrôle moteur (Phase 3)
│   ├── core/               # Utilitaires communs
│   └── visualization/      # Affichage debug
│
├── config/                 # CONFIGURATION (réglages)
│   ├── robot.yaml          # Dimensions du robot
│   ├── sensors.yaml        # Réglages capteurs
│   └── navigation.yaml     # Paramètres navigation
│
├── scripts/                # PROGRAMMES À LANCER
│   ├── test_lidar.py       # Tester le LiDAR
│   ├── test_gps.py         # Tester le GPS
│   ├── demo_perception.py  # Démo perception
│   └── demo_slam.py        # Démo SLAM
│
├── docs/                   # DOCUMENTATION
│   ├── PROJECT_PLAN.md     # Plan du projet
│   └── ROS2_INTEGRATION.md # Guide migration ROS2
│
└── requirements.txt        # Dépendances Python
```

### Pourquoi cette organisation ?

```
PRINCIPE: Séparation des responsabilités

Chaque dossier a UN seul rôle:

drivers/     → Parle au hardware (et c'est tout)
perception/  → Comprend les données capteurs (et c'est tout)
slam/        → Fait la carte et localise (et c'est tout)
navigation/  → Planifie le chemin (et c'est tout)

AVANTAGE:
• Si le LiDAR change → on modifie SEULEMENT drivers/
• Si l'algo SLAM change → on modifie SEULEMENT slam/
• Facile à tester chaque partie séparément
• Facile à comprendre
```

---

## 5. Explication de chaque fichier

### PHASE 0 : Les drivers (src/drivers/)

#### `lidar_ld19.py` - Parler au LiDAR

```python
# CE QUE FAIT CE FICHIER:
# 1. Se connecte au LiDAR via USB (port série)
# 2. Lit les données brutes (octets)
# 3. Décode ces octets en distances
# 4. Fournit une interface simple

# EXEMPLE D'UTILISATION:
lidar = LD19Driver('/dev/ttyUSB0')  # Connexion USB
lidar.start()                        # Démarrer la lecture

scan = lidar.get_latest_scan()       # Obtenir les données
# scan.points = [(0°, 2.5m), (1°, 2.4m), ...]

lidar.stop()                         # Arrêter
```

**Le protocole LD19 expliqué:**
```
Le LiDAR envoie des paquets de 47 octets:

[Entête][Longueur][Vitesse][Angle_début][Données...][Angle_fin][CRC]
   1        1        2          2          36           2        1

Les 36 octets de données = 12 points × 3 octets chacun
Chaque point = [Distance_low][Distance_high][Intensité]

Notre code:
1. Cherche l'entête (0x54)
2. Lit 47 octets
3. Vérifie le CRC (somme de contrôle)
4. Décode les distances
5. Stocke dans une liste Python
```

#### `gps_pointone.py` - Parler au GPS

```python
# CE QUE FAIT CE FICHIER:
# 1. Se connecte au GPS via USB
# 2. Lit les phrases NMEA (format texte standard GPS)
# 3. Décode latitude, longitude, précision
# 4. Convertit en coordonnées locales (mètres)

# EXEMPLE D'UTILISATION:
gps = PointOneGPS('/dev/ttyUSB1')
gps.start()

position = gps.get_position()
# position.latitude = 48.8566
# position.longitude = 2.3522
# position.quality = "RTK_FIXED"  # Précision centimétrique!
```

**Le format NMEA expliqué:**
```
Le GPS envoie du texte comme ça:

$GNGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,47.0,M,,*47

Décodé:
- 123519 = 12h35min19s
- 4807.038,N = 48°07.038' Nord (latitude)
- 01131.000,E = 11°31.000' Est (longitude)
- 4 = Qualité (4 = RTK Fixed = très précis)
- 08 = 8 satellites visibles
- 0.9 = Précision horizontale
```

#### `vesc_motor.py` - Contrôler les moteurs

```python
# CE QUE FAIT CE FICHIER:
# 1. Se connecte au VESC via USB
# 2. Envoie des commandes de vitesse
# 3. Envoie des commandes de direction
# 4. Lit l'odométrie (distance parcourue)

# EXEMPLE D'UTILISATION:
vesc = VESCController('/dev/ttyACM0')
vesc.start()

vesc.set_duty(0.1)        # 10% de puissance (avancer doucement)
vesc.set_servo(0.5)       # Direction au centre
vesc.set_servo(0.3)       # Tourner à gauche
vesc.set_servo(0.7)       # Tourner à droite

distance, vitesse = vesc.get_odometry()
# distance = 1.5 mètres parcourus
# vitesse = 0.3 m/s
```

**Sécurités intégrées:**
```python
MAX_DUTY_CYCLE = 0.2   # Maximum 20% de puissance (sécurité!)
RAMP_STEP = 0.02       # Accélération progressive (pas brutal)

# Le code fait:
# - Rampe d'accélération (smooth)
# - Limite de vitesse
# - Arrêt d'urgence
```

---

### PHASE 1 : La perception (src/perception/)

#### `lidar_processor.py` - Nettoyer les données LiDAR

```
PROBLÈME: Les données brutes du LiDAR sont "sales"

Données brutes:          Données nettoyées:
    •  •                      • • •
  •      • ← bruit           •     •
    •  •  •                  •       •
  •    X   • ← erreur       •    X    •
    •  •  •                  •       •
  •      •                    •     •
    •  •                       • • •

CE QUE FAIT CE FICHIER:
1. Filtre médian (enlève les points isolés aberrants)
2. Filtre de distance (ignore trop proche ou trop loin)
3. Filtre d'intensité (ignore signaux trop faibles)
4. Segmentation (groupe les points proches ensemble)
```

```python
# EXEMPLE:
processor = LidarProcessor(
    min_range=0.05,      # Ignore < 5cm (trop proche)
    max_range=10.0,      # Ignore > 10m (trop loin)
    min_intensity=10     # Ignore signaux faibles
)

processed = processor.process(raw_scan)
# processed.points = points nettoyés et filtrés
```

#### `obstacle_detector.py` - Trouver les obstacles

```
CE QUE FAIT CE FICHIER:
Transforme des points en "obstacles" compréhensibles

Points LiDAR:                    Obstacles détectés:

    • • •                        ┌─────────┐
  •       •                      │ OBSTACLE│
  •       •      ──────►         │  #1     │
    • • •                        └─────────┘

• • • • • • • •                  ══════════════
                                    MUR #1

Informations extraites:
- Position (x, y)
- Taille (largeur, hauteur)
- Type (mur, poteau, objet mobile)
- Vitesse (si ça bouge)
- Distance minimale
```

```python
# EXEMPLE:
detector = ObstacleDetector()
obstacles = detector.detect(processed_scan)

for obs in obstacles:
    print(f"Obstacle à {obs.distance}m, type={obs.obstacle_type}")
    print(f"Taille: {obs.bbox.width}m x {obs.bbox.height}m")

    if obs.obstacle_type == ObstacleType.DYNAMIC:
        print(f"Il bouge à {obs.speed} m/s !")

# Vérifier le risque de collision:
is_danger, obstacle, time_to_crash = detector.check_collision_risk(obstacles)
if is_danger:
    print(f"ATTENTION! Collision dans {time_to_crash} secondes!")
```

#### `sensor_fusion.py` - Combiner tous les capteurs

```
PROBLÈME: Chaque capteur a des forces et faiblesses

GPS RTK:
  ✓ Position absolue (où sur Terre)
  ✗ Peut perdre le signal
  ✗ Pas d'orientation

LiDAR:
  ✓ Orientation relative
  ✓ Fonctionne partout
  ✗ Dérive avec le temps

Odométrie (roues):
  ✓ Très rapide (100 Hz)
  ✗ Glissement des roues
  ✗ Erreur cumulative

SOLUTION: Fusion de capteurs (Filtre de Kalman)

┌─────────┐
│   GPS   │──┐
└─────────┘  │
             ├──► FUSION (EKF) ──► Position optimale
┌─────────┐  │                     (le meilleur des 3)
│  LiDAR  │──┤
└─────────┘  │
             │
┌─────────┐  │
│ Odométrie│──┘
└─────────┘
```

```python
# EXEMPLE:
fusion = SensorFusion()
fusion.set_gps_origin(48.8566, 2.3522)  # Point de départ

# Dans la boucle principale:
fusion.update_gps(lat, lon, accuracy=0.02)
fusion.update_odometry(distance, angle, dt)

state = fusion.get_state()
# state.pose = (x, y, theta) - position fusionnée
# state.velocity = vitesse estimée
# state.covariance = incertitude (on sait qu'on ne sait pas tout!)
```

#### `transforms.py` - Conversions de coordonnées

```
PROBLÈME: Différents "points de vue"

Le LiDAR voit en coordonnées ROBOT:
  - "L'obstacle est à 2m devant MOI"

Le GPS donne des coordonnées MONDE:
  - "Je suis à 48.8566°N, 2.3522°E"

La carte est en coordonnées CARTE:
  - "L'obstacle est au pixel (150, 230)"

CE QUE FAIT CE FICHIER:
Convertit entre tous ces systèmes

          GPS (lat, lon)
               │
               ▼
         ┌───────────┐
         │ transforms │
         └───────────┘
          ╱    │    ╲
         ▼     ▼     ▼
     Robot   Monde   Carte
     (x,y)   (x,y)   (pixel)
```

---

### PHASE 2 : Le SLAM (src/slam/)

#### Qu'est-ce que le SLAM ?

```
SLAM = Simultaneous Localization And Mapping
     = Localisation et Cartographie Simultanées

C'est le problème de la poule et l'œuf:
- Pour savoir OÙ je suis, j'ai besoin d'une CARTE
- Pour faire une CARTE, je dois savoir OÙ je suis

Le SLAM résout les deux EN MÊME TEMPS!

Comment?
1. Je fais un premier scan LiDAR
2. J'avance un peu
3. Je fais un deuxième scan
4. Je compare les deux scans pour savoir combien j'ai bougé
5. J'ajoute les nouveaux points à ma carte
6. Je répète...
```

#### `slam_core.py` - Le SLAM principal

```
CE QUE FAIT CE FICHIER:
Wrapper autour de BreezySLAM (algorithme prouvé)

BreezySLAM est basé sur "CoreSLAM" (aussi appelé "tinySLAM"):
- Créé par des chercheurs en robotique
- Utilisé dans des vrais robots
- Optimisé pour processeurs ARM (= Jetson Nano!)

┌─────────────────────────────────────────────────────────┐
│                    BreezySLAM                            │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  Entrée: Scan LiDAR (360 distances)                     │
│          Odométrie (distance + angle)                    │
│                                                          │
│  Sortie: Position estimée (x, y, theta)                 │
│          Carte (image noir/blanc)                        │
│                                                          │
│  Algorithme:                                             │
│  1. Scan Matching (RMHC = Random Mutation Hill Climb)   │
│  2. Mise à jour de la carte                             │
│  3. Mise à jour de la position                          │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

```python
# EXEMPLE SIMPLE:
slam = SLAM()

# Boucle principale
while True:
    scan_mm = get_lidar_scan()  # Distances en millimètres

    pose = slam.update(
        scan_mm,
        velocity=(distance_mm, angle_deg, dt)
    )

    print(f"Je suis à ({pose.x}, {pose.y}), orientation {pose.theta}")

    map_image = slam.get_map()  # Image de la carte
```

#### `particle_filter.py` - Localisation précise

```
PROBLÈME: Le SLAM donne une position, mais avec de l'incertitude

SOLUTION: Particle Filter (Filtre Particulaire)

Idée: Au lieu d'une seule position, on en garde 500!

Étape 1: INITIALISATION
         Créer 500 "particules" autour de la position estimée

         Chaque particule = une hypothèse "je suis peut-être ici"

              • • • • •
            • • • • • • •
           • • • • • • • •
            • • • X • • •     X = position probable
              • • • • •

Étape 2: MOUVEMENT
         Quand le robot bouge, toutes les particules bougent
         (avec un peu de bruit aléatoire)

Étape 3: OBSERVATION
         On compare ce que chaque particule "verrait" avec ce
         que le LiDAR voit vraiment

         Les particules qui "voient" la même chose = gros poids
         Les particules qui "voient" différemment = petit poids

Étape 4: RESAMPLING
         On garde les bonnes particules, on jette les mauvaises

         Résultat: les particules CONVERGENT vers la vraie position

              • •
             • • •
              • X •      ← Beaucoup plus précis!
              • •
```

#### `occupancy_grid.py` - La carte

```
CE QUE FAIT CE FICHIER:
Gère une carte 2D où chaque cellule est:
- LIBRE (blanc) = on peut passer
- OCCUPÉ (noir) = obstacle
- INCONNU (gris) = pas encore exploré

Exemple de carte 10x10:
┌─────────────────────────┐
│ ? ? ? ? ? ? ? ? ? ? │  ? = inconnu
│ ? ? ? ? ? ? ? ? ? ? │  · = libre
│ ? ? · · · · # # ? ? │  # = occupé
│ ? ? · · · · # # ? ? │
│ ? ? · · R · · · ? ? │  R = robot
│ ? ? · · · · · · ? ? │
│ ? ? · · · · · # ? ? │
│ ? ? # # # # # # ? ? │
│ ? ? ? ? ? ? ? ? ? ? │
│ ? ? ? ? ? ? ? ? ? ? │
└─────────────────────────┘

COMMENT ÇA SE REMPLIT:

1. Le LiDAR voit un obstacle à 3m devant
2. On trace un rayon du robot jusqu'à l'obstacle
3. Tout le rayon = LIBRE (on a vu à travers)
4. Le point final = OCCUPÉ (l'obstacle)

     Robot                    Obstacle
        R · · · · · · · · · · #
          ↑                   ↑
       LIBRE              OCCUPÉ
    (le laser est         (le laser
     passé par là)        s'est arrêté)
```

---

## 6. Pourquoi ça marche sur Jetson Nano

### Ressources nécessaires vs disponibles

```
JETSON NANO 4GB:
┌────────────────────────────────────────┐
│ CPU: 4 coeurs ARM @ 1.4 GHz            │
│ RAM: 4 GB                              │
│ GPU: 128 coeurs CUDA                   │
└────────────────────────────────────────┘

NOTRE CODE UTILISE:
┌────────────────────────────────────────┐
│ RAM: ~300-500 MB (carte + particules)  │
│ CPU: 1-2 coeurs à ~50%                 │
│ GPU: Non utilisé (optionnel)           │
└────────────────────────────────────────┘

MARGE: Énorme! On utilise ~10-15% des capacités
```

### Optimisations intégrées

```python
# 1. NUMPY pour les calculs (optimisé en C)
distances = np.array(scan)  # Rapide!
# Au lieu de:
distances = [p.distance for p in scan]  # Lent

# 2. Pas de traitement inutile
if distance > MAX_RANGE:
    continue  # On ignore, pas de calcul

# 3. Fréquence adaptée
# LiDAR: 10 Hz (suffisant)
# GPS: 5 Hz (suffisant)
# Contrôle: 20 Hz (suffisant pour une voiture RC)

# 4. BreezySLAM est optimisé pour ARM
# Il utilise SIMD (instructions parallèles) sur ARM
# Spécifiquement testé sur Raspberry Pi et Jetson
```

### Comparaison avec F1Tenth

```
F1TENTH = Compétition de voitures RC autonomes
         Utilise EXACTEMENT le même hardware!

┌─────────────────────────────────────────────────┐
│           F1TENTH (référence)                    │
├─────────────────────────────────────────────────┤
│ Hardware: Jetson Nano/Xavier                     │
│ LiDAR: Hokuyo ou RPLidar (similaire à LD19)     │
│ SLAM: SLAM Toolbox ou Hector SLAM               │
│ Vitesse: jusqu'à 5 m/s !                        │
├─────────────────────────────────────────────────┤
│           Notre projet                           │
├─────────────────────────────────────────────────┤
│ Hardware: Jetson Nano 4GB                        │
│ LiDAR: LD19 (similaire)                         │
│ SLAM: BreezySLAM (même famille d'algorithmes)   │
│ Vitesse cible: 0.5-1 m/s (bien plus lent)       │
└─────────────────────────────────────────────────┘

Si F1Tenth fait 5 m/s, on peut facilement faire 1 m/s!
```

---

## 7. Pourquoi c'est scalable

### Architecture modulaire

```
AUJOURD'HUI:                    DEMAIN (facile à changer):

drivers/lidar_ld19.py           drivers/lidar_rplidar.py
        │                               │
        └──► même interface ◄───────────┘

Si tu changes de LiDAR:
1. Crée un nouveau driver
2. Garde la même interface (get_scan(), etc.)
3. Change UNE ligne de config
4. Le reste du code ne change pas!
```

### Configurations externalisées

```yaml
# config/robot.yaml
robot:
  wheelbase: 0.26      # ← Change ça si tu changes de voiture
  max_speed: 1.0       # ← Augmente quand tu es confiant

# Pas besoin de toucher au code!
```

### Prêt pour ROS2

```
Notre structure:
src/drivers/lidar_ld19.py

ROS2 structure:
src/robocar_drivers/lidar_node.py
    └── Utilise lidar_ld19.py dedans!

On peut réutiliser 90% du code en migrant vers ROS2
```

---

## 8. Les fichiers "vides" (__init__.py)

### Pourquoi ces fichiers existent ?

```python
# src/visualization/__init__.py
"""
Visualization module for debug and monitoring.
"""

# C'est (presque) vide! Pourquoi?
```

### Réponse: C'est un "package Python"

```
Sans __init__.py:              Avec __init__.py:

src/                           src/
├── perception/                ├── perception/
│   ├── obstacle_detector.py   │   ├── __init__.py      ← MAGIC!
│   └── lidar_processor.py     │   ├── obstacle_detector.py
│                              │   └── lidar_processor.py

# Sans __init__.py:
from perception.obstacle_detector import ObstacleDetector
# ERREUR! Python ne sait pas que c'est un package

# Avec __init__.py:
from perception.obstacle_detector import ObstacleDetector
# OK! Python reconnaît "perception" comme un package
```

### Le contenu de __init__.py

```python
# src/slam/__init__.py

"""Documentation du module"""

# On importe les classes principales pour un accès facile
from .slam_core import SLAM, SLAMConfig
from .particle_filter import ParticleFilter
from .occupancy_grid import OccupancyGrid

# Maintenant on peut faire:
from slam import SLAM  # Simple!

# Au lieu de:
from slam.slam_core import SLAM  # Plus long
```

### Pourquoi visualization/ est "vide" ?

```
C'est un PLACEHOLDER (espace réservé)

visualization/
├── __init__.py     ← "Ce dossier existe et sera rempli plus tard"

On l'a créé pour:
1. Montrer la structure prévue
2. Pouvoir importer depuis ce package plus tard
3. Organiser le code proprement dès le début

C'est comme avoir une boîte étiquetée "OUTILS" vide
qu'on remplira quand on aura les outils.
```

---

## 9. Comment tout s'assemble

### Le flux complet de données

```
┌─────────────────────────────────────────────────────────────────┐
│                    BOUCLE PRINCIPALE (10 Hz)                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 1. LECTURE CAPTEURS                                              │
│                                                                  │
│    LiDAR ───► lidar_ld19.py ───► 360 distances                  │
│    GPS   ───► gps_pointone.py ──► latitude, longitude           │
│    VESC  ───► vesc_motor.py ────► odométrie                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. PERCEPTION                                                    │
│                                                                  │
│    360 distances ──► lidar_processor.py ──► points filtrés      │
│    points filtrés ──► obstacle_detector.py ──► liste obstacles  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. SLAM & LOCALISATION                                           │
│                                                                  │
│    points + odométrie ──► slam_core.py ──► position + carte     │
│    position + GPS ──► sensor_fusion.py ──► position fusionnée   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. NAVIGATION (Phase 3 - à faire)                                │
│                                                                  │
│    position + carte + destination ──► chemin à suivre           │
│    chemin + obstacles ──► commandes moteur                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. CONTRÔLE                                                      │
│                                                                  │
│    commandes ──► vesc_motor.py ──► MOTEUR + DIRECTION           │
└─────────────────────────────────────────────────────────────────┘
                              │
                              └──────────── RÉPÉTER ──────────────┘
```

### Exemple de code principal (simplifié)

```python
# main.py (ce qu'on aura à la fin)

from drivers import LD19Driver, PointOneGPS, VESCController
from perception import LidarProcessor, ObstacleDetector, SensorFusion
from slam import SLAM
from navigation import PathPlanner, PathFollower  # Phase 3

# Initialisation
lidar = LD19Driver('/dev/ttyUSB0')
gps = PointOneGPS('/dev/ttyUSB1')
vesc = VESCController('/dev/ttyACM0')

processor = LidarProcessor()
detector = ObstacleDetector()
fusion = SensorFusion()
slam = SLAM()
planner = PathPlanner()
follower = PathFollower()

# Démarrage
lidar.start()
gps.start()
vesc.start()

# Destination
destination = (10.0, 5.0)  # 10m devant, 5m à droite

# Boucle principale
while not arrived:
    # 1. Lire les capteurs
    scan = lidar.get_latest_scan()
    position = gps.get_position()
    odom = vesc.get_odometry()

    # 2. Traiter les données
    processed = processor.process(scan)
    obstacles = detector.detect(processed)

    # 3. Mettre à jour SLAM
    pose = slam.update(scan, odom)
    fusion.update_gps(position)
    fusion.update_odometry(odom)
    state = fusion.get_state()

    # 4. Planifier le chemin
    path = planner.plan(state.pose, destination, slam.get_map())

    # 5. Suivre le chemin
    velocity, steering = follower.compute(state, path, obstacles)

    # 6. Envoyer les commandes
    vesc.set_duty(velocity)
    vesc.set_steering(steering)

    # 7. Vérifier si arrivé
    arrived = distance_to(destination) < 0.5

    time.sleep(0.1)  # 10 Hz

print("Arrivé!")
```

---

## Résumé

### Ce qu'on a fait

| Phase | Contenu | Status |
|-------|---------|--------|
| **Phase 0** | Drivers (LiDAR, GPS, VESC) | ✓ Fait |
| **Phase 1** | Perception (filtrage, obstacles, fusion) | ✓ Fait |
| **Phase 2** | SLAM (carte + localisation) | ✓ Fait |
| **Phase 3** | Navigation (planification + suivi) | À faire |
| **Phase 4** | Intégration (tout ensemble) | À faire |

### Pourquoi ça va marcher

1. **Algorithmes prouvés**: BreezySLAM, Particle Filter (utilisés en vrai)
2. **Hardware suffisant**: Jetson Nano fait tourner F1Tenth à 5 m/s
3. **Architecture propre**: Facile à débugger et améliorer
4. **Scalable**: Peut évoluer vers ROS2 facilement

### Prochaine étape

La Phase 3 (Navigation) ajoutera:
- `navigation/global_planner.py` - Calcul du chemin (A*)
- `navigation/local_planner.py` - Évitement d'obstacles (DWA)
- `navigation/path_follower.py` - Suivi de trajectoire (Pure Pursuit)

---

*Document créé pour comprendre le projet Robocar de A à Z*