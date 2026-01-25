# Robocar - Plan de Projet Navigation Autonome

## Vue d'ensemble

Transformer la voiture RC en robot de livraison autonome capable de naviguer d'un point A à un point B en utilisant:
- **LiDAR LD19** : Détection d'obstacles et cartographie
- **GPS RTK Point One** : Localisation précise (centimétrique)
- **Jetson Nano** : Traitement embarqué

---

## Architecture Système

```
┌─────────────────────────────────────────────────────────────────────┐
│                         JETSON NANO                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │   LiDAR      │  │   GPS RTK    │  │   Caméra     │              │
│  │   LD19       │  │  Point One   │  │   OAK-D      │              │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘              │
│         │                 │                 │                        │
│         ▼                 ▼                 ▼                        │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │              COUCHE DRIVERS / HAL                        │        │
│  │  • ld19_driver.py  • gps_driver.py  • camera_driver.py  │        │
│  └─────────────────────────┬───────────────────────────────┘        │
│                            │                                         │
│                            ▼                                         │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │                  SLAM & LOCALISATION                     │        │
│  │  • Cartographie (mapping)                                │        │
│  │  • Localisation (où suis-je?)                           │        │
│  │  • Fusion GPS + LiDAR + Odométrie                       │        │
│  └─────────────────────────┬───────────────────────────────┘        │
│                            │                                         │
│                            ▼                                         │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │                   NAVIGATION                             │        │
│  │  • Planification globale (A* / Dijkstra)                │        │
│  │  • Planification locale (DWA / TEB)                     │        │
│  │  • Évitement d'obstacles                                │        │
│  └─────────────────────────┬───────────────────────────────┘        │
│                            │                                         │
│                            ▼                                         │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │                   CONTRÔLE                               │        │
│  │  • Contrôleur PID vitesse/direction                     │        │
│  │  • Interface VESC                                        │        │
│  └─────────────────────────────────────────────────────────┘        │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Qu'est-ce que le SLAM ?

### SLAM = Simultaneous Localization And Mapping

Le SLAM répond à deux questions simultanément:
1. **Où suis-je ?** (Localisation)
2. **À quoi ressemble mon environnement ?** (Cartographie)

### Le problème du poulet et de l'œuf

- Pour savoir où je suis, j'ai besoin d'une carte
- Pour construire une carte, j'ai besoin de savoir où je suis
- → Le SLAM résout les deux en même temps !

### Types de SLAM pour ton projet

| Type | Capteur | Avantages | Inconvénients |
|------|---------|-----------|---------------|
| **LiDAR SLAM** | LD19 | Précis, fonctionne dans le noir | 2D seulement |
| **Visual SLAM** | OAK-D | 3D, riche en features | Sensible à la lumière |
| **GPS-aided SLAM** | RTK GPS | Position absolue | Perte de signal indoor |

### Notre approche : SLAM Hybride

```
GPS RTK (outdoor) ──┐
                    ├──► Fusion EKF ──► Position finale
LiDAR SLAM ─────────┘

Avantage:
- Extérieur : GPS RTK donne la position globale
- Obstacles : LiDAR détecte et évite
- Intérieur/tunnels : LiDAR SLAM prend le relais si GPS perdu
```

---

## Milestones du Projet

### Phase 0 : Fondations (Semaine 1-2)
**Objectif**: Avoir tous les capteurs qui fonctionnent individuellement

```
□ M0.1 - Driver LiDAR LD19
    ├── Lecture des données série
    ├── Parsing du protocole LD19
    ├── Visualisation des scans 2D
    └── Test de fréquence (10Hz minimum)

□ M0.2 - Driver GPS RTK Point One
    ├── Communication NMEA/RTCM
    ├── Parsing des trames GGA, RMC
    ├── Gestion du mode RTK Fixed
    └── Logging des positions

□ M0.3 - Refactoring contrôle moteur
    ├── Abstraction du VESC
    ├── Interface de commande unifiée
    └── Lecture odométrie VESC
```

### Phase 1 : Perception (Semaine 3-4)
**Objectif**: Le robot "voit" son environnement

```
□ M1.1 - Traitement LiDAR
    ├── Filtrage des points aberrants
    ├── Détection d'obstacles
    ├── Clustering des obstacles
    └── Estimation de distance minimale

□ M1.2 - Fusion Capteurs Simple
    ├── Synchronisation temporelle
    ├── Transformation de coordonnées
    └── Structure de données unifiée
```

### Phase 2 : SLAM & Localisation (Semaine 5-8)
**Objectif**: Le robot sait où il est et construit une carte

```
□ M2.1 - Odométrie
    ├── Modèle cinématique Ackermann
    ├── Intégration encodeurs VESC
    └── Estimation de pose (x, y, θ)

□ M2.2 - SLAM 2D avec LiDAR
    ├── Implémentation ICP (scan matching)
    ├── Construction de carte d'occupation
    ├── Sauvegarde/chargement de carte
    └── Intégration Hector SLAM ou GMapping

□ M2.3 - Fusion GPS-SLAM
    ├── Extended Kalman Filter (EKF)
    ├── Géoréférencement de la carte
    ├── Gestion perte GPS
    └── Recalage automatique
```

### Phase 3 : Navigation (Semaine 9-12)
**Objectif**: Le robot planifie et suit un chemin

```
□ M3.1 - Planification Globale
    ├── Représentation de la carte (grille)
    ├── Algorithme A* / Dijkstra
    ├── Waypoints intermédiaires
    └── Replanification si obstacle

□ M3.2 - Planification Locale
    ├── Dynamic Window Approach (DWA)
    ├── Évitement d'obstacles temps réel
    ├── Lissage de trajectoire
    └── Contraintes cinématiques

□ M3.3 - Contrôleur de suivi
    ├── Pure Pursuit ou Stanley
    ├── Régulation vitesse adaptative
    └── Gestion des virages serrés
```

### Phase 4 : Intégration (Semaine 13-14)
**Objectif**: Tout fonctionne ensemble

```
□ M4.1 - Machine à états
    ├── États: IDLE, MAPPING, NAVIGATING, STOPPED
    ├── Transitions sécurisées
    └── Mode d'arrêt d'urgence

□ M4.2 - Interface utilisateur
    ├── Envoi de destination GPS
    ├── Visualisation carte/position
    └── Monitoring état robot

□ M4.3 - Tests terrain
    ├── Tests unitaires
    ├── Tests d'intégration
    └── Validation sur parcours réel
```

### Phase 5 : Optimisation (Semaine 15-16)
**Objectif**: Performance et fiabilité

```
□ M5.1 - Optimisation Jetson
    ├── Profiling CPU/GPU
    ├── TensorRT pour inférence
    └── Optimisation mémoire

□ M5.2 - Robustesse
    ├── Gestion des erreurs capteurs
    ├── Recovery automatique
    └── Watchdog système
```

---

## Par où commencer ?

### Étape 1 : Driver LiDAR LD19 (PRIORITÉ 1)

Le LiDAR est le capteur le plus important pour la navigation. Commence par là.

```python
# Structure suggérée: src/lidar/ld19_driver.py

class LD19Driver:
    """
    Driver pour le LiDAR LD19 (Ldrobot)
    - Connexion série USB
    - Protocole: 47 bytes par paquet
    - Fréquence: ~10Hz (scan complet)
    - Résolution: 1° (360 points par scan)
    """

    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        pass

    def get_scan(self) -> List[Tuple[float, float]]:
        """Retourne [(angle_rad, distance_m), ...]"""
        pass
```

### Étape 2 : Driver GPS RTK (PRIORITÉ 2)

```python
# Structure suggérée: src/gps/rtk_driver.py

class PointOneGPS:
    """
    Driver pour GPS RTK Point One
    - NMEA 0183 protocol
    - Précision: 1-2cm en mode RTK Fixed
    """

    def get_position(self) -> Tuple[float, float, float]:
        """Retourne (latitude, longitude, altitude)"""
        pass

    def get_rtk_status(self) -> str:
        """Retourne: 'NO_FIX', 'GPS', 'DGPS', 'RTK_FLOAT', 'RTK_FIXED'"""
        pass
```

### Étape 3 : Visualisation de base

Crée un script de test qui affiche:
- Le scan LiDAR en temps réel (matplotlib ou pygame)
- La position GPS sur une carte

### Étape 4 : Intégration progressive

Une fois les drivers fonctionnels, intègre le SLAM étape par étape.

---

## Choix Technologiques Recommandés

### Framework

| Option | Pour | Contre | Recommandation |
|--------|------|--------|----------------|
| **ROS2** | Écosystème riche, Nav2 | Complexe, lourd | Pour projet sérieux long terme |
| **Custom Python** | Simple, léger, éducatif | Réinventer la roue | Pour apprendre et prototyper |
| **ROS1 Noetic** | Beaucoup de ressources | Fin de vie 2025 | Non recommandé |

**Ma recommandation**: Commence en **Python pur** pour comprendre les concepts, puis migre vers **ROS2** quand tu auras un prototype fonctionnel.

### Bibliothèques SLAM

| Bibliothèque | Type | Jetson Compatible | Difficulté |
|--------------|------|-------------------|------------|
| **Hector SLAM** | LiDAR 2D | Oui | Facile |
| **GMapping** | LiDAR 2D + Odométrie | Oui | Moyen |
| **Cartographer** | LiDAR 2D/3D | Oui (optimisé) | Difficile |
| **ORB-SLAM3** | Visuel | Oui (GPU) | Difficile |

**Pour commencer**: Implémente un **ICP simple** (Iterative Closest Point) en Python, puis utilise **Hector SLAM** pour la production.

### Navigation

- **Planification globale**: A* sur grille d'occupation
- **Planification locale**: DWA (Dynamic Window Approach)
- **Suivi de trajectoire**: Pure Pursuit (simple et efficace)

---

## Structure de Fichiers Proposée

```
Robocar/
├── src/
│   ├── drivers/                    # NEW: Couche hardware
│   │   ├── __init__.py
│   │   ├── lidar_ld19.py          # Driver LiDAR
│   │   ├── gps_pointone.py        # Driver GPS RTK
│   │   └── vesc_motor.py          # Refactor du contrôle moteur
│   │
│   ├── perception/                 # NEW: Traitement capteurs
│   │   ├── __init__.py
│   │   ├── obstacle_detector.py   # Détection obstacles LiDAR
│   │   ├── sensor_fusion.py       # Fusion multi-capteurs
│   │   └── transforms.py          # Transformations de coordonnées
│   │
│   ├── slam/                       # NEW: SLAM & Localisation
│   │   ├── __init__.py
│   │   ├── odometry.py            # Odométrie Ackermann
│   │   ├── icp.py                 # Scan matching
│   │   ├── occupancy_grid.py      # Carte d'occupation
│   │   ├── ekf_localization.py    # Filtre de Kalman
│   │   └── map_io.py              # Sauvegarde/chargement cartes
│   │
│   ├── navigation/                 # NEW: Navigation autonome
│   │   ├── __init__.py
│   │   ├── global_planner.py      # A* path planning
│   │   ├── local_planner.py       # DWA obstacle avoidance
│   │   ├── path_follower.py       # Pure Pursuit controller
│   │   └── waypoint_manager.py    # Gestion des destinations
│   │
│   ├── control/                    # NEW: Contrôle bas niveau
│   │   ├── __init__.py
│   │   ├── pid_controller.py      # Régulateurs PID
│   │   └── ackermann_model.py     # Modèle cinématique
│   │
│   ├── core/                       # NEW: Infrastructure
│   │   ├── __init__.py
│   │   ├── config.py              # Configuration centralisée
│   │   ├── state_machine.py       # Machine à états
│   │   ├── logger.py              # Logging structuré
│   │   └── safety.py              # Watchdog, arrêt urgence
│   │
│   ├── visualization/              # NEW: Debug et monitoring
│   │   ├── __init__.py
│   │   ├── lidar_viz.py           # Visualisation LiDAR
│   │   ├── map_viz.py             # Visualisation carte
│   │   └── dashboard.py           # Interface monitoring
│   │
│   └── [modules existants...]     # camera/, mask_generator/, etc.
│
├── config/                         # NEW: Fichiers de configuration
│   ├── robot.yaml                 # Paramètres physiques
│   ├── sensors.yaml               # Config capteurs
│   ├── navigation.yaml            # Paramètres navigation
│   └── maps/                      # Cartes sauvegardées
│
├── scripts/                        # NEW: Scripts d'exécution
│   ├── run_mapping.py             # Mode cartographie
│   ├── run_navigation.py          # Mode navigation
│   ├── test_lidar.py              # Test LiDAR
│   └── test_gps.py                # Test GPS
│
├── tests/                          # NEW: Tests unitaires
│   ├── test_drivers.py
│   ├── test_slam.py
│   └── test_navigation.py
│
└── docs/                           # NEW: Documentation
    ├── PROJECT_PLAN.md            # Ce fichier
    ├── HARDWARE_SETUP.md          # Guide hardware
    └── API.md                     # Documentation API
```

---

## Ressources Utiles

### Documentation Capteurs
- **LD19 LiDAR**: [Ldrobot LD19 Protocol](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros)
- **Point One GPS**: [Point One Navigation Docs](https://pointonenav.com/docs)

### Algorithmes
- **ICP**: [Iterative Closest Point Explained](https://www.youtube.com/watch?v=QWDM4cFdKUc)
- **EKF SLAM**: [Cyrill Stachniss Course](https://www.youtube.com/watch?v=U6vr3iNrwRA)
- **A***: [Red Blob Games A* Tutorial](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- **Pure Pursuit**: [Paper original](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)

### Cours Complets
- [Coursera: Self-Driving Cars Specialization](https://www.coursera.org/specializations/self-driving-cars)
- [ETH Zurich: Autonomous Mobile Robots](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQEn33QDVGJpiZLi-SlL7vA)

---

## Prochaines Actions Immédiates

1. **Aujourd'hui**: Créer la structure de dossiers
2. **Cette semaine**: Implémenter le driver LiDAR LD19
3. **Semaine prochaine**: Implémenter le driver GPS RTK
4. **Dans 2 semaines**: Première visualisation temps réel des deux capteurs

---

*Document créé le 25/01/2026 - Robocar Autonomous Navigation Project*