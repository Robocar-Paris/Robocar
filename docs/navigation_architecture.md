# Architecture de la Navigation Autonome

Documentation des fichiers lies au systeme de navigation (`scripts/start_navigation.py`).

---

## Vue d'ensemble

La voiture avance tout droit et utilise le LiDAR pour detecter et eviter les obstacles en temps reel.

```
scripts/start_navigation.py          <- Point d'entree
|
+-- scripts/check_components.py      <- Verification hardware
|
+-- src/navigation_controller.py     <- Controleur principal (mode basique)
|   +-- src/interface/               <- Interfaces abstraites + adaptateurs
|   |   +-- sensor_interface.py      <- Interfaces ILidarSensor, IMotorController
|   |   +-- hardware_adapters.py     <- Adaptateurs hardware reels
|   |   |   +-- src/driver/lidar.py  <- Driver LD19
|   |   |   +-- src/driver/vesc_motor.py <- Driver VESC
|   |   +-- simulation_adapters.py   <- Adaptateurs simulation
|   |       +-- src/simulation/      <- Environnements de test
|   +-- src/perception/
|       +-- lidar_processor.py       <- Filtrage et traitement LiDAR
|       +-- obstacle_detector.py     <- Detection et classification obstacles
|
+-- src/navigation_controller_slam.py <- Controleur SLAM (mode avance)
|   +-- src/slam/slam_core.py        <- Cartographie SLAM
|   +-- src/slam/occupancy_grid.py   <- Grille d'occupation
|   +-- src/navigation/global_planner.py <- Planification A*
|   +-- src/navigation/local_planner.py  <- Evitement DWA
|
+-- config/robot.yaml                <- Configuration
```

---

## Commande de lancement

```bash
# Mode reel (sur le Robocar)
python3 scripts/start_navigation.py --lidar-port /dev/ttyUSB0 --vesc-port /dev/ttyACM0

# Mode simulation (sur PC)
python3 scripts/start_navigation.py --simulation --env epitech

# Mode SLAM avance
python3 scripts/start_navigation.py --slam --lidar-port /dev/ttyUSB0 --vesc-port /dev/ttyACM0
```

Options disponibles :
- `--skip-check` : ignorer la verification des composants
- `--no-confirm` : demarrer sans demander confirmation
- `--no-gui` : pas d'interface graphique (mode headless)
- `--max-speed 0.5` : vitesse maximale (0.0 a 1.0)
- `--env {parking,corridor,epitech}` : environnement de simulation

---

## Fichiers detailles

### 1. scripts/start_navigation.py

**Role :** Point d'entree principal. Parse les arguments, initialise les capteurs, et lance le mode de navigation choisi.

**Classe principale :** `NavigationLauncher`

| Methode | Description |
|---------|-------------|
| `check_components()` | Verifie LiDAR et VESC via `ComponentChecker` |
| `run_real()` | Mode reel : cree `RealLidarAdapter` + `RealMotorAdapter`, lance `NavigationController` |
| `run_slam()` | Mode SLAM : cree les adaptateurs, lance `SLAMNavigationController` |
| `run_simulation()` | Mode simulation : cree un environnement virtuel et des adaptateurs simules |

**Flux d'execution (mode reel) :**
1. Affiche la banniere
2. Verifie les composants (LiDAR + VESC)
3. Initialise `RealLidarAdapter` et `RealMotorAdapter`
4. Cree `NavigationController(lidar, motor, mode='car')`
5. Demande confirmation utilisateur
6. Lance `controller.run_real()`

---

### 2. src/navigation_controller.py

**Role :** Controleur de navigation principal. Avance tout droit et evite les obstacles avec le LiDAR.

**Classe principale :** `NavigationController`

| Constante | Valeur | Description |
|-----------|--------|-------------|
| `MAX_SPEED` | 0.5 | Vitesse max normalisee |
| `MIN_SPEED` | 0.1 | Vitesse min normalisee |
| `OBSTACLE_STOP_DIST` | 0.3m | Arret d'urgence |
| `OBSTACLE_SLOW_DIST` | 1.0m | Zone de ralentissement |
| `OBSTACLE_WARN_DIST` | 2.0m | Zone de vigilance |
| `LOOP_RATE` | 20 Hz | Frequence de la boucle |

| Methode | Description |
|---------|-------------|
| `navigation_step()` | Une iteration : lire LiDAR, detecter obstacles, calculer vitesse/direction |
| `check_obstacles()` | Traite le scan LiDAR, detecte les obstacles par zone (avant, gauche, droite) |
| `run_real()` | Boucle principale mode reel |
| `run_headless()` | Boucle sans GUI |
| `run_with_gui()` | Boucle avec visualisation matplotlib |
| `init_sensors()` | Demarre LiDAR et moteur |
| `shutdown()` | Arret d'urgence + fermeture capteurs |

**Logique de `navigation_step()` :**
1. Lire le scan LiDAR via `check_obstacles()`
2. Si danger imminent (< 0.3m) : arret + tourner pour se degager
3. Sinon : avancer tout droit a `MAX_SPEED`
4. Si obstacle dans la zone de vigilance (< 2.0m) : ralentir progressivement
5. Si obstacle dans la zone de ralentissement (< 1.0m) : mixer evitement + avance

**Zones angulaires :**
```
         +pi/2 (gauche)
           |
  +135     |     +45
    \      |      /
     \  FRONT_L  /
      \    |    /
       \   |   /
--------+--+--+-------- 0 (avant)
       /   |   \
      / FRONT_R \
     /     |     \
    /      |      \
  -135     |     -45
           |
         -pi/2 (droite)
```

---

### 3. src/interface/sensor_interface.py

**Role :** Interfaces abstraites pour les capteurs et actionneurs. Permet au meme code de fonctionner en simulation et en reel.

| Interface | Methodes | Description |
|-----------|----------|-------------|
| `ILidarSensor` | `start()`, `stop()`, `get_scan()` | Capteur LiDAR |
| `IMotorController` | `start()`, `stop()`, `set_speed()`, `set_steering()`, `emergency_stop()` | Moteur |

| Dataclass | Champs | Description |
|-----------|--------|-------------|
| `LidarPoint` | `angle`, `distance`, `intensity`, `valid` | Un point LiDAR |
| `LidarData` | `timestamp`, `points[]`, `scan_frequency` | Un scan complet |

---

### 4. src/interface/hardware_adapters.py

**Role :** Adapte les drivers hardware reels aux interfaces abstraites.

| Adaptateur | Driver | Port par defaut |
|------------|--------|-----------------|
| `RealLidarAdapter` | `LidarDriver` (LD19) | `/dev/ttyUSB0` |
| `RealMotorAdapter` | `VESCController` | `/dev/ttyACM0` |

---

### 5. src/interface/simulation_adapters.py

**Role :** Adaptateurs pour la simulation sur PC (sans hardware).

| Adaptateur | Description |
|------------|-------------|
| `SimulatedLidarAdapter` | Ray-casting dans un environnement virtuel |
| `SimulatedMotorAdapter` | Modele cinematique bicyclette (wheelbase=0.26m) |

`SimulatedMotorAdapter` gere la physique : `update_physics(dt)` calcule la nouvelle position via le modele bicyclette, puis met a jour la pose du LiDAR simule.

---

### 6. src/perception/lidar_processor.py

**Role :** Filtre et traite les scans LiDAR bruts avant la detection d'obstacles.

**Classe :** `LidarProcessor`

**Pipeline de traitement :**
1. Filtrage par distance (0.05m - 8.0m)
2. Filtrage par intensite (> 10)
3. Filtrage median (anti-bruit)
4. Conversion en coordonnees cartesiennes (x, y)
5. Segmentation en groupes de points

**Sortie :** `ProcessedScan` contenant :
- `angles[]`, `distances[]` : donnees polaires filtrees
- `points[]` : coordonnees cartesiennes (x, y)
- `valid_mask[]` : masque de validite
- `segments[]` : groupes de points proches

---

### 7. src/perception/obstacle_detector.py

**Role :** Detecte, classifie et suit les obstacles a partir des scans traites.

**Classe :** `ObstacleDetector`

**Pipeline :**
1. Clustering des points proches (< 0.3m entre eux)
2. Filtrage (minimum 3 points par cluster)
3. Classification : `WALL`, `POLE`, `STATIC`, `DYNAMIC`
4. Tracking inter-frames (association par distance < 0.5m)

**Types d'obstacles (`ObstacleType`) :**
| Type | Critere |
|------|---------|
| `WALL` | Longueur > 0.5m, points alignes |
| `POLE` | Petit, compact |
| `STATIC` | Immobile entre frames |
| `DYNAMIC` | En mouvement |

---

### 8. src/driver/lidar.py

**Role :** Driver bas-niveau pour le LiDAR LD19 via UART.

**Classe :** `LidarDriver`

| Parametre | Valeur |
|-----------|--------|
| Baudrate | 230400 bps |
| Taille paquet | 47 octets |
| Points par paquet | 12 |
| Portee | 0.02m - 12m |
| Resolution angulaire | ~1 deg |
| Frequence scan | 5-15 Hz |
| Header | `0x54` |

**Fonctionnement :**
- Thread de lecture continue des paquets serie
- Verification CRC de chaque paquet
- Assemblage des paquets en scans 360 complets
- Buffer circulaire des derniers scans

---

### 9. src/driver/vesc_motor.py

**Role :** Driver pour le controleur moteur VESC via PyVESC.

**Classe :** `VESCController`

| Parametre | Valeur |
|-----------|--------|
| Baudrate | 115200 bps |
| Duty cycle max | 0.05 (5%) |
| Steering max | 0.3 rad |
| Rampe acceleration | 0.02 par step |

| Methode | Description |
|---------|-------------|
| `set_speed(v)` | Vitesse [-1.0, 1.0] -> duty cycle |
| `set_steering(s)` | Direction [-1.0, 1.0] -> position servo |
| `emergency_stop()` | Coupe le moteur immediatement |

Le ramping assure une acceleration progressive pour eviter les a-coups.

---

### 10. scripts/check_components.py

**Role :** Verification pre-vol des composants hardware.

**Classe :** `ComponentChecker`

**Verifications effectuees :**
1. Dependances Python : `numpy`, `pyserial`, `pyyaml`, (optionnel: `pyvesc`, `matplotlib`)
2. Ports serie disponibles (`/dev/ttyUSB*`, `/dev/ttyACM*`)
3. Detection du LiDAR LD19 (test de lecture sur les ports USB)
4. Detection du VESC (test de connexion sur les ports ACM)

---

### 11. config/robot.yaml

**Role :** Configuration centralisee du robot, chargee au demarrage par `NavigationController`.

**Sections principales :**

```yaml
robot:
  wheelbase: 0.26          # Empattement (m)
  track: 0.18              # Voie (m)

safety:
  emergency_stop_distance: 0.3   # Arret immediat (m)
  slow_distance: 1.0             # Ralentissement (m)
  warning_distance: 2.0          # Vigilance (m)
  max_no_lidar_time: 1.0         # Arret si pas de LiDAR (s)

navigation:
  max_speed: 0.5
  min_speed: 0.1
  loop_rate: 20

perception:
  lidar:
    min_range: 0.05
    max_range: 8.0
    min_intensity: 10
  obstacle_detection:
    cluster_threshold: 0.3
    min_cluster_points: 3
```

---

### 12. src/simulation/environment.py

**Role :** Definit les environnements virtuels pour la simulation.

**Environnements disponibles :**

| Fonction | Description |
|----------|-------------|
| `create_parking_env()` | Parking avec places et murs |
| `create_corridor_env()` | Couloir etroit avec murs |
| `create_epitech_env()` | Campus Epitech simplifie |
| `create_empty_env()` | Espace vide |

**Types d'obstacles :**
- `WALL` : segment de ligne (mur)
- `BOX` : rectangle (voiture, meuble)
- `CYLINDER` : cercle (poteau, arbre)

Le `SimulatedLidarAdapter` utilise le ray-casting contre ces obstacles pour generer des scans LiDAR realistes.

---

### 13. src/navigation_controller_slam.py

**Role :** Controleur avance avec SLAM, planification A*, et evitement DWA.

**Classe :** `SLAMNavigationController`

**Pipeline SLAM :**
1. Construction de carte en temps reel (occupancy grid via LiDAR)
2. Localisation par filtre a particules
3. Planification globale A* sur la grille
4. Evitement local DWA (Dynamic Window Approach)

**Note :** Le GPS est passe comme `None` dans le mode actuel. Le SLAM fonctionne uniquement avec le LiDAR pour la cartographie et la localisation.

---

## Flux de donnees en mode reel

```
LiDAR LD19 (/dev/ttyUSB0)
    |
    v
LidarDriver (driver/lidar.py)
    | paquets serie -> LidarScan
    v
RealLidarAdapter (interface/hardware_adapters.py)
    | LidarScan -> LidarData
    v
NavigationController.check_obstacles()
    |
    +-> LidarProcessor.process()     -> ProcessedScan (filtre)
    +-> ObstacleDetector.detect()    -> List[Obstacle] (classifie)
    +-> Calcul distances par zone    -> (danger, distance, steering)
    |
    v
NavigationController.navigation_step()
    | (vitesse, direction)
    v
RealMotorAdapter (interface/hardware_adapters.py)
    | set_speed(), set_steering()
    v
VESCController (driver/vesc_motor.py)
    | duty cycle + servo position
    v
VESC (/dev/ttyACM0) -> Moteur + Servo
```
