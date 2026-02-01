# Documentation Drivers et Perception - Robocar

## Vue d'ensemble

Cette documentation couvre les trois composants principaux du système de perception de la voiture RC:

1. **GPS Point One** - Positionnement GPS RTK centimetrique
2. **LiDAR LD19** - Detection d'obstacles 360 degres
3. **Perception** - Traitement des donnees capteurs et detection d'obstacles

---

## Architecture des fichiers

```
Robocar/
├── src/
│   ├── driver/                    # Drivers hardware
│   │   ├── gps.py                 # Driver GPS Point One (PRINCIPAL)
│   │   ├── gps_rtk.py             # GPS avec corrections RTK Polaris (optionnel)
│   │   ├── polaris_client.py      # Client Polaris pour RTK (optionnel)
│   │   ├── ntrip_client.py        # Client NTRIP Centipede/RTK2GO (optionnel)
│   │   ├── lidar.py               # Driver LiDAR LD19
│   │   └── vesc_motor.py          # Controleur moteur VESC
│   │
│   └── perception/                # Traitement perception
│       ├── lidar_processor.py     # Traitement scans LiDAR
│       ├── obstacle_detector.py   # Detection et tracking obstacles
│       ├── sensor_fusion.py       # Fusion capteurs (EKF)
│       └── transforms.py          # Transformations coordonnees
│
├── lidar_test/                    # Tests et simulation LiDAR
│   └── src/
│       ├── lidar_simulator.py     # Simulateur environnements
│       ├── obstacle_detector.py   # Detection avec zones securite
│       ├── ld19_parser.py         # Parser paquets LD19
│       └── lidar_main.py          # Suite de tests
│
├── config/
│   ├── gps.yaml                   # Configuration GPS
│   └── robot.yaml                 # Configuration robot
│
└── scripts/
    ├── test_gps_pointone.py       # Test GPS Point One
    └── test_lidar.py              # Test LiDAR
```

---

## 1. GPS Point One

### Description
Driver pour le GPS RTK Point One Navigation. Parse les messages NMEA (GGA pour position, GST pour precision).

### Specifications
- **Port serie**: `/dev/ttyUSB1` (par defaut)
- **Baudrate**: 460800
- **Protocole**: NMEA 0183
- **Niveaux de qualite**:
  - `0` - NO_FIX (pas de fix)
  - `1` - GPS (~2-5m de precision)
  - `2` - DGPS (~0.5-2m)
  - `4` - RTK_FIXED (~1-2cm)
  - `5` - RTK_FLOAT (~10-50cm)

### Fichier principal: `src/driver/gps.py`

### Classes

#### `GPSPosition` (dataclass)
```python
@dataclass
class GPSPosition:
    timestamp: float      # Unix timestamp
    latitude: float       # Degres (positif = Nord)
    longitude: float      # Degres (positif = Est)
    altitude: float       # Metres au-dessus du niveau de la mer
    quality: int          # Qualite du fix (0, 1, 2, 4, 5)
    satellites: int       # Nombre de satellites
    accuracy_h: float     # Precision horizontale (metres)
    accuracy_v: float     # Precision verticale (metres)
```

**Proprietes:**
- `quality_string` - Chaine lisible ("NO_FIX", "GPS", "RTK_FIXED", etc.)
- `is_rtk_fixed` - True si qualite RTK Fixed
- `is_valid` - True si fix valide (quality > 0)

#### `GPSDriver`
```python
class GPSDriver:
    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 460800)
    def start(self) -> bool           # Demarre le driver
    def stop(self)                    # Arrete le driver
    def get_position(self) -> GPSPosition  # Position actuelle
    def wait_for_fix(self, timeout: float = 30.0) -> bool
    def wait_for_rtk_fixed(self, timeout: float = 60.0) -> bool
    def set_position_callback(self, callback: Callable[[GPSPosition], None])
```

### Utilisation

```python
from driver.gps import GPSDriver

# Creer et demarrer le GPS
gps = GPSDriver('/dev/ttyUSB1')
gps.start()

# Attendre un fix
if gps.wait_for_fix(timeout=30):
    pos = gps.get_position()
    print(f"Position: {pos.latitude}, {pos.longitude}")
    print(f"Qualite: {pos.quality_string}")
    print(f"Precision: {pos.accuracy_h}m")

# Avec callback
def on_position(pos):
    print(f"Nouvelle position: {pos.latitude}, {pos.longitude}")

gps.set_position_callback(on_position)

# Arreter
gps.stop()
```

### Test
```bash
python scripts/test_gps_pointone.py --port /dev/ttyUSB1 --duration 30
```

### Options RTK (optionnel)

Si vous voulez des corrections RTK pour une precision centimetrique:

**Avec Polaris (Point One - payant):**
```python
from driver.gps_rtk import GPSRTKDriver

gps = GPSRTKDriver(
    port='/dev/ttyUSB1',
    polaris_api_key='votre_cle_api'
)
gps.start()
```

**Avec Centipede (gratuit, France):**
```python
from driver.ntrip_client import NTRIPClient

# Trouver votre base sur https://centipede.fr/index.php/view/map
client = NTRIPClient.centipede("LIENSS")  # Nom de la base proche
client.set_position(48.8566, 2.3522)
client.start()
```

---

## 2. LiDAR LD19

### Description
Driver pour le LiDAR LD19 de LDROBOT. Scanner laser 2D 360 degres.

### Specifications
- **Portee**: 0.02m a 12m
- **Resolution angulaire**: ~1 degre
- **Frequence de scan**: 5-15 Hz
- **Interface**: UART 230400 baud
- **Port serie**: `/dev/ttyUSB0` (par defaut)
- **Taille paquet**: 47 bytes, 12 points par paquet

### Fichier principal: `src/driver/lidar.py`

### Classes

#### `LidarPoint` (dataclass)
```python
@dataclass
class LidarPoint:
    angle: float      # Angle en radians
    distance: float   # Distance en metres
    intensity: int    # Intensite du retour (0-255)
    valid: bool       # True si mesure valide
```

#### `LidarScan` (dataclass)
```python
@dataclass
class LidarScan:
    points: List[LidarPoint]   # Liste des points
    timestamp: float           # Timestamp du scan
    scan_frequency: float      # Frequence du scan
```

#### `LidarDriver`
```python
class LidarDriver:
    def __init__(self, port: str, baudrate: int = 230400, buffer_size: int = 10)
    def start(self) -> bool              # Demarre le driver
    def stop(self)                       # Arrete le driver
    def get_scan(self, timeout: float = 1.0) -> LidarScan  # Scan suivant
    def get_latest_scan(self) -> LidarScan  # Dernier scan (ignore anciens)

    @staticmethod
    def get_scan_as_arrays(scan) -> Tuple[List, List]  # (angles, distances)
```

### Utilisation

```python
from driver.lidar import LidarDriver, scan_to_cartesian

# Creer et demarrer le LiDAR
lidar = LidarDriver('/dev/ttyUSB0')
if lidar.start():
    # Recuperer un scan
    scan = lidar.get_scan(timeout=1.0)
    if scan:
        # Nombre de points valides
        valid_points = sum(1 for p in scan.points if p.valid)
        print(f"Points valides: {valid_points}")

        # Convertir en coordonnees cartesiennes
        cartesian = scan_to_cartesian(scan)
        for x, y in cartesian[:5]:
            print(f"Point: x={x:.2f}m, y={y:.2f}m")

        # Trouver l'obstacle le plus proche devant
        front_points = [p for p in scan.points
                       if p.valid and -0.5 < p.angle < 0.5]
        if front_points:
            nearest = min(front_points, key=lambda p: p.distance)
            print(f"Obstacle devant: {nearest.distance:.2f}m")

lidar.stop()
```

### Test
```bash
python scripts/test_lidar.py --port /dev/ttyUSB0
```

### Mode Simulation (pour PC)

Le dossier `lidar_test/` contient un simulateur pour tester sans hardware:

```python
# Depuis le dossier Robocar
import sys
sys.path.insert(0, 'lidar_test/src')

from lidar_simulator import LidarSimulator, create_room_with_furniture

# Creer environnement simule
env = create_room_with_furniture()
simulator = LidarSimulator(env)

# Generer un scan
scan = simulator.generate_scan(480)  # 480 points
print(f"Points generes: {scan.point_count}")
```

**Environnements disponibles:**
- `create_corridor_environment()` - Couloir
- `create_parking_environment()` - Parking avec voitures
- `create_room_with_furniture()` - Piece avec meubles
- `create_obstacle_course()` - Parcours d'obstacles

### Lancer les tests simulation
```bash
python run_lidar.py --demo      # Demo rapide
python run_lidar.py --simulate  # Tests complets
python run_lidar.py --real      # Avec vrai LiDAR
```

---

## 3. Perception

### Description
Module de traitement des donnees capteurs pour la navigation autonome.

### Composants

#### 3.1 LidarProcessor (`src/perception/lidar_processor.py`)

Traitement et filtrage des scans LiDAR.

```python
from perception import LidarProcessor, ProcessedScan

# Creer le processeur
processor = LidarProcessor(
    min_range=0.05,      # Distance min (m)
    max_range=10.0,      # Distance max (m)
    min_intensity=10,    # Intensite min
    median_filter_size=3 # Taille filtre median
)

# Traiter un scan
processed = processor.process(raw_scan)

# Trouver l'obstacle le plus proche
dist, angle = processor.find_nearest_obstacle(processed)
print(f"Obstacle a {dist:.2f}m, angle {math.degrees(angle):.1f} deg")

# Obstacles dans une zone (devant, 2m max)
front_obstacles = processor.find_obstacles_in_zone(
    processed,
    angle_min=-math.pi/4,  # -45 deg
    angle_max=math.pi/4,   # +45 deg
    max_distance=2.0
)

# Segmentation (grouper points contigus)
segments = processor.segment_scan(processed)
for seg in segments:
    print(f"Segment: {len(seg.points)} pts, longueur={seg.length:.2f}m")
```

#### 3.2 ObstacleDetector (`src/perception/obstacle_detector.py`)

Detection, classification et tracking des obstacles.

```python
from perception import ObstacleDetector, Obstacle, ObstacleType

# Creer le detecteur
detector = ObstacleDetector(
    cluster_threshold=0.3,      # Distance max entre points d'un cluster
    min_cluster_points=3,       # Points min pour un obstacle
    tracking_distance=0.5,      # Distance max pour matcher obstacles
    tracking_max_age=10         # Frames avant suppression
)

# Detecter les obstacles
obstacles = detector.detect(processed_scan)

for obs in obstacles:
    print(f"Obstacle ID={obs.id}")
    print(f"  Type: {obs.obstacle_type.name}")  # STATIC, DYNAMIC, WALL, POLE
    print(f"  Distance: {obs.distance:.2f}m")
    print(f"  Angle: {math.degrees(obs.angle):.1f} deg")
    print(f"  Vitesse: {obs.speed:.2f}m/s")
    print(f"  Taille: {obs.bbox.width:.2f}x{obs.bbox.height:.2f}m")

# Obstacles proches (max 2m, devant)
danger_zone = detector.get_obstacles_in_range(
    obstacles,
    max_distance=2.0,
    angle_min=-math.pi/4,
    angle_max=math.pi/4
)

# Verifier risque de collision
is_risk, dangerous_obs, ttc = detector.check_collision_risk(
    obstacles,
    robot_velocity=(1.0, 0),  # 1 m/s vers l'avant
    time_horizon=2.0,
    safety_distance=0.3
)

if is_risk:
    print(f"DANGER! Collision dans {ttc:.1f}s avec obstacle {dangerous_obs.id}")
```

**Types d'obstacles:**
- `UNKNOWN` - Non classifie
- `STATIC` - Obstacle statique
- `DYNAMIC` - Obstacle en mouvement
- `WALL` - Mur (long et lineaire)
- `POLE` - Poteau (fin et haut)

#### 3.3 Detection avec Zones de Securite (`lidar_test/src/obstacle_detector.py`)

Pour les alertes de securite avec zones configurables.

```python
import sys
sys.path.insert(0, 'lidar_test/src')

from obstacle_detector import ObstacleDetector, SafetyZone, AlertLevel

# Creer detecteur avec zones par defaut
detector = ObstacleDetector()

# Ou personnaliser les zones
zones = [
    SafetyZone(
        name="front",
        angle_start=315, angle_end=45,     # -45 a +45 deg
        distance_warning=2.0,              # Alerte warning
        distance_danger=1.0,               # Alerte danger
        distance_critical=0.3              # Arret d'urgence
    ),
    SafetyZone(
        name="rear",
        angle_start=135, angle_end=225,
        distance_warning=1.0,
        distance_danger=0.5,
        distance_critical=0.2
    )
]
detector = ObstacleDetector(zones=zones)

# Detecter
result = detector.detect(scan)

print(f"Niveau alerte max: {result.max_alert_level.name}")
# NONE, WARNING, DANGER, CRITICAL

for zone_name, level in result.zones_status.items():
    print(f"  {zone_name}: {level.name}")

# Callback pour alertes
def on_alert(result):
    if result.max_alert_level == AlertLevel.CRITICAL:
        print("STOP!")
        motor.stop()

detector.register_alert_callback(on_alert)
```

---

## Pipeline complet

Exemple d'utilisation des trois composants ensemble:

```python
from driver.gps import GPSDriver
from driver.lidar import LidarDriver
from perception import LidarProcessor, ObstacleDetector

# Initialiser les drivers
gps = GPSDriver('/dev/ttyUSB1')
lidar = LidarDriver('/dev/ttyUSB0')
processor = LidarProcessor()
detector = ObstacleDetector()

# Demarrer
gps.start()
lidar.start()

try:
    while True:
        # Position GPS
        pos = gps.get_position()
        if pos:
            print(f"GPS: {pos.latitude:.6f}, {pos.longitude:.6f} ({pos.quality_string})")

        # Scan LiDAR
        scan = lidar.get_latest_scan()
        if scan:
            # Traiter
            processed = processor.process(scan)

            # Detecter obstacles
            obstacles = detector.detect(processed)

            # Obstacle le plus proche devant
            front = detector.get_nearest_obstacle(obstacles, -0.5, 0.5)
            if front:
                print(f"Obstacle devant: {front.min_distance:.2f}m")

                if front.min_distance < 0.5:
                    print("STOP - Obstacle trop proche!")
                    break

        time.sleep(0.1)

finally:
    gps.stop()
    lidar.stop()
```

---

## Configuration

### `config/gps.yaml`
```yaml
serial:
  port: /dev/ttyUSB1
  baudrate: 460800

polaris:
  api_key: "votre_cle_api"
  host: polaris.pointonenav.com
  port: 2101

accuracy_thresholds:
  rtk_fixed: 0.02   # 2 cm
  rtk_float: 0.5    # 50 cm
  gps: 2.5          # 2.5 m
```

### `config/robot.yaml`
```yaml
robot:
  wheelbase: 0.26   # Distance entre essieux (m)
  max_velocity: 2.0 # Vitesse max (m/s)

lidar:
  position:
    x: 0.1   # 10cm devant le centre
    y: 0.0
    z: 0.15  # 15cm de hauteur
```

---

## Depannage

### GPS ne recoit pas de signal
1. Verifier que l'antenne a une vue degagee du ciel
2. Verifier le port: `ls /dev/ttyUSB*`
3. Verifier les permissions: `sudo chmod 666 /dev/ttyUSB1`
4. Tester manuellement: `screen /dev/ttyUSB1 460800`

### LiDAR ne demarre pas
1. Verifier le port: `ls /dev/ttyUSB*`
2. Verifier l'alimentation (5V)
3. Tester: `screen /dev/ttyUSB0 230400`

### Points LiDAR invalides
1. Nettoyer la lentille du LiDAR
2. Verifier qu'il n'y a pas d'obstruction
3. Augmenter `min_intensity` dans le processeur

---

## Jetson Nano vs PC

| Composant | Jetson Nano | PC (Simulation) |
|-----------|-------------|-----------------|
| GPS | `/dev/ttyUSB1` | Non disponible |
| LiDAR | `/dev/ttyUSB0` | `run_lidar.py --simulate` |
| Perception | Temps reel | Simulation |

Pour tester sur PC avant deploiement:
```bash
# Mode simulation complete
python run_lidar.py --simulate --interactive

# Tests unitaires
python run_lidar.py --test
```
