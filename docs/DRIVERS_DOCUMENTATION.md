# Documentation Drivers et Perception - Robocar

## Vue d'ensemble

Cette documentation couvre les trois composants principaux du systeme de perception de la voiture RC:

1. **GPS Point One RTK** - Positionnement GPS RTK centimetrique via Polaris
2. **LiDAR LD19** - Detection d'obstacles 360 degres
3. **Perception** - Traitement des donnees capteurs et detection d'obstacles

---

## Architecture des fichiers

```
Robocar/
├── src/
│   ├── driver/                    # Drivers hardware
│   │   ├── gps.py                 # Driver GPS Point One (parsing NMEA)
│   │   ├── gps_rtk.py             # GPS avec corrections RTK Polaris
│   │   ├── polaris_client.py      # Client Polaris pour RTK
│   │   ├── lidar.py               # Driver LiDAR LD19
│   │   └── vesc_motor.py          # Controleur moteur VESC
│   │
│   └── perception/                # Traitement perception
│       ├── lidar_processor.py     # Traitement scans LiDAR
│       ├── obstacle_detector.py   # Detection et tracking obstacles
│       ├── sensor_fusion.py       # Fusion capteurs (EKF)
│       └── transforms.py          # Transformations coordonnees
│
├── config/
│   ├── gps.yaml                   # Configuration GPS + Polaris
│   └── robot.yaml                 # Configuration robot
│
└── scripts/
    ├── test_gps_pointone.py       # Test GPS Point One
    ├── test_gps_rtk.py            # Test GPS RTK avec Polaris
    └── test_lidar.py              # Test LiDAR
```

---

## 1. GPS Point One RTK

### Description
Driver pour le GPS RTK Point One Navigation avec corrections Polaris.
- `gps.py` - Parse les messages NMEA (GGA pour position, GST pour precision)
- `gps_rtk.py` - Integre les corrections RTK via Polaris
- `polaris_client.py` - Client pour le service Polaris

### Specifications
- **Port serie**: `/dev/ttyUSB0` (configurable dans gps.yaml)
- **Baudrate**: 460800
- **Protocole**: NMEA 0183
- **Service RTK**: Polaris (Point One Navigation)
- **Niveaux de qualite**:
  - `0` - NO_FIX (pas de fix)
  - `1` - GPS (~2-5m de precision)
  - `2` - DGPS (~0.5-2m)
  - `4` - RTK_FIXED (~1-2cm)
  - `5` - RTK_FLOAT (~10-50cm)

### Configuration (`config/gps.yaml`)
```yaml
gps:
  port: "/dev/ttyUSB0"
  baudrate: 460800

polaris:
  api_key: "votre_cle_api"    # Cle API Polaris
  device_id: 67
  host: "polaris.pointonenav.com"
  port: 2101

initial_position:
  latitude: 48.8968           # Position EPITA
  longitude: 2.2190
  altitude: 35.0
```

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

#### `GPSDriver` (gps.py)
```python
class GPSDriver:
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 460800)
    def start(self) -> bool
    def stop(self)
    def get_position(self) -> GPSPosition
    def wait_for_fix(self, timeout: float = 30.0) -> bool
    def wait_for_rtk_fixed(self, timeout: float = 60.0) -> bool
    def set_position_callback(self, callback: Callable[[GPSPosition], None])
```

#### `GPSRTKDriver` (gps_rtk.py)
```python
class GPSRTKDriver:
    def __init__(self, port: str, polaris_api_key: str, ...)
    def start(self) -> bool
    def stop(self)
    def get_position(self) -> GPSPosition
    # Integre automatiquement les corrections Polaris
```

### Utilisation

**Mode simple (sans RTK):**
```python
from driver.gps import GPSDriver

gps = GPSDriver('/dev/ttyUSB0')
gps.start()

if gps.wait_for_fix(timeout=30):
    pos = gps.get_position()
    print(f"Position: {pos.latitude}, {pos.longitude}")
    print(f"Qualite: {pos.quality_string}")

gps.stop()
```

**Mode RTK avec Polaris (precision centimetrique):**
```python
from driver.gps_rtk import GPSRTKDriver

# Charger config depuis gps.yaml
gps = GPSRTKDriver(
    port='/dev/ttyUSB0',
    polaris_api_key='030ade8ca4f04445ba0a7bc20f5e53be'
)
gps.start()

# Attendre RTK Fixed
if gps.wait_for_rtk_fixed(timeout=120):
    pos = gps.get_position()
    print(f"RTK FIXED! Precision: {pos.accuracy_h}m")

gps.stop()
```

### Test
```bash
# Test GPS simple
python scripts/test_gps_pointone.py --port /dev/ttyUSB0

# Test GPS RTK avec Polaris
python scripts/test_gps_rtk.py --port /dev/ttyUSB0
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
- **Port serie**: `/dev/ttyUSB1` (par defaut)
- **Taille paquet**: 47 bytes, 12 points par paquet

### Fichier: `src/driver/lidar.py`

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
    points: List[LidarPoint]
    timestamp: float
    scan_frequency: float
```

#### `LidarDriver`
```python
class LidarDriver:
    def __init__(self, port: str, baudrate: int = 230400, buffer_size: int = 10)
    def start(self) -> bool
    def stop(self)
    def get_scan(self, timeout: float = 1.0) -> LidarScan
    def get_latest_scan(self) -> LidarScan

    @staticmethod
    def get_scan_as_arrays(scan) -> Tuple[List, List]  # (angles, distances)
```

### Utilisation

```python
from driver.lidar import LidarDriver

lidar = LidarDriver('/dev/ttyUSB1')
if lidar.start():
    scan = lidar.get_scan(timeout=1.0)
    if scan:
        valid_points = sum(1 for p in scan.points if p.valid)
        print(f"Points valides: {valid_points}")

        # Obstacle le plus proche devant (-45 a +45 deg)
        front_points = [p for p in scan.points
                       if p.valid and -0.78 < p.angle < 0.78]
        if front_points:
            nearest = min(front_points, key=lambda p: p.distance)
            print(f"Obstacle devant: {nearest.distance:.2f}m")

lidar.stop()
```

### Test
```bash
python scripts/test_lidar.py --port /dev/ttyUSB1
```

---

## 3. Perception

### Description
Module de traitement des donnees capteurs pour la navigation autonome.

### 3.1 LidarProcessor (`src/perception/lidar_processor.py`)

Traitement et filtrage des scans LiDAR.

```python
from perception import LidarProcessor

processor = LidarProcessor(
    min_range=0.05,      # Distance min (m)
    max_range=10.0,      # Distance max (m)
    min_intensity=10,    # Intensite min
    median_filter_size=3
)

# Traiter un scan
processed = processor.process(raw_scan)

# Trouver l'obstacle le plus proche
dist, angle = processor.find_nearest_obstacle(processed)

# Obstacles dans une zone (devant)
front_obstacles = processor.find_obstacles_in_zone(
    processed,
    angle_min=-0.78,  # -45 deg
    angle_max=0.78,   # +45 deg
    max_distance=2.0
)

# Segmentation (grouper points contigus)
segments = processor.segment_scan(processed)
```

### 3.2 ObstacleDetector (`src/perception/obstacle_detector.py`)

Detection, classification et tracking des obstacles.

```python
from perception import ObstacleDetector, ObstacleType

detector = ObstacleDetector(
    cluster_threshold=0.3,
    min_cluster_points=3,
    tracking_distance=0.5,
    tracking_max_age=10
)

obstacles = detector.detect(processed_scan)

for obs in obstacles:
    print(f"Obstacle ID={obs.id}")
    print(f"  Type: {obs.obstacle_type.name}")  # STATIC, DYNAMIC, WALL, POLE
    print(f"  Distance: {obs.distance:.2f}m")
    print(f"  Vitesse: {obs.speed:.2f}m/s")

# Verifier risque de collision
is_risk, dangerous_obs, ttc = detector.check_collision_risk(
    obstacles,
    robot_velocity=(1.0, 0),
    time_horizon=2.0,
    safety_distance=0.3
)
```

**Types d'obstacles:**
- `STATIC` - Obstacle statique
- `DYNAMIC` - Obstacle en mouvement
- `WALL` - Mur (long et lineaire)
- `POLE` - Poteau (fin)

---

## Pipeline complet pour la voiture RC

```python
import time
from driver import GPSRTKDriver, LidarDriver, VESCController
from perception import LidarProcessor, ObstacleDetector

# Initialiser les drivers
gps = GPSRTKDriver('/dev/ttyUSB0', polaris_api_key='votre_cle')
lidar = LidarDriver('/dev/ttyUSB1')
motor = VESCController('/dev/ttyACM0')
processor = LidarProcessor()
detector = ObstacleDetector()

# Demarrer
gps.start()
lidar.start()
motor.start()

try:
    while True:
        # Position GPS RTK
        pos = gps.get_position()
        if pos and pos.is_valid:
            print(f"GPS: {pos.latitude:.6f}, {pos.longitude:.6f} ({pos.quality_string})")

        # Scan LiDAR
        scan = lidar.get_latest_scan()
        if scan:
            processed = processor.process(scan)
            obstacles = detector.detect(processed)

            # Verifier obstacles devant
            front = detector.get_nearest_obstacle(obstacles, -0.5, 0.5)
            if front and front.min_distance < 0.5:
                print("STOP - Obstacle!")
                motor.set_duty_cycle(0)
            else:
                motor.set_duty_cycle(0.1)  # Avancer doucement

        time.sleep(0.1)

finally:
    motor.set_duty_cycle(0)
    gps.stop()
    lidar.stop()
    motor.stop()
```

---

## Configuration

### `config/gps.yaml`
```yaml
gps:
  port: "/dev/ttyUSB0"
  baudrate: 460800

polaris:
  api_key: "030ade8ca4f04445ba0a7bc20f5e53be"
  device_id: 67
  host: "polaris.pointonenav.com"
  port: 2101

initial_position:
  latitude: 48.8968    # EPITA
  longitude: 2.2190
  altitude: 35.0
```

### `config/robot.yaml`
```yaml
robot:
  wheelbase: 0.26
  max_velocity: 2.0

lidar:
  position:
    x: 0.1
    y: 0.0
    z: 0.15
```

---

## Depannage

### GPS ne recoit pas de signal
1. Verifier que l'antenne a une vue degagee du ciel
2. Verifier le port: `ls /dev/ttyUSB*`
3. Permissions: `sudo chmod 666 /dev/ttyUSB0`
4. Test manuel: `screen /dev/ttyUSB0 460800`

### GPS RTK ne passe pas en RTK Fixed
1. Verifier la cle API Polaris dans `config/gps.yaml`
2. Verifier la connexion internet
3. Attendre 1-2 minutes (convergence RTK)

### LiDAR ne demarre pas
1. Verifier le port: `ls /dev/ttyUSB*`
2. Verifier l'alimentation (5V)
3. Test: `screen /dev/ttyUSB1 230400`

### Points LiDAR invalides
1. Nettoyer la lentille du LiDAR
2. Verifier qu'il n'y a pas d'obstruction
3. Augmenter `min_intensity` dans le processeur
