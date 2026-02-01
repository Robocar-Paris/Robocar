# Documentation Robocar - Voiture RC Autonome

## Vue d'ensemble

Systeme de navigation autonome pour voiture RC avec:
- **GPS Point One RTK** - Positionnement centimetrique via Polaris
- **LiDAR LD19** - Detection d'obstacles 360 degres
- **VESC** - Controle moteur

---

## Structure du projet

```
Robocar/
├── src/
│   ├── driver/                 # Drivers hardware
│   │   ├── gps.py              # GPS Point One (NMEA)
│   │   ├── gps_rtk.py          # GPS RTK avec Polaris
│   │   ├── polaris_client.py   # Client Polaris
│   │   ├── lidar.py            # LiDAR LD19
│   │   └── vesc_motor.py       # Moteur VESC
│   │
│   ├── perception/             # Traitement capteurs
│   │   ├── lidar_processor.py  # Traitement scans
│   │   ├── obstacle_detector.py # Detection obstacles
│   │   ├── sensor_fusion.py    # Fusion EKF
│   │   └── transforms.py       # Coordonnees
│   │
│   ├── slam/                   # Cartographie
│   │   ├── slam_core.py        # SLAM principal
│   │   ├── occupancy_grid.py   # Grille occupation
│   │   └── particle_filter.py  # Filtre particules
│   │
│   ├── navigation/             # Navigation
│   │   ├── global_planner.py   # Planification A*
│   │   ├── local_planner.py    # Evitement DWA
│   │   ├── path_follower.py    # Suivi Pure Pursuit
│   │   └── waypoint_manager.py # Gestion waypoints
│   │
│   └── core/                   # Infrastructure
│       ├── state_machine.py    # Machine d'etats
│       └── safety.py           # Securite
│
├── scripts/                    # Scripts executables
│   ├── run_navigation.py       # Navigation autonome
│   ├── run_mapping.py          # Cartographie
│   ├── test_gps_pointone.py    # Test GPS
│   ├── test_gps_rtk.py         # Test GPS RTK
│   ├── test_lidar.py           # Test LiDAR
│   ├── test_all_sensors.py     # Test tous capteurs
│   ├── test_diagnostic.py      # Diagnostic complet
│   ├── demo_perception.py      # Demo perception
│   ├── demo_slam.py            # Demo SLAM
│   └── demo_navigation.py      # Demo navigation
│
├── config/
│   ├── robot.yaml              # Config robot
│   ├── navigation.yaml         # Config navigation
│   └── gps.yaml                # Config GPS + Polaris
│
└── docs/                       # Documentation
```

---

## 1. GPS Point One RTK

### Fichiers
- `gps.py` - Driver de base (parsing NMEA)
- `gps_rtk.py` - Driver RTK avec corrections Polaris
- `polaris_client.py` - Client service Polaris

### Configuration (`config/gps.yaml`)
```yaml
gps:
  port: "/dev/ttyUSB0"
  baudrate: 460800

polaris:
  api_key: "030ade8ca4f04445ba0a7bc20f5e53be"
  host: "polaris.pointonenav.com"
  port: 2101
```

### Utilisation
```python
from driver import GPSDriver, GPSRTKDriver

# Mode simple
gps = GPSDriver('/dev/ttyUSB0')
gps.start()
pos = gps.get_position()
print(f"{pos.latitude}, {pos.longitude} - {pos.quality_string}")

# Mode RTK (precision cm)
gps = GPSRTKDriver('/dev/ttyUSB0', polaris_api_key='votre_cle')
gps.start()
gps.wait_for_rtk_fixed(timeout=120)
```

### Test
```bash
python scripts/test_gps_pointone.py
python scripts/test_gps_rtk.py
```

---

## 2. LiDAR LD19

### Fichier: `lidar.py`

### Specifications
- Portee: 0.02m - 12m
- Resolution: ~1 degre
- Frequence: 5-15 Hz
- Port: `/dev/ttyUSB1`
- Baudrate: 230400

### Utilisation
```python
from driver import LidarDriver

lidar = LidarDriver('/dev/ttyUSB1')
lidar.start()

scan = lidar.get_scan()
for point in scan.points:
    if point.valid:
        print(f"Angle: {point.angle:.2f}, Distance: {point.distance:.2f}m")

lidar.stop()
```

### Test
```bash
python scripts/test_lidar.py
```

---

## 3. Perception

### LidarProcessor
Filtrage et traitement des scans LiDAR.

```python
from perception import LidarProcessor

processor = LidarProcessor(min_range=0.05, max_range=10.0)
processed = processor.process(scan)
dist, angle = processor.find_nearest_obstacle(processed)
```

### ObstacleDetector
Detection et tracking des obstacles.

```python
from perception import ObstacleDetector

detector = ObstacleDetector()
obstacles = detector.detect(processed_scan)

for obs in obstacles:
    print(f"Obstacle: {obs.distance:.2f}m, type={obs.obstacle_type.name}")
```

---

## 4. Navigation

### Scripts principaux

**Navigation autonome:**
```bash
python scripts/run_navigation.py
```

**Cartographie:**
```bash
python scripts/run_mapping.py
```

**Test complet:**
```bash
python scripts/test_diagnostic.py
```

---

## 5. Configuration

### `config/robot.yaml`
```yaml
robot:
  wheelbase: 0.26        # Distance entre essieux (m)
  max_velocity: 2.0      # Vitesse max (m/s)
  max_steering: 0.5      # Angle braquage max (rad)

lidar:
  position:
    x: 0.1               # Position LiDAR
    y: 0.0
    z: 0.15
```

### `config/navigation.yaml`
Parametres SLAM, planification, controle.

---

## 6. Ports serie

| Composant | Port | Baudrate |
|-----------|------|----------|
| GPS | /dev/ttyUSB0 | 460800 |
| LiDAR | /dev/ttyUSB1 | 230400 |
| VESC | /dev/ttyACM0 | 115200 |

---

## 7. Depannage

### GPS pas de signal
```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
screen /dev/ttyUSB0 460800
```

### LiDAR pas de donnees
```bash
sudo chmod 666 /dev/ttyUSB1
screen /dev/ttyUSB1 230400
```

### VESC pas de connexion
```bash
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0
```
