# 🚗 LiDAR LD19 Test Suite pour Robocar

Suite de tests complète pour le LiDAR LDROBOT LD19, conçue pour le projet Robocar SLAM.

## 📦 Structure du projet

```
lidar_test_suite/
├── src/
│   ├── ld19_parser.py       # Parser du protocole LD19
│   ├── obstacle_detector.py  # Détection d'obstacles avec zones
│   ├── lidar_simulator.py    # Simulateur pour tests sans hardware
│   ├── lidar_visualizer.py   # Visualisation (polar, cartesian, zones)
│   └── lidar_test_main.py    # Script principal de test
├── tests/
│   └── test_lidar.py         # Tests unitaires
├── data/
│   └── *.json                # Scans sauvegardés
├── visualizations/
│   └── *.png                 # Images générées
└── requirements.txt
```

## 🚀 Installation

```bash
# Cloner le projet
git clone https://github.com/Robocar-Paris/Robocar.git

# Installer les dépendances
pip install -r requirements.txt

# Ou pour Jetson
pip install -r requirements-jetson.txt
```

## 📋 Dépendances

- Python 3.8+
- numpy
- matplotlib
- pyserial (pour le vrai LiDAR)

## 🔧 Utilisation

### Mode Simulation (sans hardware)

```bash
# Lancer tous les tests avec le simulateur
python src/lidar_test_main.py --simulate

# Mode interactif
python src/lidar_test_main.py --simulate --interactive
```

### Mode Réel (avec LiDAR connecté)

```bash
# Avec le LiDAR sur /dev/ttyUSB0
python src/lidar_test_main.py --port /dev/ttyUSB0

# Test spécifique
python src/lidar_test_main.py --port /dev/ttyUSB0 --test obstacles
```

### Tests Unitaires

```bash
python tests/test_lidar.py
```

## 📊 Tests Disponibles

| Test | Description |
|------|-------------|
| `connection` | Vérifie la connexion et réception de données |
| `quality` | Analyse la qualité des données (couverture, intensité) |
| `obstacles` | Teste la détection d'obstacles |
| `performance` | Mesure le FPS et la latence |
| `coverage` | Vérifie la couverture de toutes les zones |

## 🎯 Zones de Sécurité

Les zones de sécurité par défaut pour la voiture RC :

```
                    FRONT
                  ╔═══════╗
                 /   0°    \
    FRONT_LEFT /             \ FRONT_RIGHT
              /               \
      LEFT   |       🚗       |   RIGHT
              \               /
               \             /
                \   180°    /
                 ╚═════════╝
                    REAR
```

Chaque zone a 3 niveaux de distance :
- **Warning** : Obstacle détecté, vigilance
- **Danger** : Obstacle proche, ralentir
- **Critical** : Obstacle très proche, arrêt d'urgence

## 🔌 Connexion du LiDAR LD19

| Pin | Fonction | Couleur typique |
|-----|----------|-----------------|
| 1 | TX (Data) | Blanc |
| 2 | PWM | Bleu |
| 3 | VCC (5V) | Rouge |
| 4 | GND | Noir |

**Configuration série :**
- Baud rate: 230400
- Data bits: 8
- Parity: None
- Stop bits: 1

## 📈 Visualisations

Le module de visualisation génère :
- Vue cartésienne (X/Y en mètres)
- Vue polaire (style radar)
- Vue avec zones de sécurité colorées
- Histogrammes (distance, intensité)

```python
from lidar_visualizer import LidarVisualizer
from lidar_simulator import LidarSimulator, create_room_with_furniture

# Créer un scan simulé
sim = LidarSimulator(create_room_with_furniture())
scan = sim.generate_scan(480)

# Visualiser
viz = LidarVisualizer()
viz.plot_scan_cartesian(scan, save_path="scan.png")
viz.plot_with_zones(scan, detector, result, save_path="zones.png")
```

## 🧪 Environnements de Simulation

4 environnements prédéfinis :
1. **Pièce meublée** - Table, canapé, armoire, lampes
2. **Couloir** - Long et étroit
3. **Parking** - Voitures stationnées, piliers
4. **Parcours d'obstacles** - Cônes, slalom, passages étroits

```python
from lidar_simulator import (
    create_corridor_environment,
    create_parking_environment,
    create_room_with_furniture,
    create_obstacle_course
)
```

## 🛠️ Intégration avec le SLAM

Pour intégrer avec votre système SLAM :

```python
from ld19_parser import LD19Parser, LidarScan
from obstacle_detector import ObstacleDetector, EmergencyStop

# Parser pour les données série
parser = LD19Parser()

# Détecteur avec arrêt d'urgence
detector = ObstacleDetector()
emergency = EmergencyStop(detector)

emergency.set_stop_callback(lambda: motors.stop())
emergency.set_resume_callback(lambda: motors.resume())

# Boucle principale
while running:
    raw_data = serial_port.read(serial_port.in_waiting)
    packets = parser.add_data(raw_data)
    
    for packet in packets:
        scan.add_packet(packet)
    
    if scan.point_count >= 400:
        # Détection automatique avec callbacks
        result = detector.detect(scan)
        
        # Données pour le SLAM
        x, y, _ = scan.to_cartesian_arrays()
        slam.update(x, y)
        
        scan = LidarScan()  # Reset
```

## 📝 Format des Données Sauvegardées

```json
{
  "timestamp": 1706540123.456,
  "point_count": 480,
  "measurements": [
    {
      "angle_deg": 0.0,
      "distance_mm": 1234,
      "intensity": 180
    },
    ...
  ]
}
```

## 🐛 Dépannage

### Le LiDAR ne répond pas
1. Vérifier l'alimentation 5V
2. Vérifier que le port série est correct (`ls /dev/ttyUSB*`)
3. Vérifier les permissions (`sudo chmod 666 /dev/ttyUSB0`)

### Données erratiques
1. Nettoyer la lentille du LiDAR
2. Éviter les surfaces réfléchissantes directes
3. Vérifier qu'il n'y a pas de vibrations excessives

### Performances insuffisantes
1. Réduire le nombre de points par scan
2. Augmenter le seuil de clustering
3. Désactiver les zones non nécessaires

## 📜 Licence

MIT License - Robocar Paris 2024

## 👥 Contributeurs

- Équipe Robocar Paris
