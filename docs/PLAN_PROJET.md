# Plan du Projet Robocar

## Objectif Final
Voiture RC autonome capable de naviguer d'un point A a un point B en detectant et contournant les obstacles.

---

## Etat Actuel

### Ce qui est FAIT

| Composant | Status | Description |
|-----------|--------|-------------|
| **Driver GPS** | OK | `gps.py`, `gps_rtk.py` - Point One RTK via Polaris |
| **Driver LiDAR** | OK | `lidar.py` - LD19 360 degres |
| **Driver VESC** | OK | `vesc_motor.py` - Controle moteur |
| **Perception** | OK | `lidar_processor.py`, `obstacle_detector.py` |
| **Simulation** | OK | `LidarSimulator`, `GPSSimulator` pour tests PC |
| **Script voiture** | OK | `run_car.py` - Navigation basique |
| **Script simulation** | OK | `run_simulation.py` - Test sur PC |

### Ce qui FONCTIONNE (teste)
- GPS recoit les coordonnees (capture d'ecran)
- LiDAR detecte les obstacles

---

## Ce qu'il RESTE A FAIRE

### Phase 1: Integration Hardware (Priorite HAUTE)

#### 1.1 Calibration LiDAR
- [ ] Tester detection obstacles en conditions reelles
- [ ] Ajuster `min_intensity` si trop de faux positifs
- [ ] Verifier orientation (angle 0 = devant)
- [ ] Calibrer distance min/max

**Fichier:** `src/driver/lidar.py`

#### 1.2 Calibration GPS RTK
- [ ] Tester avec antenne EPITA + Polaris
- [ ] Verifier precision RTK FIXED (~2cm)
- [ ] Mesurer temps de convergence RTK
- [ ] Tester stabilite du signal

**Fichier:** `scripts/test_gps_rtk.py`

#### 1.3 Calibration Moteur VESC
- [ ] Calibrer duty cycle vs vitesse reelle
- [ ] Calibrer servo vs angle de braquage
- [ ] Tester rampe acceleration/freinage
- [ ] Definir limites de securite

**Fichier:** `src/driver/vesc_motor.py`

---

### Phase 2: Navigation (Priorite HAUTE)

#### 2.1 Odometrie / Estimation de pose
- [ ] Implementer fusion GPS + odometrie VESC
- [ ] Ajouter compas/IMU pour heading (actuellement estime)
- [ ] Calibrer modele cinematique Ackermann

**Fichiers:**
- `src/perception/sensor_fusion.py` (existe, a integrer)
- `src/navigation/path_follower.py`

#### 2.2 Controleur de suivi de trajectoire
- [ ] Tester Pure Pursuit sur voiture reelle
- [ ] Ajuster gain de direction (`STEERING_GAIN`)
- [ ] Tester Stanley controller comme alternative

**Fichier:** `src/navigation/path_follower.py`

#### 2.3 Evitement d'obstacles
- [ ] Tester evitement basique (tourner a gauche/droite)
- [ ] Implementer DWA (Dynamic Window Approach) pour evitement fluide
- [ ] Ajouter planification locale si obstacle bloquant

**Fichier:** `src/navigation/local_planner.py`

---

### Phase 3: Interface Utilisateur (Priorite MOYENNE)

#### 3.1 Definition des waypoints
- [ ] Interface pour definir point A et B sur carte
- [ ] Format fichier waypoints (GPS ou local)
- [ ] Conversion GPS <-> coordonnees locales

**A creer:** Interface simple (fichier texte ou app mobile)

#### 3.2 Monitoring temps reel
- [ ] Affichage position GPS en temps reel
- [ ] Affichage scan LiDAR
- [ ] Logs et alerts

**Option:** Page web simple ou app terminal

---

### Phase 4: Robustesse (Priorite MOYENNE)

#### 4.1 Gestion des erreurs
- [ ] Perte signal GPS -> arret
- [ ] Perte LiDAR -> arret
- [ ] Perte VESC -> arret
- [ ] Timeout waypoint -> abandon

**Fichier:** `src/core/safety.py`

#### 4.2 Arret d'urgence
- [ ] Bouton physique arret urgence
- [ ] Arret si obstacle < 30cm
- [ ] Arret si hors zone autorisee

#### 4.3 Tests terrain
- [ ] Test en ligne droite
- [ ] Test avec virage
- [ ] Test avec obstacle fixe
- [ ] Test avec obstacle mobile (personne)

---

### Phase 5: Optimisation (Priorite BASSE)

#### 5.1 Performance
- [ ] Optimiser frequence boucle controle (actuellement ~10Hz)
- [ ] Reduire latence GPS -> commande
- [ ] Profiler sur Jetson Nano

#### 5.2 SLAM (optionnel)
- [ ] Cartographie environnement
- [ ] Localisation sans GPS (indoor)

**Fichiers:** `src/slam/` (existe, a integrer si besoin)

---

## Ordre de Priorite

```
1. [MAINTENANT] Tester GPS RTK avec Polaris sur terrain
2. [MAINTENANT] Tester LiDAR detection obstacles
3. [SEMAINE 1]  Calibrer VESC (vitesse, direction)
4. [SEMAINE 1]  Premier test navigation simple
5. [SEMAINE 2]  Ajuster controleur Pure Pursuit
6. [SEMAINE 2]  Tester evitement obstacles
7. [SEMAINE 3]  Integration complete
8. [SEMAINE 3]  Tests terrain intensifs
```

---

## Commandes Utiles

### Test GPS
```bash
python scripts/test_gps_pointone.py --port /dev/ttyUSB0
python scripts/test_gps_rtk.py --port /dev/ttyUSB0
```

### Test LiDAR
```bash
python scripts/test_lidar.py --port /dev/ttyUSB1
```

### Simulation PC
```bash
python scripts/run_simulation.py --env parking --visualize
python scripts/run_simulation.py --env corridor --visualize
```

### Voiture reelle
```bash
# Navigation vers un point
python scripts/run_car.py --target-lat 48.8970 --target-lon 2.2195

# Navigation avec fichier waypoints
python scripts/run_car.py --waypoints waypoints.txt
```

---

## Architecture Finale

```
                    +------------------+
                    |   run_car.py     |  <-- Script principal
                    +------------------+
                            |
            +---------------+---------------+
            |               |               |
    +-------v------+ +------v------+ +------v------+
    |   GPS RTK    | |   LiDAR     | |    VESC     |
    |   Driver     | |   Driver    | |   Driver    |
    +--------------+ +-------------+ +-------------+
            |               |               |
            v               v               v
    +---------------+ +-------------+
    | Sensor Fusion | | Perception  |
    | (GPS + Odom)  | | (Obstacles) |
    +---------------+ +-------------+
            |               |
            +-------+-------+
                    |
            +-------v-------+
            |  Navigation   |
            | (Path Follow) |
            +---------------+
                    |
                    v
            +---------------+
            |   Commandes   |
            | (Vitesse+Dir) |
            +---------------+
```

---

## Risques et Solutions

| Risque | Probabilite | Solution |
|--------|-------------|----------|
| GPS perd le fix | Moyenne | Arret securite + attente |
| Obstacle non detecte | Faible | Reduire min_intensity, ajouter camera |
| Derive odometrie | Haute | Fusion GPS obligatoire |
| Latence trop haute | Moyenne | Optimiser code, reduire frequence |
| Batterie faible | Moyenne | Monitoring tension VESC |

---

## Checklist Pre-Test Terrain

- [ ] Batterie chargee
- [ ] Antenne GPS vue du ciel
- [ ] LiDAR propre et degage
- [ ] Zone de test securisee
- [ ] Bouton arret urgence accessible
- [ ] Waypoints definis et valides
- [ ] Laptop pour monitoring
