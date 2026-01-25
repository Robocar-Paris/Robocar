# Robocar - Installation sur Jetson Nano

Ce guide explique comment configurer le projet Robocar sur une Jetson Nano 4GB.

## Prérequis

- Jetson Nano 4GB Developer Kit
- Carte microSD 64GB minimum (recommandé: 128GB)
- JetPack 4.6 ou supérieur

## Pourquoi `apt upgrade` casse tout ?

La Jetson Nano utilise **L4T (Linux for Tegra)**, une version spéciale d'Ubuntu 18.04 avec des drivers NVIDIA personnalisés. Quand tu fais `apt upgrade`:

1. Les paquets Ubuntu standard **écrasent** les versions NVIDIA
2. Les drivers CUDA et GPU sont corrompus
3. La carte ne boot plus correctement

**Solution**: Ne jamais faire `apt upgrade` sans protéger les paquets NVIDIA.

```bash
# Protéger les paquets NVIDIA
sudo apt-mark hold nvidia-* cuda-* libcudnn* libnv*

# Maintenant tu peux mettre à jour en sécurité
sudo apt update
```

## Installation Automatique

```bash
cd ~/Robocar
chmod +x scripts/setup_jetson.sh
./scripts/setup_jetson.sh
```

## Installation Manuelle

### Étape 1: Protéger les paquets NVIDIA

```bash
sudo apt-mark hold nvidia-* cuda-* libcudnn* libnv*
```

### Étape 2: Installer les dépendances système

```bash
sudo apt update

sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-yaml \
    python3-serial \
    libopenblas-base
```

### Étape 3: Configurer les ports série

```bash
# Ajouter l'utilisateur au groupe dialout
sudo usermod -a -G dialout $USER

# Redémarrer pour appliquer
sudo reboot
```

### Étape 4: Installer les dépendances Python

```bash
pip3 install --user -r requirements-jetson.txt
```

### Étape 5: Installer PyTorch (spécifique Jetson)

PyTorch doit être installé depuis les wheels officielles NVIDIA:

```bash
# Télécharger PyTorch 1.10 pour JetPack 4.6
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl \
    -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl

# Installer
pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl

# Compiler TorchVision
sudo apt install -y libjpeg-dev zlib1g-dev
git clone --branch v0.11.1 https://github.com/pytorch/vision torchvision
cd torchvision
python3 setup.py install --user
```

## Connexion des Capteurs

### LiDAR LD19

| Pin LiDAR | Connexion |
|-----------|-----------|
| VCC (5V)  | Pin 2 (5V) ou USB 5V |
| GND       | Pin 6 (GND) |
| TX        | USB-UART RX |
| PWM       | Optionnel |

**Port série**: `/dev/ttyUSB0` à 230400 bauds

### GPS RTK Point One

| Pin GPS | Connexion |
|---------|-----------|
| VCC     | 5V |
| GND     | GND |
| TX      | USB-UART RX |
| RX      | USB-UART TX |

**Port série**: `/dev/ttyUSB1` ou `/dev/ttyACM0` à 115200 bauds

### VESC Motor Controller

Connexion via USB directement.

**Port série**: `/dev/ttyACM0` ou `/dev/ttyUSB2`

## Vérification de l'Installation

```bash
# Vérifier les ports série
ls -la /dev/ttyUSB* /dev/ttyACM*

# Tester le LiDAR
python3 scripts/test_lidar.py --port /dev/ttyUSB0

# Tester le GPS
python3 scripts/test_gps.py --port /dev/ttyUSB1

# Tester tous les capteurs
python3 scripts/test_all_sensors.py
```

## Optimisation Performances

### Activer le mode performance

```bash
# Mode performance maximum
sudo nvpmodel -m 0
sudo jetson_clocks

# Vérifier le mode
sudo nvpmodel -q
```

### Augmenter la mémoire swap

```bash
# Créer un fichier swap de 4GB
sudo fallocate -l 4G /var/swapfile
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile

# Rendre permanent
echo '/var/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```

## Problèmes Courants

### Erreur: "Permission denied" sur /dev/ttyUSB*

```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

### Erreur: "No module named 'cv2'"

```bash
# Utiliser la version système (pas pip)
sudo apt install python3-opencv
```

### Erreur: PyTorch ne trouve pas CUDA

```bash
# Vérifier CUDA
nvcc --version

# Vérifier que PyTorch voit CUDA
python3 -c "import torch; print(torch.cuda.is_available())"
```

### Mémoire insuffisante

```bash
# Fermer l'interface graphique
sudo systemctl set-default multi-user.target
sudo reboot

# Pour revenir à l'interface graphique
sudo systemctl set-default graphical.target
```

## Architecture du Projet sur Jetson

```
Robocar/
├── src/
│   ├── driver/          # Drivers capteurs (LiDAR, GPS, VESC)
│   ├── perception/      # Traitement données capteurs
│   ├── slam/            # Cartographie et localisation
│   ├── navigation/      # Planification de chemin (à compléter)
│   └── control/         # Contrôle moteur (à compléter)
├── config/              # Fichiers de configuration YAML
├── scripts/             # Scripts de test et démos
└── docs/                # Documentation
```

## Ressources

- [PyTorch pour Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Jetson Nano Getting Started](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)
- [BreezySLAM](https://github.com/simondlevy/BreezySLAM)