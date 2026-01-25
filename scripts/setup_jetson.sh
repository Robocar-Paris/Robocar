#!/bin/bash
# =============================================================================
# Robocar - Jetson Nano Setup Script
# =============================================================================
# Ce script configure l'environnement Jetson Nano pour le projet Robocar.
#
# IMPORTANT: NE PAS faire "sudo apt upgrade" sur Jetson Nano car cela peut
# casser les drivers NVIDIA et CUDA.
#
# Usage: chmod +x setup_jetson.sh && ./setup_jetson.sh
# =============================================================================

set -e

echo "=============================================="
echo "   Robocar - Jetson Nano Setup"
echo "=============================================="

# Couleurs pour les messages
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Vérifier qu'on est sur une Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo -e "${RED}ERREUR: Ce script est conçu pour Jetson Nano uniquement${NC}"
    exit 1
fi

echo -e "${GREEN}Jetson Nano détectée${NC}"
cat /etc/nv_tegra_release

# =============================================================================
# Étape 1: Protéger les paquets NVIDIA contre les upgrades accidentels
# =============================================================================
echo ""
echo -e "${YELLOW}[1/6] Protection des paquets NVIDIA...${NC}"

sudo apt-mark hold nvidia-* cuda-* libcudnn* libnv* 2>/dev/null || true
echo -e "${GREEN}Paquets NVIDIA protégés contre les upgrades${NC}"

# =============================================================================
# Étape 2: Mise à jour sécurisée (sans upgrade des paquets système)
# =============================================================================
echo ""
echo -e "${YELLOW}[2/6] Mise à jour des listes de paquets...${NC}"

sudo apt update

# =============================================================================
# Étape 3: Installation des dépendances système
# =============================================================================
echo ""
echo -e "${YELLOW}[3/6] Installation des dépendances système...${NC}"

sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-yaml \
    python3-serial \
    libopenblas-base \
    libopenblas-dev \
    libjpeg-dev \
    zlib1g-dev \
    libpython3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

echo -e "${GREEN}Dépendances système installées${NC}"

# =============================================================================
# Étape 4: Configuration des ports série (LiDAR, GPS, VESC)
# =============================================================================
echo ""
echo -e "${YELLOW}[4/6] Configuration des permissions série...${NC}"

# Ajouter l'utilisateur au groupe dialout pour accès aux ports série
sudo usermod -a -G dialout $USER

# Créer les règles udev pour les capteurs
sudo tee /etc/udev/rules.d/99-robocar-sensors.rules > /dev/null << 'EOF'
# LiDAR LD19 - généralement sur ttyUSB0
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"

# GPS Point One - généralement sur ttyUSB1 ou ttyACM0
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"

# VESC Motor Controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="vesc", MODE="0666"

# Règle générique pour tous les USB-série
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", MODE="0666"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "${GREEN}Règles udev créées. Les capteurs seront accessibles via /dev/lidar, /dev/gps, /dev/vesc${NC}"

# =============================================================================
# Étape 5: Installation des dépendances Python
# =============================================================================
echo ""
echo -e "${YELLOW}[5/6] Installation des dépendances Python...${NC}"

# Upgrade pip
python3 -m pip install --upgrade pip

# Installer les dépendances Python (sauf PyTorch qui nécessite une version spéciale)
pip3 install --user \
    pyserial \
    pyvesc \
    pyyaml \
    breezyslam \
    dataclasses

echo -e "${GREEN}Dépendances Python installées${NC}"

# =============================================================================
# Étape 6: Installation de PyTorch pour Jetson
# =============================================================================
echo ""
echo -e "${YELLOW}[6/6] Installation de PyTorch pour Jetson...${NC}"

# Déterminer la version de JetPack
JETPACK_VERSION=$(cat /etc/nv_tegra_release | grep -oP 'R\d+' | head -1)

echo "Version L4T détectée: $JETPACK_VERSION"

# PyTorch pour Jetson doit être installé depuis les wheels NVIDIA
# https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

if [ "$JETPACK_VERSION" = "R32" ]; then
    echo "JetPack 4.x détecté (L4T R32)"
    echo ""
    echo -e "${YELLOW}Pour installer PyTorch sur JetPack 4.x:${NC}"
    echo ""
    echo "# PyTorch 1.10 pour JetPack 4.6:"
    echo "wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl"
    echo "pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl"
    echo ""
    echo "# TorchVision compatible:"
    echo "sudo apt install -y libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev"
    echo "git clone --branch v0.11.1 https://github.com/pytorch/vision torchvision"
    echo "cd torchvision && python3 setup.py install --user"
elif [ "$JETPACK_VERSION" = "R35" ]; then
    echo "JetPack 5.x détecté (L4T R35)"
    echo ""
    echo -e "${YELLOW}Pour installer PyTorch sur JetPack 5.x:${NC}"
    echo ""
    echo "# PyTorch 2.0 pour JetPack 5.1:"
    echo "wget https://nvidia.box.com/shared/static/i8pukc49h3lhak4kkn67tg9j4goqm0m7.whl -O torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
    echo "pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
fi

echo ""
echo -e "${YELLOW}Note: L'installation de PyTorch est manuelle car elle dépend de ta version JetPack.${NC}"
echo "Consulte: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"

# =============================================================================
# Résumé
# =============================================================================
echo ""
echo "=============================================="
echo -e "${GREEN}   Installation terminée !${NC}"
echo "=============================================="
echo ""
echo "Prochaines étapes:"
echo "  1. Redémarre pour appliquer les permissions: sudo reboot"
echo "  2. Installe PyTorch manuellement (voir instructions ci-dessus)"
echo "  3. Vérifie les capteurs: ls -la /dev/ttyUSB* /dev/ttyACM*"
echo "  4. Teste le LiDAR: python3 scripts/test_lidar.py"
echo "  5. Teste le GPS: python3 scripts/test_gps.py"
echo ""
echo -e "${YELLOW}IMPORTANT: Ne jamais faire 'sudo apt upgrade' sur Jetson !${NC}"
echo ""
