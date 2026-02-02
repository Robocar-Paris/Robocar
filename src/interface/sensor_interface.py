"""
Interfaces abstraites pour les capteurs et actionneurs.

Ces interfaces definissent le contrat que doivent respecter
les implementations reelles (hardware) et simulees.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Callable
import time


@dataclass
class LidarPoint:
    """Point de mesure LiDAR."""
    angle: float        # Angle en radians
    distance: float     # Distance en metres
    intensity: int = 0  # Intensite (0-255)
    valid: bool = True  # Point valide


@dataclass
class LidarData:
    """Scan LiDAR complet."""
    points: List[LidarPoint] = field(default_factory=list)
    timestamp: float = 0.0
    scan_frequency: float = 10.0

    @property
    def valid_points(self) -> List[LidarPoint]:
        """Retourne uniquement les points valides."""
        return [p for p in self.points if p.valid]


@dataclass
class GPSData:
    """Position GPS."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    heading: float = 0.0           # Cap en radians
    speed: float = 0.0             # Vitesse en m/s
    accuracy_h: float = 100.0      # Precision horizontale (m)
    accuracy_v: float = 100.0      # Precision verticale (m)
    satellites: int = 0
    quality: int = 0               # 0=no fix, 1=GPS, 4=RTK fixed, 5=RTK float
    timestamp: float = 0.0
    is_valid: bool = False

    @property
    def is_rtk_fixed(self) -> bool:
        """RTK fixe (precision centimetrique)."""
        return self.quality == 4

    @property
    def quality_string(self) -> str:
        """Description de la qualite."""
        qualities = {
            0: "No Fix",
            1: "GPS",
            2: "DGPS",
            4: "RTK Fixed",
            5: "RTK Float",
        }
        return qualities.get(self.quality, f"Unknown ({self.quality})")


class ILidarSensor(ABC):
    """Interface abstraite pour capteur LiDAR."""

    @abstractmethod
    def start(self) -> bool:
        """Demarre le capteur. Retourne True si succes."""
        pass

    @abstractmethod
    def stop(self):
        """Arrete le capteur."""
        pass

    @abstractmethod
    def get_scan(self) -> Optional[LidarData]:
        """Retourne le dernier scan complet."""
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """Indique si le capteur est actif."""
        pass


class IGPSSensor(ABC):
    """Interface abstraite pour capteur GPS."""

    @abstractmethod
    def start(self) -> bool:
        """Demarre le capteur. Retourne True si succes."""
        pass

    @abstractmethod
    def stop(self):
        """Arrete le capteur."""
        pass

    @abstractmethod
    def get_position(self) -> Optional[GPSData]:
        """Retourne la position actuelle."""
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """Indique si le capteur est actif."""
        pass

    def wait_for_fix(self, timeout: float = 60.0) -> bool:
        """
        Attend un fix GPS valide.

        Args:
            timeout: Temps max d'attente en secondes

        Returns:
            True si fix obtenu, False si timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            pos = self.get_position()
            if pos and pos.is_valid:
                return True
            time.sleep(0.5)
        return False


class IMotorController(ABC):
    """Interface abstraite pour controleur moteur."""

    @abstractmethod
    def start(self) -> bool:
        """Initialise le controleur. Retourne True si succes."""
        pass

    @abstractmethod
    def stop(self):
        """Arrete le controleur (et coupe les moteurs)."""
        pass

    @abstractmethod
    def set_speed(self, speed: float):
        """
        Definit la vitesse.

        Args:
            speed: Vitesse normalisee [-1.0, 1.0]
                   Positif = avant, negatif = arriere
        """
        pass

    @abstractmethod
    def set_steering(self, steering: float):
        """
        Definit l'angle de braquage.

        Args:
            steering: Braquage normalise [-1.0, 1.0]
                      Positif = gauche, negatif = droite
        """
        pass

    @abstractmethod
    def emergency_stop(self):
        """Arret d'urgence immediat."""
        pass

    @abstractmethod
    def is_running(self) -> bool:
        """Indique si le controleur est actif."""
        pass
