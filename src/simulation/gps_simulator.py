"""
Simulateur GPS Point One RTK.

Simule un GPS avec position et trajectoire pour tests sur PC.
"""

import math
import time
import random
import threading
from dataclasses import dataclass
from typing import Optional, Callable, List, Tuple
from queue import Queue


@dataclass
class SimulatedGPSPosition:
    """Position GPS simulee."""
    timestamp: float
    latitude: float
    longitude: float
    altitude: float
    quality: int          # 0=no fix, 1=GPS, 4=RTK_FIXED, 5=RTK_FLOAT
    satellites: int
    accuracy_h: float     # Precision horizontale (m)
    accuracy_v: float     # Precision verticale (m)

    @property
    def quality_string(self) -> str:
        """Retourne la qualite sous forme de chaine."""
        names = {0: "NO_FIX", 1: "GPS", 2: "DGPS", 4: "RTK_FIXED", 5: "RTK_FLOAT"}
        return names.get(self.quality, "UNKNOWN")

    @property
    def is_rtk_fixed(self) -> bool:
        return self.quality == 4

    @property
    def is_valid(self) -> bool:
        return self.quality > 0


class GPSSimulator:
    """
    Simulateur de GPS RTK.

    Simule un GPS avec une position qui peut etre mise a jour
    ou suivre une trajectoire.

    Usage:
        gps = GPSSimulator()
        gps.set_origin(48.8566, 2.3522)  # Paris
        gps.start()

        pos = gps.get_position()
        print(f"Position: {pos.latitude}, {pos.longitude}")

        gps.stop()
    """

    # Constantes pour conversion
    METERS_PER_DEG_LAT = 111320.0
    METERS_PER_DEG_LON_EQUATOR = 111320.0

    def __init__(self, rtk_mode: bool = True, update_rate: float = 10.0):
        """
        Initialise le simulateur GPS.

        Args:
            rtk_mode: Si True, simule RTK FIXED (precision cm)
            update_rate: Frequence de mise a jour (Hz)
        """
        self.rtk_mode = rtk_mode
        self.update_rate = update_rate

        # Position origine (reference)
        self.origin_lat = 48.8566   # Paris par defaut
        self.origin_lon = 2.3522
        self.origin_alt = 35.0

        # Position locale (metres depuis origine)
        self.local_x = 0.0
        self.local_y = 0.0

        # Orientation du robot
        self.heading = 0.0  # radians

        # Thread
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._position: Optional[SimulatedGPSPosition] = None
        self._lock = threading.Lock()

        # Callback
        self._position_callback: Optional[Callable[[SimulatedGPSPosition], None]] = None

        # Trajectoire
        self._waypoints: List[Tuple[float, float]] = []
        self._waypoint_index = 0

    def set_origin(self, latitude: float, longitude: float, altitude: float = 35.0):
        """
        Definit le point d'origine GPS.

        Args:
            latitude: Latitude en degres
            longitude: Longitude en degres
            altitude: Altitude en metres
        """
        self.origin_lat = latitude
        self.origin_lon = longitude
        self.origin_alt = altitude

    def set_local_position(self, x: float, y: float, heading: float = 0.0):
        """
        Definit la position locale (metres depuis origine).

        Args:
            x: Position X (Est positif)
            y: Position Y (Nord positif)
            heading: Orientation en radians
        """
        with self._lock:
            self.local_x = x
            self.local_y = y
            self.heading = heading

    def move(self, dx: float, dy: float, dheading: float = 0.0):
        """
        Deplace le robot relativement.

        Args:
            dx: Deplacement X dans le repere robot
            dy: Deplacement Y dans le repere robot
            dheading: Rotation
        """
        with self._lock:
            # Rotation dans le repere monde
            cos_h = math.cos(self.heading)
            sin_h = math.sin(self.heading)

            self.local_x += dx * cos_h - dy * sin_h
            self.local_y += dx * sin_h + dy * cos_h
            self.heading += dheading

    def _local_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        """Convertit coordonnees locales en GPS."""
        # Calcul de la longueur d'un degre de longitude a cette latitude
        meters_per_deg_lon = self.METERS_PER_DEG_LON_EQUATOR * math.cos(math.radians(self.origin_lat))

        lat = self.origin_lat + y / self.METERS_PER_DEG_LAT
        lon = self.origin_lon + x / meters_per_deg_lon

        return lat, lon

    def start(self) -> bool:
        """Demarre le simulateur."""
        if self._running:
            return True

        self._running = True
        self._thread = threading.Thread(
            target=self._update_loop,
            daemon=True,
            name="GPSSimulator"
        )
        self._thread.start()
        return True

    def stop(self):
        """Arrete le simulateur."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _update_loop(self):
        """Boucle de mise a jour GPS."""
        period = 1.0 / self.update_rate

        while self._running:
            start = time.time()

            # Generer position
            pos = self._generate_position()

            with self._lock:
                self._position = pos

            # Callback
            if self._position_callback:
                self._position_callback(pos)

            # Attendre
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _generate_position(self) -> SimulatedGPSPosition:
        """Genere une position GPS."""
        with self._lock:
            x = self.local_x
            y = self.local_y

        # Bruit selon le mode
        if self.rtk_mode:
            noise = 0.02  # 2 cm
            quality = 4   # RTK_FIXED
            accuracy_h = 0.02
        else:
            noise = 2.0   # 2 m
            quality = 1   # GPS
            accuracy_h = 2.5

        x += random.gauss(0, noise)
        y += random.gauss(0, noise)

        lat, lon = self._local_to_gps(x, y)

        return SimulatedGPSPosition(
            timestamp=time.time(),
            latitude=lat,
            longitude=lon,
            altitude=self.origin_alt + random.gauss(0, 0.1),
            quality=quality,
            satellites=random.randint(10, 14),
            accuracy_h=accuracy_h,
            accuracy_v=accuracy_h * 1.5
        )

    def get_position(self) -> Optional[SimulatedGPSPosition]:
        """Retourne la derniere position."""
        with self._lock:
            return self._position

    def wait_for_fix(self, timeout: float = 10.0) -> bool:
        """Attend un fix GPS (toujours True en simulation)."""
        time.sleep(0.5)  # Simule un petit delai
        return True

    def wait_for_rtk_fixed(self, timeout: float = 30.0) -> bool:
        """Attend RTK FIXED (True si rtk_mode active)."""
        time.sleep(1.0)  # Simule un delai de convergence
        return self.rtk_mode

    def set_position_callback(self, callback: Callable[[SimulatedGPSPosition], None]):
        """Definit le callback de position."""
        self._position_callback = callback

    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """
        Definit une liste de waypoints a suivre.

        Args:
            waypoints: Liste de (x, y) en metres depuis l'origine
        """
        self._waypoints = waypoints
        self._waypoint_index = 0

    def get_current_waypoint(self) -> Optional[Tuple[float, float]]:
        """Retourne le waypoint actuel."""
        if self._waypoint_index < len(self._waypoints):
            return self._waypoints[self._waypoint_index]
        return None

    def advance_waypoint(self) -> bool:
        """Passe au waypoint suivant. Retourne False si termine."""
        self._waypoint_index += 1
        return self._waypoint_index < len(self._waypoints)


if __name__ == "__main__":
    # Test du simulateur
    print("=== Test GPSSimulator ===\n")

    gps = GPSSimulator(rtk_mode=True)
    gps.set_origin(48.8968, 2.2190)  # EPITA
    gps.set_local_position(0, 0)

    gps.start()

    print("Position initiale:")
    time.sleep(0.5)
    pos = gps.get_position()
    if pos:
        print(f"  Lat: {pos.latitude:.6f}, Lon: {pos.longitude:.6f}")
        print(f"  Qualite: {pos.quality_string}, Precision: {pos.accuracy_h}m")

    # Deplacer le robot
    print("\nDeplacement de 10m vers l'Est...")
    gps.move(10, 0)
    time.sleep(0.5)

    pos = gps.get_position()
    if pos:
        print(f"  Lat: {pos.latitude:.6f}, Lon: {pos.longitude:.6f}")

    print("\nDeplacement de 5m vers le Nord...")
    gps.move(0, 5)
    time.sleep(0.5)

    pos = gps.get_position()
    if pos:
        print(f"  Lat: {pos.latitude:.6f}, Lon: {pos.longitude:.6f}")

    gps.stop()
    print("\nSimulateur arrete.")
