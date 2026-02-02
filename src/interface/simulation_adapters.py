"""
Adaptateurs pour la simulation (PC).

Ces classes adaptent les simulateurs aux interfaces abstraites.
"""

from typing import Optional
import math
import time

from .sensor_interface import (
    ILidarSensor, IGPSSensor, IMotorController,
    LidarData, LidarPoint, GPSData
)


class SimulatedLidarAdapter(ILidarSensor):
    """Adaptateur pour le LiDAR simule."""

    def __init__(self, environment):
        """
        Args:
            environment: Environnement de simulation
        """
        self.environment = environment
        self._simulator = None
        self._running = False

    def start(self) -> bool:
        try:
            from simulation import LidarSimulator
            self._simulator = LidarSimulator(self.environment)
            self._simulator.start()
            self._running = True
            return True
        except Exception as e:
            print(f"[LiDAR Sim] Erreur demarrage: {e}")
            return False

    def stop(self):
        if self._simulator:
            self._simulator.stop()
        self._running = False

    def get_scan(self) -> Optional[LidarData]:
        if not self._simulator:
            return None

        scan = self._simulator.get_single_scan()
        if not scan:
            return None

        # Convertir au format interface
        points = []
        for p in scan.points:
            points.append(LidarPoint(
                angle=p.angle,
                distance=p.distance,
                intensity=p.intensity,
                valid=p.valid
            ))

        return LidarData(
            points=points,
            timestamp=scan.timestamp,
            scan_frequency=scan.scan_frequency
        )

    def set_robot_pose(self, x: float, y: float, theta: float):
        """Met a jour la pose du robot dans le simulateur."""
        if self._simulator:
            self._simulator.set_robot_pose(x, y, theta)

    def is_running(self) -> bool:
        return self._running


class SimulatedGPSAdapter(IGPSSensor):
    """Adaptateur pour le GPS simule."""

    # Coordonnees de reference Epitech Paris (Le Kremlin-Bicetre)
    EPITECH_LAT = 48.8156
    EPITECH_LON = 2.3631

    def __init__(self, origin_lat: float = None, origin_lon: float = None):
        """
        Args:
            origin_lat: Latitude de l'origine (defaut: Epitech Paris)
            origin_lon: Longitude de l'origine (defaut: Epitech Paris)
        """
        self.origin_lat = origin_lat or self.EPITECH_LAT
        self.origin_lon = origin_lon or self.EPITECH_LON
        self._simulator = None
        self._running = False
        self._x = 0.0  # Position locale X
        self._y = 0.0  # Position locale Y
        self._heading = 0.0
        self._speed = 0.0

    def start(self) -> bool:
        try:
            from simulation import GPSSimulator
            self._simulator = GPSSimulator(rtk_mode=True)
            self._simulator.set_origin(self.origin_lat, self.origin_lon)
            self._simulator.start()
            self._running = True
            return True
        except Exception as e:
            print(f"[GPS Sim] Erreur demarrage: {e}")
            return False

    def stop(self):
        if self._simulator:
            self._simulator.stop()
        self._running = False

    def set_local_position(self, x: float, y: float, heading: float, speed: float = 0.0):
        """
        Met a jour la position locale.

        Args:
            x, y: Position en metres par rapport a l'origine
            heading: Cap en radians
            speed: Vitesse en m/s
        """
        self._x = x
        self._y = y
        self._heading = heading
        self._speed = speed
        if self._simulator:
            self._simulator.set_local_position(x, y, heading)

    def get_position(self) -> Optional[GPSData]:
        if not self._running:
            return None

        # Convertir position locale en coordonnees GPS
        # 1 degre latitude = ~111320 m
        # 1 degre longitude = ~111320 * cos(lat) m
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        lat = self.origin_lat + (self._y / meters_per_deg_lat)
        lon = self.origin_lon + (self._x / meters_per_deg_lon)

        return GPSData(
            latitude=lat,
            longitude=lon,
            altitude=0.0,
            heading=self._heading,
            speed=self._speed,
            accuracy_h=0.02,  # RTK simule = 2cm
            accuracy_v=0.04,
            satellites=12,
            quality=4,  # RTK Fixed
            timestamp=time.time(),
            is_valid=True
        )

    def is_running(self) -> bool:
        return self._running


class SimulatedMotorAdapter(IMotorController):
    """Adaptateur pour le moteur simule (physique simplifiee)."""

    def __init__(self, wheelbase: float = 0.26):
        """
        Args:
            wheelbase: Empattement du vehicule en metres
        """
        self.wheelbase = wheelbase
        self._running = False
        self._speed = 0.0
        self._steering = 0.0

        # Etat du robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Callbacks pour mise a jour
        self._position_callback = None

    def start(self) -> bool:
        self._running = True
        return True

    def stop(self):
        self.emergency_stop()
        self._running = False

    def set_speed(self, speed: float):
        self._speed = max(-1.0, min(1.0, speed))

    def set_steering(self, steering: float):
        self._steering = max(-1.0, min(1.0, steering))

    def emergency_stop(self):
        self._speed = 0.0
        self._steering = 0.0

    def is_running(self) -> bool:
        return self._running

    def set_position_callback(self, callback):
        """Definit un callback appele apres chaque update physique."""
        self._position_callback = callback

    def update_physics(self, dt: float):
        """
        Met a jour la physique du vehicule (modele bicyclette).

        Args:
            dt: Pas de temps en secondes
        """
        if not self._running:
            return

        # Vitesse reelle (m/s) - max 2 m/s
        v = self._speed * 2.0

        # Angle de braquage (rad) - max 0.5 rad (~30 deg)
        delta = self._steering * 0.5

        # Modele cinematique bicyclette
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += (v / self.wheelbase) * math.tan(delta) * dt

        # Normaliser theta entre -pi et pi
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        if self._position_callback:
            self._position_callback(self.x, self.y, self.theta, v)

    def set_pose(self, x: float, y: float, theta: float):
        """Definit la pose du robot."""
        self.x = x
        self.y = y
        self.theta = theta

    def get_pose(self) -> tuple:
        """Retourne la pose actuelle (x, y, theta)."""
        return self.x, self.y, self.theta

    def get_velocity(self) -> tuple:
        """Retourne la vitesse actuelle (speed, steering)."""
        return self._speed * 2.0, self._steering
