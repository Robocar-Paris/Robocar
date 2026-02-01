"""
Simulateur LiDAR LD19.

Genere des scans LiDAR virtuels a partir d'un environnement simule.
"""

import math
import time
import random
from dataclasses import dataclass
from typing import List, Optional
import threading
from queue import Queue

from .environment import Environment, create_parking_env


@dataclass
class LidarSimulatorConfig:
    """Configuration du simulateur LiDAR."""
    num_points: int = 360          # Points par scan
    min_range: float = 0.02        # Portee min (m)
    max_range: float = 12.0        # Portee max (m)
    scan_frequency: float = 10.0   # Frequence de scan (Hz)
    noise_std: float = 0.02        # Bruit gaussien (m)
    miss_rate: float = 0.02        # Taux de points manques


@dataclass
class SimulatedLidarPoint:
    """Point LiDAR simule."""
    angle: float      # Angle en radians
    distance: float   # Distance en metres
    intensity: int    # Intensite (0-255)
    valid: bool       # Point valide


@dataclass
class SimulatedLidarScan:
    """Scan LiDAR simule."""
    points: List[SimulatedLidarPoint]
    timestamp: float
    scan_frequency: float


class LidarSimulator:
    """
    Simulateur de LiDAR LD19.

    Genere des scans LiDAR a partir d'un environnement virtuel.
    Compatible avec l'interface du vrai driver LiDAR.

    Usage:
        env = create_parking_env()
        lidar = LidarSimulator(env)
        lidar.set_robot_pose(0, 0, 0)
        lidar.start()

        scan = lidar.get_scan()
        for point in scan.points:
            print(f"Angle: {point.angle:.2f}, Distance: {point.distance:.2f}")

        lidar.stop()
    """

    def __init__(self, environment: Optional[Environment] = None,
                 config: Optional[LidarSimulatorConfig] = None):
        """
        Initialise le simulateur.

        Args:
            environment: Environnement de simulation
            config: Configuration du simulateur
        """
        self.env = environment or create_parking_env()
        self.config = config or LidarSimulatorConfig()

        # Position du robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Orientation en radians

        # Thread de scan
        self._running = False
        self._scan_thread: Optional[threading.Thread] = None
        self._scan_queue: Queue = Queue(maxsize=10)

    def set_robot_pose(self, x: float, y: float, theta: float = 0.0):
        """
        Definit la position du robot.

        Args:
            x, y: Position en metres
            theta: Orientation en radians
        """
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

    def move_robot(self, dx: float, dy: float, dtheta: float = 0.0):
        """Deplace le robot relativement a sa position actuelle."""
        # Rotation dans le repere robot
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)

        self.robot_x += dx * cos_t - dy * sin_t
        self.robot_y += dx * sin_t + dy * cos_t
        self.robot_theta += dtheta

    def start(self) -> bool:
        """Demarre le simulateur."""
        if self._running:
            return True

        self._running = True
        self._scan_thread = threading.Thread(
            target=self._scan_loop,
            daemon=True,
            name="LidarSimulator"
        )
        self._scan_thread.start()
        return True

    def stop(self):
        """Arrete le simulateur."""
        self._running = False
        if self._scan_thread:
            self._scan_thread.join(timeout=1.0)

    def _scan_loop(self):
        """Boucle de generation de scans."""
        period = 1.0 / self.config.scan_frequency

        while self._running:
            start = time.time()

            scan = self._generate_scan()

            # Ajouter a la queue
            try:
                if self._scan_queue.full():
                    self._scan_queue.get_nowait()
                self._scan_queue.put_nowait(scan)
            except:
                pass

            # Attendre pour respecter la frequence
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _generate_scan(self) -> SimulatedLidarScan:
        """Genere un scan LiDAR."""
        points = []
        angle_step = 2 * math.pi / self.config.num_points

        for i in range(self.config.num_points):
            # Angle dans le repere robot
            angle_robot = i * angle_step - math.pi  # -pi a +pi

            # Angle dans le repere monde
            angle_world = angle_robot + self.robot_theta

            # Raycast
            distance = self.env.raycast(
                self.robot_x, self.robot_y,
                angle_world,
                self.config.max_range
            )

            # Ajouter du bruit
            if distance < self.config.max_range:
                distance += random.gauss(0, self.config.noise_std)
                distance = max(self.config.min_range, distance)

            # Point manque aleatoirement
            valid = True
            if random.random() < self.config.miss_rate:
                valid = False
                distance = 0.0

            # Hors portee
            if distance >= self.config.max_range:
                valid = False

            # Intensite (plus proche = plus intense)
            if valid:
                intensity = int(255 * (1 - distance / self.config.max_range))
                intensity = max(50, min(255, intensity + random.randint(-20, 20)))
            else:
                intensity = 0

            points.append(SimulatedLidarPoint(
                angle=angle_robot,
                distance=distance,
                intensity=intensity,
                valid=valid
            ))

        return SimulatedLidarScan(
            points=points,
            timestamp=time.time(),
            scan_frequency=self.config.scan_frequency
        )

    def get_scan(self, timeout: float = 1.0) -> Optional[SimulatedLidarScan]:
        """
        Recupere le prochain scan.

        Args:
            timeout: Temps d'attente max

        Returns:
            Scan LiDAR ou None
        """
        try:
            return self._scan_queue.get(timeout=timeout)
        except:
            return None

    def get_latest_scan(self) -> Optional[SimulatedLidarScan]:
        """Recupere le dernier scan (vide la queue)."""
        scan = None
        while not self._scan_queue.empty():
            try:
                scan = self._scan_queue.get_nowait()
            except:
                break
        return scan

    def get_single_scan(self) -> SimulatedLidarScan:
        """Genere un scan unique sans demarrer le thread."""
        return self._generate_scan()


if __name__ == "__main__":
    # Test du simulateur
    from .environment import create_corridor_env

    print("=== Test LidarSimulator ===\n")

    env = create_corridor_env()
    lidar = LidarSimulator(env)
    lidar.set_robot_pose(0, 0, 0)

    # Test scan unique
    scan = lidar.get_single_scan()
    valid_count = sum(1 for p in scan.points if p.valid)
    print(f"Scan genere: {len(scan.points)} points, {valid_count} valides")

    # Test avec thread
    lidar.start()
    time.sleep(0.5)

    for i in range(3):
        scan = lidar.get_scan()
        if scan:
            # Trouver l'obstacle le plus proche
            min_dist = float('inf')
            min_angle = 0
            for p in scan.points:
                if p.valid and p.distance < min_dist:
                    min_dist = p.distance
                    min_angle = p.angle

            print(f"Scan {i+1}: obstacle le plus proche a {min_dist:.2f}m, "
                  f"angle {math.degrees(min_angle):.1f} deg")

    lidar.stop()
    print("\nSimulateur arrete.")
