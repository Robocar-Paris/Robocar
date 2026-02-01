#!/usr/bin/env python3
"""
Simulation de la voiture RC sur PC.

Teste la navigation autonome sans hardware reel.

Usage:
    python scripts/run_simulation.py
    python scripts/run_simulation.py --env parking
    python scripts/run_simulation.py --env corridor --visualize

Environnements disponibles:
    - parking: Parking avec voitures garees
    - corridor: Couloir avec obstacles
"""

import sys
import time
import argparse
import math
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from simulation import (
    LidarSimulator, LidarSimulatorConfig,
    GPSSimulator,
    Environment, create_parking_env, create_corridor_env
)
from perception import LidarProcessor, ObstacleDetector


class SimulatedCar:
    """
    Voiture RC simulee.

    Meme logique que run_car.py mais avec capteurs simules.
    """

    # Parametres navigation
    WAYPOINT_REACHED_DIST = 0.5
    MAX_SPEED = 0.5           # m/s en simulation
    MIN_SPEED = 0.1
    OBSTACLE_STOP_DIST = 0.5
    OBSTACLE_SLOW_DIST = 1.5
    STEERING_GAIN = 2.0

    def __init__(self, environment: Environment):
        """
        Initialise la simulation.

        Args:
            environment: Environnement de simulation
        """
        self.env = environment

        # Simulateurs
        self.lidar = LidarSimulator(environment)
        self.gps = GPSSimulator(rtk_mode=True)

        # Perception
        self.processor = LidarProcessor(min_range=0.05, max_range=10.0)
        self.detector = ObstacleDetector()

        # Etat robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.steering = 0.0

        # Waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0

        # Stats
        self.total_distance = 0.0
        self.start_time = 0.0

    def set_start_position(self, x: float, y: float, theta: float = 0.0):
        """Definit la position de depart."""
        self.x = x
        self.y = y
        self.theta = theta

        self.lidar.set_robot_pose(x, y, theta)
        self.gps.set_local_position(x, y, theta)

    def add_waypoint(self, x: float, y: float):
        """Ajoute un waypoint (coordonnees locales en metres)."""
        self.waypoints.append((x, y))

    def distance_to_waypoint(self, target_x: float, target_y: float) -> float:
        """Distance jusqu'au waypoint."""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def bearing_to_waypoint(self, target_x: float, target_y: float) -> float:
        """Cap vers le waypoint (radians)."""
        dx = target_x - self.x
        dy = target_y - self.y
        return math.atan2(dy, dx)

    def check_obstacles(self) -> tuple:
        """Verifie les obstacles."""
        scan = self.lidar.get_single_scan()

        min_dist_front = float('inf')
        min_dist_left = float('inf')
        min_dist_right = float('inf')

        for point in scan.points:
            if not point.valid:
                continue

            angle_deg = math.degrees(point.angle)

            if -45 < angle_deg < 45:
                if point.distance < min_dist_front:
                    min_dist_front = point.distance

            if 45 < angle_deg < 90:
                if point.distance < min_dist_left:
                    min_dist_left = point.distance

            if -90 < angle_deg < -45:
                if point.distance < min_dist_right:
                    min_dist_right = point.distance

        # Direction d'evitement
        steering = 0.0
        if min_dist_front < self.OBSTACLE_SLOW_DIST:
            if min_dist_left > min_dist_right:
                steering = 0.5
            else:
                steering = -0.5

        obstacle = min_dist_front < self.OBSTACLE_STOP_DIST

        return obstacle, min_dist_front, steering

    def compute_control(self, target_x: float, target_y: float) -> tuple:
        """Calcule les commandes."""
        distance = self.distance_to_waypoint(target_x, target_y)
        target_bearing = self.bearing_to_waypoint(target_x, target_y)

        # Erreur de cap
        heading_error = target_bearing - self.theta
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        steering = self.STEERING_GAIN * heading_error
        steering = max(-1.0, min(1.0, steering))

        speed = self.MAX_SPEED
        if distance < 2.0:
            speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * (distance / 2.0)

        return speed, steering

    def update_physics(self, speed: float, steering: float, dt: float):
        """
        Met a jour la physique du robot.

        Modele cinematique Ackermann simplifie.
        """
        wheelbase = 0.26  # metres

        # Vitesse et angle de braquage
        v = speed
        delta = steering * 0.5  # Max 0.5 rad de braquage

        # Mise a jour position
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += (v / wheelbase) * math.tan(delta) * dt

        # Normaliser theta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        # Distance parcourue
        self.total_distance += abs(v) * dt

        # Mettre a jour simulateurs
        self.lidar.set_robot_pose(self.x, self.y, self.theta)
        self.gps.set_local_position(self.x, self.y, self.theta)

    def print_status(self, distance: float, obs_dist: float):
        """Affiche le statut."""
        wp_idx = self.current_waypoint_idx + 1
        total_wp = len(self.waypoints)

        print(f"\r[Sim] WP{wp_idx}/{total_wp}: "
              f"pos=({self.x:.1f},{self.y:.1f}) "
              f"dist={distance:.1f}m "
              f"heading={math.degrees(self.theta):.0f}° "
              f"obs={obs_dist:.1f}m "
              f"total={self.total_distance:.1f}m", end='')

    def print_map(self):
        """Affiche une mini-carte ASCII."""
        width = 40
        height = 20
        scale = min(width / self.env.width, height / self.env.height)

        # Creer grille
        grid = [['.' for _ in range(width)] for _ in range(height)]

        # Position robot
        rx = int((self.x + self.env.width / 2) * scale)
        ry = int((self.y + self.env.height / 2) * scale)
        if 0 <= rx < width and 0 <= ry < height:
            # Direction du robot
            arrows = {0: '>', 1: '^', 2: '<', 3: 'v'}
            direction = int((self.theta + math.pi / 4) / (math.pi / 2)) % 4
            grid[height - 1 - ry][rx] = arrows.get(direction, 'R')

        # Waypoints
        for i, (wx, wy) in enumerate(self.waypoints):
            wpx = int((wx + self.env.width / 2) * scale)
            wpy = int((wy + self.env.height / 2) * scale)
            if 0 <= wpx < width and 0 <= wpy < height:
                if i == self.current_waypoint_idx:
                    grid[height - 1 - wpy][wpx] = 'X'
                else:
                    grid[height - 1 - wpy][wpx] = 'o'

        # Afficher
        print("\n" + "=" * (width + 2))
        for row in grid:
            print("|" + "".join(row) + "|")
        print("=" * (width + 2))
        print(f"R=Robot, X=Waypoint actuel, o=Waypoint suivant")

    def run(self, visualize: bool = False):
        """Boucle principale de simulation."""
        if not self.waypoints:
            print("[Sim] Aucun waypoint defini!")
            return

        print(f"\n[Sim] Demarrage simulation")
        print(f"[Sim] Position: ({self.x:.1f}, {self.y:.1f})")
        print(f"[Sim] Waypoints: {len(self.waypoints)}")
        print("[Sim] Appuyez sur Ctrl+C pour arreter\n")

        if visualize:
            self.print_map()

        self.lidar.start()
        self.gps.start()
        self.start_time = time.time()

        dt = 0.05  # 50ms timestep
        last_viz_time = 0

        try:
            while self.current_waypoint_idx < len(self.waypoints):
                target_x, target_y = self.waypoints[self.current_waypoint_idx]
                distance = self.distance_to_waypoint(target_x, target_y)

                # Waypoint atteint?
                if distance < self.WAYPOINT_REACHED_DIST:
                    print(f"\n[Sim] Waypoint {self.current_waypoint_idx + 1} atteint!")
                    self.current_waypoint_idx += 1

                    if visualize:
                        self.print_map()

                    if self.current_waypoint_idx >= len(self.waypoints):
                        break
                    continue

                # Verifier obstacles
                obstacle, obs_dist, avoid_steering = self.check_obstacles()

                if obstacle:
                    # STOP
                    self.speed = 0
                    self.steering = avoid_steering
                    print(f"\n[Sim] STOP! Obstacle a {obs_dist:.2f}m - evitement...")
                    time.sleep(0.1)

                    # Essayer de contourner
                    self.update_physics(0.1, avoid_steering * 2, dt)
                    continue

                # Calculer commandes
                speed, steering = self.compute_control(target_x, target_y)

                # Ajuster pour evitement
                if obs_dist < self.OBSTACLE_SLOW_DIST:
                    speed *= 0.5
                    steering += avoid_steering * 0.5

                # Mettre a jour physique
                self.update_physics(speed, steering, dt)

                # Affichage
                self.print_status(distance, obs_dist)

                # Visualisation periodique
                if visualize and time.time() - last_viz_time > 2.0:
                    print()
                    self.print_map()
                    last_viz_time = time.time()

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n\n[Sim] Arret demande par l'utilisateur")

        finally:
            self.lidar.stop()
            self.gps.stop()

            elapsed = time.time() - self.start_time
            print(f"\n[Sim] === RESUME ===")
            print(f"[Sim] Temps: {elapsed:.1f}s")
            print(f"[Sim] Distance: {self.total_distance:.1f}m")
            print(f"[Sim] Waypoints atteints: {self.current_waypoint_idx}/{len(self.waypoints)}")
            print(f"[Sim] Position finale: ({self.x:.2f}, {self.y:.2f})")


def main():
    parser = argparse.ArgumentParser(description='Simulation voiture RC')
    parser.add_argument('--env', choices=['parking', 'corridor'], default='parking',
                        help='Environnement de simulation')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Afficher la carte ASCII')
    parser.add_argument('--start-x', type=float, default=-7.0, help='Position X depart')
    parser.add_argument('--start-y', type=float, default=0.0, help='Position Y depart')

    args = parser.parse_args()

    # Creer environnement
    if args.env == 'parking':
        env = create_parking_env()
        # Waypoints: traverser le parking
        waypoints = [
            (-3, 0),    # Milieu gauche
            (0, 0),     # Centre
            (3, 0),     # Milieu droite
            (7, 0),     # Sortie droite
        ]
        start_x, start_y = -7, 0
    else:
        env = create_corridor_env()
        # Waypoints: traverser le couloir
        waypoints = [
            (-5, 0),
            (-2, 0),
            (0, 0),
            (2, 0),
            (5, 0),
        ]
        start_x, start_y = -6, 0

    # Override position depart si specifiee
    if args.start_x != -7.0:
        start_x = args.start_x
    if args.start_y != 0.0:
        start_y = args.start_y

    # Creer simulation
    car = SimulatedCar(env)
    car.set_start_position(start_x, start_y, 0)

    for x, y in waypoints:
        car.add_waypoint(x, y)

    print("=" * 50)
    print("   SIMULATION VOITURE RC AUTONOME")
    print("=" * 50)
    print(f"Environnement: {args.env}")
    print(f"Depart: ({start_x}, {start_y})")
    print(f"Waypoints: {len(waypoints)}")

    # Lancer simulation
    car.run(visualize=args.visualize)

    return 0


if __name__ == '__main__':
    sys.exit(main())
