#!/usr/bin/env python3
"""
Script principal pour la voiture RC autonome.

Navigation de point A vers point B avec detection et evitement d'obstacles.

Usage:
    python scripts/run_car.py
    python scripts/run_car.py --target-lat 48.8970 --target-lon 2.2195
    python scripts/run_car.py --waypoints waypoints.txt

Fichier waypoints.txt (un waypoint par ligne):
    48.8968,2.2190
    48.8970,2.2195
    48.8972,2.2192
"""

import sys
import time
import argparse
import math
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from driver import GPSDriver, LidarDriver, VESCController
from perception import LidarProcessor, ObstacleDetector


class RobocarController:
    """
    Controleur principal de la voiture RC autonome.

    Fonctionnalites:
    - Navigation GPS vers waypoints
    - Detection d'obstacles par LiDAR
    - Evitement d'obstacles
    - Arret d'urgence
    """

    # Configuration
    GPS_PORT = '/dev/ttyUSB0'
    LIDAR_PORT = '/dev/ttyUSB0'
    VESC_PORT = '/dev/ttyACM0'

    # Parametres navigation
    WAYPOINT_REACHED_DIST = 1.0    # Distance pour considerer waypoint atteint (m)
    MAX_SPEED = 0.15               # Vitesse max (duty cycle)
    MIN_SPEED = 0.05               # Vitesse min
    OBSTACLE_STOP_DIST = 0.5       # Distance arret obstacle (m)
    OBSTACLE_SLOW_DIST = 1.5       # Distance ralentissement (m)
    STEERING_GAIN = 1.5            # Gain de direction

    def __init__(self):
        """Initialise le controleur."""
        self.gps = None
        self.lidar = None
        self.motor = None
        self.processor = LidarProcessor(min_range=0.05, max_range=10.0)
        self.detector = ObstacleDetector()

        # Etat
        self.running = False
        self.waypoints = []
        self.current_waypoint_idx = 0

        # Position actuelle
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0  # Estime par mouvement

    def init_hardware(self) -> bool:
        """Initialise les drivers hardware."""
        print("[Car] Initialisation du hardware...")

        # GPS
        print(f"  GPS sur {self.GPS_PORT}...")
        try:
            self.gps = GPSDriver(self.GPS_PORT, baudrate=460800)
            if not self.gps.start():
                print("  [ERREUR] GPS non connecte")
                return False
            print("  GPS OK")
        except Exception as e:
            print(f"  [ERREUR] GPS: {e}")
            return False

        # LiDAR
        print(f"  LiDAR sur {self.LIDAR_PORT}...")
        try:
            self.lidar = LidarDriver(self.LIDAR_PORT)
            if not self.lidar.start():
                print("  [ERREUR] LiDAR non connecte")
                return False
            print("  LiDAR OK")
        except Exception as e:
            print(f"  [ERREUR] LiDAR: {e}")
            return False

        # VESC
        print(f"  VESC sur {self.VESC_PORT}...")
        try:
            self.motor = VESCController(self.VESC_PORT)
            if not self.motor.start():
                print("  [ERREUR] VESC non connecte")
                return False
            print("  VESC OK")
        except Exception as e:
            print(f"  [ERREUR] VESC: {e}")
            return False

        print("[Car] Hardware initialise!")
        return True

    def shutdown(self):
        """Arrete proprement tous les composants."""
        print("[Car] Arret...")

        if self.motor:
            self.motor.set_duty_cycle(0)
            self.motor.stop()

        if self.lidar:
            self.lidar.stop()

        if self.gps:
            self.gps.stop()

        print("[Car] Arret termine.")

    def add_waypoint(self, lat: float, lon: float):
        """Ajoute un waypoint."""
        self.waypoints.append((lat, lon))

    def load_waypoints(self, filepath: str):
        """Charge les waypoints depuis un fichier."""
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split(',')
                    if len(parts) >= 2:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        self.add_waypoint(lat, lon)

        print(f"[Car] {len(self.waypoints)} waypoints charges")

    def distance_to_waypoint(self, target_lat: float, target_lon: float) -> float:
        """Calcule la distance jusqu'au waypoint (m)."""
        # Formule Haversine simplifiee
        dlat = math.radians(target_lat - self.current_lat)
        dlon = math.radians(target_lon - self.current_lon)

        lat1 = math.radians(self.current_lat)

        dx = dlon * 111320 * math.cos(lat1)
        dy = dlat * 111320

        return math.sqrt(dx * dx + dy * dy)

    def bearing_to_waypoint(self, target_lat: float, target_lon: float) -> float:
        """Calcule le cap vers le waypoint (radians)."""
        dlat = target_lat - self.current_lat
        dlon = target_lon - self.current_lon

        lat1 = math.radians(self.current_lat)
        dx = dlon * math.cos(lat1)
        dy = dlat

        return math.atan2(dx, dy)

    def check_obstacles(self) -> tuple:
        """
        Verifie les obstacles devant.

        Returns:
            (obstacle_detected, min_distance, suggested_steering)
        """
        scan = self.lidar.get_latest_scan()
        if not scan:
            return False, float('inf'), 0.0

        # Zone avant: -45 a +45 degres
        min_dist_front = float('inf')
        min_dist_left = float('inf')
        min_dist_right = float('inf')

        for point in scan.points:
            if not point.valid:
                continue

            angle_deg = math.degrees(point.angle)

            # Zone avant
            if -45 < angle_deg < 45:
                if point.distance < min_dist_front:
                    min_dist_front = point.distance

            # Zone gauche
            if 45 < angle_deg < 90:
                if point.distance < min_dist_left:
                    min_dist_left = point.distance

            # Zone droite
            if -90 < angle_deg < -45:
                if point.distance < min_dist_right:
                    min_dist_right = point.distance

        # Determiner la direction d'evitement
        steering = 0.0
        if min_dist_front < self.OBSTACLE_SLOW_DIST:
            # Tourner vers le cote le plus libre
            if min_dist_left > min_dist_right:
                steering = 0.3  # Tourner a gauche
            else:
                steering = -0.3  # Tourner a droite

        obstacle_detected = min_dist_front < self.OBSTACLE_STOP_DIST

        return obstacle_detected, min_dist_front, steering

    def compute_control(self, target_lat: float, target_lon: float) -> tuple:
        """
        Calcule les commandes de controle.

        Returns:
            (speed, steering)
        """
        # Distance et cap vers le waypoint
        distance = self.distance_to_waypoint(target_lat, target_lon)
        target_bearing = self.bearing_to_waypoint(target_lat, target_lon)

        # Erreur de cap (simplifie - on utiliserait un compas en realite)
        heading_error = target_bearing - self.current_heading
        # Normaliser entre -pi et pi
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Commande de direction
        steering = self.STEERING_GAIN * heading_error
        steering = max(-0.5, min(0.5, steering))

        # Vitesse de base
        speed = self.MAX_SPEED

        # Ralentir si proche du waypoint
        if distance < 3.0:
            speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * (distance / 3.0)

        return speed, steering

    def run(self):
        """Boucle principale de navigation."""
        if not self.waypoints:
            print("[Car] Aucun waypoint defini!")
            return

        self.running = True
        self.current_waypoint_idx = 0

        print(f"\n[Car] Demarrage navigation vers {len(self.waypoints)} waypoints")
        print("[Car] Appuyez sur Ctrl+C pour arreter\n")

        # Attendre fix GPS
        print("[Car] Attente fix GPS...")
        if not self.gps.wait_for_fix(timeout=60):
            print("[Car] ERREUR: Pas de fix GPS!")
            return

        try:
            while self.running and self.current_waypoint_idx < len(self.waypoints):
                # Position GPS actuelle
                pos = self.gps.get_position()
                if pos and pos.is_valid:
                    self.current_lat = pos.latitude
                    self.current_lon = pos.longitude

                # Waypoint cible
                target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
                distance = self.distance_to_waypoint(target_lat, target_lon)

                # Verifier si waypoint atteint
                if distance < self.WAYPOINT_REACHED_DIST:
                    print(f"[Car] Waypoint {self.current_waypoint_idx + 1} atteint!")
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.waypoints):
                        print("[Car] Destination finale atteinte!")
                        break
                    continue

                # Verifier obstacles
                obstacle, obs_dist, avoid_steering = self.check_obstacles()

                if obstacle:
                    # STOP - obstacle trop proche
                    print(f"[Car] STOP! Obstacle a {obs_dist:.2f}m")
                    self.motor.set_duty_cycle(0)
                    time.sleep(0.5)
                    continue

                # Calculer commandes
                speed, steering = self.compute_control(target_lat, target_lon)

                # Ajuster pour evitement
                if obs_dist < self.OBSTACLE_SLOW_DIST:
                    speed *= 0.5
                    steering += avoid_steering

                # Appliquer commandes
                self.motor.set_duty_cycle(speed)
                self.motor.set_servo(0.5 + steering)  # 0.5 = centre

                # Affichage
                print(f"\r[Car] WP{self.current_waypoint_idx + 1}: "
                      f"dist={distance:.1f}m, speed={speed:.2f}, steer={steering:.2f}, "
                      f"obs={obs_dist:.1f}m", end='')

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[Car] Arret demande par l'utilisateur")

        finally:
            # Arreter le moteur
            self.motor.set_duty_cycle(0)
            print("[Car] Navigation terminee.")


def main():
    parser = argparse.ArgumentParser(description='Voiture RC autonome')
    parser.add_argument('--target-lat', type=float, help='Latitude cible')
    parser.add_argument('--target-lon', type=float, help='Longitude cible')
    parser.add_argument('--waypoints', type=str, help='Fichier waypoints')
    parser.add_argument('--gps-port', default='/dev/ttyUSB0', help='Port GPS')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB1', help='Port LiDAR')
    parser.add_argument('--vesc-port', default='/dev/ttyACM0', help='Port VESC')

    args = parser.parse_args()

    # Creer controleur
    car = RobocarController()
    car.GPS_PORT = args.gps_port
    car.LIDAR_PORT = args.lidar_port
    car.VESC_PORT = args.vesc_port

    # Charger waypoints
    if args.waypoints:
        car.load_waypoints(args.waypoints)
    elif args.target_lat and args.target_lon:
        car.add_waypoint(args.target_lat, args.target_lon)
    else:
        # Waypoint par defaut (10m devant position EPITA)
        print("[Car] Aucune destination specifiee. Utilisez --target-lat/--target-lon")
        print("      ou --waypoints fichier.txt")
        return 1

    # Initialiser hardware
    if not car.init_hardware():
        print("[Car] Echec initialisation hardware")
        return 1

    try:
        car.run()
    finally:
        car.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
