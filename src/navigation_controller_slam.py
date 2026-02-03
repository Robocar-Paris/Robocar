#!/usr/bin/env python3
"""
Controleur de Navigation Avance avec SLAM
==========================================

Ce controleur integre:
1. SLAM pour construire la carte en temps reel
2. Planificateur Global (A*) pour trouver le chemin optimal
3. Planificateur Local (DWA) pour evitement dynamique d'obstacles

Le robot:
- Construit une carte au fur et a mesure avec le LiDAR
- Utilise le GPS RTK pour la localisation globale
- Planifie le chemin optimal vers la destination
- Evite les obstacles en temps reel

Usage:
    controller = SLAMNavigationController(lidar, gps, motor)
    controller.set_destination_gps(lat, lon)
    controller.run()
"""

import math
import time
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

from interface import (
    ILidarSensor, IGPSSensor, IMotorController,
    LidarData, GPSData
)
from slam.slam_core import SLAM, SLAMConfig, SLAMPose
from slam.occupancy_grid import OccupancyGrid
from navigation.global_planner import GlobalPlanner, PlannerConfig
from navigation.local_planner import DWAPlanner, DWAConfig, VelocityCommand


@dataclass
class NavigationConfig:
    """Configuration de la navigation."""
    # Tolerances
    waypoint_tolerance: float = 0.5          # metres
    goal_tolerance: float = 0.3              # metres

    # Vitesses
    max_speed: float = 0.8                   # m/s
    min_speed: float = 0.1                   # m/s

    # Securite
    emergency_stop_distance: float = 0.3     # metres
    slow_down_distance: float = 1.0          # metres

    # Replanification
    replan_distance: float = 0.5             # replanifier tous les X metres
    replan_on_obstacle: bool = True

    # Frequence
    control_rate: float = 20.0               # Hz

    # GPS
    origin_lat: float = 48.8156              # Latitude origine
    origin_lon: float = 2.3631               # Longitude origine


@dataclass
class NavigationStatus:
    """Status de la navigation."""
    # Position
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    speed: float = 0.0

    # GPS
    latitude: float = 0.0
    longitude: float = 0.0
    gps_quality: int = 0
    gps_accuracy: float = 99.0

    # Navigation
    state: str = "IDLE"                      # IDLE, MAPPING, NAVIGATING, REACHED, ERROR
    current_waypoint: int = 0
    total_waypoints: int = 0
    distance_to_goal: float = float('inf')
    distance_traveled: float = 0.0

    # SLAM
    map_coverage: float = 0.0                # %
    slam_updates: int = 0

    # Obstacles
    nearest_obstacle: float = float('inf')
    obstacles_detected: int = 0

    # Planification
    path_length: float = 0.0
    path_points: int = 0

    # Temps
    elapsed_time: float = 0.0


class SLAMNavigationController:
    """
    Controleur de navigation avec SLAM integre.

    Flux de donnees:
    1. LiDAR → SLAM → Carte d'occupation
    2. GPS → Position globale → Conversion locale
    3. Carte + Position + Destination → A* → Chemin global
    4. Chemin + Obstacles → DWA → Commandes moteur
    """

    def __init__(
        self,
        lidar: ILidarSensor,
        gps: IGPSSensor,
        motor: IMotorController,
        config: Optional[NavigationConfig] = None
    ):
        self.lidar = lidar
        self.gps = gps
        self.motor = motor
        self.config = config or NavigationConfig()

        # SLAM
        slam_config = SLAMConfig(
            map_size_pixels=800,
            map_size_meters=40.0,
            scan_size=360
        )
        self.slam = SLAM(slam_config)

        # Grille d'occupation pour la planification
        self.occupancy_grid = OccupancyGrid(
            width=800,
            height=800,
            resolution=0.05  # 5cm par pixel
        )

        # Planificateurs
        planner_config = PlannerConfig(
            robot_radius=0.15,
            safety_margin=0.10
        )
        self.global_planner = GlobalPlanner(planner_config)

        dwa_config = DWAConfig(
            max_vel_x=self.config.max_speed,
            min_vel_x=0.0,
            min_obstacle_distance=self.config.emergency_stop_distance
        )
        self.local_planner = DWAPlanner(dwa_config)

        # Etat
        self.status = NavigationStatus()
        self.running = False
        self.paused = False

        # Destination et chemin
        self.destination: Optional[Tuple[float, float]] = None
        self.global_path: List[Tuple[float, float]] = []
        self.waypoints: List[Tuple[float, float]] = []

        # Historique
        self.trajectory: List[Tuple[float, float]] = []
        self.obstacle_points: List[Tuple[float, float]] = []

        # Derniere position pour replanification
        self._last_replan_pos: Tuple[float, float] = (0, 0)
        self._start_time: float = 0

        # Velocite actuelle pour DWA
        self._current_vel: Tuple[float, float] = (0.0, 0.0)

    def set_gps_origin(self, lat: float, lon: float):
        """Definit l'origine GPS pour conversion locale."""
        self.config.origin_lat = lat
        self.config.origin_lon = lon

    def gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convertit coordonnees GPS en position locale (metres)."""
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.config.origin_lat))

        x = (lon - self.config.origin_lon) * meters_per_deg_lon
        y = (lat - self.config.origin_lat) * meters_per_deg_lat

        return x, y

    def local_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        """Convertit position locale en coordonnees GPS."""
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.config.origin_lat))

        lat = self.config.origin_lat + (y / meters_per_deg_lat)
        lon = self.config.origin_lon + (x / meters_per_deg_lon)

        return lat, lon

    def set_destination_gps(self, lat: float, lon: float):
        """Definit la destination en coordonnees GPS."""
        self.destination = self.gps_to_local(lat, lon)
        self.waypoints = [self.destination]
        self.status.total_waypoints = 1
        print(f"[NAV-SLAM] Destination GPS: ({lat:.6f}, {lon:.6f})")
        print(f"[NAV-SLAM] Destination locale: ({self.destination[0]:.1f}, {self.destination[1]:.1f})")

    def set_destination_local(self, x: float, y: float):
        """Definit la destination en coordonnees locales."""
        self.destination = (x, y)
        self.waypoints = [self.destination]
        self.status.total_waypoints = 1
        print(f"[NAV-SLAM] Destination locale: ({x:.1f}, {y:.1f})")

    def add_waypoint_gps(self, lat: float, lon: float):
        """Ajoute un waypoint GPS."""
        local = self.gps_to_local(lat, lon)
        self.waypoints.append(local)
        self.status.total_waypoints = len(self.waypoints)

    def set_initial_pose(self, x: float, y: float, theta: float):
        """Definit la pose initiale."""
        self.status.x = x
        self.status.y = y
        self.status.theta = theta
        self._last_replan_pos = (x, y)

    def init_sensors(self) -> bool:
        """Initialise les capteurs."""
        print("[NAV-SLAM] Initialisation des capteurs...")

        if not self.lidar.start():
            print("[ERREUR] LiDAR non disponible")
            return False
        print("  [OK] LiDAR")

        if not self.gps.start():
            print("[ERREUR] GPS non disponible")
            return False
        print("  [OK] GPS")

        if not self.motor.start():
            print("[ERREUR] Moteur non disponible")
            return False
        print("  [OK] Moteur")

        return True

    def update_from_gps(self):
        """Met a jour la position depuis le GPS."""
        pos = self.gps.get_position()
        if pos and pos.is_valid:
            self.status.latitude = pos.latitude
            self.status.longitude = pos.longitude
            self.status.gps_quality = pos.quality
            self.status.gps_accuracy = getattr(pos, 'accuracy_h', 99.0)

            # Convertir en local
            x, y = self.gps_to_local(pos.latitude, pos.longitude)
            self.status.x = x
            self.status.y = y

            # Heading depuis GPS si disponible
            if hasattr(pos, 'heading') and pos.heading != 0:
                self.status.theta = math.radians(pos.heading)

    def update_slam(self, scan: LidarData):
        """Met a jour le SLAM avec un nouveau scan."""
        if not scan or not scan.points:
            return

        # Convertir scan en mm pour SLAM
        scan_mm = []
        for point in scan.points:
            if point.valid:
                scan_mm.append(int(point.distance * 1000))
            else:
                scan_mm.append(12000)  # Max range

        # Assurer qu'on a 360 points
        if len(scan_mm) < 360:
            # Interpoler
            indices = np.linspace(0, len(scan_mm)-1, 360).astype(int)
            scan_mm = [scan_mm[i] for i in indices]
        elif len(scan_mm) > 360:
            # Sous-echantillonner
            indices = np.linspace(0, len(scan_mm)-1, 360).astype(int)
            scan_mm = [scan_mm[i] for i in indices]

        # Update SLAM
        pose = self.slam.update(scan_mm)
        self.status.slam_updates += 1

        # Mettre a jour grille d'occupation
        ranges = np.array([p.distance for p in scan.points if p.valid])
        angles = np.array([p.angle for p in scan.points if p.valid])

        if len(ranges) > 0:
            self.occupancy_grid.update_from_scan(
                self.status.x, self.status.y, self.status.theta,
                ranges, angles
            )

    def extract_obstacles(self, scan: LidarData) -> List[Tuple[float, float]]:
        """Extrait les obstacles du scan LiDAR."""
        obstacles = []
        self.obstacle_points = []

        if not scan:
            return obstacles

        min_dist = float('inf')

        for point in scan.points:
            if not point.valid:
                continue

            # Convertir en coordonnees monde
            angle_world = point.angle + self.status.theta
            ox = self.status.x + point.distance * math.cos(angle_world)
            oy = self.status.y + point.distance * math.sin(angle_world)

            obstacles.append((ox, oy))
            self.obstacle_points.append((ox, oy))

            # Trouver obstacle le plus proche (devant)
            angle_rel = math.degrees(point.angle)
            if -60 < angle_rel < 60 and point.distance < min_dist:
                min_dist = point.distance

        self.status.nearest_obstacle = min_dist
        self.status.obstacles_detected = len(obstacles)

        return obstacles

    def plan_path(self) -> bool:
        """Planifie le chemin global avec A*."""
        if not self.destination:
            return False

        start = (self.status.x, self.status.y)
        goal = self.destination

        # Mettre a jour la carte dans le planificateur
        self.global_planner.set_map(self.occupancy_grid)

        # Planifier
        path = self.global_planner.plan(start, goal)

        if path:
            self.global_path = path
            self.status.path_points = len(path)
            self.status.path_length = self.global_planner.path_length(path)
            print(f"[NAV-SLAM] Chemin trouve: {len(path)} points, {self.status.path_length:.1f}m")
            return True
        else:
            print("[NAV-SLAM] Aucun chemin trouve!")
            self.global_path = []
            return False

    def compute_control(self, obstacles: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Calcule les commandes de controle avec DWA."""
        if not self.global_path:
            return 0.0, 0.0

        # Position et vitesse actuelles
        current_pose = (self.status.x, self.status.y, self.status.theta)

        # Calculer commande DWA
        cmd = self.local_planner.compute_velocity(
            current_pose=current_pose,
            current_vel=self._current_vel,
            global_path=self.global_path,
            obstacles=obstacles,
            goal=self.destination
        )

        if cmd.is_valid:
            self._current_vel = (cmd.linear, cmd.angular)
            return cmd.linear, cmd.angular
        else:
            # Pas de trajectoire valide - stop
            self._current_vel = (0.0, 0.0)
            return 0.0, 0.0

    def apply_control(self, speed: float, steering: float):
        """Applique les commandes au moteur."""
        # Convertir vitesse angulaire en angle de direction
        # (approximation pour Ackermann)
        wheelbase = 0.26  # metres
        if abs(speed) > 0.01:
            steering_angle = math.atan(wheelbase * steering / speed)
        else:
            steering_angle = 0.0

        # Normaliser
        speed_norm = speed / self.config.max_speed
        steering_norm = (steering_angle / 0.3 + 1.0) / 2.0  # 0-1

        self.motor.set_speed(speed_norm)
        self.motor.set_steering(steering_norm)

    def check_should_replan(self) -> bool:
        """Verifie si on doit replanifier."""
        dx = self.status.x - self._last_replan_pos[0]
        dy = self.status.y - self._last_replan_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        return dist > self.config.replan_distance

    def check_goal_reached(self) -> bool:
        """Verifie si la destination est atteinte."""
        if not self.destination:
            return False

        dx = self.destination[0] - self.status.x
        dy = self.destination[1] - self.status.y
        dist = math.sqrt(dx*dx + dy*dy)

        self.status.distance_to_goal = dist

        return dist < self.config.goal_tolerance

    def navigation_step(self) -> bool:
        """Execute une iteration de navigation."""
        # 1. Lire capteurs
        scan = self.lidar.get_scan()

        # 2. Mise a jour GPS
        self.update_from_gps()

        # 3. Mise a jour SLAM
        if scan:
            self.update_slam(scan)

        # 4. Extraire obstacles
        obstacles = self.extract_obstacles(scan) if scan else []

        # 5. Verifier si destination atteinte
        if self.check_goal_reached():
            print("\n[NAV-SLAM] DESTINATION ATTEINTE!")
            self.motor.emergency_stop()
            self.status.state = "REACHED"
            return False

        # 6. Replanifier si necessaire
        if self.check_should_replan() or not self.global_path:
            self._last_replan_pos = (self.status.x, self.status.y)
            self.plan_path()

        # 7. Calculer controle
        speed, steering = self.compute_control(obstacles)

        # 8. Securite: verifier obstacle proche
        if self.status.nearest_obstacle < self.config.emergency_stop_distance:
            print("\n[NAV-SLAM] ARRET D'URGENCE - Obstacle!")
            speed = 0.0
        elif self.status.nearest_obstacle < self.config.slow_down_distance:
            speed *= 0.5

        # 9. Appliquer controle
        self.apply_control(speed, steering)
        self.status.speed = speed

        # 10. Mettre a jour trajectoire
        self.trajectory.append((self.status.x, self.status.y))
        if len(self.trajectory) > 1:
            prev = self.trajectory[-2]
            d = math.sqrt((self.status.x - prev[0])**2 + (self.status.y - prev[1])**2)
            self.status.distance_traveled += d

        return True

    def run(self, visualize: bool = False):
        """Execute la navigation."""
        if not self.destination:
            print("[NAV-SLAM] Aucune destination definie!")
            return

        if not self.init_sensors():
            return

        # Attendre fix GPS
        print("[NAV-SLAM] Attente fix GPS...")
        if hasattr(self.gps, 'wait_for_fix'):
            if not self.gps.wait_for_fix(timeout=60):
                print("[ERREUR] Pas de fix GPS!")
                return

        # Initialiser position depuis GPS
        self.update_from_gps()
        print(f"[NAV-SLAM] Position initiale: ({self.status.x:.1f}, {self.status.y:.1f})")

        # Planification initiale
        if not self.plan_path():
            print("[NAV-SLAM] Impossible de planifier un chemin initial")
            return

        self.running = True
        self.status.state = "NAVIGATING"
        self._start_time = time.time()
        dt = 1.0 / self.config.control_rate

        print("\n[NAV-SLAM] Navigation demarree!")
        print("Appuyez sur Ctrl+C pour arreter\n")

        try:
            while self.running:
                if self.paused:
                    time.sleep(0.1)
                    continue

                # Iteration de navigation
                if not self.navigation_step():
                    break

                # Mise a jour temps
                self.status.elapsed_time = time.time() - self._start_time

                # Affichage status
                self._print_status()

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n[NAV-SLAM] Interruption utilisateur")

        finally:
            self.shutdown()

    def run_with_visualization(self):
        """Execute la navigation avec visualisation matplotlib."""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, FancyArrow
        import matplotlib.colors as mcolors

        if not self.destination:
            print("[NAV-SLAM] Aucune destination definie!")
            return

        if not self.init_sensors():
            return

        # Attendre fix GPS
        print("[NAV-SLAM] Attente fix GPS...")
        if hasattr(self.gps, 'wait_for_fix'):
            self.gps.wait_for_fix(timeout=30)

        self.update_from_gps()

        # Planification initiale
        self.plan_path()

        # Setup matplotlib
        plt.ion()
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))
        ax_map, ax_local = axes

        fig.canvas.manager.set_window_title('Robocar - Navigation SLAM')

        self.running = True
        self.status.state = "NAVIGATING"
        self._start_time = time.time()
        dt = 1.0 / self.config.control_rate

        def on_key(event):
            if event.key == ' ':
                self.paused = not self.paused
            elif event.key == 'q':
                self.running = False

        fig.canvas.mpl_connect('key_press_event', on_key)

        print("\n[NAV-SLAM] Navigation avec visualisation")
        print("[ESPACE] Pause | [Q] Quitter\n")

        try:
            while self.running and plt.fignum_exists(fig.number):
                if self.paused:
                    plt.pause(0.1)
                    continue

                # Navigation step
                if not self.navigation_step():
                    break

                self.status.elapsed_time = time.time() - self._start_time

                # === VISUALISATION ===
                ax_map.clear()
                ax_local.clear()

                # Carte SLAM
                slam_map = self.slam.get_map_as_image()
                ax_map.imshow(slam_map, origin='lower', extent=[
                    -20, 20, -20, 20
                ])
                ax_map.set_title('Carte SLAM')
                ax_map.set_xlabel('X (m)')
                ax_map.set_ylabel('Y (m)')

                # Robot sur carte
                ax_map.plot(self.status.x, self.status.y, 'bo', markersize=10)

                # Destination
                if self.destination:
                    ax_map.plot(self.destination[0], self.destination[1],
                               'r*', markersize=15)

                # Chemin global
                if self.global_path:
                    path = np.array(self.global_path)
                    ax_map.plot(path[:, 0], path[:, 1], 'g-', linewidth=2, alpha=0.7)

                # Vue locale
                ax_local.set_xlim(-5, 5)
                ax_local.set_ylim(-5, 5)
                ax_local.set_aspect('equal')
                ax_local.grid(True, alpha=0.3)
                ax_local.set_title('Vue Locale (LiDAR)')

                # Points LiDAR
                if self.obstacle_points:
                    rel_obs = []
                    for ox, oy in self.obstacle_points:
                        # Convertir en relatif robot
                        dx = ox - self.status.x
                        dy = oy - self.status.y
                        cos_t = math.cos(-self.status.theta)
                        sin_t = math.sin(-self.status.theta)
                        rx = dx * cos_t - dy * sin_t
                        ry = dx * sin_t + dy * cos_t
                        rel_obs.append((rx, ry))

                    if rel_obs:
                        obs = np.array(rel_obs)
                        ax_local.scatter(obs[:, 0], obs[:, 1], c='red', s=2, alpha=0.5)

                # Robot au centre
                ax_local.plot(0, 0, 'bo', markersize=15)
                ax_local.arrow(0, 0, 0.5, 0, head_width=0.2, color='blue')

                # Info text
                info = (
                    f"Position: ({self.status.x:.1f}, {self.status.y:.1f})\n"
                    f"GPS: ({self.status.latitude:.6f}, {self.status.longitude:.6f})\n"
                    f"Qualite GPS: {self.status.gps_quality}\n"
                    f"Distance but: {self.status.distance_to_goal:.1f}m\n"
                    f"Obstacle proche: {self.status.nearest_obstacle:.2f}m\n"
                    f"SLAM updates: {self.status.slam_updates}\n"
                    f"Temps: {self.status.elapsed_time:.1f}s"
                )
                ax_local.text(-4.8, 4.5, info, fontsize=9, verticalalignment='top',
                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

                fig.tight_layout()
                fig.canvas.draw()
                fig.canvas.flush_events()

                plt.pause(dt)

        except KeyboardInterrupt:
            print("\n[NAV-SLAM] Interruption")

        finally:
            self.shutdown()
            plt.ioff()
            plt.show()

    def _print_status(self):
        """Affiche le status dans la console."""
        gps_str = f"Q{self.status.gps_quality}"
        if self.status.gps_quality == 4:
            gps_str = "\033[92mRTK\033[0m"
        elif self.status.gps_quality == 5:
            gps_str = "\033[93mFLOAT\033[0m"

        print(f"\r[NAV-SLAM] "
              f"Pos:({self.status.x:6.1f},{self.status.y:6.1f}) "
              f"GPS:{gps_str} "
              f"Dist:{self.status.distance_to_goal:5.1f}m "
              f"Obs:{self.status.nearest_obstacle:4.1f}m "
              f"Spd:{self.status.speed:4.2f}", end='')

    def shutdown(self):
        """Arret propre."""
        print("\n\n[NAV-SLAM] Arret...")
        self.running = False
        self.status.state = "IDLE"

        if self.motor:
            self.motor.emergency_stop()
            self.motor.stop()

        if self.lidar:
            self.lidar.stop()

        if self.gps:
            self.gps.stop()

        self._print_summary()

    def _print_summary(self):
        """Affiche le resume de la navigation."""
        print("\n" + "=" * 60)
        print("   RESUME NAVIGATION SLAM")
        print("=" * 60)
        print(f"Status final: {self.status.state}")
        print(f"Temps total: {self.status.elapsed_time:.1f}s")
        print(f"Distance parcourue: {self.status.distance_traveled:.1f}m")
        print(f"SLAM updates: {self.status.slam_updates}")
        if self.status.elapsed_time > 0:
            avg_speed = self.status.distance_traveled / self.status.elapsed_time
            print(f"Vitesse moyenne: {avg_speed:.2f} m/s")
        print("=" * 60)

    def save_map(self, filepath: str):
        """Sauvegarde la carte construite."""
        self.occupancy_grid.save(filepath, filepath.replace('.png', '.yaml'))
        print(f"[NAV-SLAM] Carte sauvegardee: {filepath}")


# Fonction utilitaire pour creer le controleur facilement
def create_slam_navigation(
    lidar_port: str = '/dev/ttyUSB0',
    gps_port: str = '/dev/ttyUSB1',
    vesc_port: str = '/dev/ttyACM0',
    polaris_key: Optional[str] = None
) -> SLAMNavigationController:
    """
    Cree un controleur de navigation SLAM avec les vrais capteurs.

    Args:
        lidar_port: Port du LiDAR
        gps_port: Port du GPS
        vesc_port: Port du VESC
        polaris_key: Cle API Polaris (optionnel)

    Returns:
        SLAMNavigationController configure
    """
    from interface import RealLidarAdapter, RealGPSAdapter, RealMotorAdapter

    lidar = RealLidarAdapter(lidar_port)
    gps = RealGPSAdapter(gps_port, polaris_api_key=polaris_key)
    motor = RealMotorAdapter(vesc_port)

    return SLAMNavigationController(lidar, gps, motor)


if __name__ == '__main__':
    import sys
    import argparse

    parser = argparse.ArgumentParser(description='Navigation SLAM Robocar')
    parser.add_argument('--target-lat', type=float, help='Latitude destination')
    parser.add_argument('--target-lon', type=float, help='Longitude destination')
    parser.add_argument('--simulation', action='store_true', help='Mode simulation')
    parser.add_argument('--visualize', action='store_true', help='Avec visualisation')

    args = parser.parse_args()

    if args.simulation:
        # Mode simulation
        from interface import (
            SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
        )
        from simulation import create_epitech_env

        env = create_epitech_env()
        lidar = SimulatedLidarAdapter(env)
        gps = SimulatedGPSAdapter()
        motor = SimulatedMotorAdapter()

        controller = SLAMNavigationController(lidar, gps, motor)
        controller.set_initial_pose(0, 0, 0)
        controller.set_destination_local(10, 5)

        if args.visualize:
            controller.run_with_visualization()
        else:
            controller.run()

    elif args.target_lat and args.target_lon:
        # Mode reel
        controller = create_slam_navigation()
        controller.set_destination_gps(args.target_lat, args.target_lon)

        if args.visualize:
            controller.run_with_visualization()
        else:
            controller.run()

    else:
        print("Usage:")
        print("  Mode simulation: python navigation_controller_slam.py --simulation")
        print("  Mode reel: python navigation_controller_slam.py --target-lat 48.8970 --target-lon 2.2195")
