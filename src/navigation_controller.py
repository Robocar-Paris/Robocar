#!/usr/bin/env python3
"""
Controleur de navigation unifie pour Robocar.

Ce controleur fonctionne avec les interfaces abstraites,
permettant d'utiliser le meme code de navigation en:
- Mode simulation (PC avec matplotlib)
- Mode reel (Jetson Nano avec vrais capteurs)
"""

import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

from interface import (
    ILidarSensor, IGPSSensor, IMotorController,
    LidarData, GPSData,
    SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
)


@dataclass
class Waypoint:
    """Point de passage."""
    # Position locale (metres)
    x: float = 0.0
    y: float = 0.0
    # Position GPS (si disponible)
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    # Etat
    reached: bool = False


@dataclass
class NavigationState:
    """Etat de la navigation."""
    # Position actuelle
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    speed: float = 0.0

    # GPS
    latitude: float = 0.0
    longitude: float = 0.0
    gps_quality: int = 0

    # Navigation
    current_waypoint_idx: int = 0
    distance_to_waypoint: float = 0.0
    heading_error: float = 0.0

    # Obstacles
    obstacle_detected: bool = False
    obstacle_distance: float = float('inf')

    # Statistiques
    total_distance: float = 0.0
    elapsed_time: float = 0.0


class NavigationController:
    """
    Controleur de navigation autonome.

    Utilise les interfaces abstraites pour fonctionner
    independamment du mode (simulation ou reel).
    """

    # Configuration
    WAYPOINT_REACHED_DIST = 0.5      # Distance pour considerer waypoint atteint (m)
    MAX_SPEED = 0.5                   # Vitesse max normalisee
    MIN_SPEED = 0.1                   # Vitesse min normalisee
    OBSTACLE_STOP_DIST = 0.5          # Distance arret obstacle (m)
    OBSTACLE_SLOW_DIST = 1.5          # Distance ralentissement (m)
    STEERING_GAIN = 2.0               # Gain de direction
    LOOP_RATE = 20                    # Hz

    # Coordonnees de reference (Epitech Paris)
    ORIGIN_LAT = 48.8156
    ORIGIN_LON = 2.3631

    def __init__(self,
                 lidar: ILidarSensor,
                 gps: IGPSSensor,
                 motor: IMotorController,
                 mode: str = 'simulation'):
        """
        Initialise le controleur.

        Args:
            lidar: Capteur LiDAR (reel ou simule)
            gps: Capteur GPS (reel ou simule)
            motor: Controleur moteur (reel ou simule)
            mode: 'simulation' ou 'car'
        """
        self.lidar = lidar
        self.gps = gps
        self.motor = motor
        self.mode = mode

        # Waypoints
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_idx = 0

        # Etat
        self.state = NavigationState()
        self.running = False
        self.paused = False
        self.start_time = 0.0

        # Historique de la trajectoire (pour visualisation)
        self.trajectory: List[Tuple[float, float]] = []

        # Points LiDAR actuels (pour visualisation)
        self.scan_points: List[Tuple[float, float]] = []

        # Origine GPS
        self.origin_lat = self.ORIGIN_LAT
        self.origin_lon = self.ORIGIN_LON

    def set_initial_pose(self, x: float, y: float, theta: float):
        """Definit la pose initiale."""
        self.state.x = x
        self.state.y = y
        self.state.theta = theta

        # Mettre a jour les simulateurs si applicable
        if isinstance(self.motor, SimulatedMotorAdapter):
            self.motor.set_pose(x, y, theta)
        if isinstance(self.lidar, SimulatedLidarAdapter):
            self.lidar.set_robot_pose(x, y, theta)
        if isinstance(self.gps, SimulatedGPSAdapter):
            self.gps.set_local_position(x, y, theta)

    def set_gps_origin(self, lat: float, lon: float):
        """Definit l'origine GPS pour la conversion locale."""
        self.origin_lat = lat
        self.origin_lon = lon

    def add_waypoint_local(self, x: float, y: float):
        """Ajoute un waypoint en coordonnees locales (metres)."""
        self.waypoints.append(Waypoint(x=x, y=y))

    def add_waypoint_gps(self, lat: float, lon: float):
        """Ajoute un waypoint en coordonnees GPS."""
        # Convertir en local
        x, y = self._gps_to_local(lat, lon)
        self.waypoints.append(Waypoint(x=x, y=y, latitude=lat, longitude=lon))

    def _gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convertit des coordonnees GPS en position locale."""
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        x = (lon - self.origin_lon) * meters_per_deg_lon
        y = (lat - self.origin_lat) * meters_per_deg_lat

        return x, y

    def _local_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        """Convertit une position locale en coordonnees GPS."""
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        lat = self.origin_lat + (y / meters_per_deg_lat)
        lon = self.origin_lon + (x / meters_per_deg_lon)

        return lat, lon

    def distance_to_waypoint(self, wp: Waypoint) -> float:
        """Calcule la distance jusqu'au waypoint."""
        dx = wp.x - self.state.x
        dy = wp.y - self.state.y
        return math.sqrt(dx * dx + dy * dy)

    def bearing_to_waypoint(self, wp: Waypoint) -> float:
        """Calcule le cap vers le waypoint (radians)."""
        dx = wp.x - self.state.x
        dy = wp.y - self.state.y
        return math.atan2(dy, dx)

    def check_obstacles(self) -> Tuple[bool, float, float]:
        """
        Verifie les obstacles via LiDAR.

        Returns:
            (obstacle_detected, min_distance, suggested_steering)
        """
        scan = self.lidar.get_scan()
        if not scan:
            return False, float('inf'), 0.0

        # Reinitialiser les points pour visualisation
        self.scan_points = []

        min_dist_front = float('inf')
        min_dist_left = float('inf')
        min_dist_right = float('inf')

        for point in scan.points:
            if not point.valid:
                continue

            # Coordonnees dans le repere monde pour visualisation
            angle_world = point.angle + self.state.theta
            px = self.state.x + point.distance * math.cos(angle_world)
            py = self.state.y + point.distance * math.sin(angle_world)
            self.scan_points.append((px, py))

            angle_deg = math.degrees(point.angle)

            # Zone avant (-45 a +45 deg)
            if -45 < angle_deg < 45:
                if point.distance < min_dist_front:
                    min_dist_front = point.distance

            # Zone gauche (45 a 90 deg)
            if 45 < angle_deg < 90:
                if point.distance < min_dist_left:
                    min_dist_left = point.distance

            # Zone droite (-90 a -45 deg)
            if -90 < angle_deg < -45:
                if point.distance < min_dist_right:
                    min_dist_right = point.distance

        # Determiner la direction d'evitement
        steering = 0.0
        if min_dist_front < self.OBSTACLE_SLOW_DIST:
            # Tourner vers le cote le plus libre
            if min_dist_left > min_dist_right:
                steering = 0.5  # Tourner a gauche
            else:
                steering = -0.5  # Tourner a droite

        obstacle_detected = min_dist_front < self.OBSTACLE_STOP_DIST

        # Mettre a jour l'etat
        self.state.obstacle_detected = obstacle_detected
        self.state.obstacle_distance = min_dist_front

        return obstacle_detected, min_dist_front, steering

    def compute_control(self, wp: Waypoint) -> Tuple[float, float]:
        """
        Calcule les commandes de controle.

        Args:
            wp: Waypoint cible

        Returns:
            (speed, steering)
        """
        # Distance et cap vers le waypoint
        distance = self.distance_to_waypoint(wp)
        target_bearing = self.bearing_to_waypoint(wp)

        # Erreur de cap
        heading_error = target_bearing - self.state.theta
        # Normaliser entre -pi et pi
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        self.state.heading_error = heading_error
        self.state.distance_to_waypoint = distance

        # Commande de direction
        steering = self.STEERING_GAIN * heading_error
        steering = max(-1.0, min(1.0, steering))

        # Vitesse de base
        speed = self.MAX_SPEED

        # Ralentir si proche du waypoint
        if distance < 2.0:
            speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * (distance / 2.0)

        return speed, steering

    def update_state_from_gps(self):
        """Met a jour l'etat depuis le GPS (mode reel)."""
        pos = self.gps.get_position()
        if pos and pos.is_valid:
            self.state.latitude = pos.latitude
            self.state.longitude = pos.longitude
            self.state.gps_quality = pos.quality

            # Convertir en local
            self.state.x, self.state.y = self._gps_to_local(pos.latitude, pos.longitude)

            # Cap depuis le GPS si disponible
            if hasattr(pos, 'heading') and pos.heading != 0:
                self.state.theta = pos.heading

    def update_state_from_motor(self, dt: float):
        """Met a jour l'etat depuis le moteur simule."""
        if isinstance(self.motor, SimulatedMotorAdapter):
            self.motor.update_physics(dt)
            x, y, theta = self.motor.get_pose()
            speed, _ = self.motor.get_velocity()

            self.state.x = x
            self.state.y = y
            self.state.theta = theta
            self.state.speed = speed

            # Mettre a jour les autres simulateurs
            if isinstance(self.lidar, SimulatedLidarAdapter):
                self.lidar.set_robot_pose(x, y, theta)
            if isinstance(self.gps, SimulatedGPSAdapter):
                self.gps.set_local_position(x, y, theta, speed)

            # Calculer GPS simule
            lat, lon = self._local_to_gps(x, y)
            self.state.latitude = lat
            self.state.longitude = lon

            # Ajouter a la trajectoire
            self.trajectory.append((x, y))

            # Mettre a jour distance totale
            if len(self.trajectory) > 1:
                prev_x, prev_y = self.trajectory[-2]
                d = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                self.state.total_distance += d

    def navigation_step(self) -> bool:
        """
        Execute une iteration de navigation.

        Returns:
            True si navigation en cours, False si terminee
        """
        if not self.waypoints or self.current_waypoint_idx >= len(self.waypoints):
            return False

        # Waypoint courant
        wp = self.waypoints[self.current_waypoint_idx]
        distance = self.distance_to_waypoint(wp)

        # Verifier si waypoint atteint
        if distance < self.WAYPOINT_REACHED_DIST:
            wp.reached = True
            self.current_waypoint_idx += 1
            print(f"[NAV] Waypoint {self.current_waypoint_idx} atteint!")

            if self.current_waypoint_idx >= len(self.waypoints):
                print("[NAV] Destination finale atteinte!")
                self.motor.emergency_stop()
                return False
            return True

        # Verifier obstacles
        obstacle, obs_dist, avoid_steering = self.check_obstacles()

        if obstacle:
            # STOP - obstacle trop proche
            self.motor.set_speed(0)
            self.motor.set_steering(avoid_steering)
            return True

        # Calculer commandes de navigation
        speed, steering = self.compute_control(wp)

        # Ajuster pour evitement
        if obs_dist < self.OBSTACLE_SLOW_DIST:
            speed *= 0.5
            steering += avoid_steering * 0.5

        # Appliquer commandes
        self.motor.set_speed(speed)
        self.motor.set_steering(steering)

        return True

    def init_sensors(self) -> bool:
        """Initialise tous les capteurs."""
        print("[NAV] Initialisation des capteurs...")

        if not self.lidar.start():
            print("[ERREUR] LiDAR non disponible")
            return False
        print("  LiDAR OK")

        if not self.gps.start():
            print("[ERREUR] GPS non disponible")
            return False
        print("  GPS OK")

        if not self.motor.start():
            print("[ERREUR] Moteur non disponible")
            return False
        print("  Moteur OK")

        return True

    def shutdown(self):
        """Arrete proprement tous les composants."""
        print("[NAV] Arret...")
        self.running = False

        if self.motor:
            self.motor.emergency_stop()
            self.motor.stop()

        if self.lidar:
            self.lidar.stop()

        if self.gps:
            self.gps.stop()

        print("[NAV] Arret termine.")

    def run_headless(self):
        """Execute la navigation sans visualisation."""
        if not self.waypoints:
            print("[NAV] Aucun waypoint defini!")
            return

        if not self.init_sensors():
            return

        self.running = True
        self.start_time = time.time()
        dt = 1.0 / self.LOOP_RATE

        print(f"\n[NAV] Demarrage navigation vers {len(self.waypoints)} waypoints")
        print("[NAV] Appuyez sur Ctrl+C pour arreter\n")

        try:
            while self.running:
                # Mise a jour etat
                if self.mode == 'simulation':
                    self.update_state_from_motor(dt)
                else:
                    self.update_state_from_gps()

                # Iteration de navigation
                if not self.navigation_step():
                    break

                # Affichage etat
                self.state.elapsed_time = time.time() - self.start_time
                wp_idx = min(self.current_waypoint_idx, len(self.waypoints) - 1)
                print(f"\r[NAV] WP{wp_idx + 1}/{len(self.waypoints)}: "
                      f"pos=({self.state.x:.1f},{self.state.y:.1f}) "
                      f"dist={self.state.distance_to_waypoint:.1f}m "
                      f"obs={self.state.obstacle_distance:.1f}m", end='')

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n[NAV] Interruption utilisateur")

        finally:
            self.shutdown()
            self._print_summary()

    def run_with_gui(self):
        """Execute la navigation avec visualisation matplotlib."""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Circle
        import numpy as np

        if not self.waypoints:
            print("[NAV] Aucun waypoint defini!")
            return

        if not self.init_sensors():
            return

        # Configuration matplotlib
        plt.ion()
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        fig.canvas.manager.set_window_title('Robocar - Navigation')

        # Determiner les limites de la zone
        all_x = [self.state.x] + [wp.x for wp in self.waypoints]
        all_y = [self.state.y] + [wp.y for wp in self.waypoints]
        margin = 5
        x_min, x_max = min(all_x) - margin, max(all_x) + margin
        y_min, y_max = min(all_y) - margin, max(all_y) + margin

        # Agrandir si environnement plus grand
        x_min, x_max = min(x_min, -20), max(x_max, 20)
        y_min, y_max = min(y_min, -20), max(y_max, 20)

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (metres)')
        ax.set_ylabel('Y (metres)')

        # Elements graphiques
        robot_patch = None
        robot_arrow = None
        lidar_scatter = None
        trajectory_line = None
        waypoint_markers = []
        info_text = None

        self.running = True
        self.start_time = time.time()
        dt = 1.0 / self.LOOP_RATE

        def on_key(event):
            if event.key == ' ':
                self.paused = not self.paused
            elif event.key == 'q':
                self.running = False

        def on_close(event):
            self.running = False

        fig.canvas.mpl_connect('key_press_event', on_key)
        fig.canvas.mpl_connect('close_event', on_close)

        print(f"\n[NAV] Demarrage navigation vers {len(self.waypoints)} waypoints")
        print("Controles: [ESPACE] Pause | [Q] Quitter\n")

        try:
            while self.running and plt.fignum_exists(fig.number):
                if self.paused:
                    plt.pause(0.1)
                    continue

                # Mise a jour etat
                self.update_state_from_motor(dt)

                # Iteration de navigation
                nav_continue = self.navigation_step()

                # === MISE A JOUR GRAPHIQUE ===

                # Effacer elements precedents
                if robot_patch:
                    robot_patch.remove()
                if robot_arrow:
                    robot_arrow.remove()
                if lidar_scatter:
                    lidar_scatter.remove()
                if trajectory_line:
                    trajectory_line.remove()
                if info_text:
                    info_text.remove()
                for marker in waypoint_markers:
                    marker.remove()
                waypoint_markers.clear()

                # Dessiner trajectoire
                if len(self.trajectory) > 1:
                    traj = np.array(self.trajectory)
                    trajectory_line, = ax.plot(
                        traj[:, 0], traj[:, 1],
                        'b-', alpha=0.5, linewidth=2, zorder=2
                    )

                # Dessiner points LiDAR
                if self.scan_points:
                    points = np.array(self.scan_points)
                    distances = np.sqrt(
                        (points[:, 0] - self.state.x)**2 +
                        (points[:, 1] - self.state.y)**2
                    )
                    lidar_scatter = ax.scatter(
                        points[:, 0], points[:, 1],
                        c=distances, cmap='RdYlGn', s=5, alpha=0.6, zorder=3
                    )

                # Dessiner waypoints
                for i, wp in enumerate(self.waypoints):
                    if i < self.current_waypoint_idx:
                        color, marker, size = 'green', 'o', 80
                    elif i == self.current_waypoint_idx:
                        color, marker, size = 'red', 'X', 150
                    else:
                        color, marker, size = 'orange', 'o', 100

                    scatter = ax.scatter(wp.x, wp.y, c=color, marker=marker,
                                        s=size, zorder=5, edgecolors='black')
                    waypoint_markers.append(scatter)

                    text = ax.text(wp.x + 0.3, wp.y + 0.3, str(i + 1),
                                  fontsize=10, fontweight='bold')
                    waypoint_markers.append(text)

                # Dessiner robot
                robot_length, robot_width = 0.4, 0.25
                corners = np.array([
                    [-robot_length/2, -robot_width/2],
                    [robot_length/2, -robot_width/2],
                    [robot_length/2, robot_width/2],
                    [-robot_length/2, robot_width/2]
                ])
                cos_t = np.cos(self.state.theta)
                sin_t = np.sin(self.state.theta)
                rot_matrix = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
                corners = corners @ rot_matrix.T
                corners += np.array([self.state.x, self.state.y])

                robot_patch = Polygon(corners, closed=True,
                                     facecolor='blue', edgecolor='darkblue',
                                     alpha=0.8, zorder=10)
                ax.add_patch(robot_patch)

                # Fleche de direction
                arrow_length = 0.5
                dx = arrow_length * cos_t
                dy = arrow_length * sin_t
                robot_arrow = ax.arrow(self.state.x, self.state.y, dx, dy,
                                      head_width=0.15, head_length=0.1,
                                      fc='yellow', ec='black', zorder=11)

                # Informations
                self.state.elapsed_time = time.time() - self.start_time
                wp_idx = min(self.current_waypoint_idx, len(self.waypoints) - 1)
                wp_info = (f"Waypoint {wp_idx + 1}/{len(self.waypoints)}: "
                          f"{self.state.distance_to_waypoint:.1f}m")
                if self.current_waypoint_idx >= len(self.waypoints):
                    wp_info = "TERMINE!"

                info = (
                    f"Position: ({self.state.x:.2f}, {self.state.y:.2f})\n"
                    f"GPS: ({self.state.latitude:.6f}, {self.state.longitude:.6f})\n"
                    f"Heading: {math.degrees(self.state.theta):.0f} deg\n"
                    f"Vitesse: {self.state.speed:.2f} m/s\n"
                    f"{wp_info}\n"
                    f"Obstacle: {self.state.obstacle_distance:.1f}m\n"
                    f"Distance totale: {self.state.total_distance:.1f}m\n"
                    f"Temps: {self.state.elapsed_time:.1f}s\n"
                    f"\n[ESPACE] Pause | [Q] Quitter"
                )

                info_text = ax.text(
                    0.02, 0.98, info,
                    transform=ax.transAxes,
                    verticalalignment='top',
                    fontfamily='monospace',
                    fontsize=10,
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
                )

                ax.set_title(
                    f'Robocar Navigation - {"PAUSE" if self.paused else "EN COURS"}',
                    fontsize=14, fontweight='bold'
                )

                fig.canvas.draw()
                fig.canvas.flush_events()

                if not nav_continue:
                    # Navigation terminee, attendre fermeture
                    while self.running and plt.fignum_exists(fig.number):
                        plt.pause(0.1)
                    break

                plt.pause(dt)

        except KeyboardInterrupt:
            print("\n[NAV] Interruption utilisateur")

        finally:
            self.shutdown()
            self._print_summary()

            plt.ioff()
            if plt.fignum_exists(fig.number):
                plt.show()

    def run_real(self):
        """Execute la navigation en mode reel (Jetson Nano)."""
        if not self.waypoints:
            print("[NAV] Aucun waypoint defini!")
            return

        if not self.init_sensors():
            return

        # Attendre fix GPS
        print("[NAV] Attente fix GPS...")
        if not self.gps.wait_for_fix(timeout=60):
            print("[ERREUR] Pas de fix GPS!")
            return

        pos = self.gps.get_position()
        print(f"[NAV] Fix GPS obtenu: ({pos.latitude:.6f}, {pos.longitude:.6f})")
        print(f"      Qualite: {pos.quality_string}, Precision: {pos.accuracy_h:.2f}m")

        # Initialiser position
        self.state.latitude = pos.latitude
        self.state.longitude = pos.longitude
        self.state.x, self.state.y = self._gps_to_local(pos.latitude, pos.longitude)

        self.running = True
        self.start_time = time.time()
        dt = 1.0 / self.LOOP_RATE

        print(f"\n[NAV] Demarrage navigation vers {len(self.waypoints)} waypoints")
        print("[NAV] Appuyez sur Ctrl+C pour arreter\n")

        try:
            while self.running:
                # Mise a jour etat depuis GPS
                self.update_state_from_gps()

                # Iteration de navigation
                if not self.navigation_step():
                    break

                # Affichage etat
                self.state.elapsed_time = time.time() - self.start_time
                wp_idx = min(self.current_waypoint_idx, len(self.waypoints) - 1)
                print(f"\r[NAV] WP{wp_idx + 1}/{len(self.waypoints)}: "
                      f"GPS=({self.state.latitude:.6f},{self.state.longitude:.6f}) "
                      f"dist={self.state.distance_to_waypoint:.1f}m "
                      f"obs={self.state.obstacle_distance:.1f}m "
                      f"qual={self.state.gps_quality}", end='')

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n[NAV] Interruption utilisateur")

        finally:
            self.shutdown()
            self._print_summary()

    def _print_summary(self):
        """Affiche le resume de la navigation."""
        print("\n" + "=" * 50)
        print("   RESUME NAVIGATION")
        print("=" * 50)
        print(f"Temps total: {self.state.elapsed_time:.1f}s")
        print(f"Distance parcourue: {self.state.total_distance:.1f}m")
        print(f"Waypoints atteints: {self.current_waypoint_idx}/{len(self.waypoints)}")
        if self.state.total_distance > 0 and self.state.elapsed_time > 0:
            avg_speed = self.state.total_distance / self.state.elapsed_time
            print(f"Vitesse moyenne: {avg_speed:.2f} m/s")
        print("=" * 50)
