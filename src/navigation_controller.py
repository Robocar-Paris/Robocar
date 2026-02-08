#!/usr/bin/env python3
"""
Controleur de navigation LiDAR pour Robocar.

La voiture avance tout droit et utilise le LiDAR pour
l'evitement d'obstacles en temps reel.
Fonctionne en mode simulation (PC) ou reel (Jetson Nano).
"""

import math
import time
import os
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

from interface import (
    ILidarSensor, IMotorController,
    LidarData,
    SimulatedLidarAdapter, SimulatedMotorAdapter
)

# Import perception modules
from perception.lidar_processor import LidarProcessor, ProcessedScan
from perception.obstacle_detector import ObstacleDetector, Obstacle, ObstacleType


# Dataclass locale pour la conversion vers le format du processeur
@dataclass
class _ProcessorLidarPoint:
    """Point LiDAR pour le processeur."""
    angle: float
    distance: float
    intensity: int
    valid: bool


@dataclass
class _ProcessorLidarScan:
    """Scan LiDAR pour le processeur."""
    timestamp: float
    points: List['_ProcessorLidarPoint']
    scan_frequency: float


@dataclass
class NavigationState:
    """Etat de la navigation."""
    # Position actuelle
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    speed: float = 0.0

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
    MAX_SPEED = 0.5                   # Vitesse max normalisee
    MIN_SPEED = 0.1                   # Vitesse min normalisee
    OBSTACLE_STOP_DIST = 0.3          # Distance arret obstacle (m) - EMERGENCY STOP
    OBSTACLE_SLOW_DIST = 1.0          # Distance ralentissement (m)
    OBSTACLE_WARN_DIST = 2.0          # Distance avertissement (m)
    SIDE_OBSTACLE_DIST = 0.5          # Distance evitement lateral (m)
    AVOIDANCE_SPEED = 0.15            # Vitesse lente pendant evitement (m/s norm)
    LOOP_RATE = 20                    # Hz

    # Zones angulaires pour detection d'obstacles (en radians)
    # Convention: 0 = avant, pi/2 = gauche, -pi/2 = droite
    FRONT_ZONE = (-math.pi/4, math.pi/4)       # -45° a +45°
    FRONT_LEFT_ZONE = (math.pi/4, math.pi/2)   # +45° a +90°
    FRONT_RIGHT_ZONE = (-math.pi/2, -math.pi/4) # -90° a -45°
    LEFT_ZONE = (math.pi/2, 3*math.pi/4)       # +90° a +135°
    RIGHT_ZONE = (-3*math.pi/4, -math.pi/2)    # -135° a -90°

    def __init__(self,
                 lidar: ILidarSensor,
                 motor: IMotorController,
                 mode: str = 'simulation'):
        """
        Initialise le controleur.

        Args:
            lidar: Capteur LiDAR (reel ou simule)
            motor: Controleur moteur (reel ou simule)
            mode: 'simulation' ou 'car'
        """
        self.lidar = lidar
        self.motor = motor
        self.mode = mode

        # Etat
        self.state = NavigationState()
        self.running = False
        self.paused = False
        self.start_time = 0.0

        # Historique de la trajectoire (pour visualisation)
        self.trajectory: List[Tuple[float, float]] = []

        # Points LiDAR actuels (pour visualisation)
        self.scan_points: List[Tuple[float, float]] = []

        # === PERCEPTION MODULES ===
        # Processeur LiDAR pour filtrage et traitement des donnees
        self.lidar_processor = LidarProcessor(
            min_range=0.05,      # 5cm minimum (evite les points trop proches)
            max_range=8.0,       # 8m maximum
            min_intensity=10,    # Filtre les points faibles
            segmentation_threshold=0.3  # 30cm entre segments
        )

        # Detecteur d'obstacles pour clustering et classification
        self.obstacle_detector = ObstacleDetector(
            cluster_threshold=0.3,      # 30cm entre points d'un cluster
            min_cluster_points=3,       # Minimum 3 points pour un obstacle
            wall_min_length=0.5,        # 50cm pour classifier comme mur
            tracking_distance=0.5,      # 50cm pour le tracking
            tracking_max_age=10         # 10 frames avant d'oublier un obstacle
        )

        # Liste des obstacles detectes
        self.detected_obstacles: List[Obstacle] = []
        self.last_processed_scan: Optional[ProcessedScan] = None

        # Charger la configuration si disponible
        self._load_config()

    def _load_config(self, config_path: Optional[str] = None):
        """
        Charge la configuration depuis robot.yaml.

        Args:
            config_path: Chemin vers le fichier de config (optionnel)
        """
        if not YAML_AVAILABLE:
            return

        # Chercher le fichier de config
        if config_path is None:
            # Chercher dans le repertoire config relatif au script
            script_dir = Path(__file__).parent.parent
            config_path = script_dir / 'config' / 'robot.yaml'

        if not os.path.exists(config_path):
            return

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Charger les parametres de securite
            if 'safety' in config:
                safety = config['safety']
                self.OBSTACLE_STOP_DIST = safety.get('emergency_stop_distance', self.OBSTACLE_STOP_DIST)
                self.OBSTACLE_SLOW_DIST = safety.get('slow_distance', self.OBSTACLE_SLOW_DIST)
                self.OBSTACLE_WARN_DIST = safety.get('warning_distance', self.OBSTACLE_WARN_DIST)

            # Charger les parametres de navigation
            if 'navigation' in config:
                nav = config['navigation']
                self.MAX_SPEED = nav.get('max_speed', self.MAX_SPEED)
                self.MIN_SPEED = nav.get('min_speed', self.MIN_SPEED)
                self.SIDE_OBSTACLE_DIST = nav.get('side_obstacle_distance', self.SIDE_OBSTACLE_DIST)
                self.AVOIDANCE_SPEED = nav.get('avoidance_speed', self.AVOIDANCE_SPEED)
                self.LOOP_RATE = nav.get('loop_rate', self.LOOP_RATE)

            # Charger les parametres de perception
            if 'perception' in config:
                perc = config['perception']

                # LiDAR processor
                if 'lidar' in perc:
                    lidar_cfg = perc['lidar']
                    self.lidar_processor = LidarProcessor(
                        min_range=lidar_cfg.get('min_range', 0.05),
                        max_range=lidar_cfg.get('max_range', 8.0),
                        min_intensity=lidar_cfg.get('min_intensity', 10),
                        angle_offset=lidar_cfg.get('angle_offset', 0.0)
                    )

                # Obstacle detector
                if 'obstacle_detection' in perc:
                    obs_cfg = perc['obstacle_detection']
                    self.obstacle_detector = ObstacleDetector(
                        cluster_threshold=obs_cfg.get('cluster_threshold', 0.3),
                        min_cluster_points=obs_cfg.get('min_cluster_points', 3),
                        wall_min_length=obs_cfg.get('wall_min_length', 0.5),
                        tracking_distance=obs_cfg.get('tracking_distance', 0.5),
                        tracking_max_age=obs_cfg.get('tracking_max_age', 10)
                    )

        except Exception:
            pass

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

    def _normalize_angle(self, angle: float) -> float:
        """
        Normalise un angle entre -pi et +pi.
        Convention: 0 = avant, +pi/2 = gauche, -pi/2 = droite
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _convert_lidar_angle(self, lidar_angle: float) -> float:
        """
        Convertit l'angle LiDAR (0-2pi, sens horaire depuis l'avant)
        vers notre convention (-pi a +pi, 0 = avant).

        Le LiDAR LD19 mesure de 0° a 360° dans le sens horaire.
        On doit convertir pour que:
        - 0° LiDAR -> 0 rad (avant)
        - 90° LiDAR -> -pi/2 (droite)
        - 270° LiDAR -> +pi/2 (gauche)
        """
        # Normaliser d'abord
        normalized = self._normalize_angle(lidar_angle)
        return normalized

    def _is_angle_in_zone(self, angle: float, zone: Tuple[float, float]) -> bool:
        """Verifie si un angle est dans une zone donnee."""
        angle = self._normalize_angle(angle)
        zone_min, zone_max = zone
        if zone_min <= zone_max:
            return zone_min <= angle <= zone_max
        else:
            # Zone qui traverse -pi/+pi
            return angle >= zone_min or angle <= zone_max

    def _convert_scan_to_processor_format(self, scan: LidarData) -> '_ProcessorLidarScan':
        """Convertit le scan de l'interface vers le format du processeur."""
        processor_points = []
        for p in scan.points:
            processor_points.append(_ProcessorLidarPoint(
                angle=p.angle,
                distance=p.distance,
                intensity=p.intensity,
                valid=p.valid
            ))
        return _ProcessorLidarScan(
            timestamp=scan.timestamp,
            points=processor_points,
            scan_frequency=scan.scan_frequency
        )

    def check_obstacles(self) -> Tuple[bool, float, float, float, float]:
        """
        Verifie les obstacles via LiDAR avec perception avancee.

        Utilise LidarProcessor pour filtrer les donnees et
        ObstacleDetector pour detecter et classifier les obstacles.

        Returns:
            (danger_imminent, min_distance_front, suggested_steering,
             min_dist_left, min_dist_right)
        """
        scan = self.lidar.get_scan()
        if not scan or len(scan.points) == 0:
            return True, 0.0, 0.0, 0.0, 0.0

        # Reinitialiser les points pour visualisation
        self.scan_points = []

        # === ETAPE 1: Convertir et traiter le scan ===
        try:
            processor_scan = self._convert_scan_to_processor_format(scan)
            processed = self.lidar_processor.process(processor_scan)
            self.last_processed_scan = processed
        except Exception:
            return True, 0.0, 0.0, 0.0, 0.0

        # === ETAPE 2: Detecter les obstacles ===
        try:
            self.detected_obstacles = self.obstacle_detector.detect(processed)
        except Exception:
            self.detected_obstacles = []

        # === ETAPE 3: Calculer les distances par zone ===
        # Utiliser les points valides du scan traite
        valid_mask = processed.valid_mask
        angles = processed.angles
        distances = processed.distances
        points = processed.points

        min_dist_front = float('inf')
        min_dist_front_left = float('inf')
        min_dist_front_right = float('inf')
        min_dist_left = float('inf')
        min_dist_right = float('inf')

        for i in range(len(angles)):
            if not valid_mask[i]:
                continue

            # Convertir en coordonnees monde pour visualisation
            angle_world = angles[i] + self.state.theta
            px = self.state.x + distances[i] * math.cos(angle_world)
            py = self.state.y + distances[i] * math.sin(angle_world)
            self.scan_points.append((px, py))

            # Convertir l'angle pour notre convention
            angle_normalized = self._convert_lidar_angle(angles[i])

            # Classifier dans les zones
            if self._is_angle_in_zone(angle_normalized, self.FRONT_ZONE):
                if distances[i] < min_dist_front:
                    min_dist_front = distances[i]

            if self._is_angle_in_zone(angle_normalized, self.FRONT_LEFT_ZONE):
                if distances[i] < min_dist_front_left:
                    min_dist_front_left = distances[i]

            if self._is_angle_in_zone(angle_normalized, self.FRONT_RIGHT_ZONE):
                if distances[i] < min_dist_front_right:
                    min_dist_front_right = distances[i]

            if self._is_angle_in_zone(angle_normalized, self.LEFT_ZONE):
                if distances[i] < min_dist_left:
                    min_dist_left = distances[i]

            if self._is_angle_in_zone(angle_normalized, self.RIGHT_ZONE):
                if distances[i] < min_dist_right:
                    min_dist_right = distances[i]

        # === ETAPE 4: Verifier les obstacles detectes ===
        danger_from_obstacles = False
        nearest_obstacle_dist = float('inf')

        for obs in self.detected_obstacles:
            if obs.min_distance < self.OBSTACLE_STOP_DIST:
                danger_from_obstacles = True
            if obs.min_distance < nearest_obstacle_dist:
                nearest_obstacle_dist = obs.min_distance

        # === ETAPE 5: Determiner la direction d'evitement ===
        steering = 0.0

        # Distance effective devant (prend en compte aussi les cotes)
        effective_front_dist = min(
            min_dist_front,
            min_dist_front_left * 0.8,  # Pondere les cotes
            min_dist_front_right * 0.8
        )

        if effective_front_dist < self.OBSTACLE_WARN_DIST:
            # Calculer un score pour chaque direction
            left_score = min_dist_front_left + min_dist_left * 0.5
            right_score = min_dist_front_right + min_dist_right * 0.5

            # Tourner vers le cote le plus libre
            if left_score > right_score:
                # Plus de place a gauche
                avoidance_strength = 1.0 - (effective_front_dist / self.OBSTACLE_WARN_DIST)
                steering = 0.3 + 0.5 * avoidance_strength  # 0.3 a 0.8
            else:
                # Plus de place a droite
                avoidance_strength = 1.0 - (effective_front_dist / self.OBSTACLE_WARN_DIST)
                steering = -(0.3 + 0.5 * avoidance_strength)  # -0.3 a -0.8

        # === ETAPE 6: Determiner si danger imminent ===
        danger_imminent = (
            min_dist_front < self.OBSTACLE_STOP_DIST or
            danger_from_obstacles or
            (min_dist_front_left < self.OBSTACLE_STOP_DIST * 0.7) or
            (min_dist_front_right < self.OBSTACLE_STOP_DIST * 0.7)
        )

        # Mettre a jour l'etat
        self.state.obstacle_detected = danger_imminent
        self.state.obstacle_distance = min(min_dist_front, nearest_obstacle_dist)

        return danger_imminent, effective_front_dist, steering, min_dist_left, min_dist_right

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

            # Mettre a jour le simulateur LiDAR
            if isinstance(self.lidar, SimulatedLidarAdapter):
                self.lidar.set_robot_pose(x, y, theta)

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

        Mode LiDAR uniquement: avance tout droit et evite les obstacles.

        Returns:
            True si navigation en cours, False si terminee
        """
        # === PERCEPTION: Verifier obstacles ===
        danger_imminent, obs_dist, avoid_steering, dist_left, dist_right = self.check_obstacles()

        # === SECURITE: Obstacle tres proche - avancer lentement en tournant ===
        if danger_imminent:
            # Choisir la direction d'evitement
            if abs(avoid_steering) > 0.1:
                steer = avoid_steering
            else:
                # Pas de direction claire depuis l'avant, utiliser les cotes
                if dist_left > dist_right:
                    steer = 0.7   # Plus de place a gauche, tourner a gauche
                else:
                    steer = -0.7  # Plus de place a droite, tourner a droite

            # Avancer lentement en tournant fort pour contourner l'obstacle
            # (avec speed=0 le modele bicyclette ne tourne pas du tout)
            self.motor.set_speed(self.AVOIDANCE_SPEED)
            self.motor.set_steering(max(-1.0, min(1.0, steer * 1.3)))
            return True

        # === NAVIGATION: Avancer tout droit ===
        final_speed = self.MAX_SPEED
        final_steering = 0.0

        # --- Evitement lateral: obstacle proche sur un cote ---
        if dist_left < self.SIDE_OBSTACLE_DIST:
            side_strength = 1.0 - (dist_left / self.SIDE_OBSTACLE_DIST)
            final_steering -= 0.5 * side_strength  # Tourner a droite

        if dist_right < self.SIDE_OBSTACLE_DIST:
            side_strength = 1.0 - (dist_right / self.SIDE_OBSTACLE_DIST)
            final_steering += 0.5 * side_strength  # Tourner a gauche

        # --- Evitement frontal ---
        if obs_dist < self.OBSTACLE_WARN_DIST:
            # Facteur de ralentissement progressif
            slowdown = (obs_dist - self.OBSTACLE_STOP_DIST) / (self.OBSTACLE_WARN_DIST - self.OBSTACLE_STOP_DIST)
            slowdown = max(0.2, min(1.0, slowdown))

            final_speed = self.MAX_SPEED * slowdown

            # Evitement d'obstacles - plus agressif quand l'obstacle est proche
            if obs_dist < self.OBSTACLE_SLOW_DIST:
                avoidance_weight = 1.0 - (obs_dist / self.OBSTACLE_SLOW_DIST)
                avoidance_weight = max(0.0, min(1.0, avoidance_weight))
                final_steering += avoidance_weight * avoid_steering

        # Limiter les valeurs
        final_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, final_speed))
        final_steering = max(-1.0, min(1.0, final_steering))

        # === APPLIQUER COMMANDES ===
        self.motor.set_speed(final_speed)
        self.motor.set_steering(final_steering)

        return True

    def init_sensors(self) -> bool:
        """Initialise tous les capteurs."""
        if not self.lidar.start():
            return False

        if not self.motor.start():
            return False

        return True

    def shutdown(self):
        """Arrete proprement tous les composants."""
        self.running = False

        if self.motor:
            self.motor.emergency_stop()
            self.motor.stop()

        if self.lidar:
            self.lidar.stop()

    def run_headless(self):
        """Execute la navigation sans visualisation."""
        if not self.init_sensors():
            return

        self.running = True
        self.start_time = time.time()
        dt = 1.0 / self.LOOP_RATE

        try:
            while self.running:
                if self.mode == 'simulation':
                    self.update_state_from_motor(dt)

                if not self.navigation_step():
                    break

                self.state.elapsed_time = time.time() - self.start_time
                time.sleep(dt)

        except KeyboardInterrupt:
            pass

        finally:
            self.shutdown()

    def run_with_gui(self):
        """Execute la navigation avec visualisation matplotlib."""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Circle
        import numpy as np

        if not self.init_sensors():
            return

        # Configuration matplotlib
        plt.ion()
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        fig.canvas.manager.set_window_title('Robocar - Navigation LiDAR')

        # Limites de la zone
        x_min, x_max = -20, 20
        y_min, y_max = -20, 20

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

                info = (
                    f"Position: ({self.state.x:.2f}, {self.state.y:.2f})\n"
                    f"Heading: {math.degrees(self.state.theta):.0f} deg\n"
                    f"Vitesse: {self.state.speed:.2f} m/s\n"
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
            pass

        finally:
            self.shutdown()

            plt.ioff()
            if plt.fignum_exists(fig.number):
                plt.show()

    def run_real(self):
        """Execute la navigation en mode reel (Jetson Nano)."""
        if not self.init_sensors():
            return

        self.running = True
        self.start_time = time.time()
        dt = 1.0 / self.LOOP_RATE

        try:
            while self.running:
                if not self.navigation_step():
                    break

                self.state.elapsed_time = time.time() - self.start_time
                time.sleep(dt)

        except KeyboardInterrupt:
            pass

        finally:
            self.shutdown()

