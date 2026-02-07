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
    ILidarSensor, IGPSSensor, IMotorController,
    LidarData, GPSData,
    SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
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
class ObstacleState:
    """Etat detaille des obstacles par zone."""
    # Distances minimales par zone
    front: float = float('inf')
    front_left: float = float('inf')
    front_right: float = float('inf')
    left: float = float('inf')
    right: float = float('inf')

    # Direction d'evitement suggeree (-1 = droite, +1 = gauche)
    avoid_steering: float = 0.0

    # Etats de blocage
    front_blocked: bool = False       # Obstacle devant
    completely_blocked: bool = False   # Bloque de partout, nulle part ou aller

    @property
    def best_escape_direction(self) -> float:
        """Retourne la meilleure direction d'echappement."""
        # Score pour chaque cote : plus c'est grand, plus c'est libre
        left_score = self.front_left + self.left * 0.5
        right_score = self.front_right + self.right * 0.5

        if left_score > right_score:
            return 1.0   # Tourner a gauche
        elif right_score > left_score:
            return -1.0  # Tourner a droite
        else:
            return 1.0   # Par defaut, gauche


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
    OBSTACLE_STOP_DIST = 0.3          # Distance arret obstacle (m) - EMERGENCY STOP
    OBSTACLE_SLOW_DIST = 1.0          # Distance ralentissement (m)
    OBSTACLE_WARN_DIST = 2.0          # Distance avertissement (m)
    STEERING_GAIN = 2.0               # Gain de direction
    LOOP_RATE = 20                    # Hz

    # Zones angulaires pour detection d'obstacles (en radians)
    # Convention: 0 = avant, pi/2 = gauche, -pi/2 = droite
    FRONT_ZONE = (-math.pi/4, math.pi/4)       # -45° a +45°
    FRONT_LEFT_ZONE = (math.pi/4, math.pi/2)   # +45° a +90°
    FRONT_RIGHT_ZONE = (-math.pi/2, -math.pi/4) # -90° a -45°
    LEFT_ZONE = (math.pi/2, 3*math.pi/4)       # +90° a +135°
    RIGHT_ZONE = (-3*math.pi/4, -math.pi/2)    # -135° a -90°

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
            print("[NAV] PyYAML non disponible, utilisation des valeurs par defaut")
            return

        # Chercher le fichier de config
        if config_path is None:
            # Chercher dans le repertoire config relatif au script
            script_dir = Path(__file__).parent.parent
            config_path = script_dir / 'config' / 'robot.yaml'

        if not os.path.exists(config_path):
            print(f"[NAV] Fichier config non trouve: {config_path}")
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
                print(f"[NAV] Config securite: stop={self.OBSTACLE_STOP_DIST}m, "
                      f"slow={self.OBSTACLE_SLOW_DIST}m, warn={self.OBSTACLE_WARN_DIST}m")

            # Charger les parametres de navigation
            if 'navigation' in config:
                nav = config['navigation']
                self.WAYPOINT_REACHED_DIST = nav.get('waypoint_reached_distance', self.WAYPOINT_REACHED_DIST)
                self.MAX_SPEED = nav.get('max_speed', self.MAX_SPEED)
                self.MIN_SPEED = nav.get('min_speed', self.MIN_SPEED)
                self.STEERING_GAIN = nav.get('steering_gain', self.STEERING_GAIN)
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

            print("[NAV] Configuration chargee avec succes")

        except Exception as e:
            print(f"[NAV] Erreur chargement config: {e}")

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

    def check_obstacles(self) -> ObstacleState:
        """
        Verifie les obstacles via LiDAR avec perception avancee.

        Utilise LidarProcessor pour filtrer les donnees et
        ObstacleDetector pour detecter et classifier les obstacles.

        Retourne un ObstacleState detaille avec:
        - Distance minimale par zone (front, front_left, front_right, left, right)
        - front_blocked: obstacle devant necessitant un contournement
        - completely_blocked: murs partout, impossible d'avancer
        - avoid_steering: direction d'evitement suggeree
        """
        obs_state = ObstacleState()

        scan = self.lidar.get_scan()
        if not scan or len(scan.points) == 0:
            # Pas de donnees LiDAR - securite, on signale bloque
            print("[PERCEPTION] Pas de donnees LiDAR!")
            obs_state.completely_blocked = True
            obs_state.front_blocked = True
            obs_state.front = 0.0
            return obs_state

        # Reinitialiser les points pour visualisation
        self.scan_points = []

        # === ETAPE 1: Convertir et traiter le scan ===
        try:
            processor_scan = self._convert_scan_to_processor_format(scan)
            processed = self.lidar_processor.process(processor_scan)
            self.last_processed_scan = processed
        except Exception as e:
            print(f"[PERCEPTION] Erreur traitement scan: {e}")
            obs_state.completely_blocked = True
            obs_state.front_blocked = True
            obs_state.front = 0.0
            return obs_state

        # === ETAPE 2: Detecter les obstacles ===
        try:
            self.detected_obstacles = self.obstacle_detector.detect(processed)
        except Exception as e:
            print(f"[PERCEPTION] Erreur detection obstacles: {e}")
            self.detected_obstacles = []

        # === ETAPE 3: Calculer les distances par zone ===
        valid_mask = processed.valid_mask
        angles = processed.angles
        distances = processed.distances

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
                obs_state.front = min(obs_state.front, distances[i])

            if self._is_angle_in_zone(angle_normalized, self.FRONT_LEFT_ZONE):
                obs_state.front_left = min(obs_state.front_left, distances[i])

            if self._is_angle_in_zone(angle_normalized, self.FRONT_RIGHT_ZONE):
                obs_state.front_right = min(obs_state.front_right, distances[i])

            if self._is_angle_in_zone(angle_normalized, self.LEFT_ZONE):
                obs_state.left = min(obs_state.left, distances[i])

            if self._is_angle_in_zone(angle_normalized, self.RIGHT_ZONE):
                obs_state.right = min(obs_state.right, distances[i])

        # === ETAPE 4: Determiner si le front est bloque ===
        # Le front est bloque si un obstacle est dans la zone de ralentissement
        effective_front_dist = min(
            obs_state.front,
            obs_state.front_left * 0.8,
            obs_state.front_right * 0.8
        )

        obs_state.front_blocked = effective_front_dist < self.OBSTACLE_SLOW_DIST

        # === ETAPE 5: Determiner si completement bloque ===
        # On est bloque partout si TOUTES les directions proches sont sous le seuil d'arret
        blocked_threshold = self.OBSTACLE_STOP_DIST
        all_blocked = (
            obs_state.front < blocked_threshold and
            obs_state.front_left < blocked_threshold and
            obs_state.front_right < blocked_threshold and
            obs_state.left < blocked_threshold * 1.5 and
            obs_state.right < blocked_threshold * 1.5
        )
        obs_state.completely_blocked = all_blocked

        # === ETAPE 6: Calculer la direction d'evitement ===
        if obs_state.front_blocked:
            left_score = obs_state.front_left + obs_state.left * 0.5
            right_score = obs_state.front_right + obs_state.right * 0.5

            avoidance_strength = max(0.0, 1.0 - (effective_front_dist / self.OBSTACLE_SLOW_DIST))

            if left_score > right_score:
                obs_state.avoid_steering = 0.3 + 0.7 * avoidance_strength
            else:
                obs_state.avoid_steering = -(0.3 + 0.7 * avoidance_strength)

        # Mettre a jour l'etat global
        self.state.obstacle_detected = obs_state.front_blocked
        self.state.obstacle_distance = effective_front_dist

        # Debug info
        if obs_state.front_blocked:
            status = "BLOQUE PARTOUT" if obs_state.completely_blocked else "CONTOURNEMENT"
            if self.mode == 'car':
                print(f"\n[PERCEPTION] {status}: front={obs_state.front:.2f}m, "
                      f"FL={obs_state.front_left:.2f}m, FR={obs_state.front_right:.2f}m, "
                      f"L={obs_state.left:.2f}m, R={obs_state.right:.2f}m, "
                      f"steer={obs_state.avoid_steering:.2f}")

        return obs_state

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

        Logique de contournement d'obstacles:
        - Obstacle devant: la voiture tourne pour le contourner tout en avancant
        - Bloque partout (murs de tous les cotes): arret complet
        - Sinon: navigation normale vers le waypoint

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

        # === PERCEPTION: Analyser les obstacles ===
        obs = self.check_obstacles()

        # === CAS 1: Completement bloque (murs partout) -> ARRET ===
        if obs.completely_blocked:
            self.motor.set_speed(0)
            self.motor.set_steering(0)
            if self.mode == 'car':
                print("\n[NAV] ARRET: bloque de tous les cotes, aucune issue")
            return True

        # === NAVIGATION: Calculer commandes vers waypoint ===
        nav_speed, nav_steering = self.compute_control(wp)

        # === CAS 2: Obstacle devant -> CONTOURNER en avancant ===
        if obs.front_blocked:
            # Vitesse reduite mais on continue d'avancer
            # Plus l'obstacle est proche, plus on ralentit
            proximity = max(0.0, obs.front - self.OBSTACLE_STOP_DIST)
            range_dist = self.OBSTACLE_SLOW_DIST - self.OBSTACLE_STOP_DIST
            if range_dist > 0:
                speed_factor = max(0.3, proximity / range_dist)
            else:
                speed_factor = 0.3

            contour_speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * speed_factor * 0.6

            # Direction: on tourne fort vers le cote libre
            # L'evitement domine la navigation quand on est proche
            avoidance_weight = max(0.5, 1.0 - (proximity / range_dist if range_dist > 0 else 0))
            contour_steering = (1 - avoidance_weight) * nav_steering + avoidance_weight * obs.avoid_steering

            # Limiter
            contour_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, contour_speed))
            contour_steering = max(-1.0, min(1.0, contour_steering))

            self.motor.set_speed(contour_speed)
            self.motor.set_steering(contour_steering)
            return True

        # === CAS 3: Pas d'obstacle proche -> navigation normale ===
        final_speed = nav_speed
        final_steering = nav_steering

        # Legere influence de l'evitement si obstacle dans la zone d'avertissement
        obs_dist = min(obs.front, obs.front_left * 0.8, obs.front_right * 0.8)
        if obs_dist < self.OBSTACLE_WARN_DIST:
            # Ralentissement progressif
            slowdown = (obs_dist - self.OBSTACLE_SLOW_DIST) / (self.OBSTACLE_WARN_DIST - self.OBSTACLE_SLOW_DIST)
            slowdown = max(0.5, min(1.0, slowdown))
            final_speed = nav_speed * slowdown

            # Legere correction de direction
            blend = 0.3 * (1.0 - obs_dist / self.OBSTACLE_WARN_DIST)
            final_steering = (1 - blend) * nav_steering + blend * obs.avoid_steering

        # Limiter les valeurs
        final_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, final_speed))
        final_steering = max(-1.0, min(1.0, final_steering))

        # === APPLIQUER COMMANDES ===
        self.motor.set_speed(final_speed)
        self.motor.set_steering(final_steering)

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
