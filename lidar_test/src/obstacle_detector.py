#!/usr/bin/env python3
"""
Module de Détection d'Obstacles pour LiDAR LD19
================================================
Système de détection d'obstacles avec:
- Zones de sécurité configurables (avant, arrière, côtés)
- Alertes multi-niveaux (warning, danger, critical)
- Filtrage et clustering des points
- Tracking simple des obstacles
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Callable
from enum import Enum
from collections import deque
import time

from ld19_parser import LidarMeasurement, LidarScan


class AlertLevel(Enum):
    """Niveaux d'alerte pour les obstacles."""
    NONE = 0
    WARNING = 1    # Obstacle détecté, pas de danger immédiat
    DANGER = 2     # Obstacle proche, ralentir
    CRITICAL = 3   # Obstacle très proche, arrêt d'urgence


@dataclass
class SafetyZone:
    """
    Définit une zone de sécurité.
    
    La zone est définie en coordonnées polaires relatives au LiDAR:
    - Angle 0° = devant le véhicule
    - Angles positifs = sens horaire
    """
    name: str
    angle_start: float      # Angle de début en degrés
    angle_end: float        # Angle de fin en degrés
    distance_warning: float # Distance d'avertissement en mètres
    distance_danger: float  # Distance de danger en mètres
    distance_critical: float # Distance critique en mètres
    enabled: bool = True
    
    def contains_angle(self, angle: float) -> bool:
        """Vérifie si un angle est dans cette zone."""
        angle = angle % 360
        if self.angle_start <= self.angle_end:
            return self.angle_start <= angle <= self.angle_end
        else:
            # Zone qui traverse 0°
            return angle >= self.angle_start or angle <= self.angle_end
    
    def get_alert_level(self, distance: float) -> AlertLevel:
        """Retourne le niveau d'alerte pour une distance donnée."""
        if distance <= self.distance_critical:
            return AlertLevel.CRITICAL
        elif distance <= self.distance_danger:
            return AlertLevel.DANGER
        elif distance <= self.distance_warning:
            return AlertLevel.WARNING
        return AlertLevel.NONE


@dataclass
class Obstacle:
    """Représente un obstacle détecté."""
    points: List[LidarMeasurement]
    zone: SafetyZone
    alert_level: AlertLevel
    timestamp: float = field(default_factory=time.time)
    
    @property
    def center_angle(self) -> float:
        """Angle central de l'obstacle."""
        if not self.points:
            return 0.0
        angles = [p.angle_deg for p in self.points]
        # Gestion du wrap-around
        if max(angles) - min(angles) > 180:
            angles = [(a if a < 180 else a - 360) for a in angles]
        return (sum(angles) / len(angles)) % 360
    
    @property
    def min_distance(self) -> float:
        """Distance minimale de l'obstacle en mètres."""
        if not self.points:
            return float('inf')
        return min(p.distance_m for p in self.points)
    
    @property
    def center_position(self) -> Tuple[float, float]:
        """Position centrale de l'obstacle (x, y) en mètres."""
        if not self.points:
            return (0.0, 0.0)
        x_coords = [p.to_cartesian()[0] for p in self.points]
        y_coords = [p.to_cartesian()[1] for p in self.points]
        return (np.mean(x_coords), np.mean(y_coords))
    
    @property
    def width(self) -> float:
        """Largeur estimée de l'obstacle en mètres."""
        if len(self.points) < 2:
            return 0.0
        positions = [p.to_cartesian() for p in self.points]
        x_coords = [p[0] for p in positions]
        y_coords = [p[1] for p in positions]
        return max(
            max(x_coords) - min(x_coords),
            max(y_coords) - min(y_coords)
        )


@dataclass
class DetectionResult:
    """Résultat de la détection d'obstacles."""
    obstacles: List[Obstacle]
    max_alert_level: AlertLevel
    zones_status: Dict[str, AlertLevel]
    scan_timestamp: float
    processing_time_ms: float


class ObstacleDetector:
    """
    Détecteur d'obstacles pour voiture RC avec LiDAR LD19.
    """
    
    # Zones par défaut pour une voiture RC
    DEFAULT_ZONES = [
        SafetyZone(
            name="front",
            angle_start=315,  # -45°
            angle_end=45,     # +45°
            distance_warning=2.0,
            distance_danger=1.0,
            distance_critical=0.3
        ),
        SafetyZone(
            name="front_left",
            angle_start=45,
            angle_end=90,
            distance_warning=1.5,
            distance_danger=0.8,
            distance_critical=0.3
        ),
        SafetyZone(
            name="front_right",
            angle_start=270,
            angle_end=315,
            distance_warning=1.5,
            distance_danger=0.8,
            distance_critical=0.3
        ),
        SafetyZone(
            name="left",
            angle_start=90,
            angle_end=135,
            distance_warning=1.0,
            distance_danger=0.5,
            distance_critical=0.2
        ),
        SafetyZone(
            name="right",
            angle_start=225,
            angle_end=270,
            distance_warning=1.0,
            distance_danger=0.5,
            distance_critical=0.2
        ),
        SafetyZone(
            name="rear",
            angle_start=135,
            angle_end=225,
            distance_warning=1.0,
            distance_danger=0.5,
            distance_critical=0.2
        ),
    ]
    
    def __init__(self, zones: List[SafetyZone] = None,
                 min_points_for_obstacle: int = 3,
                 cluster_distance: float = 0.15,  # 15cm
                 min_intensity: int = 10):
        """
        Initialise le détecteur d'obstacles.
        
        Args:
            zones: Liste des zones de sécurité (utilise les défauts si None)
            min_points_for_obstacle: Nombre min de points pour considérer un obstacle
            cluster_distance: Distance max entre points du même cluster (mètres)
            min_intensity: Intensité minimale pour considérer un point valide
        """
        self.zones = zones or self.DEFAULT_ZONES.copy()
        self.min_points = min_points_for_obstacle
        self.cluster_distance = cluster_distance
        self.min_intensity = min_intensity
        
        # Historique pour le filtrage temporel
        self._obstacle_history: deque = deque(maxlen=5)
        
        # Callbacks pour les alertes
        self._alert_callbacks: List[Callable[[DetectionResult], None]] = []
        
    def add_zone(self, zone: SafetyZone):
        """Ajoute une zone de sécurité."""
        self.zones.append(zone)
        
    def remove_zone(self, name: str):
        """Supprime une zone par son nom."""
        self.zones = [z for z in self.zones if z.name != name]
        
    def get_zone(self, name: str) -> Optional[SafetyZone]:
        """Récupère une zone par son nom."""
        for zone in self.zones:
            if zone.name == name:
                return zone
        return None
    
    def register_alert_callback(self, callback: Callable[[DetectionResult], None]):
        """Enregistre un callback appelé lors de la détection."""
        self._alert_callbacks.append(callback)
        
    def detect(self, scan: LidarScan) -> DetectionResult:
        """
        Effectue la détection d'obstacles sur un scan LiDAR.
        
        Args:
            scan: Scan LiDAR complet
            
        Returns:
            Résultat de la détection avec les obstacles et alertes
        """
        start_time = time.time()
        
        # Filtrer les points valides
        valid_points = scan.get_valid_measurements(
            min_distance_mm=10,
            max_distance_mm=12000,
            min_intensity=self.min_intensity
        )
        
        obstacles = []
        zones_status = {z.name: AlertLevel.NONE for z in self.zones}
        
        for zone in self.zones:
            if not zone.enabled:
                continue
                
            # Points dans cette zone
            zone_points = [
                p for p in valid_points 
                if zone.contains_angle(p.angle_deg) and 
                   p.distance_m <= zone.distance_warning
            ]
            
            if not zone_points:
                continue
            
            # Clustering des points
            clusters = self._cluster_points(zone_points)
            
            for cluster in clusters:
                if len(cluster) < self.min_points:
                    continue
                    
                # Trouver la distance min dans le cluster
                min_dist = min(p.distance_m for p in cluster)
                alert_level = zone.get_alert_level(min_dist)
                
                if alert_level != AlertLevel.NONE:
                    obstacle = Obstacle(
                        points=cluster,
                        zone=zone,
                        alert_level=alert_level
                    )
                    obstacles.append(obstacle)
                    
                    # Mettre à jour le statut de la zone
                    if alert_level.value > zones_status[zone.name].value:
                        zones_status[zone.name] = alert_level
        
        # Niveau d'alerte maximum
        max_alert = AlertLevel.NONE
        for level in zones_status.values():
            if level.value > max_alert.value:
                max_alert = level
        
        processing_time = (time.time() - start_time) * 1000
        
        result = DetectionResult(
            obstacles=obstacles,
            max_alert_level=max_alert,
            zones_status=zones_status,
            scan_timestamp=time.time(),
            processing_time_ms=processing_time
        )
        
        # Appeler les callbacks
        for callback in self._alert_callbacks:
            try:
                callback(result)
            except Exception as e:
                print(f"Erreur callback: {e}")
        
        return result
    
    def _cluster_points(self, points: List[LidarMeasurement]) -> List[List[LidarMeasurement]]:
        """
        Regroupe les points proches en clusters.
        Utilise un algorithme simple basé sur la distance angulaire et radiale.
        """
        if not points:
            return []
        
        # Trier par angle
        sorted_points = sorted(points, key=lambda p: p.angle_deg)
        
        clusters = []
        current_cluster = [sorted_points[0]]
        
        for i in range(1, len(sorted_points)):
            point = sorted_points[i]
            last_point = current_cluster[-1]
            
            # Calculer la distance entre les points
            x1, y1 = last_point.to_cartesian()
            x2, y2 = point.to_cartesian()
            distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            if distance <= self.cluster_distance:
                current_cluster.append(point)
            else:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = [point]
        
        if current_cluster:
            clusters.append(current_cluster)
            
        return clusters
    
    def get_closest_obstacle_in_zone(self, result: DetectionResult, 
                                     zone_name: str) -> Optional[Obstacle]:
        """Retourne l'obstacle le plus proche dans une zone donnée."""
        zone_obstacles = [o for o in result.obstacles if o.zone.name == zone_name]
        if not zone_obstacles:
            return None
        return min(zone_obstacles, key=lambda o: o.min_distance)
    
    def get_path_clear(self, result: DetectionResult, 
                       zones: List[str] = None) -> bool:
        """
        Vérifie si le chemin est dégagé dans les zones spécifiées.
        Par défaut, vérifie uniquement la zone avant.
        """
        if zones is None:
            zones = ["front"]
        
        for zone_name in zones:
            if zone_name in result.zones_status:
                if result.zones_status[zone_name].value >= AlertLevel.DANGER.value:
                    return False
        return True


class EmergencyStop:
    """
    Gestionnaire d'arrêt d'urgence basé sur la détection d'obstacles.
    """
    
    def __init__(self, detector: ObstacleDetector):
        self.detector = detector
        self.is_stopped = False
        self._stop_callback: Optional[Callable[[], None]] = None
        self._resume_callback: Optional[Callable[[], None]] = None
        
        # Enregistrer le callback de détection
        detector.register_alert_callback(self._on_detection)
        
    def set_stop_callback(self, callback: Callable[[], None]):
        """Définit le callback d'arrêt d'urgence."""
        self._stop_callback = callback
        
    def set_resume_callback(self, callback: Callable[[], None]):
        """Définit le callback de reprise."""
        self._resume_callback = callback
        
    def _on_detection(self, result: DetectionResult):
        """Appelé lors de chaque détection."""
        if result.max_alert_level == AlertLevel.CRITICAL:
            if not self.is_stopped:
                self.is_stopped = True
                print(f"⛔ ARRÊT D'URGENCE - Obstacle critique détecté!")
                if self._stop_callback:
                    self._stop_callback()
        elif result.max_alert_level.value < AlertLevel.DANGER.value:
            if self.is_stopped:
                self.is_stopped = False
                print("✅ Voie libre - Reprise possible")
                if self._resume_callback:
                    self._resume_callback()


def create_custom_zones_for_slam() -> List[SafetyZone]:
    """
    Crée des zones optimisées pour le SLAM.
    Plus de granularité angulaire pour la navigation autonome.
    """
    zones = []
    
    # Zone avant principale (navigation)
    zones.append(SafetyZone(
        name="nav_front",
        angle_start=340,
        angle_end=20,
        distance_warning=3.0,
        distance_danger=1.5,
        distance_critical=0.5
    ))
    
    # Zones latérales pour le mapping
    for i in range(8):
        angle_start = i * 45
        angle_end = (i + 1) * 45
        zones.append(SafetyZone(
            name=f"sector_{i}",
            angle_start=angle_start,
            angle_end=angle_end,
            distance_warning=4.0,  # Mapping à plus longue distance
            distance_danger=1.0,
            distance_critical=0.3
        ))
    
    return zones


if __name__ == "__main__":
    # Test du détecteur d'obstacles
    print("=== Test du Détecteur d'Obstacles ===\n")
    
    # Créer des mesures de test simulant des obstacles
    test_measurements = []
    
    # Obstacle devant (distance critique)
    for angle in range(350, 360):
        test_measurements.append(LidarMeasurement(
            angle_deg=angle,
            distance_mm=250,  # 25cm - critique!
            intensity=200
        ))
    for angle in range(0, 10):
        test_measurements.append(LidarMeasurement(
            angle_deg=angle,
            distance_mm=250,
            intensity=200
        ))
    
    # Obstacle sur le côté gauche (warning)
    for angle in range(80, 100):
        test_measurements.append(LidarMeasurement(
            angle_deg=angle,
            distance_mm=800,  # 80cm - warning
            intensity=180
        ))
    
    # Points lointains (pas d'obstacle)
    for angle in range(120, 340):
        test_measurements.append(LidarMeasurement(
            angle_deg=angle,
            distance_mm=5000,  # 5m - ok
            intensity=150
        ))
    
    # Créer un scan et tester
    scan = LidarScan(test_measurements)
    detector = ObstacleDetector()
    
    # Callback de test
    def on_alert(result: DetectionResult):
        if result.max_alert_level != AlertLevel.NONE:
            print(f"  🚨 Alerte niveau: {result.max_alert_level.name}")
    
    detector.register_alert_callback(on_alert)
    
    result = detector.detect(scan)
    
    print(f"Niveau d'alerte max: {result.max_alert_level.name}")
    print(f"Temps de traitement: {result.processing_time_ms:.2f} ms")
    print(f"\nStatut des zones:")
    for zone_name, level in result.zones_status.items():
        status = "🟢" if level == AlertLevel.NONE else "🟡" if level == AlertLevel.WARNING else "🟠" if level == AlertLevel.DANGER else "🔴"
        print(f"  {status} {zone_name}: {level.name}")
    
    print(f"\nObstacles détectés: {len(result.obstacles)}")
    for i, obstacle in enumerate(result.obstacles):
        print(f"\n  Obstacle {i+1}:")
        print(f"    Zone: {obstacle.zone.name}")
        print(f"    Niveau: {obstacle.alert_level.name}")
        print(f"    Distance min: {obstacle.min_distance:.2f} m")
        print(f"    Angle central: {obstacle.center_angle:.1f}°")
        print(f"    Largeur estimée: {obstacle.width:.2f} m")
        print(f"    Nb points: {len(obstacle.points)}")
    
    # Test du path clear
    print(f"\nChemin avant dégagé: {detector.get_path_clear(result)}")
    print(f"Chemin latéral dégagé: {detector.get_path_clear(result, ['left', 'right'])}")
    print(f"Chemin arrière dégagé: {detector.get_path_clear(result, ['rear'])}")