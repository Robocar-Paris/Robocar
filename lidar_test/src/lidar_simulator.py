#!/usr/bin/env python3
"""
Simulateur LiDAR LD19
======================
Simule un LiDAR LD19 pour tester sans le hardware physique.

Fonctionnalités:
- Environnements prédéfinis (couloir, parking, pièce)
- Obstacles dynamiques
- Bruit réaliste
- Export de données pour replay
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import json
import time
import struct
from pathlib import Path

from ld19_parser import (
    LidarMeasurement, LidarPacket, LidarScan, 
    HEADER_BYTE, LENGTH_BYTE, MEASUREMENTS_PER_PACKET, CRC8
)


@dataclass
class Wall:
    """Un mur dans l'environnement simulé."""
    start: Tuple[float, float]  # Point de départ (x, y) en mètres
    end: Tuple[float, float]    # Point de fin (x, y) en mètres
    reflectivity: float = 0.8   # Réflectivité (0-1)
    

@dataclass 
class CircleObstacle:
    """Un obstacle circulaire (pilier, poteau, etc.)."""
    center: Tuple[float, float]  # Centre (x, y) en mètres
    radius: float                # Rayon en mètres
    reflectivity: float = 0.7


@dataclass
class RectangleObstacle:
    """Un obstacle rectangulaire (boîte, voiture, etc.)."""
    center: Tuple[float, float]  # Centre (x, y) en mètres
    width: float                 # Largeur en mètres (axe X)
    height: float                # Hauteur en mètres (axe Y)
    rotation: float = 0.0        # Rotation en degrés
    reflectivity: float = 0.75


@dataclass
class SimulatedEnvironment:
    """Environnement simulé complet."""
    name: str
    walls: List[Wall] = field(default_factory=list)
    circles: List[CircleObstacle] = field(default_factory=list)
    rectangles: List[RectangleObstacle] = field(default_factory=list)
    
    def add_wall(self, x1: float, y1: float, x2: float, y2: float, 
                 reflectivity: float = 0.8):
        """Ajoute un mur à l'environnement."""
        self.walls.append(Wall((x1, y1), (x2, y2), reflectivity))
        
    def add_circle(self, x: float, y: float, radius: float,
                   reflectivity: float = 0.7):
        """Ajoute un obstacle circulaire."""
        self.circles.append(CircleObstacle((x, y), radius, reflectivity))
        
    def add_rectangle(self, x: float, y: float, width: float, height: float,
                      rotation: float = 0.0, reflectivity: float = 0.75):
        """Ajoute un obstacle rectangulaire."""
        self.rectangles.append(RectangleObstacle(
            (x, y), width, height, rotation, reflectivity
        ))


class LidarSimulator:
    """
    Simulateur de LiDAR LD19.
    """
    
    # Spécifications du LD19
    MAX_RANGE = 12.0          # Portée max en mètres
    MIN_RANGE = 0.02          # Portée min en mètres
    ANGULAR_RESOLUTION = 0.75 # Résolution angulaire en degrés (à 10Hz)
    ROTATION_SPEED = 10.0     # Vitesse de rotation en Hz
    
    def __init__(self, environment: SimulatedEnvironment = None,
                 noise_stddev: float = 0.01,  # Bruit en mètres
                 angular_noise: float = 0.1,   # Bruit angulaire en degrés
                 miss_rate: float = 0.02):     # Taux de points manquants
        """
        Initialise le simulateur.
        
        Args:
            environment: Environnement à simuler
            noise_stddev: Écart-type du bruit de distance (mètres)
            angular_noise: Bruit angulaire (degrés)
            miss_rate: Probabilité qu'un point soit manquant
        """
        self.environment = environment or self._create_default_environment()
        self.noise_stddev = noise_stddev
        self.angular_noise = angular_noise
        self.miss_rate = miss_rate
        
        self._current_angle = 0.0
        self._timestamp = 0
        self._crc = CRC8()
        
    def _create_default_environment(self) -> SimulatedEnvironment:
        """Crée un environnement par défaut (pièce simple)."""
        env = SimulatedEnvironment(name="default_room")
        
        # Pièce de 6m x 6m
        env.add_wall(-3, -3, 3, -3)   # Mur sud
        env.add_wall(3, -3, 3, 3)     # Mur est
        env.add_wall(3, 3, -3, 3)     # Mur nord
        env.add_wall(-3, 3, -3, -3)   # Mur ouest
        
        # Quelques obstacles
        env.add_circle(1.5, 1.5, 0.2)  # Pilier
        env.add_rectangle(-1, 0, 0.5, 1.0, 45)  # Boîte
        
        return env
    
    def _ray_wall_intersection(self, angle_rad: float, wall: Wall) -> Optional[Tuple[float, float]]:
        """
        Calcule l'intersection d'un rayon avec un mur.
        Retourne (distance, reflectivity) ou None.
        """
        # Direction du rayon (partant de l'origine)
        dx = np.sin(angle_rad)
        dy = np.cos(angle_rad)
        
        # Vecteur du mur
        x1, y1 = wall.start
        x2, y2 = wall.end
        wx = x2 - x1
        wy = y2 - y1
        
        # Résolution paramétrique
        denom = dx * wy - dy * wx
        if abs(denom) < 1e-10:
            return None  # Rayon parallèle au mur
        
        t = (x1 * wy - y1 * wx) / (-denom)
        s = (dx * y1 - dy * x1) / (-denom)
        
        if t > 0 and 0 <= s <= 1:
            distance = t
            if self.MIN_RANGE <= distance <= self.MAX_RANGE:
                return (distance, wall.reflectivity)
        
        return None
    
    def _ray_circle_intersection(self, angle_rad: float, 
                                  circle: CircleObstacle) -> Optional[Tuple[float, float]]:
        """
        Calcule l'intersection d'un rayon avec un cercle.
        """
        dx = np.sin(angle_rad)
        dy = np.cos(angle_rad)
        cx, cy = circle.center
        r = circle.radius
        
        # Équation quadratique
        a = dx * dx + dy * dy
        b = 2 * (dx * (-cx) + dy * (-cy))
        c = cx * cx + cy * cy - r * r
        
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return None
        
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        
        # Prendre la plus proche intersection positive
        t = None
        if t1 > 0 and self.MIN_RANGE <= t1 <= self.MAX_RANGE:
            t = t1
        elif t2 > 0 and self.MIN_RANGE <= t2 <= self.MAX_RANGE:
            t = t2
            
        if t is not None:
            return (t, circle.reflectivity)
        return None
    
    def _ray_rectangle_intersection(self, angle_rad: float,
                                     rect: RectangleObstacle) -> Optional[Tuple[float, float]]:
        """
        Calcule l'intersection d'un rayon avec un rectangle.
        """
        # Convertir le rectangle en 4 murs
        cx, cy = rect.center
        w, h = rect.width / 2, rect.height / 2
        rot = np.radians(rect.rotation)
        
        # Coins du rectangle
        corners = [
            (-w, -h), (w, -h), (w, h), (-w, h)
        ]
        
        # Rotation et translation
        rotated_corners = []
        for x, y in corners:
            rx = x * np.cos(rot) - y * np.sin(rot) + cx
            ry = x * np.sin(rot) + y * np.cos(rot) + cy
            rotated_corners.append((rx, ry))
        
        # Vérifier l'intersection avec chaque côté
        min_dist = None
        for i in range(4):
            wall = Wall(rotated_corners[i], rotated_corners[(i+1) % 4], rect.reflectivity)
            result = self._ray_wall_intersection(angle_rad, wall)
            if result:
                if min_dist is None or result[0] < min_dist[0]:
                    min_dist = result
        
        return min_dist
    
    def _cast_ray(self, angle_deg: float) -> Tuple[int, int]:
        """
        Lance un rayon et retourne (distance_mm, intensity).
        """
        # Ajouter du bruit angulaire
        noisy_angle = angle_deg + np.random.normal(0, self.angular_noise)
        angle_rad = np.radians(noisy_angle)
        
        # Trouver l'intersection la plus proche
        min_distance = self.MAX_RANGE
        reflectivity = 0.0
        
        # Vérifier tous les obstacles
        for wall in self.environment.walls:
            result = self._ray_wall_intersection(angle_rad, wall)
            if result and result[0] < min_distance:
                min_distance, reflectivity = result
                
        for circle in self.environment.circles:
            result = self._ray_circle_intersection(angle_rad, circle)
            if result and result[0] < min_distance:
                min_distance, reflectivity = result
                
        for rect in self.environment.rectangles:
            result = self._ray_rectangle_intersection(angle_rad, rect)
            if result and result[0] < min_distance:
                min_distance, reflectivity = result
        
        # Ajouter du bruit de distance
        if min_distance < self.MAX_RANGE:
            min_distance += np.random.normal(0, self.noise_stddev)
            min_distance = max(self.MIN_RANGE, min(self.MAX_RANGE, min_distance))
        
        # Simuler des points manquants
        if np.random.random() < self.miss_rate:
            return (0, 0)
        
        # Convertir en mm et calculer l'intensité
        distance_mm = int(min_distance * 1000)
        intensity = int(reflectivity * 200 + np.random.normal(0, 10))
        intensity = max(0, min(255, intensity))
        
        return (distance_mm, intensity)
    
    def generate_scan(self, num_points: int = 480) -> LidarScan:
        """
        Génère un scan complet de 360°.
        
        Args:
            num_points: Nombre de points dans le scan
            
        Returns:
            LidarScan avec les mesures simulées
        """
        measurements = []
        angle_step = 360.0 / num_points
        
        for i in range(num_points):
            angle = i * angle_step
            distance_mm, intensity = self._cast_ray(angle)
            
            measurements.append(LidarMeasurement(
                angle_deg=angle,
                distance_mm=distance_mm,
                intensity=intensity
            ))
        
        return LidarScan(measurements)
    
    def generate_packet(self) -> LidarPacket:
        """
        Génère un paquet de données comme le vrai LiDAR.
        """
        measurements = []
        start_angle = self._current_angle
        angle_step = 360.0 / (480 / MEASUREMENTS_PER_PACKET)  # ~30°/paquet
        
        for i in range(MEASUREMENTS_PER_PACKET):
            angle = (start_angle + i * angle_step / MEASUREMENTS_PER_PACKET) % 360
            distance_mm, intensity = self._cast_ray(angle)
            
            measurements.append(LidarMeasurement(
                angle_deg=angle,
                distance_mm=distance_mm,
                intensity=intensity
            ))
        
        end_angle = (start_angle + angle_step) % 360
        self._current_angle = end_angle
        self._timestamp = (self._timestamp + 8) % 65536  # ~8ms par paquet
        
        return LidarPacket(
            speed_dps=3600,
            start_angle_deg=start_angle,
            end_angle_deg=end_angle,
            timestamp_ms=self._timestamp,
            measurements=measurements,
            crc_valid=True
        )
    
    def generate_raw_packet(self) -> bytes:
        """
        Génère des données brutes comme le port série du LiDAR.
        """
        packet = self.generate_packet()
        
        data = bytearray([HEADER_BYTE, LENGTH_BYTE])
        data.extend(struct.pack("<H", packet.speed_dps))
        data.extend(struct.pack("<H", int(packet.start_angle_deg * 100)))
        
        for m in packet.measurements:
            data.extend(struct.pack("<HB", m.distance_mm, m.intensity))
        
        data.extend(struct.pack("<H", int(packet.end_angle_deg * 100)))
        data.extend(struct.pack("<H", packet.timestamp_ms))
        
        crc = self._crc.calculate(bytes(data))
        data.append(crc)
        
        return bytes(data)


# Environnements prédéfinis

def create_corridor_environment(length: float = 10.0, 
                                width: float = 2.0) -> SimulatedEnvironment:
    """Crée un environnement de couloir."""
    env = SimulatedEnvironment(name="corridor")
    
    half_width = width / 2
    half_length = length / 2
    
    # Murs du couloir
    env.add_wall(-half_width, -half_length, -half_width, half_length)  # Gauche
    env.add_wall(half_width, -half_length, half_width, half_length)    # Droite
    env.add_wall(-half_width, half_length, half_width, half_length)    # Fond
    
    return env


def create_parking_environment() -> SimulatedEnvironment:
    """Crée un environnement de parking avec plusieurs voitures."""
    env = SimulatedEnvironment(name="parking")
    
    # Délimitation du parking
    env.add_wall(-10, -5, 10, -5)
    env.add_wall(10, -5, 10, 5)
    env.add_wall(10, 5, -10, 5)
    env.add_wall(-10, 5, -10, -5)
    
    # Voitures stationnées (rectangles)
    car_positions = [
        (-7, 3, 0), (-4, 3, 0), (-1, 3, 0),
        (-7, -3, 0), (-4, -3, 0), (2, -3, 0), (5, -3, 0)
    ]
    
    for x, y, rot in car_positions:
        env.add_rectangle(x, y, 1.8, 4.5, rot, 0.6)  # Taille voiture
    
    # Piliers
    env.add_circle(0, 0, 0.3)
    env.add_circle(6, 0, 0.3)
    
    return env


def create_room_with_furniture() -> SimulatedEnvironment:
    """Crée une pièce avec des meubles."""
    env = SimulatedEnvironment(name="room_furniture")
    
    # Pièce 8m x 6m
    env.add_wall(-4, -3, 4, -3)
    env.add_wall(4, -3, 4, 3)
    env.add_wall(4, 3, -4, 3)
    env.add_wall(-4, 3, -4, -3)
    
    # Table centrale
    env.add_rectangle(0, 0, 1.5, 0.8, 0, 0.5)
    
    # Canapé
    env.add_rectangle(-2.5, 0, 0.8, 2.0, 0, 0.7)
    
    # Armoire
    env.add_rectangle(3, 0, 0.6, 2.5, 0, 0.8)
    
    # Lampadaire
    env.add_circle(2, 2, 0.15)
    
    # Plantes
    env.add_circle(-3, 2, 0.3)
    env.add_circle(-3, -2, 0.25)
    
    return env


def create_obstacle_course() -> SimulatedEnvironment:
    """Crée un parcours d'obstacles pour tester la navigation."""
    env = SimulatedEnvironment(name="obstacle_course")
    
    # Terrain 15m x 10m
    env.add_wall(-7.5, -5, 7.5, -5)
    env.add_wall(7.5, -5, 7.5, 5)
    env.add_wall(7.5, 5, -7.5, 5)
    env.add_wall(-7.5, 5, -7.5, -5)
    
    # Obstacles variés
    # Ligne de cônes
    for x in range(-5, 6, 2):
        env.add_circle(x, -3, 0.15)
    
    # Slalom
    env.add_rectangle(-4, 0, 0.3, 1.5, 0)
    env.add_rectangle(-2, 1, 0.3, 1.5, 0)
    env.add_rectangle(0, 0, 0.3, 1.5, 0)
    env.add_rectangle(2, 1, 0.3, 1.5, 0)
    env.add_rectangle(4, 0, 0.3, 1.5, 0)
    
    # Zone étroite
    env.add_wall(5, 2, 5, 4)
    env.add_wall(6, 2, 6, 4)
    
    return env


def save_scan_to_file(scan: LidarScan, filepath: str):
    """Sauvegarde un scan dans un fichier JSON."""
    data = {
        "timestamp": time.time(),
        "point_count": scan.point_count,
        "measurements": [
            {
                "angle_deg": m.angle_deg,
                "distance_mm": m.distance_mm,
                "intensity": m.intensity
            }
            for m in scan.measurements
        ]
    }
    
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)


def load_scan_from_file(filepath: str) -> LidarScan:
    """Charge un scan depuis un fichier JSON."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    measurements = [
        LidarMeasurement(
            angle_deg=m["angle_deg"],
            distance_mm=m["distance_mm"],
            intensity=m["intensity"]
        )
        for m in data["measurements"]
    ]
    
    return LidarScan(measurements)


if __name__ == "__main__":
    print("=== Test du Simulateur LiDAR LD19 ===\n")
    
    # Tester différents environnements
    environments = [
        create_corridor_environment(),
        create_parking_environment(),
        create_room_with_furniture(),
        create_obstacle_course()
    ]
    
    for env in environments:
        print(f"\n📍 Environnement: {env.name}")
        print(f"   Murs: {len(env.walls)}")
        print(f"   Cercles: {len(env.circles)}")
        print(f"   Rectangles: {len(env.rectangles)}")
        
        simulator = LidarSimulator(env, noise_stddev=0.01)
        scan = simulator.generate_scan(480)
        
        valid_points = scan.get_valid_measurements()
        print(f"   Points générés: {scan.point_count}")
        print(f"   Points valides: {len(valid_points)}")
        
        if scan.min_distance:
            print(f"   Distance min: {scan.min_distance:.2f} m")
        
        # Sauvegarder un scan de test
        save_path = f"/home/claude/lidar_test_suite/data/scan_{env.name}.json"
        save_scan_to_file(scan, save_path)
        print(f"   Scan sauvegardé: {save_path}")
    
    # Test de génération de paquets bruts
    print("\n\n📦 Test génération paquets bruts:")
    sim = LidarSimulator(create_room_with_furniture())
    
    for i in range(3):
        raw_data = sim.generate_raw_packet()
        print(f"   Paquet {i+1}: {len(raw_data)} bytes, "
              f"Header: 0x{raw_data[0]:02X}, Length: 0x{raw_data[1]:02X}")