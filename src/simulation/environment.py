"""
Environnement virtuel pour simulation.

Definit les obstacles et murs pour simuler un environnement reel.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum


class ObstacleType(Enum):
    """Types d'obstacles."""
    WALL = "wall"           # Mur (ligne)
    BOX = "box"             # Boite rectangulaire
    CYLINDER = "cylinder"   # Cylindre (poteau)
    CIRCLE = "circle"       # Cercle


@dataclass
class Obstacle:
    """
    Obstacle dans l'environnement.

    Pour WALL: (x1, y1) -> (x2, y2)
    Pour BOX: centre (x, y), largeur, hauteur, rotation
    Pour CYLINDER/CIRCLE: centre (x, y), rayon
    """
    obstacle_type: ObstacleType
    x: float
    y: float
    # Pour WALL: point final
    x2: Optional[float] = None
    y2: Optional[float] = None
    # Pour BOX: dimensions
    width: Optional[float] = None
    height: Optional[float] = None
    rotation: float = 0.0
    # Pour CYLINDER/CIRCLE: rayon
    radius: Optional[float] = None

    def intersect_ray(self, origin_x: float, origin_y: float, angle: float) -> Optional[float]:
        """
        Calcule l'intersection d'un rayon avec cet obstacle.

        Args:
            origin_x, origin_y: Point d'origine du rayon
            angle: Angle du rayon en radians

        Returns:
            Distance jusqu'a l'intersection, ou None si pas d'intersection
        """
        if self.obstacle_type == ObstacleType.WALL:
            return self._intersect_wall(origin_x, origin_y, angle)
        elif self.obstacle_type == ObstacleType.CIRCLE or self.obstacle_type == ObstacleType.CYLINDER:
            return self._intersect_circle(origin_x, origin_y, angle)
        elif self.obstacle_type == ObstacleType.BOX:
            return self._intersect_box(origin_x, origin_y, angle)
        return None

    def _intersect_wall(self, ox: float, oy: float, angle: float) -> Optional[float]:
        """Intersection rayon-segment."""
        dx = math.cos(angle)
        dy = math.sin(angle)

        x1, y1 = self.x, self.y
        x2, y2 = self.x2, self.y2

        # Vecteur du segment
        sx = x2 - x1
        sy = y2 - y1

        # Calcul intersection parametrique
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-10:
            return None  # Parallele

        t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom

        if t > 0.001 and 0 <= u <= 1:
            return t
        return None

    def _intersect_circle(self, ox: float, oy: float, angle: float) -> Optional[float]:
        """Intersection rayon-cercle."""
        dx = math.cos(angle)
        dy = math.sin(angle)

        cx, cy = self.x, self.y
        r = self.radius

        # Vecteur origine -> centre
        fx = ox - cx
        fy = oy - cy

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return None

        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)

        if t1 > 0.001:
            return t1
        if t2 > 0.001:
            return t2
        return None

    def _intersect_box(self, ox: float, oy: float, angle: float) -> Optional[float]:
        """Intersection rayon-boite (4 murs)."""
        # Creer 4 murs pour la boite
        cx, cy = self.x, self.y
        w, h = self.width / 2, self.height / 2
        rot = self.rotation

        cos_r = math.cos(rot)
        sin_r = math.sin(rot)

        # Coins de la boite
        corners = [
            (-w, -h), (w, -h), (w, h), (-w, h)
        ]

        # Rotation et translation des coins
        rotated = []
        for px, py in corners:
            rx = px * cos_r - py * sin_r + cx
            ry = px * sin_r + py * cos_r + cy
            rotated.append((rx, ry))

        # Test intersection avec chaque cote
        min_dist = None
        for i in range(4):
            x1, y1 = rotated[i]
            x2, y2 = rotated[(i + 1) % 4]

            wall = Obstacle(ObstacleType.WALL, x1, y1, x2, y2)
            dist = wall._intersect_wall(ox, oy, angle)

            if dist is not None:
                if min_dist is None or dist < min_dist:
                    min_dist = dist

        return min_dist


@dataclass
class Environment:
    """
    Environnement de simulation avec obstacles.
    """
    obstacles: List[Obstacle] = field(default_factory=list)
    width: float = 20.0   # Largeur en metres
    height: float = 20.0  # Hauteur en metres

    def add_obstacle(self, obstacle: Obstacle):
        """Ajoute un obstacle."""
        self.obstacles.append(obstacle)

    def add_wall(self, x1: float, y1: float, x2: float, y2: float):
        """Ajoute un mur."""
        self.obstacles.append(Obstacle(ObstacleType.WALL, x1, y1, x2, y2))

    def add_box(self, x: float, y: float, width: float, height: float, rotation: float = 0):
        """Ajoute une boite."""
        self.obstacles.append(Obstacle(
            ObstacleType.BOX, x, y,
            width=width, height=height, rotation=rotation
        ))

    def add_cylinder(self, x: float, y: float, radius: float):
        """Ajoute un cylindre/poteau."""
        self.obstacles.append(Obstacle(ObstacleType.CYLINDER, x, y, radius=radius))

    def raycast(self, origin_x: float, origin_y: float, angle: float,
                max_range: float = 12.0) -> float:
        """
        Lance un rayon et retourne la distance jusqu'au premier obstacle.

        Args:
            origin_x, origin_y: Position du robot
            angle: Angle du rayon (radians)
            max_range: Portee maximale

        Returns:
            Distance jusqu'a l'obstacle (ou max_range si rien)
        """
        min_dist = max_range

        for obstacle in self.obstacles:
            dist = obstacle.intersect_ray(origin_x, origin_y, angle)
            if dist is not None and dist < min_dist:
                min_dist = dist

        return min_dist

    def add_boundary_walls(self):
        """Ajoute les murs de bordure de l'environnement."""
        w, h = self.width / 2, self.height / 2
        self.add_wall(-w, -h, w, -h)   # Bas
        self.add_wall(w, -h, w, h)     # Droite
        self.add_wall(w, h, -w, h)     # Haut
        self.add_wall(-w, h, -w, -h)   # Gauche


def create_parking_env() -> Environment:
    """
    Cree un environnement de parking avec voitures.
    """
    env = Environment(width=20, height=15)
    env.add_boundary_walls()

    # Voitures garees (boites)
    env.add_box(-6, -4, 4.0, 2.0, 0)       # Voiture 1
    env.add_box(-6, 0, 4.0, 2.0, 0)        # Voiture 2
    env.add_box(-6, 4, 4.0, 2.0, 0)        # Voiture 3

    env.add_box(6, -4, 4.0, 2.0, 0)        # Voiture 4
    env.add_box(6, 0, 4.0, 2.0, 0)         # Voiture 5
    env.add_box(6, 4, 4.0, 2.0, 0)         # Voiture 6

    # Poteaux
    env.add_cylinder(0, -5, 0.15)
    env.add_cylinder(0, 5, 0.15)

    return env


def create_corridor_env() -> Environment:
    """
    Cree un environnement de couloir avec obstacles.
    """
    env = Environment(width=15, height=6)
    env.add_boundary_walls()

    # Obstacles dans le couloir
    env.add_box(-4, 1, 1.0, 1.0, 0.3)      # Boite 1
    env.add_box(0, -1, 1.5, 1.0, -0.2)     # Boite 2
    env.add_box(4, 0.5, 1.0, 1.5, 0.1)     # Boite 3

    # Poteaux
    env.add_cylinder(-2, -1.5, 0.2)
    env.add_cylinder(2, 1.5, 0.2)

    return env


def create_empty_env(width: float = 20, height: float = 20) -> Environment:
    """Cree un environnement vide avec juste les murs."""
    env = Environment(width=width, height=height)
    env.add_boundary_walls()
    return env


if __name__ == "__main__":
    # Test rapide
    env = create_parking_env()

    # Test raycast depuis le centre
    for angle_deg in range(0, 360, 45):
        angle = math.radians(angle_deg)
        dist = env.raycast(0, 0, angle)
        print(f"Angle {angle_deg:3d} deg: {dist:.2f}m")
