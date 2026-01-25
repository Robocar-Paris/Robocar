"""
Coordinate Transformations

Handles coordinate transformations between different frames:
- Robot frame (body frame)
- World frame (global/map frame)
- GPS frame (WGS84 / local ENU)
- Sensor frames (LiDAR, camera, etc.)

Conventions:
- X = forward (front of robot)
- Y = left
- Z = up (not used in 2D)
- Angles are counter-clockwise from X axis
"""

import math
from typing import Tuple, List, Optional
from dataclasses import dataclass
import numpy as np


@dataclass
class Pose2D:
    """2D pose (position + orientation)."""
    x: float
    y: float
    theta: float  # Orientation in radians

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta])

    @staticmethod
    def from_array(arr: np.ndarray) -> 'Pose2D':
        return Pose2D(float(arr[0]), float(arr[1]), float(arr[2]))

    def __add__(self, other: 'Pose2D') -> 'Pose2D':
        """Compose two poses (other relative to self)."""
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        x = self.x + cos_t * other.x - sin_t * other.y
        y = self.y + sin_t * other.x + cos_t * other.y
        theta = normalize_angle(self.theta + other.theta)
        return Pose2D(x, y, theta)

    def inverse(self) -> 'Pose2D':
        """Return inverse transformation."""
        cos_t = math.cos(-self.theta)
        sin_t = math.sin(-self.theta)
        x = -(cos_t * self.x - sin_t * self.y)
        y = -(sin_t * self.x + cos_t * self.y)
        return Pose2D(x, y, -self.theta)

    def distance_to(self, other: 'Pose2D') -> float:
        """Euclidean distance to another pose."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle_to(self, other: 'Pose2D') -> float:
        """Angle from this pose to another pose."""
        return math.atan2(other.y - self.y, other.x - self.x)


@dataclass
class Transform2D:
    """2D rigid transformation (rotation + translation)."""
    x: float        # Translation X
    y: float        # Translation Y
    theta: float    # Rotation angle

    def to_matrix(self) -> np.ndarray:
        """Convert to 3x3 homogeneous transformation matrix."""
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        return np.array([
            [cos_t, -sin_t, self.x],
            [sin_t,  cos_t, self.y],
            [0,      0,     1]
        ])

    @staticmethod
    def from_matrix(matrix: np.ndarray) -> 'Transform2D':
        """Create from 3x3 homogeneous transformation matrix."""
        theta = math.atan2(matrix[1, 0], matrix[0, 0])
        x = matrix[0, 2]
        y = matrix[1, 2]
        return Transform2D(x, y, theta)

    def apply(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """Apply transformation to a point."""
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        x = cos_t * point[0] - sin_t * point[1] + self.x
        y = sin_t * point[0] + cos_t * point[1] + self.y
        return (x, y)

    def apply_batch(self, points: np.ndarray) -> np.ndarray:
        """Apply transformation to multiple points (Nx2 array)."""
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        rotation = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        translation = np.array([self.x, self.y])

        return points @ rotation.T + translation

    def inverse(self) -> 'Transform2D':
        """Return inverse transformation."""
        cos_t = math.cos(-self.theta)
        sin_t = math.sin(-self.theta)
        x = -(cos_t * self.x - sin_t * self.y)
        y = -(sin_t * self.x + cos_t * self.y)
        return Transform2D(x, y, -self.theta)

    def compose(self, other: 'Transform2D') -> 'Transform2D':
        """Compose with another transformation (self * other)."""
        result = self.to_matrix() @ other.to_matrix()
        return Transform2D.from_matrix(result)

    @staticmethod
    def identity() -> 'Transform2D':
        """Return identity transformation."""
        return Transform2D(0.0, 0.0, 0.0)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def angle_difference(a: float, b: float) -> float:
    """Calculate shortest angular difference from a to b."""
    diff = b - a
    return normalize_angle(diff)


class FrameManager:
    """
    Manages coordinate frames and transformations.

    Maintains a tree of coordinate frames and provides
    transformations between any two frames.

    Usage:
        fm = FrameManager()

        # Define sensor positions relative to robot
        fm.set_transform('robot', 'lidar', Transform2D(0.1, 0, 0))
        fm.set_transform('robot', 'gps', Transform2D(0, 0, 0))

        # Update robot position in world
        fm.set_transform('world', 'robot', Transform2D(x, y, theta))

        # Transform LiDAR point to world frame
        world_point = fm.transform_point('lidar', 'world', lidar_point)
    """

    def __init__(self):
        """Initialize frame manager."""
        self._transforms: dict = {}  # (from_frame, to_frame) -> Transform2D
        self._frame_tree: dict = {}  # frame -> parent_frame

    def set_transform(
        self,
        from_frame: str,
        to_frame: str,
        transform: Transform2D
    ):
        """
        Set transformation from one frame to another.

        Args:
            from_frame: Parent frame
            to_frame: Child frame
            transform: Transformation from parent to child
        """
        self._transforms[(from_frame, to_frame)] = transform
        self._transforms[(to_frame, from_frame)] = transform.inverse()
        self._frame_tree[to_frame] = from_frame

    def get_transform(
        self,
        from_frame: str,
        to_frame: str
    ) -> Optional[Transform2D]:
        """
        Get transformation between two frames.

        Args:
            from_frame: Source frame
            to_frame: Target frame

        Returns:
            Transform2D or None if no path exists
        """
        if from_frame == to_frame:
            return Transform2D.identity()

        # Direct transform
        if (from_frame, to_frame) in self._transforms:
            return self._transforms[(from_frame, to_frame)]

        # Find path through frame tree
        path = self._find_path(from_frame, to_frame)
        if path is None:
            return None

        # Compose transforms along path
        result = Transform2D.identity()
        for i in range(len(path) - 1):
            t = self._transforms.get((path[i], path[i+1]))
            if t is None:
                return None
            result = result.compose(t)

        return result

    def _find_path(self, from_frame: str, to_frame: str) -> Optional[List[str]]:
        """Find path between frames using BFS."""
        if from_frame == to_frame:
            return [from_frame]

        # Build adjacency list
        neighbors: dict = {}
        for (f1, f2) in self._transforms:
            if f1 not in neighbors:
                neighbors[f1] = []
            neighbors[f1].append(f2)

        # BFS
        visited = {from_frame}
        queue = [(from_frame, [from_frame])]

        while queue:
            current, path = queue.pop(0)

            for neighbor in neighbors.get(current, []):
                if neighbor == to_frame:
                    return path + [neighbor]

                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return None

    def transform_point(
        self,
        from_frame: str,
        to_frame: str,
        point: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        """Transform a point between frames."""
        transform = self.get_transform(from_frame, to_frame)
        if transform is None:
            return None
        return transform.apply(point)

    def transform_points(
        self,
        from_frame: str,
        to_frame: str,
        points: np.ndarray
    ) -> Optional[np.ndarray]:
        """Transform multiple points between frames."""
        transform = self.get_transform(from_frame, to_frame)
        if transform is None:
            return None
        return transform.apply_batch(points)

    def transform_pose(
        self,
        from_frame: str,
        to_frame: str,
        pose: Pose2D
    ) -> Optional[Pose2D]:
        """Transform a pose between frames."""
        transform = self.get_transform(from_frame, to_frame)
        if transform is None:
            return None

        # Transform position
        x, y = transform.apply((pose.x, pose.y))

        # Transform orientation
        theta = normalize_angle(pose.theta + transform.theta)

        return Pose2D(x, y, theta)


class GPSConverter:
    """
    Converts between GPS (WGS84) and local ENU coordinates.

    ENU = East-North-Up coordinate system centered at origin.
    """

    # WGS84 ellipsoid parameters
    WGS84_A = 6378137.0             # Semi-major axis (m)
    WGS84_F = 1 / 298.257223563     # Flattening
    WGS84_B = WGS84_A * (1 - WGS84_F)  # Semi-minor axis

    def __init__(
        self,
        origin_lat: float = 0.0,
        origin_lon: float = 0.0,
        origin_alt: float = 0.0
    ):
        """
        Initialize GPS converter.

        Args:
            origin_lat: Origin latitude (degrees)
            origin_lon: Origin longitude (degrees)
            origin_alt: Origin altitude (meters)
        """
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt
        self._origin_set = (origin_lat != 0.0 or origin_lon != 0.0)

        # Precompute scale factors at origin
        self._update_scale_factors()

    def set_origin(self, lat: float, lon: float, alt: float = 0.0):
        """Set the origin for local coordinate system."""
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt
        self._origin_set = True
        self._update_scale_factors()

    def _update_scale_factors(self):
        """Update meters-per-degree scale factors."""
        lat_rad = math.radians(self.origin_lat)

        # Radius of curvature
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)

        # Meters per degree at this latitude
        self._m_per_deg_lat = (
            math.pi * self.WGS84_A * (1 - self.WGS84_F**2) /
            (180 * (1 - self.WGS84_F**2 * sin_lat**2)**1.5)
        )
        self._m_per_deg_lon = (
            math.pi * self.WGS84_A * cos_lat /
            (180 * math.sqrt(1 - self.WGS84_F**2 * sin_lat**2))
        )

    def gps_to_local(
        self,
        lat: float,
        lon: float,
        alt: float = 0.0
    ) -> Tuple[float, float, float]:
        """
        Convert GPS to local ENU coordinates.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude (meters)

        Returns:
            (east, north, up) in meters
        """
        if not self._origin_set:
            self.set_origin(lat, lon, alt)

        east = (lon - self.origin_lon) * self._m_per_deg_lon
        north = (lat - self.origin_lat) * self._m_per_deg_lat
        up = alt - self.origin_alt

        return (east, north, up)

    def local_to_gps(
        self,
        east: float,
        north: float,
        up: float = 0.0
    ) -> Tuple[float, float, float]:
        """
        Convert local ENU coordinates to GPS.

        Args:
            east: East offset (meters)
            north: North offset (meters)
            up: Up offset (meters)

        Returns:
            (latitude, longitude, altitude) in degrees/meters
        """
        lon = self.origin_lon + east / self._m_per_deg_lon
        lat = self.origin_lat + north / self._m_per_deg_lat
        alt = self.origin_alt + up

        return (lat, lon, alt)

    def distance(
        self,
        lat1: float, lon1: float,
        lat2: float, lon2: float
    ) -> float:
        """
        Calculate distance between two GPS points (Haversine).

        Returns distance in meters.
        """
        R = 6371000  # Earth radius in meters

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (math.sin(dlat/2)**2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def bearing(
        self,
        lat1: float, lon1: float,
        lat2: float, lon2: float
    ) -> float:
        """
        Calculate bearing from point 1 to point 2.

        Returns bearing in radians (0 = North, pi/2 = East).
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)

        x = math.sin(dlon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))

        return math.atan2(x, y)


if __name__ == '__main__':
    # Test transformations
    print("Testing Coordinate Transformations...")

    # Test Transform2D
    t1 = Transform2D(1.0, 0.0, math.pi/2)  # Move 1m, rotate 90°
    point = (1.0, 0.0)
    transformed = t1.apply(point)
    print(f"Transform (1,0) with T(1,0,90°): ({transformed[0]:.2f}, {transformed[1]:.2f})")
    # Expected: (1, 1) - move 1m forward, then point is 1m to the left

    # Test inverse
    t_inv = t1.inverse()
    back = t_inv.apply(transformed)
    print(f"Inverse transform back: ({back[0]:.2f}, {back[1]:.2f})")

    # Test FrameManager
    print("\nTesting FrameManager...")
    fm = FrameManager()

    # Set up frames
    fm.set_transform('world', 'robot', Transform2D(5.0, 3.0, math.pi/4))
    fm.set_transform('robot', 'lidar', Transform2D(0.1, 0.0, 0.0))

    # Transform point from lidar to world
    lidar_point = (2.0, 0.0)  # 2m in front of lidar
    world_point = fm.transform_point('lidar', 'world', lidar_point)
    print(f"LiDAR point (2,0) in world: ({world_point[0]:.2f}, {world_point[1]:.2f})")

    # Test GPSConverter
    print("\nTesting GPSConverter...")
    gps = GPSConverter()

    # Paris coordinates
    origin = (48.8566, 2.3522)  # Eiffel Tower area
    gps.set_origin(*origin)

    # Point 100m north
    test_lat = origin[0] + 100 / gps._m_per_deg_lat
    test_lon = origin[1]

    local = gps.gps_to_local(test_lat, test_lon)
    print(f"100m north in local: ({local[0]:.1f}, {local[1]:.1f}, {local[2]:.1f})")

    # Convert back
    back_gps = gps.local_to_gps(local[0], local[1])
    print(f"Back to GPS: ({back_gps[0]:.6f}, {back_gps[1]:.6f})")

    print("\nTest complete!")
