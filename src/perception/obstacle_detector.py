"""
Obstacle Detector

Detects, classifies, and tracks obstacles from LiDAR data.

Features:
- Obstacle clustering (DBSCAN-like)
- Obstacle classification (static, dynamic, wall)
- Bounding box estimation
- Obstacle tracking across frames
- Collision prediction
"""

import math
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from enum import Enum
import numpy as np

from .lidar_processor import LidarProcessor, ProcessedScan, ScanSegment


class ObstacleType(Enum):
    """Type of detected obstacle."""
    UNKNOWN = 0
    STATIC = 1      # Static obstacle (doesn't move)
    DYNAMIC = 2     # Moving obstacle
    WALL = 3        # Wall or large flat surface
    POLE = 4        # Thin vertical obstacle (pole, leg)


@dataclass
class BoundingBox:
    """Axis-aligned bounding box."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def area(self) -> float:
        return self.width * self.height

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def distance_to(self, x: float, y: float) -> float:
        """Distance from point to bounding box."""
        cx = max(self.x_min, min(x, self.x_max))
        cy = max(self.y_min, min(y, self.y_max))
        return math.sqrt((x - cx)**2 + (y - cy)**2)

    def intersects(self, other: 'BoundingBox') -> bool:
        return not (self.x_max < other.x_min or self.x_min > other.x_max or
                    self.y_max < other.y_min or self.y_min > other.y_max)


@dataclass
class Obstacle:
    """Detected obstacle."""
    id: int
    timestamp: float
    points: np.ndarray           # Nx2 array of (x, y)
    centroid: Tuple[float, float]
    bbox: BoundingBox
    obstacle_type: ObstacleType
    velocity: Tuple[float, float] = (0.0, 0.0)  # Estimated velocity (vx, vy)
    confidence: float = 1.0
    age: int = 0                 # Number of frames since first detected

    @property
    def distance(self) -> float:
        """Distance from robot (origin) to obstacle centroid."""
        return math.sqrt(self.centroid[0]**2 + self.centroid[1]**2)

    @property
    def angle(self) -> float:
        """Angle from robot to obstacle centroid (radians)."""
        return math.atan2(self.centroid[1], self.centroid[0])

    @property
    def min_distance(self) -> float:
        """Minimum distance from robot to any point in obstacle."""
        if len(self.points) == 0:
            return self.distance
        distances = np.linalg.norm(self.points, axis=1)
        return float(np.min(distances))

    @property
    def speed(self) -> float:
        """Speed of obstacle."""
        return math.sqrt(self.velocity[0]**2 + self.velocity[1]**2)

    def predict_position(self, dt: float) -> Tuple[float, float]:
        """Predict obstacle position after dt seconds."""
        return (
            self.centroid[0] + self.velocity[0] * dt,
            self.centroid[1] + self.velocity[1] * dt
        )

    def time_to_collision(self, robot_velocity: Tuple[float, float] = (0, 0)) -> float:
        """
        Estimate time to collision with robot.

        Args:
            robot_velocity: Robot velocity (vx, vy)

        Returns:
            Time to collision in seconds, or inf if no collision
        """
        # Relative velocity
        rel_vx = self.velocity[0] - robot_velocity[0]
        rel_vy = self.velocity[1] - robot_velocity[1]

        # Simple estimation: time for obstacle to reach origin
        # Using closest point approach
        min_dist = self.min_distance
        rel_speed = math.sqrt(rel_vx**2 + rel_vy**2)

        if rel_speed < 0.01:
            return float('inf')

        # Check if obstacle is approaching
        dot = self.centroid[0] * rel_vx + self.centroid[1] * rel_vy
        if dot >= 0:  # Moving away
            return float('inf')

        return min_dist / rel_speed


class ObstacleDetector:
    """
    Detects and tracks obstacles from LiDAR scans.

    Usage:
        detector = ObstacleDetector()

        # Detect obstacles in a scan
        obstacles = detector.detect(processed_scan)

        # Get obstacles in danger zone
        danger = detector.get_obstacles_in_range(obstacles, max_distance=1.0)

        # Check for collision risk
        if detector.check_collision_risk(obstacles, robot_velocity):
            emergency_stop()
    """

    def __init__(
        self,
        cluster_threshold: float = 0.3,
        min_cluster_points: int = 3,
        max_cluster_points: int = 500,
        wall_min_length: float = 0.5,
        wall_linearity_threshold: float = 0.95,
        tracking_distance: float = 0.5,
        tracking_max_age: int = 10
    ):
        """
        Initialize obstacle detector.

        Args:
            cluster_threshold: Maximum distance between points in a cluster (meters)
            min_cluster_points: Minimum points for a valid cluster
            max_cluster_points: Maximum points for a cluster (split if exceeded)
            wall_min_length: Minimum length to classify as wall (meters)
            wall_linearity_threshold: Minimum linearity score for walls (0-1)
            tracking_distance: Maximum distance for obstacle matching (meters)
            tracking_max_age: Maximum frames before dropping tracked obstacle
        """
        self.cluster_threshold = cluster_threshold
        self.min_cluster_points = min_cluster_points
        self.max_cluster_points = max_cluster_points
        self.wall_min_length = wall_min_length
        self.wall_linearity_threshold = wall_linearity_threshold
        self.tracking_distance = tracking_distance
        self.tracking_max_age = tracking_max_age

        self._next_id = 0
        self._tracked_obstacles: Dict[int, Obstacle] = {}
        self._last_timestamp = 0.0

    def detect(self, scan: ProcessedScan) -> List[Obstacle]:
        """
        Detect obstacles in a processed scan.

        Args:
            scan: Processed LiDAR scan

        Returns:
            List of detected obstacles
        """
        # Get valid points
        valid_mask = scan.valid_mask
        points = scan.points[valid_mask]

        if len(points) < self.min_cluster_points:
            return []

        # Cluster points
        clusters = self._cluster_points(points)

        # Create obstacles from clusters
        obstacles = []
        for cluster_points in clusters:
            obstacle = self._create_obstacle(cluster_points, scan.timestamp)
            if obstacle:
                obstacles.append(obstacle)

        # Track obstacles across frames
        obstacles = self._track_obstacles(obstacles, scan.timestamp)

        return obstacles

    def _cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster points using a simple distance-based algorithm.

        Similar to DBSCAN but optimized for ordered LiDAR points.
        """
        if len(points) == 0:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            # Distance to last point in cluster
            dist = np.linalg.norm(points[i] - current_cluster[-1])

            if dist <= self.cluster_threshold:
                current_cluster.append(points[i])
            else:
                # Save current cluster if valid
                if len(current_cluster) >= self.min_cluster_points:
                    clusters.append(np.array(current_cluster))
                # Start new cluster
                current_cluster = [points[i]]

        # Don't forget last cluster
        if len(current_cluster) >= self.min_cluster_points:
            clusters.append(np.array(current_cluster))

        # Check if first and last clusters should be merged (circular scan)
        if len(clusters) >= 2:
            first_point = clusters[0][0]
            last_point = clusters[-1][-1]
            if np.linalg.norm(first_point - last_point) <= self.cluster_threshold:
                # Merge first and last
                merged = np.vstack([clusters[-1], clusters[0]])
                clusters = clusters[1:-1]
                clusters.append(merged)

        return clusters

    def _create_obstacle(
        self,
        points: np.ndarray,
        timestamp: float
    ) -> Optional[Obstacle]:
        """Create an Obstacle from a cluster of points."""
        if len(points) < self.min_cluster_points:
            return None

        # Calculate centroid
        centroid = (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))

        # Calculate bounding box
        bbox = BoundingBox(
            x_min=float(np.min(points[:, 0])),
            x_max=float(np.max(points[:, 0])),
            y_min=float(np.min(points[:, 1])),
            y_max=float(np.max(points[:, 1]))
        )

        # Classify obstacle
        obstacle_type = self._classify_obstacle(points, bbox)

        return Obstacle(
            id=self._get_next_id(),
            timestamp=timestamp,
            points=points,
            centroid=centroid,
            bbox=bbox,
            obstacle_type=obstacle_type
        )

    def _classify_obstacle(
        self,
        points: np.ndarray,
        bbox: BoundingBox
    ) -> ObstacleType:
        """Classify the type of obstacle."""
        # Calculate linearity using PCA
        linearity = self._calculate_linearity(points)

        # Calculate dimensions
        width = bbox.width
        height = bbox.height
        aspect_ratio = max(width, height) / (min(width, height) + 0.001)
        length = max(width, height)

        # Wall: long and linear
        if length >= self.wall_min_length and linearity >= self.wall_linearity_threshold:
            return ObstacleType.WALL

        # Pole: thin with high aspect ratio
        if aspect_ratio > 5 and min(width, height) < 0.15:
            return ObstacleType.POLE

        # Default to static (will be updated by tracking if moving)
        return ObstacleType.STATIC

    def _calculate_linearity(self, points: np.ndarray) -> float:
        """
        Calculate linearity score of points using PCA.

        Returns value between 0 (not linear) and 1 (perfectly linear).
        """
        if len(points) < 3:
            return 0.0

        # Center points
        centered = points - np.mean(points, axis=0)

        # Covariance matrix
        cov = np.cov(centered.T)

        if cov.ndim < 2:
            return 0.0

        # Eigenvalues
        eigenvalues = np.linalg.eigvals(cov)
        eigenvalues = np.sort(np.abs(eigenvalues))[::-1]

        # Linearity = ratio of eigenvalues
        if eigenvalues[0] < 1e-10:
            return 0.0

        return float(1.0 - eigenvalues[1] / eigenvalues[0])

    def _track_obstacles(
        self,
        new_obstacles: List[Obstacle],
        timestamp: float
    ) -> List[Obstacle]:
        """
        Track obstacles across frames for velocity estimation.

        Matches new obstacles to previously detected ones and
        estimates velocity based on position change.
        """
        dt = timestamp - self._last_timestamp
        self._last_timestamp = timestamp

        if dt <= 0 or dt > 1.0:  # Reset tracking if too much time passed
            self._tracked_obstacles.clear()
            for obs in new_obstacles:
                self._tracked_obstacles[obs.id] = obs
            return new_obstacles

        # Match new obstacles to tracked ones
        matched_ids = set()
        result = []

        for new_obs in new_obstacles:
            best_match = None
            best_distance = self.tracking_distance

            for tracked_id, tracked_obs in self._tracked_obstacles.items():
                if tracked_id in matched_ids:
                    continue

                # Calculate distance between centroids
                dist = math.sqrt(
                    (new_obs.centroid[0] - tracked_obs.centroid[0])**2 +
                    (new_obs.centroid[1] - tracked_obs.centroid[1])**2
                )

                if dist < best_distance:
                    best_distance = dist
                    best_match = tracked_obs

            if best_match:
                # Update tracked obstacle
                matched_ids.add(best_match.id)

                # Estimate velocity
                vx = (new_obs.centroid[0] - best_match.centroid[0]) / dt
                vy = (new_obs.centroid[1] - best_match.centroid[1]) / dt

                # Smooth velocity with previous estimate
                alpha = 0.3
                vx = alpha * vx + (1 - alpha) * best_match.velocity[0]
                vy = alpha * vy + (1 - alpha) * best_match.velocity[1]

                # Create updated obstacle
                updated = Obstacle(
                    id=best_match.id,
                    timestamp=timestamp,
                    points=new_obs.points,
                    centroid=new_obs.centroid,
                    bbox=new_obs.bbox,
                    obstacle_type=new_obs.obstacle_type,
                    velocity=(vx, vy),
                    confidence=min(1.0, best_match.confidence + 0.1),
                    age=best_match.age + 1
                )

                # Update type if moving
                if updated.speed > 0.1:
                    updated = Obstacle(
                        id=updated.id,
                        timestamp=updated.timestamp,
                        points=updated.points,
                        centroid=updated.centroid,
                        bbox=updated.bbox,
                        obstacle_type=ObstacleType.DYNAMIC,
                        velocity=updated.velocity,
                        confidence=updated.confidence,
                        age=updated.age
                    )

                result.append(updated)
                self._tracked_obstacles[best_match.id] = updated
            else:
                # New obstacle
                result.append(new_obs)
                self._tracked_obstacles[new_obs.id] = new_obs

        # Age out unmatched tracked obstacles
        to_remove = []
        for tracked_id, tracked_obs in self._tracked_obstacles.items():
            if tracked_id not in matched_ids:
                if tracked_obs.age >= self.tracking_max_age:
                    to_remove.append(tracked_id)
                else:
                    # Keep but age
                    self._tracked_obstacles[tracked_id] = Obstacle(
                        id=tracked_obs.id,
                        timestamp=tracked_obs.timestamp,
                        points=tracked_obs.points,
                        centroid=tracked_obs.centroid,
                        bbox=tracked_obs.bbox,
                        obstacle_type=tracked_obs.obstacle_type,
                        velocity=tracked_obs.velocity,
                        confidence=tracked_obs.confidence * 0.9,
                        age=tracked_obs.age + 1
                    )

        for tracked_id in to_remove:
            del self._tracked_obstacles[tracked_id]

        return result

    def _get_next_id(self) -> int:
        """Get next unique obstacle ID."""
        self._next_id += 1
        return self._next_id

    def get_obstacles_in_range(
        self,
        obstacles: List[Obstacle],
        max_distance: float,
        angle_min: float = -np.pi,
        angle_max: float = np.pi
    ) -> List[Obstacle]:
        """
        Get obstacles within a specified range and angle.

        Args:
            obstacles: List of obstacles
            max_distance: Maximum distance (meters)
            angle_min: Minimum angle (radians)
            angle_max: Maximum angle (radians)

        Returns:
            Filtered list of obstacles
        """
        result = []
        for obs in obstacles:
            if obs.min_distance > max_distance:
                continue

            angle = obs.angle
            if angle_min < angle_max:
                if not (angle_min <= angle <= angle_max):
                    continue
            else:
                if not (angle >= angle_min or angle <= angle_max):
                    continue

            result.append(obs)

        return result

    def get_nearest_obstacle(
        self,
        obstacles: List[Obstacle],
        angle_min: float = -np.pi,
        angle_max: float = np.pi
    ) -> Optional[Obstacle]:
        """Get the nearest obstacle in a direction range."""
        filtered = self.get_obstacles_in_range(
            obstacles, float('inf'), angle_min, angle_max
        )

        if not filtered:
            return None

        return min(filtered, key=lambda o: o.min_distance)

    def check_collision_risk(
        self,
        obstacles: List[Obstacle],
        robot_velocity: Tuple[float, float] = (0, 0),
        time_horizon: float = 2.0,
        safety_distance: float = 0.3
    ) -> Tuple[bool, Optional[Obstacle], float]:
        """
        Check for collision risk.

        Args:
            obstacles: List of obstacles
            robot_velocity: Robot velocity (vx, vy)
            time_horizon: Time to look ahead (seconds)
            safety_distance: Minimum safe distance (meters)

        Returns:
            Tuple of (is_collision_risk, dangerous_obstacle, time_to_collision)
        """
        min_ttc = float('inf')
        dangerous = None

        for obs in obstacles:
            # Check current distance
            if obs.min_distance < safety_distance:
                return True, obs, 0.0

            # Check future collision
            ttc = obs.time_to_collision(robot_velocity)
            if ttc < time_horizon and ttc < min_ttc:
                min_ttc = ttc
                dangerous = obs

        if min_ttc < time_horizon:
            return True, dangerous, min_ttc

        return False, None, float('inf')

    def get_free_space(
        self,
        obstacles: List[Obstacle],
        angle_resolution: float = 0.1,
        max_range: float = 5.0
    ) -> List[Tuple[float, float]]:
        """
        Calculate free space around the robot.

        Returns list of (angle, distance) pairs representing
        the distance to nearest obstacle in each direction.
        """
        angles = np.arange(-np.pi, np.pi, angle_resolution)
        free_space = []

        for angle in angles:
            min_dist = max_range

            for obs in obstacles:
                # Check if obstacle is in this direction
                for point in obs.points:
                    point_angle = math.atan2(point[1], point[0])
                    angle_diff = abs(point_angle - angle)
                    angle_diff = min(angle_diff, 2*np.pi - angle_diff)

                    if angle_diff < angle_resolution:
                        dist = np.linalg.norm(point)
                        min_dist = min(min_dist, dist)

            free_space.append((float(angle), min_dist))

        return free_space


if __name__ == '__main__':
    # Test obstacle detector
    from .lidar_processor import LidarProcessor, create_simulated_scan

    print("Testing Obstacle Detector...")

    # Create simulated environment
    obstacles = [
        (2.0, 0.5, 0.3),
        (1.0, -1.0, 0.2),
        (-1.5, 1.5, 0.4),
    ]

    walls = [
        ((4.0, -3.0), (4.0, 3.0)),
        ((-4.0, -3.0), (-4.0, 3.0)),
    ]

    # Generate and process scan
    scan = create_simulated_scan(obstacles, walls)
    processor = LidarProcessor()
    processed = processor.process(scan)

    # Detect obstacles
    detector = ObstacleDetector()
    detected = detector.detect(processed)

    print(f"Detected {len(detected)} obstacles:")
    for obs in detected:
        print(f"  ID={obs.id}: type={obs.obstacle_type.name}, "
              f"dist={obs.distance:.2f}m, angle={np.degrees(obs.angle):.1f}°, "
              f"size=({obs.bbox.width:.2f}x{obs.bbox.height:.2f})m")

    # Check collision risk
    is_risk, dangerous, ttc = detector.check_collision_risk(detected)
    if is_risk:
        print(f"\nCollision risk! TTC={ttc:.2f}s with obstacle {dangerous.id}")
    else:
        print("\nNo collision risk")

    # Get nearest obstacle in front
    front = detector.get_nearest_obstacle(detected, -np.pi/4, np.pi/4)
    if front:
        print(f"\nNearest obstacle in front: {front.min_distance:.2f}m")

    print("\nTest complete!")
