"""
SLAM Module - Wrapper for proven SLAM implementations

Provides unified interface for different SLAM backends:
- BreezySLAM (Python, lightweight, ARM optimized)
- Hector SLAM compatible interface
- Preparation for ROS2 SLAM Toolbox integration

Based on algorithms proven in:
- F1Tenth autonomous racing
- Industrial mobile robots
- Research projects

References:
- BreezySLAM: https://github.com/simondlevy/BreezySLAM
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- F1Tenth: https://github.com/f1tenth/particle_filter
"""

import math
import time
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
from abc import ABC, abstractmethod

# Try to import BreezySLAM
try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser
    BREEZYSLAM_AVAILABLE = True
except ImportError:
    BREEZYSLAM_AVAILABLE = False
    print("[SLAM] BreezySLAM not installed. Install with: pip install breezyslam")
    print("[SLAM] Falling back to built-in SLAM implementation")


@dataclass
class SLAMConfig:
    """SLAM configuration parameters."""
    # Map parameters
    map_size_pixels: int = 800          # Map size in pixels
    map_size_meters: float = 40.0       # Map size in meters
    map_quality: int = 50               # Map quality (1-255)

    # Scan parameters
    scan_size: int = 360                # Number of scan points
    scan_rate_hz: float = 10.0          # Scan rate
    detection_angle_degrees: float = 360.0  # Detection angle
    distance_no_detection_mm: float = 12000  # Max range in mm
    detection_margin: int = 0
    offset_mm: float = 0.0              # Sensor offset from robot center

    # Algorithm parameters
    hole_width_mm: float = 600          # Obstacle width assumption
    sigma_xy_mm: float = 50             # Position uncertainty
    sigma_theta_degrees: float = 2.0    # Orientation uncertainty
    max_search_iter: int = 1000         # Max search iterations

    # Update thresholds
    min_travel_distance: float = 0.1    # Min distance before update (m)
    min_travel_angle: float = 0.1       # Min angle before update (rad)


@dataclass
class SLAMPose:
    """Robot pose from SLAM."""
    x: float            # X position (meters)
    y: float            # Y position (meters)
    theta: float        # Heading (radians)
    timestamp: float
    confidence: float = 1.0

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)


class SLAMBase(ABC):
    """Abstract base class for SLAM implementations."""

    @abstractmethod
    def update(self, scan_distances_mm: List[float],
               odometry: Optional[Tuple[float, float, float]] = None) -> SLAMPose:
        """
        Update SLAM with new scan.

        Args:
            scan_distances_mm: List of distances in millimeters
            odometry: Optional (dxy_mm, dtheta_degrees, dt_seconds)

        Returns:
            Current pose estimate
        """
        pass

    @abstractmethod
    def get_map(self) -> np.ndarray:
        """Get current occupancy map as numpy array."""
        pass

    @abstractmethod
    def get_pose(self) -> SLAMPose:
        """Get current pose estimate."""
        pass

    @abstractmethod
    def reset(self):
        """Reset SLAM to initial state."""
        pass


class BreezySLAMWrapper(SLAMBase):
    """
    Wrapper around BreezySLAM library.

    BreezySLAM is based on CoreSLAM/tinySLAM, a proven algorithm used in
    many real-world robots. It's optimized for ARM processors (Jetson Nano).

    Reference: https://github.com/simondlevy/BreezySLAM
    """

    def __init__(self, config: Optional[SLAMConfig] = None):
        if not BREEZYSLAM_AVAILABLE:
            raise ImportError("BreezySLAM not installed")

        self.config = config or SLAMConfig()

        # Create laser sensor model
        self.laser = Laser(
            scan_size=self.config.scan_size,
            scan_rate_hz=self.config.scan_rate_hz,
            detection_angle_degrees=self.config.detection_angle_degrees,
            distance_no_detection_mm=self.config.distance_no_detection_mm,
            detection_margin=self.config.detection_margin,
            offset_mm=self.config.offset_mm
        )

        # Create SLAM object
        self.slam = RMHC_SLAM(
            laser=self.laser,
            map_size_pixels=self.config.map_size_pixels,
            map_size_meters=self.config.map_size_meters,
            map_quality=self.config.map_quality,
            hole_width_mm=self.config.hole_width_mm,
            sigma_xy_mm=self.config.sigma_xy_mm,
            sigma_theta_degrees=self.config.sigma_theta_degrees,
            max_search_iter=self.config.max_search_iter
        )

        # Map buffer
        self._mapbytes = bytearray(self.config.map_size_pixels ** 2)

        # Current pose
        self._pose = SLAMPose(0, 0, 0, time.time())

        # For tracking updates
        self._last_update_pose = (0, 0, 0)
        self._update_count = 0

    def update(self, scan_distances_mm: List[float],
               odometry: Optional[Tuple[float, float, float]] = None) -> SLAMPose:
        """Update SLAM with new scan."""
        # Convert to velocities if odometry provided
        if odometry:
            dxy_mm, dtheta_degrees, dt = odometry
            self.slam.update(
                scan_distances_mm,
                pose_change=(dxy_mm, dtheta_degrees, dt)
            )
        else:
            self.slam.update(scan_distances_mm)

        # Get pose
        x_mm, y_mm, theta_deg = self.slam.getpos()

        self._pose = SLAMPose(
            x=x_mm / 1000.0,  # Convert to meters
            y=y_mm / 1000.0,
            theta=math.radians(theta_deg),
            timestamp=time.time()
        )

        self._update_count += 1
        return self._pose

    def get_map(self) -> np.ndarray:
        """Get occupancy map as numpy array."""
        self.slam.getmap(self._mapbytes)

        # Convert to numpy array
        map_array = np.frombuffer(self._mapbytes, dtype=np.uint8)
        map_array = map_array.reshape(
            (self.config.map_size_pixels, self.config.map_size_pixels)
        )

        return map_array

    def get_pose(self) -> SLAMPose:
        """Get current pose."""
        return self._pose

    def reset(self):
        """Reset SLAM."""
        self.__init__(self.config)

    @property
    def map_resolution(self) -> float:
        """Map resolution in meters per pixel."""
        return self.config.map_size_meters / self.config.map_size_pixels


class BuiltinSLAM(SLAMBase):
    """
    Built-in SLAM implementation when BreezySLAM is not available.

    Uses ICP-based scan matching with occupancy grid mapping.
    Simpler but functional for basic use cases.
    """

    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()

        # Map
        self.map_size = self.config.map_size_pixels
        self.resolution = self.config.map_size_meters / self.config.map_size_pixels
        self._map = np.ones((self.map_size, self.map_size), dtype=np.float32) * 0.5

        # Pose
        self._pose = SLAMPose(0, 0, 0, time.time())

        # Previous scan for matching
        self._prev_scan: Optional[np.ndarray] = None
        self._prev_points: Optional[np.ndarray] = None

        # Origin (center of map in world coordinates)
        self._origin_x = 0.0
        self._origin_y = 0.0

    def update(self, scan_distances_mm: List[float],
               odometry: Optional[Tuple[float, float, float]] = None) -> SLAMPose:
        """Update SLAM with new scan."""
        # Convert scan to numpy array
        scan = np.array(scan_distances_mm, dtype=np.float32)
        n_points = len(scan)

        # Generate angles
        angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)

        # Convert to Cartesian
        valid = scan < self.config.distance_no_detection_mm
        distances_m = scan / 1000.0

        x = distances_m * np.cos(angles)
        y = distances_m * np.sin(angles)
        points = np.column_stack([x[valid], y[valid]])

        # Predict pose from odometry
        if odometry:
            dxy_mm, dtheta_deg, _ = odometry
            dxy = dxy_mm / 1000.0
            dtheta = math.radians(dtheta_deg)

            self._pose = SLAMPose(
                x=self._pose.x + dxy * math.cos(self._pose.theta + dtheta/2),
                y=self._pose.y + dxy * math.sin(self._pose.theta + dtheta/2),
                theta=self._pose.theta + dtheta,
                timestamp=time.time()
            )

        # Scan matching (ICP) if we have previous scan
        if self._prev_points is not None and len(self._prev_points) > 10 and len(points) > 10:
            dx, dy, dtheta = self._icp_match(self._prev_points, points)

            # Apply correction
            self._pose = SLAMPose(
                x=self._pose.x + dx,
                y=self._pose.y + dy,
                theta=self._pose.theta + dtheta,
                timestamp=time.time()
            )

        # Update map
        self._update_map(points)

        # Store for next iteration
        self._prev_scan = scan
        self._prev_points = points.copy()

        return self._pose

    def _icp_match(self, source: np.ndarray, target: np.ndarray,
                   max_iterations: int = 20,
                   tolerance: float = 0.001) -> Tuple[float, float, float]:
        """
        Simple ICP (Iterative Closest Point) for scan matching.

        Returns:
            (dx, dy, dtheta) transformation from source to target
        """
        # Use subset of points for speed
        if len(source) > 100:
            idx = np.random.choice(len(source), 100, replace=False)
            source = source[idx]
        if len(target) > 100:
            idx = np.random.choice(len(target), 100, replace=False)
            target = target[idx]

        # Initialize transformation
        dx, dy, dtheta = 0.0, 0.0, 0.0

        for _ in range(max_iterations):
            # Apply current transformation to source
            cos_t, sin_t = math.cos(dtheta), math.sin(dtheta)
            transformed = np.zeros_like(source)
            transformed[:, 0] = cos_t * source[:, 0] - sin_t * source[:, 1] + dx
            transformed[:, 1] = sin_t * source[:, 0] + cos_t * source[:, 1] + dy

            # Find correspondences (nearest neighbors)
            correspondences = []
            for i, p in enumerate(transformed):
                dists = np.linalg.norm(target - p, axis=1)
                j = np.argmin(dists)
                if dists[j] < 0.5:  # Max correspondence distance
                    correspondences.append((i, j))

            if len(correspondences) < 5:
                break

            # Calculate optimal transformation
            src_pts = np.array([source[i] for i, j in correspondences])
            tgt_pts = np.array([target[j] for i, j in correspondences])

            # Centroids
            src_center = np.mean(src_pts, axis=0)
            tgt_center = np.mean(tgt_pts, axis=0)

            # Center points
            src_centered = src_pts - src_center
            tgt_centered = tgt_pts - tgt_center

            # SVD for rotation
            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            # Ensure proper rotation (det = 1)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T

            # Extract angle
            new_dtheta = math.atan2(R[1, 0], R[0, 0])

            # Translation
            t = tgt_center - R @ src_center

            # Update transformation
            ddtheta = new_dtheta - dtheta
            dx += t[0]
            dy += t[1]
            dtheta = new_dtheta

            # Check convergence
            if abs(ddtheta) < tolerance and np.linalg.norm(t) < tolerance:
                break

        return dx, dy, dtheta

    def _update_map(self, points: np.ndarray):
        """Update occupancy grid with new points."""
        # Robot position in map coordinates
        robot_mx = int((self._pose.x - self._origin_x) / self.resolution + self.map_size / 2)
        robot_my = int((self._pose.y - self._origin_y) / self.resolution + self.map_size / 2)

        # Transform points to world frame
        cos_t = math.cos(self._pose.theta)
        sin_t = math.sin(self._pose.theta)

        for point in points:
            # Transform to world
            wx = self._pose.x + cos_t * point[0] - sin_t * point[1]
            wy = self._pose.y + sin_t * point[0] + cos_t * point[1]

            # Convert to map coordinates
            mx = int((wx - self._origin_x) / self.resolution + self.map_size / 2)
            my = int((wy - self._origin_y) / self.resolution + self.map_size / 2)

            # Check bounds
            if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                # Mark as occupied
                self._map[my, mx] = min(1.0, self._map[my, mx] + 0.1)

            # Ray tracing for free space (simplified)
            self._trace_ray(robot_mx, robot_my, mx, my)

    def _trace_ray(self, x0: int, y0: int, x1: int, y1: int):
        """Bresenham's line algorithm for ray tracing."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                # Mark as free (but don't override obstacles)
                if (x, y) != (x1, y1):
                    self._map[y, x] = max(0.0, self._map[y, x] - 0.05)

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def get_map(self) -> np.ndarray:
        """Get occupancy map."""
        # Convert to 0-255 range
        return (self._map * 255).astype(np.uint8)

    def get_pose(self) -> SLAMPose:
        """Get current pose."""
        return self._pose

    def reset(self):
        """Reset SLAM."""
        self.__init__(self.config)


def create_slam(config: Optional[SLAMConfig] = None,
                backend: str = "auto") -> SLAMBase:
    """
    Factory function to create SLAM instance.

    Args:
        config: SLAM configuration
        backend: "breezyslam", "builtin", or "auto"

    Returns:
        SLAM instance
    """
    if backend == "auto":
        if BREEZYSLAM_AVAILABLE:
            print("[SLAM] Using BreezySLAM backend")
            return BreezySLAMWrapper(config)
        else:
            print("[SLAM] Using built-in SLAM backend")
            return BuiltinSLAM(config)

    elif backend == "breezyslam":
        if not BREEZYSLAM_AVAILABLE:
            raise ImportError("BreezySLAM not available")
        return BreezySLAMWrapper(config)

    elif backend == "builtin":
        return BuiltinSLAM(config)

    else:
        raise ValueError(f"Unknown backend: {backend}")


# Convenience class that matches BreezySLAM API
class SLAM:
    """
    High-level SLAM interface.

    Usage:
        slam = SLAM()

        # In loop:
        slam.update(scan_mm, velocity=(dxy_mm, dtheta_deg, dt))
        pose = slam.get_pose()
        map_img = slam.get_map()
    """

    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()
        self._backend = create_slam(self.config)

    def update(self, scan_distances_mm: List[float],
               velocity: Optional[Tuple[float, float, float]] = None) -> SLAMPose:
        """
        Update SLAM with new scan.

        Args:
            scan_distances_mm: Distances in mm (clockwise from front)
            velocity: (dxy_mm, dtheta_degrees, dt_seconds)
        """
        return self._backend.update(scan_distances_mm, velocity)

    def get_pose(self) -> SLAMPose:
        """Get current pose."""
        return self._backend.get_pose()

    def get_position(self) -> Tuple[float, float]:
        """Get current position (x, y) in meters."""
        pose = self.get_pose()
        return (pose.x, pose.y)

    def get_heading(self) -> float:
        """Get current heading in radians."""
        return self.get_pose().theta

    def get_map(self) -> np.ndarray:
        """Get occupancy map (0=free, 127=unknown, 255=occupied)."""
        return self._backend.get_map()

    def get_map_as_image(self) -> np.ndarray:
        """Get map as RGB image for visualization."""
        occupancy = self.get_map()

        # Create RGB image
        rgb = np.zeros((*occupancy.shape, 3), dtype=np.uint8)

        # Free space = white
        free_mask = occupancy < 100
        rgb[free_mask] = [255, 255, 255]

        # Unknown = gray
        unknown_mask = (occupancy >= 100) & (occupancy < 150)
        rgb[unknown_mask] = [128, 128, 128]

        # Occupied = black
        occupied_mask = occupancy >= 150
        rgb[occupied_mask] = [0, 0, 0]

        return rgb

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map pixel coordinates."""
        resolution = self.config.map_size_meters / self.config.map_size_pixels
        mx = int(x / resolution + self.config.map_size_pixels / 2)
        my = int(y / resolution + self.config.map_size_pixels / 2)
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map pixel coordinates to world coordinates."""
        resolution = self.config.map_size_meters / self.config.map_size_pixels
        x = (mx - self.config.map_size_pixels / 2) * resolution
        y = (my - self.config.map_size_pixels / 2) * resolution
        return x, y

    def save_map(self, filepath: str):
        """Save map to file."""
        import cv2
        map_img = self.get_map()
        cv2.imwrite(filepath, map_img)

    def reset(self):
        """Reset SLAM."""
        self._backend.reset()


if __name__ == '__main__':
    import sys
    sys.path.insert(0, '/home/user/Robocar/src')

    from perception.lidar_processor import create_simulated_scan

    print("Testing SLAM module...")

    # Create SLAM
    slam = SLAM()

    # Simulate robot movement
    print("\nSimulating mapping run...")

    robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0

    for step in range(50):
        # Move robot
        robot_x += 0.1 * math.cos(robot_theta)
        robot_y += 0.1 * math.sin(robot_theta)
        robot_theta += 0.05

        # Generate simulated scan
        obstacles = [(3.0, 1.0, 0.3), (2.0, -1.0, 0.4)]
        walls = [((5, -3), (5, 3)), ((-5, -3), (-5, 3))]
        scan = create_simulated_scan(obstacles, walls)

        # Convert to mm
        scan_mm = [int(p.distance * 1000) for p in scan.points]

        # Update SLAM
        velocity = (100, 3, 0.1)  # 100mm, 3 degrees, 0.1s
        pose = slam.update(scan_mm, velocity)

        if step % 10 == 0:
            print(f"Step {step}: pose=({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.theta):.1f}°)")

    # Get final map
    map_img = slam.get_map()
    print(f"\nFinal map size: {map_img.shape}")
    print(f"Map coverage: {np.sum(map_img != 127) / map_img.size * 100:.1f}%")

    print("\nTest complete!")