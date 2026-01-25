"""
Particle Filter Localization

Implementation based on the F1Tenth particle filter algorithm.
This is a Monte Carlo Localization (MCL) implementation optimized
for autonomous vehicles with LiDAR sensors.

References:
- F1Tenth Particle Filter: https://github.com/f1tenth/particle_filter
- Probabilistic Robotics (Thrun, Burgard, Fox)
- "A Computationally Efficient Method for Particle Filter Localization" - Walsh et al.

Features:
- Sensor model with ray casting
- Motion model with noise
- Adaptive resampling
- GPU acceleration ready (numpy-based)
"""

import math
import time
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
import threading


@dataclass
class ParticleFilterConfig:
    """Configuration for particle filter."""
    # Number of particles
    num_particles: int = 500

    # Motion model noise (standard deviations)
    motion_noise_x: float = 0.05        # meters
    motion_noise_y: float = 0.05        # meters
    motion_noise_theta: float = 0.05    # radians

    # Sensor model parameters
    z_hit: float = 0.7                  # Probability of correct reading
    z_short: float = 0.1                # Probability of short reading
    z_max: float = 0.1                  # Probability of max reading
    z_rand: float = 0.1                 # Probability of random reading
    sigma_hit: float = 0.2              # Std dev of hit measurement (meters)
    lambda_short: float = 0.5           # Exponential decay for short readings

    # Sensor parameters
    max_range: float = 10.0             # Maximum sensor range (meters)
    num_beams_skip: int = 10            # Use every Nth beam for speed

    # Resampling
    resample_threshold: float = 0.5     # Effective particle ratio threshold

    # Map parameters
    map_resolution: float = 0.05        # meters per pixel


@dataclass
class Particle:
    """Single particle representing possible robot pose."""
    x: float
    y: float
    theta: float
    weight: float = 1.0


class OccupancyMap:
    """
    Occupancy grid map for particle filter.

    Supports ray casting for sensor model.
    """

    def __init__(self, map_data: np.ndarray, resolution: float,
                 origin_x: float = 0.0, origin_y: float = 0.0):
        """
        Initialize occupancy map.

        Args:
            map_data: 2D numpy array (0=free, 255=occupied)
            resolution: meters per pixel
            origin_x, origin_y: world coordinates of map origin
        """
        self.data = map_data.astype(np.float32) / 255.0
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.height, self.width = map_data.shape

        # Precompute for ray casting
        self._occupied_threshold = 0.5

        # Distance transform for faster ray casting (optional)
        self._precompute_distances()

    def _precompute_distances(self):
        """Precompute distance to nearest obstacle for each cell."""
        try:
            from scipy import ndimage
            # Binary map (1 = free, 0 = occupied)
            binary = (self.data < self._occupied_threshold).astype(np.float32)
            self.distance_map = ndimage.distance_transform_edt(binary) * self.resolution
        except ImportError:
            self.distance_map = None

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map indices."""
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map indices to world coordinates."""
        x = mx * self.resolution + self.origin_x
        y = my * self.resolution + self.origin_y
        return x, y

    def is_free(self, x: float, y: float) -> bool:
        """Check if world position is free."""
        mx, my = self.world_to_map(x, y)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return self.data[my, mx] < self._occupied_threshold
        return False

    def ray_cast(self, x: float, y: float, theta: float,
                 max_range: float) -> float:
        """
        Cast ray from position in direction theta.

        Returns distance to first obstacle or max_range.
        """
        # Use distance map if available (faster)
        if self.distance_map is not None:
            return self._ray_cast_fast(x, y, theta, max_range)

        # Bresenham-style ray casting
        return self._ray_cast_bresenham(x, y, theta, max_range)

    def _ray_cast_fast(self, x: float, y: float, theta: float,
                       max_range: float) -> float:
        """Fast ray casting using distance map."""
        step_size = self.resolution * 0.5
        dist = 0.0

        while dist < max_range:
            # Current position
            px = x + dist * math.cos(theta)
            py = y + dist * math.sin(theta)

            mx, my = self.world_to_map(px, py)

            if not (0 <= mx < self.width and 0 <= my < self.height):
                return max_range

            # Get distance to nearest obstacle
            local_dist = self.distance_map[my, mx]

            if local_dist < step_size:
                return dist

            # Jump by the distance to nearest obstacle
            dist += max(step_size, local_dist * 0.9)

        return max_range

    def _ray_cast_bresenham(self, x: float, y: float, theta: float,
                            max_range: float) -> float:
        """Standard ray casting using Bresenham's algorithm."""
        mx, my = self.world_to_map(x, y)
        ex = x + max_range * math.cos(theta)
        ey = y + max_range * math.sin(theta)
        emx, emy = self.world_to_map(ex, ey)

        dx = abs(emx - mx)
        dy = abs(emy - my)
        sx = 1 if mx < emx else -1
        sy = 1 if my < emy else -1
        err = dx - dy

        cx, cy = mx, my
        steps = 0
        max_steps = int(max_range / self.resolution) + 1

        while steps < max_steps:
            if not (0 <= cx < self.width and 0 <= cy < self.height):
                break

            if self.data[cy, cx] >= self._occupied_threshold:
                # Hit obstacle
                wx, wy = self.map_to_world(cx, cy)
                return math.sqrt((wx - x)**2 + (wy - y)**2)

            if cx == emx and cy == emy:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

            steps += 1

        return max_range

    def ray_cast_batch(self, x: float, y: float, theta: float,
                       angles: np.ndarray, max_range: float) -> np.ndarray:
        """Cast multiple rays efficiently."""
        distances = np.zeros(len(angles))
        for i, angle in enumerate(angles):
            distances[i] = self.ray_cast(x, y, theta + angle, max_range)
        return distances


class ParticleFilter:
    """
    Particle filter for robot localization.

    Based on F1Tenth implementation with optimizations for
    real-time performance on Jetson Nano.

    Usage:
        # Initialize with map
        pf = ParticleFilter(map_data, resolution=0.05)

        # Set initial pose estimate
        pf.initialize(x=0, y=0, theta=0, spread=0.5)

        # In main loop:
        pf.motion_update(dx, dy, dtheta)
        pf.sensor_update(scan_ranges, scan_angles)

        # Get pose estimate
        x, y, theta = pf.get_pose()
    """

    def __init__(self, map_data: np.ndarray,
                 resolution: float = 0.05,
                 config: Optional[ParticleFilterConfig] = None,
                 origin_x: float = 0.0,
                 origin_y: float = 0.0):
        """
        Initialize particle filter.

        Args:
            map_data: Occupancy grid (0=free, 255=occupied)
            resolution: Map resolution (meters/pixel)
            config: Configuration parameters
            origin_x, origin_y: World coordinates of map origin
        """
        self.config = config or ParticleFilterConfig()
        self.map = OccupancyMap(map_data, resolution, origin_x, origin_y)

        # Particles
        self.particles: np.ndarray = None  # Nx3 array [x, y, theta]
        self.weights: np.ndarray = None    # N array

        # State
        self._initialized = False
        self._lock = threading.Lock()

        # Precompute
        self._log_z_hit = math.log(self.config.z_hit + 1e-10)
        self._log_z_rand = math.log(self.config.z_rand / self.config.max_range + 1e-10)

    def initialize(self, x: float, y: float, theta: float,
                   spread_xy: float = 0.5, spread_theta: float = 0.3):
        """
        Initialize particles around a pose estimate.

        Args:
            x, y: Initial position estimate
            theta: Initial heading estimate
            spread_xy: Position spread (meters)
            spread_theta: Heading spread (radians)
        """
        n = self.config.num_particles

        with self._lock:
            # Sample particles
            self.particles = np.zeros((n, 3))
            self.particles[:, 0] = np.random.normal(x, spread_xy, n)
            self.particles[:, 1] = np.random.normal(y, spread_xy, n)
            self.particles[:, 2] = np.random.normal(theta, spread_theta, n)

            # Normalize angles
            self.particles[:, 2] = np.mod(self.particles[:, 2] + np.pi, 2*np.pi) - np.pi

            # Filter out particles in obstacles
            valid = np.array([self.map.is_free(p[0], p[1]) for p in self.particles])
            if np.sum(valid) < n // 2:
                # Not enough valid particles, skip filtering
                pass
            else:
                # Replace invalid particles with valid ones
                valid_particles = self.particles[valid]
                if len(valid_particles) > 0:
                    replace_idx = np.where(~valid)[0]
                    for idx in replace_idx:
                        src_idx = np.random.randint(len(valid_particles))
                        self.particles[idx] = valid_particles[src_idx]
                        self.particles[idx, 0] += np.random.normal(0, spread_xy/2)
                        self.particles[idx, 1] += np.random.normal(0, spread_xy/2)

            # Uniform weights
            self.weights = np.ones(n) / n

            self._initialized = True

    def motion_update(self, dx: float, dy: float, dtheta: float):
        """
        Update particles with motion model.

        Args:
            dx: Change in x (robot frame)
            dy: Change in y (robot frame)
            dtheta: Change in heading
        """
        if not self._initialized:
            return

        with self._lock:
            n = len(self.particles)

            # Add noise to motion
            noisy_dx = dx + np.random.normal(0, self.config.motion_noise_x, n)
            noisy_dy = dy + np.random.normal(0, self.config.motion_noise_y, n)
            noisy_dtheta = dtheta + np.random.normal(0, self.config.motion_noise_theta, n)

            # Apply motion (in robot frame)
            cos_theta = np.cos(self.particles[:, 2])
            sin_theta = np.sin(self.particles[:, 2])

            self.particles[:, 0] += noisy_dx * cos_theta - noisy_dy * sin_theta
            self.particles[:, 1] += noisy_dx * sin_theta + noisy_dy * cos_theta
            self.particles[:, 2] += noisy_dtheta

            # Normalize angles
            self.particles[:, 2] = np.mod(self.particles[:, 2] + np.pi, 2*np.pi) - np.pi

    def sensor_update(self, scan_ranges: np.ndarray, scan_angles: np.ndarray):
        """
        Update particle weights based on sensor readings.

        Args:
            scan_ranges: Array of range measurements (meters)
            scan_angles: Array of corresponding angles (radians, relative to robot)
        """
        if not self._initialized:
            return

        with self._lock:
            # Downsample scan for speed
            skip = self.config.num_beams_skip
            ranges = scan_ranges[::skip]
            angles = scan_angles[::skip]

            # Filter invalid readings
            valid = (ranges > 0.05) & (ranges < self.config.max_range * 0.99)
            ranges = ranges[valid]
            angles = angles[valid]

            if len(ranges) < 5:
                return

            # Compute weights for each particle
            log_weights = np.zeros(len(self.particles))

            for i, particle in enumerate(self.particles):
                # Get expected ranges for this particle
                expected = self.map.ray_cast_batch(
                    particle[0], particle[1], particle[2],
                    angles, self.config.max_range
                )

                # Compute log likelihood
                log_weights[i] = self._compute_log_likelihood(ranges, expected)

            # Convert to weights and normalize
            # Use log-sum-exp trick for numerical stability
            max_log_weight = np.max(log_weights)
            weights = np.exp(log_weights - max_log_weight)

            # Normalize
            weight_sum = np.sum(weights)
            if weight_sum > 0:
                self.weights = weights / weight_sum
            else:
                self.weights = np.ones(len(self.particles)) / len(self.particles)

            # Resample if needed
            self._resample_if_needed()

    def _compute_log_likelihood(self, measured: np.ndarray,
                                expected: np.ndarray) -> float:
        """Compute log likelihood of measurement given expected."""
        # Beam model
        log_prob = 0.0

        for z, z_exp in zip(measured, expected):
            # Hit probability (Gaussian)
            diff = z - z_exp
            p_hit = math.exp(-diff**2 / (2 * self.config.sigma_hit**2))

            # Random probability (uniform)
            p_rand = 1.0 / self.config.max_range

            # Short probability (exponential)
            if z < z_exp:
                p_short = self.config.lambda_short * math.exp(-self.config.lambda_short * z)
            else:
                p_short = 0.0

            # Max probability
            p_max = 1.0 if z >= self.config.max_range * 0.99 else 0.0

            # Combined probability
            p = (self.config.z_hit * p_hit +
                 self.config.z_rand * p_rand +
                 self.config.z_short * p_short +
                 self.config.z_max * p_max)

            log_prob += math.log(max(p, 1e-10))

        return log_prob

    def _resample_if_needed(self):
        """Resample particles if effective particle count is low."""
        # Compute effective number of particles
        n_eff = 1.0 / np.sum(self.weights ** 2)
        n = len(self.particles)

        if n_eff < self.config.resample_threshold * n:
            self._resample()

    def _resample(self):
        """Low variance resampling."""
        n = len(self.particles)

        # Cumulative weights
        cumsum = np.cumsum(self.weights)

        # Random start
        r = np.random.uniform(0, 1.0 / n)

        # Systematic resampling
        new_particles = np.zeros_like(self.particles)
        j = 0

        for i in range(n):
            u = r + i / n
            while u > cumsum[j]:
                j += 1
                if j >= n:
                    j = n - 1
                    break
            new_particles[i] = self.particles[j]

        # Add small noise to prevent particle depletion
        noise = np.random.normal(0, 0.01, new_particles.shape)
        new_particles += noise

        self.particles = new_particles
        self.weights = np.ones(n) / n

    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get estimated pose.

        Returns:
            (x, y, theta) weighted mean of particles
        """
        if not self._initialized:
            return 0.0, 0.0, 0.0

        with self._lock:
            # Weighted mean for x, y
            x = np.sum(self.weights * self.particles[:, 0])
            y = np.sum(self.weights * self.particles[:, 1])

            # Circular mean for angle
            sin_sum = np.sum(self.weights * np.sin(self.particles[:, 2]))
            cos_sum = np.sum(self.weights * np.cos(self.particles[:, 2]))
            theta = math.atan2(sin_sum, cos_sum)

            return x, y, theta

    def get_particles(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get particles and weights for visualization."""
        with self._lock:
            return self.particles.copy(), self.weights.copy()

    def get_covariance(self) -> np.ndarray:
        """Get pose covariance matrix."""
        if not self._initialized:
            return np.eye(3)

        with self._lock:
            # Weighted covariance
            mean = np.array([
                np.sum(self.weights * self.particles[:, 0]),
                np.sum(self.weights * self.particles[:, 1]),
                math.atan2(
                    np.sum(self.weights * np.sin(self.particles[:, 2])),
                    np.sum(self.weights * np.cos(self.particles[:, 2]))
                )
            ])

            centered = self.particles - mean
            centered[:, 2] = np.mod(centered[:, 2] + np.pi, 2*np.pi) - np.pi

            cov = np.zeros((3, 3))
            for i in range(len(self.particles)):
                outer = np.outer(centered[i], centered[i])
                cov += self.weights[i] * outer

            return cov

    @property
    def is_converged(self) -> bool:
        """Check if filter has converged."""
        if not self._initialized:
            return False

        cov = self.get_covariance()
        return cov[0, 0] < 0.1 and cov[1, 1] < 0.1 and cov[2, 2] < 0.1


if __name__ == '__main__':
    print("Testing Particle Filter...")

    # Create a simple test map
    map_size = 200
    resolution = 0.1  # 0.1m per pixel = 20m x 20m map

    # Empty map with walls
    map_data = np.zeros((map_size, map_size), dtype=np.uint8)

    # Add walls
    map_data[0, :] = 255      # Top
    map_data[-1, :] = 255     # Bottom
    map_data[:, 0] = 255      # Left
    map_data[:, -1] = 255     # Right

    # Add obstacles
    map_data[80:90, 80:90] = 255  # Square obstacle
    map_data[120:130, 50:60] = 255  # Another obstacle

    # Create particle filter
    config = ParticleFilterConfig(num_particles=200)
    pf = ParticleFilter(
        map_data, resolution=resolution, config=config,
        origin_x=-10, origin_y=-10  # Center map at origin
    )

    # Initialize at known position
    pf.initialize(x=0, y=0, theta=0, spread_xy=0.5, spread_theta=0.2)
    print(f"Initial pose: {pf.get_pose()}")

    # Simulate motion and sensing
    for step in range(20):
        # Motion update (move forward)
        pf.motion_update(dx=0.1, dy=0, dtheta=0.05)

        # Simulate sensor readings
        true_x, true_y, true_theta = step * 0.1, 0, step * 0.05
        scan_angles = np.linspace(-np.pi/2, np.pi/2, 36)
        scan_ranges = np.array([
            pf.map.ray_cast(true_x, true_y, true_theta + a, 10.0)
            for a in scan_angles
        ])
        scan_ranges += np.random.normal(0, 0.05, len(scan_ranges))

        # Sensor update
        pf.sensor_update(scan_ranges, scan_angles)

        if step % 5 == 0:
            x, y, theta = pf.get_pose()
            print(f"Step {step}: pose=({x:.2f}, {y:.2f}, {np.degrees(theta):.1f}°), "
                  f"converged={pf.is_converged}")

    print("\nTest complete!")