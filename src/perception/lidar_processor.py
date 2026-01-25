"""
LiDAR Data Processor

Processes raw LiDAR scans to filter noise, detect features,
and prepare data for obstacle detection and SLAM.

Features:
- Median filtering to remove outliers
- Range-based filtering
- Intensity-based filtering
- Scan segmentation
- Feature extraction (lines, corners)
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
import numpy as np

# Import from drivers if available, otherwise define locally
try:
    from drivers.lidar_ld19 import LidarScan, LidarPoint
except ImportError:
    @dataclass
    class LidarPoint:
        angle: float
        distance: float
        intensity: int
        valid: bool

    @dataclass
    class LidarScan:
        timestamp: float
        points: List[LidarPoint]
        scan_frequency: float


@dataclass
class ProcessedScan:
    """Processed LiDAR scan with filtered points."""
    timestamp: float
    points: np.ndarray          # Nx2 array of (x, y) in robot frame
    angles: np.ndarray          # N array of angles
    distances: np.ndarray       # N array of distances
    intensities: np.ndarray     # N array of intensities
    valid_mask: np.ndarray      # N boolean array


@dataclass
class ScanSegment:
    """A segment of consecutive points (potential obstacle or wall)."""
    start_idx: int
    end_idx: int
    points: np.ndarray          # Mx2 array of (x, y)
    centroid: Tuple[float, float]
    length: float               # Arc length of segment
    is_line: bool               # True if segment is linear
    line_params: Optional[Tuple[float, float, float]]  # (a, b, c) for ax + by + c = 0


class LidarProcessor:
    """
    Processes raw LiDAR scans for navigation.

    Usage:
        processor = LidarProcessor()

        # Process a scan
        processed = processor.process(raw_scan)

        # Get obstacles
        segments = processor.segment_scan(processed)

        # Find nearest obstacle
        min_dist, min_angle = processor.find_nearest_obstacle(processed)
    """

    def __init__(
        self,
        min_range: float = 0.05,
        max_range: float = 10.0,
        min_intensity: int = 10,
        median_filter_size: int = 3,
        angle_offset: float = 0.0,
        segmentation_threshold: float = 0.3
    ):
        """
        Initialize LiDAR processor.

        Args:
            min_range: Minimum valid range (meters)
            max_range: Maximum valid range (meters)
            min_intensity: Minimum valid intensity
            median_filter_size: Size of median filter window (odd number)
            angle_offset: Rotation offset to align with robot frame (radians)
            segmentation_threshold: Max distance between points in same segment (meters)
        """
        self.min_range = min_range
        self.max_range = max_range
        self.min_intensity = min_intensity
        self.median_filter_size = median_filter_size
        self.angle_offset = angle_offset
        self.segmentation_threshold = segmentation_threshold

    def process(self, scan: LidarScan) -> ProcessedScan:
        """
        Process a raw LiDAR scan.

        Args:
            scan: Raw LidarScan from driver

        Returns:
            ProcessedScan with filtered and transformed points
        """
        n_points = len(scan.points)

        # Extract arrays
        angles = np.array([p.angle for p in scan.points])
        distances = np.array([p.distance for p in scan.points])
        intensities = np.array([p.intensity for p in scan.points])

        # Apply angle offset
        angles = (angles + self.angle_offset) % (2 * np.pi)

        # Create validity mask
        valid_mask = np.ones(n_points, dtype=bool)

        # Range filtering
        valid_mask &= (distances >= self.min_range)
        valid_mask &= (distances <= self.max_range)

        # Intensity filtering
        valid_mask &= (intensities >= self.min_intensity)

        # Apply median filter to distances (reduces noise)
        if self.median_filter_size > 1:
            distances = self._median_filter(distances, self.median_filter_size)

        # Convert to Cartesian coordinates
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        points = np.column_stack([x, y])

        return ProcessedScan(
            timestamp=scan.timestamp,
            points=points,
            angles=angles,
            distances=distances,
            intensities=intensities,
            valid_mask=valid_mask
        )

    def _median_filter(self, data: np.ndarray, size: int) -> np.ndarray:
        """Apply circular median filter."""
        result = np.copy(data)
        half = size // 2
        n = len(data)

        for i in range(n):
            # Get window indices (circular)
            indices = [(i + j - half) % n for j in range(size)]
            window = data[indices]
            result[i] = np.median(window)

        return result

    def get_valid_points(self, scan: ProcessedScan) -> np.ndarray:
        """Get only valid points from processed scan."""
        return scan.points[scan.valid_mask]

    def get_valid_polar(self, scan: ProcessedScan) -> Tuple[np.ndarray, np.ndarray]:
        """Get valid points in polar coordinates."""
        mask = scan.valid_mask
        return scan.angles[mask], scan.distances[mask]

    def find_nearest_obstacle(
        self,
        scan: ProcessedScan,
        angle_min: float = -np.pi,
        angle_max: float = np.pi
    ) -> Tuple[float, float]:
        """
        Find the nearest obstacle in a scan.

        Args:
            scan: Processed scan
            angle_min: Minimum angle to consider (radians)
            angle_max: Maximum angle to consider (radians)

        Returns:
            Tuple of (distance, angle) to nearest obstacle
        """
        # Create angle mask
        angles = scan.angles
        if angle_min < angle_max:
            angle_mask = (angles >= angle_min) & (angles <= angle_max)
        else:
            # Handle wraparound
            angle_mask = (angles >= angle_min) | (angles <= angle_max)

        # Combine with validity mask
        mask = scan.valid_mask & angle_mask

        if not np.any(mask):
            return float('inf'), 0.0

        # Find minimum distance
        valid_distances = scan.distances[mask]
        valid_angles = scan.angles[mask]

        min_idx = np.argmin(valid_distances)
        return valid_distances[min_idx], valid_angles[min_idx]

    def find_obstacles_in_zone(
        self,
        scan: ProcessedScan,
        angle_min: float,
        angle_max: float,
        max_distance: float
    ) -> List[Tuple[float, float]]:
        """
        Find all obstacles within a zone.

        Args:
            scan: Processed scan
            angle_min: Start angle (radians)
            angle_max: End angle (radians)
            max_distance: Maximum distance to consider

        Returns:
            List of (x, y) positions of obstacles
        """
        angles = scan.angles
        distances = scan.distances

        # Angle mask
        if angle_min < angle_max:
            angle_mask = (angles >= angle_min) & (angles <= angle_max)
        else:
            angle_mask = (angles >= angle_min) | (angles <= angle_max)

        # Distance mask
        dist_mask = distances <= max_distance

        # Combine masks
        mask = scan.valid_mask & angle_mask & dist_mask

        return [(scan.points[i, 0], scan.points[i, 1])
                for i in range(len(mask)) if mask[i]]

    def segment_scan(self, scan: ProcessedScan) -> List[ScanSegment]:
        """
        Segment scan into groups of consecutive points.

        Points are grouped if they are close enough to each other.
        This helps identify individual obstacles and walls.

        Args:
            scan: Processed scan

        Returns:
            List of ScanSegment objects
        """
        valid_points = self.get_valid_points(scan)
        valid_indices = np.where(scan.valid_mask)[0]

        if len(valid_points) < 2:
            return []

        segments = []
        current_segment_start = 0

        for i in range(1, len(valid_points)):
            # Calculate distance between consecutive points
            dist = np.linalg.norm(valid_points[i] - valid_points[i-1])

            if dist > self.segmentation_threshold:
                # End current segment, start new one
                if i - current_segment_start >= 2:
                    segment = self._create_segment(
                        valid_points[current_segment_start:i],
                        valid_indices[current_segment_start],
                        valid_indices[i-1]
                    )
                    segments.append(segment)
                current_segment_start = i

        # Don't forget last segment
        if len(valid_points) - current_segment_start >= 2:
            segment = self._create_segment(
                valid_points[current_segment_start:],
                valid_indices[current_segment_start],
                valid_indices[-1]
            )
            segments.append(segment)

        return segments

    def _create_segment(
        self,
        points: np.ndarray,
        start_idx: int,
        end_idx: int
    ) -> ScanSegment:
        """Create a ScanSegment from points."""
        centroid = (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))

        # Calculate arc length
        diffs = np.diff(points, axis=0)
        length = float(np.sum(np.linalg.norm(diffs, axis=1)))

        # Fit line and check linearity
        is_line, line_params = self._fit_line(points)

        return ScanSegment(
            start_idx=start_idx,
            end_idx=end_idx,
            points=points,
            centroid=centroid,
            length=length,
            is_line=is_line,
            line_params=line_params
        )

    def _fit_line(
        self,
        points: np.ndarray,
        threshold: float = 0.05
    ) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        """
        Fit a line to points using least squares.

        Returns:
            Tuple of (is_line, line_params) where line_params is (a, b, c)
            for the line equation ax + by + c = 0
        """
        if len(points) < 2:
            return False, None

        # Use PCA to find principal direction
        centroid = np.mean(points, axis=0)
        centered = points - centroid

        # Covariance matrix
        cov = np.cov(centered.T)

        if cov.ndim < 2:
            return False, None

        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # Principal direction is eigenvector with largest eigenvalue
        principal_idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, principal_idx]

        # Line normal
        normal = np.array([-direction[1], direction[0]])

        # Line equation: normal . (p - centroid) = 0
        # a*x + b*y + c = 0
        a, b = normal
        c = -np.dot(normal, centroid)

        # Check if points are close to line
        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c)
        max_distance = np.max(distances)

        is_line = max_distance < threshold

        return is_line, (float(a), float(b), float(c))

    def extract_lines(
        self,
        scan: ProcessedScan,
        min_points: int = 5,
        max_error: float = 0.05
    ) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Extract line segments from scan using RANSAC-like approach.

        Args:
            scan: Processed scan
            min_points: Minimum points for a valid line
            max_error: Maximum distance from line for inliers

        Returns:
            List of (start_point, end_point) tuples
        """
        segments = self.segment_scan(scan)
        lines = []

        for segment in segments:
            if len(segment.points) >= min_points and segment.is_line:
                # Project points onto the fitted line
                a, b, c = segment.line_params

                # Find endpoints by projecting first and last points
                p1 = segment.points[0]
                p2 = segment.points[-1]

                # Project onto line
                def project_point(p):
                    t = -(a * p[0] + b * p[1] + c) / (a*a + b*b)
                    return np.array([p[0] + a*t, p[1] + b*t])

                start = project_point(p1)
                end = project_point(p2)

                lines.append((start, end))

        return lines


def create_simulated_scan(
    obstacles: List[Tuple[float, float, float]],  # (x, y, radius)
    walls: List[Tuple[Tuple[float, float], Tuple[float, float]]],  # ((x1,y1), (x2,y2))
    num_rays: int = 360,
    max_range: float = 10.0,
    noise_std: float = 0.02
) -> LidarScan:
    """
    Create a simulated LiDAR scan for testing.

    Args:
        obstacles: List of circular obstacles (x, y, radius)
        walls: List of wall segments ((x1,y1), (x2,y2))
        num_rays: Number of rays in scan
        max_range: Maximum range
        noise_std: Standard deviation of range noise

    Returns:
        Simulated LidarScan
    """
    import time

    points = []

    for i in range(num_rays):
        angle = 2 * np.pi * i / num_rays

        # Ray direction
        dx = np.cos(angle)
        dy = np.sin(angle)

        min_dist = max_range

        # Check obstacles (circles)
        for ox, oy, radius in obstacles:
            # Ray-circle intersection
            # Ray: p = t * (dx, dy)
            # Circle: |p - (ox, oy)|² = radius²

            a = dx*dx + dy*dy
            b = -2 * (ox*dx + oy*dy)
            c = ox*ox + oy*oy - radius*radius

            discriminant = b*b - 4*a*c

            if discriminant >= 0:
                t1 = (-b - np.sqrt(discriminant)) / (2*a)
                t2 = (-b + np.sqrt(discriminant)) / (2*a)

                for t in [t1, t2]:
                    if 0 < t < min_dist:
                        min_dist = t

        # Check walls (line segments)
        for (x1, y1), (x2, y2) in walls:
            # Ray-segment intersection
            # Ray: p = t * (dx, dy)
            # Segment: p = (x1, y1) + s * (x2-x1, y2-y1), s in [0, 1]

            wx = x2 - x1
            wy = y2 - y1

            denom = dx * wy - dy * wx

            if abs(denom) > 1e-10:
                t = (x1 * wy - y1 * wx + (y1 - 0) * wx - (x1 - 0) * wy) / denom
                t = (wx * (0 - y1) - wy * (0 - x1)) / (-denom)
                s = (dx * (0 - y1) - dy * (0 - x1)) / (-denom)

                # Recalculate properly
                t = ((x1) * (-wy) - (y1) * (-wx)) / denom
                s = ((x1) * (-dy) - (y1) * (-dx)) / denom

                # Proper calculation
                denom = (-wx) * dy - (-wy) * dx
                if abs(denom) > 1e-10:
                    t = ((-wx) * (-y1) - (-wy) * (-x1)) / denom
                    s = (dx * (-y1) - dy * (-x1)) / denom

                    if 0 <= s <= 1 and t > 0 and t < min_dist:
                        min_dist = t

        # Add noise
        if min_dist < max_range:
            min_dist += np.random.normal(0, noise_std)
            min_dist = max(0.02, min_dist)

        point = LidarPoint(
            angle=angle,
            distance=min_dist,
            intensity=200 if min_dist < max_range else 0,
            valid=min_dist < max_range
        )
        points.append(point)

    return LidarScan(
        timestamp=time.time(),
        points=points,
        scan_frequency=10.0
    )


if __name__ == '__main__':
    # Test with simulated data
    print("Testing LiDAR Processor with simulated data...")

    # Create simulated environment
    obstacles = [
        (2.0, 1.0, 0.3),   # Obstacle at (2, 1) with radius 0.3
        (-1.5, 2.0, 0.5),  # Obstacle at (-1.5, 2) with radius 0.5
    ]

    walls = [
        ((3.0, -2.0), (3.0, 3.0)),   # Wall on the right
        ((-3.0, -2.0), (-3.0, 3.0)), # Wall on the left
    ]

    # Generate scan
    scan = create_simulated_scan(obstacles, walls)
    print(f"Generated scan with {len(scan.points)} points")

    # Process
    processor = LidarProcessor()
    processed = processor.process(scan)

    valid_count = np.sum(processed.valid_mask)
    print(f"Valid points after processing: {valid_count}")

    # Find nearest obstacle
    dist, angle = processor.find_nearest_obstacle(processed)
    print(f"Nearest obstacle: {dist:.2f}m at {np.degrees(angle):.1f}°")

    # Segment scan
    segments = processor.segment_scan(processed)
    print(f"Found {len(segments)} segments")

    for i, seg in enumerate(segments):
        print(f"  Segment {i+1}: {len(seg.points)} points, "
              f"length={seg.length:.2f}m, is_line={seg.is_line}")

    print("\nTest complete!")
