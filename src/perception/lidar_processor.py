"""
LiDAR Data Processor
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
import numpy as np

try:
    from lidar_processor import LidarScan, LidarPoint
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
        self.min_range = min_range
        self.max_range = max_range
        self.min_intensity = min_intensity
        self.median_filter_size = median_filter_size
        self.angle_offset = angle_offset
        self.segmentation_threshold = segmentation_threshold

    def process(self, scan: LidarScan) -> ProcessedScan:
        """
        Process a raw LiDAR scan.
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

        # --- FILTRAGE DISTANCE (CORRECTION CHASSIS) ---
        # On ignore tout ce qui est a moins de 40cm (0.40m)
        # Cela evite que la voiture detecte son propre nez et recule par peur.
        valid_mask &= (distances >= 0.15)

        # On ignore ce qui est trop loin
        valid_mask &= (distances <= self.max_range)
        # ---------------------------------------------

        # Intensity filtering
        valid_mask &= (intensities >= self.min_intensity)

        # --- FILTRAGE CONE (VISION TUNNEL) ---
        # On ne garde que ce qui est DEVANT dans un angle donné.
        angle_limit_deg = 45.0   # Angle de vue de +/- 45 degres
        angle_limit_rad = np.radians(angle_limit_deg)
        
        # On garde [0, limit] (Gauche) ET [2pi-limit, 2pi] (Droite)
        cone_mask = (angles <= angle_limit_rad) | (angles >= (2 * np.pi - angle_limit_rad))
        valid_mask &= cone_mask
        # -------------------------------------

        # Apply median filter
        if self.median_filter_size > 1:
            distances = self._median_filter(distances, self.median_filter_size)

        # Convert to Cartesian
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
        result = np.copy(data)
        half = size // 2
        n = len(data)
        for i in range(n):
            indices = [(i + j - half) % n for j in range(size)]
            window = data[indices]
            result[i] = np.median(window)
        return result

    def get_valid_points(self, scan: ProcessedScan) -> np.ndarray:
        return scan.points[scan.valid_mask]

    def get_valid_polar(self, scan: ProcessedScan) -> Tuple[np.ndarray, np.ndarray]:
        mask = scan.valid_mask
        return scan.angles[mask], scan.distances[mask]

    def find_nearest_obstacle(
        self,
        scan: ProcessedScan,
        angle_min: float = -np.pi,
        angle_max: float = np.pi
    ) -> Tuple[float, float]:
        angles = scan.angles
        if angle_min < angle_max:
            angle_mask = (angles >= angle_min) & (angles <= angle_max)
        else:
            angle_mask = (angles >= angle_min) | (angles <= angle_max)

        mask = scan.valid_mask & angle_mask

        if not np.any(mask):
            return float('inf'), 0.0

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
        angles = scan.angles
        distances = scan.distances

        if angle_min < angle_max:
            angle_mask = (angles >= angle_min) & (angles <= angle_max)
        else:
            angle_mask = (angles >= angle_min) | (angles <= angle_max)

        dist_mask = distances <= max_distance
        mask = scan.valid_mask & angle_mask & dist_mask

        return [(scan.points[i, 0], scan.points[i, 1])
                for i in range(len(mask)) if mask[i]]

    def segment_scan(self, scan: ProcessedScan) -> List[ScanSegment]:
        valid_points = self.get_valid_points(scan)
        valid_indices = np.where(scan.valid_mask)[0]

        if len(valid_points) < 2:
            return []

        segments = []
        current_segment_start = 0

        for i in range(1, len(valid_points)):
            dist = np.linalg.norm(valid_points[i] - valid_points[i-1])

            if dist > self.segmentation_threshold:
                if i - current_segment_start >= 2:
                    segment = self._create_segment(
                        valid_points[current_segment_start:i],
                        valid_indices[current_segment_start],
                        valid_indices[i-1]
                    )
                    segments.append(segment)
                current_segment_start = i

        if len(valid_points) - current_segment_start >= 2:
            segment = self._create_segment(
                valid_points[current_segment_start:],
                valid_indices[current_segment_start],
                valid_indices[-1]
            )
            segments.append(segment)

        return segments

    def _create_segment(self, points: np.ndarray, start_idx: int, end_idx: int) -> ScanSegment:
        centroid = (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))
        diffs = np.diff(points, axis=0)
        length = float(np.sum(np.linalg.norm(diffs, axis=1)))
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

    def _fit_line(self, points: np.ndarray, threshold: float = 0.05) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        if len(points) < 2:
            return False, None

        centroid = np.mean(points, axis=0)
        centered = points - centroid
        cov = np.cov(centered.T)

        if cov.ndim < 2:
            return False, None

        eigenvalues, eigenvectors = np.linalg.eig(cov)
        principal_idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, principal_idx]
        normal = np.array([-direction[1], direction[0]])
        a, b = normal
        c = -np.dot(normal, centroid)
        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c)
        max_distance = np.max(distances)
        is_line = max_distance < threshold

        return is_line, (float(a), float(b), float(c))

    def extract_lines(self, scan: ProcessedScan, min_points: int = 5, max_error: float = 0.05) -> List[Tuple[np.ndarray, np.ndarray]]:
        segments = self.segment_scan(scan)
        lines = []

        for segment in segments:
            if len(segment.points) >= min_points and segment.is_line:
                a, b, c = segment.line_params
                p1 = segment.points[0]
                p2 = segment.points[-1]

                def project_point(p):
                    t = -(a * p[0] + b * p[1] + c) / (a*a + b*b)
                    return np.array([p[0] + a*t, p[1] + b*t])

                start = project_point(p1)
                end = project_point(p2)
                lines.append((start, end))

        return lines

# --- FONCTION DE SIMULATION (REQUISE POUR L'IMPORT) ---
def create_simulated_scan(
    obstacles: List[Tuple[float, float, float]],
    walls: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    num_rays: int = 360,
    max_range: float = 10.0,
    noise_std: float = 0.02
) -> LidarScan:
    """Create a simulated LiDAR scan for testing."""
    import time
    points = []

    for i in range(num_rays):
        angle = 2 * np.pi * i / num_rays
        
        # Point par défaut (pas d'obstacle)
        point = LidarPoint(
            angle=angle,
            distance=max_range, 
            intensity=0,
            valid=False
        )
        points.append(point)

    return LidarScan(
        timestamp=time.time(),
        points=points,
        scan_frequency=10.0
    )

if __name__ == '__main__':
    print("LiDAR Processor Test Complete")
