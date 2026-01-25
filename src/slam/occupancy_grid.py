"""
Occupancy Grid Map

2D occupancy grid implementation compatible with:
- BreezySLAM
- SLAM Toolbox (ROS2)
- Navigation algorithms

Features:
- Probabilistic updates (log-odds)
- Ray tracing
- Map I/O (PNG, PGM, YAML)
- Coordinate transformations
"""

import math
import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass
import struct


@dataclass
class MapMetadata:
    """Map metadata (compatible with ROS map_server format)."""
    resolution: float       # meters per pixel
    width: int              # pixels
    height: int             # pixels
    origin_x: float         # world x of pixel (0,0)
    origin_y: float         # world y of pixel (0,0)
    origin_theta: float = 0.0  # rotation (usually 0)

    # Thresholds
    free_threshold: float = 0.2
    occupied_threshold: float = 0.65


class OccupancyGrid:
    """
    2D Occupancy Grid Map.

    Uses log-odds representation for efficient probabilistic updates.

    Values:
    - 0.0 = definitely free
    - 0.5 = unknown
    - 1.0 = definitely occupied

    Usage:
        grid = OccupancyGrid(width=800, height=800, resolution=0.05)

        # Update with scan
        grid.update_from_scan(robot_x, robot_y, robot_theta, ranges, angles)

        # Query
        is_free = grid.is_free(x, y)
        distance = grid.ray_cast(x, y, theta)

        # Save
        grid.save("map.png", "map.yaml")
    """

    def __init__(
        self,
        width: int = 800,
        height: int = 800,
        resolution: float = 0.05,
        origin_x: Optional[float] = None,
        origin_y: Optional[float] = None
    ):
        """
        Initialize occupancy grid.

        Args:
            width: Map width in pixels
            height: Map height in pixels
            resolution: Meters per pixel
            origin_x: World X of map origin (default: center)
            origin_y: World Y of map origin (default: center)
        """
        self.width = width
        self.height = height
        self.resolution = resolution

        # Origin (default: center of map at world origin)
        self.origin_x = origin_x if origin_x is not None else -width * resolution / 2
        self.origin_y = origin_y if origin_y is not None else -height * resolution / 2

        # Log-odds representation (0 = unknown)
        self._log_odds = np.zeros((height, width), dtype=np.float32)

        # Log-odds parameters
        self._l_occ = 0.85    # Log-odds for occupied
        self._l_free = -0.4   # Log-odds for free
        self._l_min = -5.0    # Minimum log-odds
        self._l_max = 5.0     # Maximum log-odds

        # Thresholds
        self.free_threshold = 0.2
        self.occupied_threshold = 0.65

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map indices."""
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map indices to world coordinates."""
        x = mx * self.resolution + self.origin_x + self.resolution / 2
        y = my * self.resolution + self.origin_y + self.resolution / 2
        return x, y

    def in_bounds(self, mx: int, my: int) -> bool:
        """Check if map indices are in bounds."""
        return 0 <= mx < self.width and 0 <= my < self.height

    def get_probability(self, mx: int, my: int) -> float:
        """Get occupancy probability at map coordinates."""
        if not self.in_bounds(mx, my):
            return 0.5  # Unknown outside bounds

        log_odds = self._log_odds[my, mx]
        return 1.0 / (1.0 + np.exp(-log_odds))

    def get_probability_world(self, x: float, y: float) -> float:
        """Get occupancy probability at world coordinates."""
        mx, my = self.world_to_map(x, y)
        return self.get_probability(mx, my)

    def is_free(self, x: float, y: float) -> bool:
        """Check if world position is free."""
        return self.get_probability_world(x, y) < self.free_threshold

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if world position is occupied."""
        return self.get_probability_world(x, y) > self.occupied_threshold

    def is_unknown(self, x: float, y: float) -> bool:
        """Check if world position is unknown."""
        p = self.get_probability_world(x, y)
        return self.free_threshold <= p <= self.occupied_threshold

    def set_occupied(self, mx: int, my: int):
        """Mark cell as occupied."""
        if self.in_bounds(mx, my):
            self._log_odds[my, mx] = min(
                self._l_max,
                self._log_odds[my, mx] + self._l_occ
            )

    def set_free(self, mx: int, my: int):
        """Mark cell as free."""
        if self.in_bounds(mx, my):
            self._log_odds[my, mx] = max(
                self._l_min,
                self._log_odds[my, mx] + self._l_free
            )

    def update_from_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        ranges: np.ndarray,
        angles: np.ndarray,
        max_range: float = 10.0
    ):
        """
        Update map from LiDAR scan.

        Args:
            robot_x, robot_y: Robot position in world frame
            robot_theta: Robot heading in world frame
            ranges: Array of range measurements (meters)
            angles: Array of corresponding angles (radians, relative to robot)
            max_range: Maximum sensor range
        """
        robot_mx, robot_my = self.world_to_map(robot_x, robot_y)

        for i, (r, angle) in enumerate(zip(ranges, angles)):
            if r <= 0.05 or r >= max_range:
                continue

            # End point in world frame
            world_angle = robot_theta + angle
            end_x = robot_x + r * math.cos(world_angle)
            end_y = robot_y + r * math.sin(world_angle)
            end_mx, end_my = self.world_to_map(end_x, end_y)

            # Ray trace from robot to end point
            self._trace_ray(robot_mx, robot_my, end_mx, end_my, mark_end=True)

    def _trace_ray(self, x0: int, y0: int, x1: int, y1: int, mark_end: bool = True):
        """Trace ray using Bresenham's algorithm."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            if (x, y) == (x1, y1):
                # End point - mark as occupied
                if mark_end:
                    self.set_occupied(x, y)
                break

            # Mark as free
            self.set_free(x, y)

            if not self.in_bounds(x, y):
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def ray_cast(
        self,
        x: float,
        y: float,
        theta: float,
        max_range: float = 10.0
    ) -> float:
        """
        Cast ray and return distance to first obstacle.

        Args:
            x, y: Start position (world coordinates)
            theta: Ray direction (radians)
            max_range: Maximum range

        Returns:
            Distance to obstacle or max_range
        """
        mx, my = self.world_to_map(x, y)

        # Calculate end point
        end_x = x + max_range * math.cos(theta)
        end_y = y + max_range * math.sin(theta)
        end_mx, end_my = self.world_to_map(end_x, end_y)

        # Bresenham
        dx = abs(end_mx - mx)
        dy = abs(end_my - my)
        sx = 1 if mx < end_mx else -1
        sy = 1 if my < end_my else -1
        err = dx - dy

        cx, cy = mx, my
        dist = 0.0

        while dist < max_range / self.resolution:
            if not self.in_bounds(cx, cy):
                return max_range

            if self.get_probability(cx, cy) > self.occupied_threshold:
                # Hit obstacle
                wx, wy = self.map_to_world(cx, cy)
                return math.sqrt((wx - x)**2 + (wy - y)**2)

            if cx == end_mx and cy == end_my:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

            dist += 1

        return max_range

    def get_map_data(self) -> np.ndarray:
        """
        Get map as probability array.

        Returns:
            2D array with values 0.0 (free) to 1.0 (occupied)
        """
        return 1.0 / (1.0 + np.exp(-self._log_odds))

    def get_map_image(self) -> np.ndarray:
        """
        Get map as image (0-255, compatible with OpenCV).

        Values:
        - 254 = free (white)
        - 205 = unknown (gray)
        - 0 = occupied (black)
        """
        prob = self.get_map_data()

        image = np.ones((self.height, self.width), dtype=np.uint8) * 205

        # Free
        image[prob < self.free_threshold] = 254

        # Occupied
        image[prob > self.occupied_threshold] = 0

        return image

    def get_map_rgb(self) -> np.ndarray:
        """Get map as RGB image for visualization."""
        prob = self.get_map_data()

        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Free = white
        free_mask = prob < self.free_threshold
        rgb[free_mask] = [255, 255, 255]

        # Unknown = gray
        unknown_mask = (prob >= self.free_threshold) & (prob <= self.occupied_threshold)
        rgb[unknown_mask] = [128, 128, 128]

        # Occupied = black
        occupied_mask = prob > self.occupied_threshold
        rgb[occupied_mask] = [0, 0, 0]

        return rgb

    def save(self, image_path: str, yaml_path: Optional[str] = None):
        """
        Save map in ROS-compatible format.

        Args:
            image_path: Path for image (PNG or PGM)
            yaml_path: Path for metadata YAML (optional)
        """
        # Save image
        image = self.get_map_image()

        if image_path.endswith('.pgm'):
            self._save_pgm(image_path, image)
        else:
            try:
                import cv2
                cv2.imwrite(image_path, image)
            except ImportError:
                # Fallback to simple PNG
                self._save_png(image_path, image)

        # Save YAML metadata
        if yaml_path:
            self._save_yaml(yaml_path, image_path)

    def _save_pgm(self, path: str, image: np.ndarray):
        """Save as PGM (Portable Gray Map)."""
        with open(path, 'wb') as f:
            f.write(f"P5\n{self.width} {self.height}\n255\n".encode())
            f.write(image.tobytes())

    def _save_png(self, path: str, image: np.ndarray):
        """Save as PNG using pure Python (fallback)."""
        import zlib

        def png_chunk(chunk_type, data):
            chunk_len = struct.pack('>I', len(data))
            chunk_crc = struct.pack('>I', zlib.crc32(chunk_type + data) & 0xffffffff)
            return chunk_len + chunk_type + data + chunk_crc

        # PNG signature
        signature = b'\x89PNG\r\n\x1a\n'

        # IHDR
        ihdr_data = struct.pack('>IIBBBBB', self.width, self.height, 8, 0, 0, 0, 0)
        ihdr = png_chunk(b'IHDR', ihdr_data)

        # IDAT
        raw_data = b''
        for row in image:
            raw_data += b'\x00' + row.tobytes()
        compressed = zlib.compress(raw_data, 9)
        idat = png_chunk(b'IDAT', compressed)

        # IEND
        iend = png_chunk(b'IEND', b'')

        with open(path, 'wb') as f:
            f.write(signature + ihdr + idat + iend)

    def _save_yaml(self, yaml_path: str, image_path: str):
        """Save YAML metadata (ROS map_server format)."""
        import os

        yaml_content = f"""image: {os.path.basename(image_path)}
resolution: {self.resolution}
origin: [{self.origin_x}, {self.origin_y}, 0.0]
negate: 0
occupied_thresh: {self.occupied_threshold}
free_thresh: {self.free_threshold}
"""
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

    @classmethod
    def load(cls, image_path: str, yaml_path: Optional[str] = None) -> 'OccupancyGrid':
        """
        Load map from files.

        Args:
            image_path: Path to image file
            yaml_path: Path to YAML metadata (optional)
        """
        # Load image
        try:
            import cv2
            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        except ImportError:
            # Simple PGM loader
            image = cls._load_pgm(image_path)

        height, width = image.shape

        # Default metadata
        resolution = 0.05
        origin_x = -width * resolution / 2
        origin_y = -height * resolution / 2

        # Load YAML if provided
        if yaml_path:
            import yaml
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
                resolution = meta.get('resolution', resolution)
                origin = meta.get('origin', [origin_x, origin_y, 0])
                origin_x, origin_y = origin[0], origin[1]

        # Create grid
        grid = cls(width, height, resolution, origin_x, origin_y)

        # Convert image to log-odds
        # 254 = free, 205 = unknown, 0 = occupied
        prob = np.where(image == 254, 0.1,
               np.where(image == 0, 0.9, 0.5))

        # Convert probability to log-odds
        grid._log_odds = np.log(prob / (1 - prob + 1e-10)).astype(np.float32)

        return grid

    @staticmethod
    def _load_pgm(path: str) -> np.ndarray:
        """Load PGM file."""
        with open(path, 'rb') as f:
            # Read header
            magic = f.readline().decode().strip()
            if magic != 'P5':
                raise ValueError("Not a PGM file")

            # Skip comments
            line = f.readline().decode()
            while line.startswith('#'):
                line = f.readline().decode()

            width, height = map(int, line.split())
            maxval = int(f.readline().decode().strip())

            # Read data
            data = np.frombuffer(f.read(), dtype=np.uint8)
            return data.reshape((height, width))

    def get_metadata(self) -> MapMetadata:
        """Get map metadata."""
        return MapMetadata(
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold
        )

    def inflate(self, radius: float) -> 'OccupancyGrid':
        """
        Create inflated map for path planning.

        Args:
            radius: Inflation radius in meters

        Returns:
            New inflated OccupancyGrid
        """
        from scipy import ndimage

        # Create inflated grid
        inflated = OccupancyGrid(
            self.width, self.height, self.resolution,
            self.origin_x, self.origin_y
        )

        # Get binary occupied mask
        prob = self.get_map_data()
        occupied = (prob > self.occupied_threshold).astype(np.float32)

        # Dilate
        radius_pixels = int(radius / self.resolution)
        kernel_size = 2 * radius_pixels + 1
        kernel = np.ones((kernel_size, kernel_size))

        # Create circular kernel
        y, x = np.ogrid[-radius_pixels:radius_pixels+1, -radius_pixels:radius_pixels+1]
        kernel = (x*x + y*y <= radius_pixels*radius_pixels).astype(np.float32)

        inflated_occupied = ndimage.maximum_filter(occupied, footprint=kernel)

        # Convert back to log-odds
        inflated._log_odds = np.where(
            inflated_occupied > 0.5,
            inflated._l_max,
            self._log_odds
        )

        return inflated


if __name__ == '__main__':
    print("Testing OccupancyGrid...")

    # Create grid
    grid = OccupancyGrid(width=400, height=400, resolution=0.05)
    print(f"Map size: {grid.width}x{grid.height} pixels")
    print(f"World size: {grid.width * grid.resolution}x{grid.height * grid.resolution} meters")

    # Simulate scan updates
    robot_x, robot_y, robot_theta = 0, 0, 0

    for step in range(50):
        # Create fake scan
        angles = np.linspace(-np.pi, np.pi, 360)
        ranges = np.random.uniform(2, 8, 360)

        # Add some structure
        for i, angle in enumerate(angles):
            world_angle = robot_theta + angle
            if abs(math.sin(world_angle * 4)) > 0.8:
                ranges[i] = 3.0

        # Update grid
        grid.update_from_scan(robot_x, robot_y, robot_theta, ranges, angles)

        # Move robot
        robot_x += 0.1 * math.cos(robot_theta)
        robot_y += 0.1 * math.sin(robot_theta)
        robot_theta += 0.1

    # Get statistics
    prob = grid.get_map_data()
    free_cells = np.sum(prob < grid.free_threshold)
    occupied_cells = np.sum(prob > grid.occupied_threshold)
    unknown_cells = grid.width * grid.height - free_cells - occupied_cells

    print(f"\nMap statistics:")
    print(f"  Free cells: {free_cells} ({100*free_cells/(grid.width*grid.height):.1f}%)")
    print(f"  Occupied cells: {occupied_cells} ({100*occupied_cells/(grid.width*grid.height):.1f}%)")
    print(f"  Unknown cells: {unknown_cells} ({100*unknown_cells/(grid.width*grid.height):.1f}%)")

    # Test ray casting
    dist = grid.ray_cast(0, 0, 0)
    print(f"\nRay cast from origin forward: {dist:.2f}m")

    # Save map
    grid.save("/tmp/test_map.png", "/tmp/test_map.yaml")
    print(f"\nSaved map to /tmp/test_map.png")

    print("\nTest complete!")
