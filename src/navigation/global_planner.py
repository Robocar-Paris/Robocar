"""
Global Path Planner - A* Algorithm

Finds the shortest path from A to B on the occupancy grid map.

Algorithm: A* (pronounced "A star")
- Proven optimal shortest path algorithm
- Used in every video game, GPS, and robot
- Guaranteed to find the shortest path if one exists

References:
- Red Blob Games A* tutorial: https://www.redblobgames.com/pathfinding/a-star/
- Nav2 NavFn Planner (ROS2 uses this same approach)
"""

import math
import heapq
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class PlannerConfig:
    """Global planner configuration."""
    # Robot dimensions
    robot_radius: float = 0.15          # meters
    safety_margin: float = 0.10         # extra margin around obstacles

    # A* parameters
    allow_diagonal: bool = True         # Allow 8-directional movement
    heuristic_weight: float = 1.2       # >1 = faster but slightly suboptimal

    # Path smoothing
    smooth_path: bool = True
    smooth_weight_data: float = 0.1     # How close to stay to original
    smooth_weight_smooth: float = 0.3   # How smooth to make path

    # Replanning
    replan_on_obstacle: bool = True


@dataclass(order=True)
class Node:
    """A* search node."""
    f_cost: float                                       # g + h (for priority queue)
    g_cost: float = field(compare=False)                # Cost from start
    x: int = field(compare=False)
    y: int = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)


class GlobalPlanner:
    """
    A* global path planner on occupancy grid.

    Finds shortest collision-free path from start to goal.

    Usage:
        planner = GlobalPlanner(occupancy_grid)

        path = planner.plan(start=(0,0), goal=(5,3))
        # path = [(0,0), (0.5,0.3), (1.0,0.6), ..., (5,3)]

        # Visualize
        planner.get_path_image()
    """

    # 8-directional movement costs
    DIRECTIONS_8 = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),          # Cardinal
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)  # Diagonal
    ]

    # 4-directional movement costs
    DIRECTIONS_4 = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)
    ]

    def __init__(self, config: Optional[PlannerConfig] = None):
        self.config = config or PlannerConfig()
        self._inflated_map: Optional[np.ndarray] = None
        self._map_resolution: float = 0.05
        self._map_origin_x: float = 0.0
        self._map_origin_y: float = 0.0
        self._map_width: int = 0
        self._map_height: int = 0

    def set_map(self, occupancy_map, inflate: bool = True):
        """
        Set the map for planning.

        Args:
            occupancy_map: OccupancyGrid instance
            inflate: Whether to inflate obstacles by robot radius
        """
        self._map_resolution = occupancy_map.resolution
        self._map_origin_x = occupancy_map.origin_x
        self._map_origin_y = occupancy_map.origin_y
        self._map_width = occupancy_map.width
        self._map_height = occupancy_map.height

        if inflate:
            try:
                inflated = occupancy_map.inflate(
                    self.config.robot_radius + self.config.safety_margin
                )
                prob = inflated.get_map_data()
            except (ImportError, Exception):
                # Fallback: simple inflation
                prob = occupancy_map.get_map_data()
                prob = self._simple_inflate(
                    prob,
                    self.config.robot_radius + self.config.safety_margin
                )
        else:
            prob = occupancy_map.get_map_data()

        # Binary map: True = obstacle/occupied
        self._inflated_map = prob > occupancy_map.occupied_threshold

    def set_map_array(self, binary_map: np.ndarray, resolution: float,
                      origin_x: float = 0.0, origin_y: float = 0.0):
        """Set map from binary array (True=obstacle)."""
        self._inflated_map = binary_map
        self._map_resolution = resolution
        self._map_origin_x = origin_x
        self._map_origin_y = origin_y
        self._map_height, self._map_width = binary_map.shape

    def _simple_inflate(self, prob_map: np.ndarray, radius: float) -> np.ndarray:
        """Simple inflation without scipy."""
        inflated = prob_map.copy()
        r_pixels = int(radius / self._map_resolution) + 1

        occupied = prob_map > 0.65
        h, w = prob_map.shape

        for y in range(h):
            for x in range(w):
                if occupied[y, x]:
                    for dy in range(-r_pixels, r_pixels + 1):
                        for dx in range(-r_pixels, r_pixels + 1):
                            if dx*dx + dy*dy <= r_pixels*r_pixels:
                                ny, nx = y + dy, x + dx
                                if 0 <= ny < h and 0 <= nx < w:
                                    inflated[ny, nx] = max(inflated[ny, nx], 0.9)
        return inflated

    def plan(self, start: Tuple[float, float],
             goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start to goal in world coordinates.

        Args:
            start: (x, y) start position in meters
            goal: (x, y) goal position in meters

        Returns:
            List of (x, y) waypoints in meters, or None if no path found
        """
        if self._inflated_map is None:
            print("[Planner] No map set!")
            return None

        # Convert to map coordinates
        start_mx, start_my = self._world_to_map(start[0], start[1])
        goal_mx, goal_my = self._world_to_map(goal[0], goal[1])

        # Validate
        if not self._in_bounds(start_mx, start_my):
            print(f"[Planner] Start {start} is outside map")
            return None
        if not self._in_bounds(goal_mx, goal_my):
            print(f"[Planner] Goal {goal} is outside map")
            return None
        if self._inflated_map[start_my, start_mx]:
            print(f"[Planner] Start {start} is inside an obstacle")
            return None
        if self._inflated_map[goal_my, goal_mx]:
            print(f"[Planner] Goal {goal} is inside an obstacle")
            return None

        # Run A*
        map_path = self._astar(start_mx, start_my, goal_mx, goal_my)

        if map_path is None:
            print("[Planner] No path found!")
            return None

        # Convert back to world coordinates
        world_path = [self._map_to_world(mx, my) for mx, my in map_path]

        # Smooth path
        if self.config.smooth_path and len(world_path) > 2:
            world_path = self._smooth_path(world_path)

        return world_path

    def _astar(self, sx: int, sy: int, gx: int, gy: int) -> Optional[List[Tuple[int, int]]]:
        """
        A* search algorithm.

        Returns list of (mx, my) map coordinates or None.
        """
        directions = self.DIRECTIONS_8 if self.config.allow_diagonal else self.DIRECTIONS_4

        # Priority queue
        start_node = Node(
            f_cost=self._heuristic(sx, sy, gx, gy),
            g_cost=0.0,
            x=sx, y=sy,
            parent=None
        )

        open_set = [start_node]
        heapq.heapify(open_set)

        # Visited set
        closed = set()

        # Best g_cost for each position
        g_costs = {}
        g_costs[(sx, sy)] = 0.0

        iterations = 0
        max_iterations = self._map_width * self._map_height  # Safety limit

        while open_set and iterations < max_iterations:
            iterations += 1

            # Get node with lowest f_cost
            current = heapq.heappop(open_set)

            # Goal reached?
            if current.x == gx and current.y == gy:
                return self._reconstruct_path(current)

            # Skip if already visited
            pos = (current.x, current.y)
            if pos in closed:
                continue
            closed.add(pos)

            # Explore neighbors
            for dx, dy, cost in directions:
                nx, ny = current.x + dx, current.y + dy

                # Bounds check
                if not self._in_bounds(nx, ny):
                    continue

                # Obstacle check
                if self._inflated_map[ny, nx]:
                    continue

                # Skip if visited
                if (nx, ny) in closed:
                    continue

                # Diagonal: check if both adjacent cells are free
                if abs(dx) + abs(dy) == 2:
                    if self._inflated_map[current.y + dy, current.x] or \
                       self._inflated_map[current.y, current.x + dx]:
                        continue

                # Calculate cost
                new_g = current.g_cost + cost

                # Skip if we already found a better path
                if (nx, ny) in g_costs and new_g >= g_costs[(nx, ny)]:
                    continue

                g_costs[(nx, ny)] = new_g
                h = self._heuristic(nx, ny, gx, gy)
                f = new_g + h

                neighbor = Node(f_cost=f, g_cost=new_g, x=nx, y=ny, parent=current)
                heapq.heappush(open_set, neighbor)

        return None  # No path found

    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Heuristic function (octile distance)."""
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        if self.config.allow_diagonal:
            # Octile distance
            h = max(dx, dy) + (1.414 - 1.0) * min(dx, dy)
        else:
            # Manhattan distance
            h = dx + dy

        return h * self.config.heuristic_weight

    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """Reconstruct path from goal node to start."""
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth path using gradient descent.

        Reference: Sebastian Thrun's path smoothing algorithm
        """
        # Convert to numpy
        smoothed = np.array(path, dtype=np.float64)
        original = smoothed.copy()

        weight_data = self.config.smooth_weight_data
        weight_smooth = self.config.smooth_weight_smooth

        tolerance = 0.0001
        max_iterations = 200

        for _ in range(max_iterations):
            change = 0.0

            # Don't modify start and end points
            for i in range(1, len(smoothed) - 1):
                for j in range(2):  # x and y
                    old = smoothed[i, j]

                    smoothed[i, j] += weight_data * (original[i, j] - smoothed[i, j])
                    smoothed[i, j] += weight_smooth * (
                        smoothed[i-1, j] + smoothed[i+1, j] - 2.0 * smoothed[i, j]
                    )

                    change += abs(old - smoothed[i, j])

            if change < tolerance:
                break

        # Verify smoothed path doesn't cross obstacles
        valid_path = [tuple(smoothed[0])]
        for i in range(1, len(smoothed)):
            mx, my = self._world_to_map(smoothed[i, 0], smoothed[i, 1])
            if self._in_bounds(mx, my) and not self._inflated_map[my, mx]:
                valid_path.append(tuple(smoothed[i]))
            else:
                # Fall back to original point
                valid_path.append(path[i])

        return valid_path

    def _world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world to map coordinates."""
        mx = int((x - self._map_origin_x) / self._map_resolution)
        my = int((y - self._map_origin_y) / self._map_resolution)
        return mx, my

    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map to world coordinates."""
        x = mx * self._map_resolution + self._map_origin_x + self._map_resolution / 2
        y = my * self._map_resolution + self._map_origin_y + self._map_resolution / 2
        return x, y

    def _in_bounds(self, mx: int, my: int) -> bool:
        """Check if map coordinates are in bounds."""
        return 0 <= mx < self._map_width and 0 <= my < self._map_height

    def path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total length of path in meters."""
        if not path or len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total += math.sqrt(dx*dx + dy*dy)
        return total


def simplify_path(path: List[Tuple[float, float]],
                  tolerance: float = 0.1) -> List[Tuple[float, float]]:
    """
    Simplify path using Ramer-Douglas-Peucker algorithm.

    Reduces number of waypoints while keeping shape.
    """
    if len(path) <= 2:
        return path

    # Find point with maximum distance from line between start and end
    start = np.array(path[0])
    end = np.array(path[-1])
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)

    if line_len < 1e-10:
        return [path[0], path[-1]]

    line_unit = line_vec / line_len

    max_dist = 0.0
    max_idx = 0

    for i in range(1, len(path) - 1):
        point = np.array(path[i])
        proj = np.dot(point - start, line_unit)
        proj = max(0, min(line_len, proj))
        closest = start + proj * line_unit
        dist = np.linalg.norm(point - closest)

        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > tolerance:
        left = simplify_path(path[:max_idx+1], tolerance)
        right = simplify_path(path[max_idx:], tolerance)
        return left[:-1] + right
    else:
        return [path[0], path[-1]]