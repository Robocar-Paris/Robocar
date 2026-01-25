#!/usr/bin/env python3
"""
SLAM Demo

Demonstrates the SLAM module with simulated data:
- Mapping from LiDAR scans
- Localization with particle filter
- Real-time visualization

This demo runs without hardware - perfect for testing before deployment.

Usage:
    python scripts/demo_slam.py
    python scripts/demo_slam.py --backend builtin  # Force built-in SLAM
    python scripts/demo_slam.py --no-viz           # Console only
"""

import sys
import time
import math
import argparse
import numpy as np

# Add src to path
sys.path.insert(0, '/home/user/Robocar/src')

from slam.slam_core import SLAM, SLAMConfig
from slam.occupancy_grid import OccupancyGrid
from slam.particle_filter import ParticleFilter, ParticleFilterConfig

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    from matplotlib.colors import LinearSegmentedColormap
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("matplotlib not installed. Run: pip install matplotlib")


class SimulatedWorld:
    """Simulated world with obstacles and walls."""

    def __init__(self):
        # World bounds
        self.x_min, self.x_max = -10, 10
        self.y_min, self.y_max = -10, 10

        # Walls (as line segments)
        self.walls = [
            # Outer walls
            ((-8, -8), (-8, 8)),
            ((-8, 8), (8, 8)),
            ((8, 8), (8, -8)),
            ((8, -8), (-8, -8)),

            # Inner walls (rooms)
            ((-8, 0), (-2, 0)),
            ((2, 0), (8, 0)),
            ((0, -8), (0, -3)),
            ((0, 3), (0, 8)),

            # Furniture
            ((-5, -5), (-3, -5)),
            ((-3, -5), (-3, -3)),
            ((4, 4), (6, 4)),
            ((6, 4), (6, 6)),
        ]

        # Circular obstacles
        self.obstacles = [
            (-4, 4, 0.5),   # x, y, radius
            (5, -3, 0.8),
            (-6, -6, 0.3),
            (3, 5, 0.4),
        ]

    def ray_cast(self, x: float, y: float, theta: float, max_range: float = 12.0) -> float:
        """Cast ray and return distance to first obstacle."""
        min_dist = max_range

        dx = math.cos(theta)
        dy = math.sin(theta)

        # Check walls (line segments)
        for (x1, y1), (x2, y2) in self.walls:
            dist = self._ray_line_intersection(x, y, dx, dy, x1, y1, x2, y2)
            if dist is not None and dist < min_dist:
                min_dist = dist

        # Check circular obstacles
        for ox, oy, r in self.obstacles:
            dist = self._ray_circle_intersection(x, y, dx, dy, ox, oy, r)
            if dist is not None and dist < min_dist:
                min_dist = dist

        return min_dist

    def _ray_line_intersection(self, x, y, dx, dy, x1, y1, x2, y2):
        """Ray-line segment intersection."""
        # Line segment direction
        lx = x2 - x1
        ly = y2 - y1

        # Solve for t (ray) and s (line segment)
        denom = dx * ly - dy * lx

        if abs(denom) < 1e-10:
            return None

        t = ((x1 - x) * ly - (y1 - y) * lx) / denom
        s = ((x1 - x) * dy - (y1 - y) * dx) / denom

        if t > 0.01 and 0 <= s <= 1:
            return t

        return None

    def _ray_circle_intersection(self, x, y, dx, dy, cx, cy, r):
        """Ray-circle intersection."""
        # Vector from ray origin to circle center
        fx = x - cx
        fy = y - cy

        a = dx*dx + dy*dy
        b = 2 * (fx*dx + fy*dy)
        c = fx*fx + fy*fy - r*r

        discriminant = b*b - 4*a*c

        if discriminant < 0:
            return None

        t1 = (-b - math.sqrt(discriminant)) / (2*a)
        t2 = (-b + math.sqrt(discriminant)) / (2*a)

        if t1 > 0.01:
            return t1
        if t2 > 0.01:
            return t2

        return None

    def get_scan(self, x: float, y: float, theta: float,
                 num_beams: int = 360, fov: float = 2*np.pi,
                 max_range: float = 12.0, noise_std: float = 0.02) -> tuple:
        """Generate LiDAR scan from position."""
        angles = np.linspace(-fov/2, fov/2, num_beams, endpoint=False)
        ranges = np.zeros(num_beams)

        for i, angle in enumerate(angles):
            world_angle = theta + angle
            dist = self.ray_cast(x, y, world_angle, max_range)
            ranges[i] = dist + np.random.normal(0, noise_std)
            ranges[i] = max(0.05, min(max_range, ranges[i]))

        return ranges, angles


class SimulatedRobot:
    """Simulated robot moving in the world."""

    def __init__(self, world: SimulatedWorld):
        self.world = world
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.velocity = 0.0
        self.angular_velocity = 0.0

        # Path history
        self.path_x = [self.x]
        self.path_y = [self.y]

    def update(self, dt: float = 0.1):
        """Update robot position."""
        # Simple motion model
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Store path
        self.path_x.append(self.x)
        self.path_y.append(self.y)

        # Keep path length reasonable
        if len(self.path_x) > 1000:
            self.path_x.pop(0)
            self.path_y.pop(0)

    def get_scan(self, num_beams: int = 360) -> tuple:
        """Get LiDAR scan."""
        return self.world.get_scan(self.x, self.y, self.theta, num_beams)

    def get_odometry(self, dt: float) -> tuple:
        """Get odometry (dx, dtheta, dt) in robot frame."""
        dx = self.velocity * dt
        dtheta = self.angular_velocity * dt
        return dx, dtheta, dt


def run_demo_console(args):
    """Run SLAM demo with console output."""
    print("\n" + "=" * 60)
    print("SLAM DEMO (Console Mode)")
    print("=" * 60)

    # Create world and robot
    world = SimulatedWorld()
    robot = SimulatedRobot(world)

    # Create SLAM
    config = SLAMConfig(
        map_size_pixels=400,
        map_size_meters=25.0,
        scan_size=360
    )
    slam = SLAM(config)

    print("\nSimulating mapping run...")
    print("-" * 60)

    # Run simulation
    for step in range(200):
        # Control robot (simple exploration pattern)
        t = step * 0.1
        robot.velocity = 0.3
        robot.angular_velocity = 0.3 * math.sin(t * 0.5)

        # Update robot
        robot.update(0.1)

        # Get sensor data
        ranges_m, angles = robot.get_scan(360)
        ranges_mm = (ranges_m * 1000).astype(int).tolist()

        # Get odometry
        dx, dtheta, dt = robot.get_odometry(0.1)

        # Update SLAM
        pose = slam.update(
            ranges_mm,
            velocity=(dx * 1000, math.degrees(dtheta), dt)
        )

        # Print progress
        if step % 20 == 0:
            error = math.sqrt(
                (pose.x - robot.x)**2 +
                (pose.y - robot.y)**2
            )
            print(f"Step {step:3d}: "
                  f"True=({robot.x:5.2f}, {robot.y:5.2f}) "
                  f"Est=({pose.x:5.2f}, {pose.y:5.2f}) "
                  f"Error={error:.3f}m")

    # Final map statistics
    map_data = slam.get_map()
    free = np.sum(map_data < 100)
    occupied = np.sum(map_data > 150)
    total = map_data.size

    print("-" * 60)
    print(f"\nMap statistics:")
    print(f"  Size: {map_data.shape[0]}x{map_data.shape[1]} pixels")
    print(f"  Free: {100*free/total:.1f}%")
    print(f"  Occupied: {100*occupied/total:.1f}%")
    print(f"  Unknown: {100*(total-free-occupied)/total:.1f}%")

    # Save map
    try:
        import cv2
        cv2.imwrite("/tmp/slam_map.png", map_data)
        print(f"\nMap saved to /tmp/slam_map.png")
    except ImportError:
        print("\nInstall opencv-python to save map")

    print("\nDemo complete!")


def run_demo_visual(args):
    """Run SLAM demo with visualization."""
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available, falling back to console mode")
        run_demo_console(args)
        return

    print("\n" + "=" * 60)
    print("SLAM DEMO (Visual Mode)")
    print("=" * 60)
    print("Close window to exit")

    # Create world and robot
    world = SimulatedWorld()
    robot = SimulatedRobot(world)

    # Create SLAM
    config = SLAMConfig(
        map_size_pixels=400,
        map_size_meters=25.0,
        scan_size=360
    )
    slam = SLAM(config)

    # Setup figure
    fig = plt.figure(figsize=(15, 6))
    fig.suptitle('SLAM Demo - Mapping a Simulated Environment', fontsize=14)

    # World view (ground truth)
    ax_world = fig.add_subplot(131)
    ax_world.set_title('Ground Truth')
    ax_world.set_xlim(-10, 10)
    ax_world.set_ylim(-10, 10)
    ax_world.set_aspect('equal')
    ax_world.grid(True, alpha=0.3)

    # Draw walls
    for (x1, y1), (x2, y2) in world.walls:
        ax_world.plot([x1, x2], [y1, y2], 'k-', linewidth=2)

    # Draw obstacles
    for ox, oy, r in world.obstacles:
        circle = plt.Circle((ox, oy), r, color='red', alpha=0.5)
        ax_world.add_patch(circle)

    # Robot marker
    robot_marker, = ax_world.plot([], [], 'b^', markersize=12)
    robot_path, = ax_world.plot([], [], 'b-', alpha=0.3, linewidth=1)
    robot_heading, = ax_world.plot([], [], 'b-', linewidth=2)

    # LiDAR view
    ax_lidar = fig.add_subplot(132, projection='polar')
    ax_lidar.set_title('LiDAR Scan')
    ax_lidar.set_ylim(0, 12)
    lidar_scatter, = ax_lidar.plot([], [], 'g.', markersize=2)

    # SLAM map view
    ax_map = fig.add_subplot(133)
    ax_map.set_title('SLAM Map')

    # Create custom colormap: black->gray->white
    cmap = LinearSegmentedColormap.from_list(
        'occupancy',
        [(0, 'black'), (0.5, 'gray'), (1, 'white')]
    )

    map_img = ax_map.imshow(
        np.ones((config.map_size_pixels, config.map_size_pixels)) * 127,
        cmap=cmap, vmin=0, vmax=255,
        extent=[-config.map_size_meters/2, config.map_size_meters/2,
                -config.map_size_meters/2, config.map_size_meters/2],
        origin='lower'
    )
    est_marker, = ax_map.plot([], [], 'r^', markersize=10)
    est_path, = ax_map.plot([], [], 'r-', alpha=0.5, linewidth=1)

    # Path storage
    est_path_x, est_path_y = [], []

    # Status text
    status_text = fig.text(
        0.02, 0.02,
        '',
        fontsize=10,
        fontfamily='monospace',
        verticalalignment='bottom'
    )

    # Animation state
    state = {'step': 0, 'start_time': time.time()}

    def update(frame):
        # Control robot
        t = state['step'] * 0.1
        robot.velocity = 0.4
        robot.angular_velocity = 0.4 * math.sin(t * 0.3) + 0.1 * math.sin(t * 0.7)

        # Update robot
        robot.update(0.1)

        # Get scan
        ranges_m, angles = robot.get_scan(360)

        # Update SLAM
        ranges_mm = (ranges_m * 1000).astype(int).tolist()
        dx, dtheta, dt = robot.get_odometry(0.1)
        pose = slam.update(ranges_mm, velocity=(dx * 1000, math.degrees(dtheta), dt))

        # Update world view
        robot_marker.set_data([robot.x], [robot.y])
        robot_path.set_data(robot.path_x, robot.path_y)

        hx = robot.x + 0.8 * math.cos(robot.theta)
        hy = robot.y + 0.8 * math.sin(robot.theta)
        robot_heading.set_data([robot.x, hx], [robot.y, hy])

        # Update LiDAR view
        lidar_scatter.set_data(angles, ranges_m)

        # Update map view
        map_data = slam.get_map()
        map_img.set_data(np.flipud(map_data))

        est_path_x.append(pose.x)
        est_path_y.append(pose.y)
        if len(est_path_x) > 500:
            est_path_x.pop(0)
            est_path_y.pop(0)

        est_marker.set_data([pose.x], [pose.y])
        est_path.set_data(est_path_x, est_path_y)

        # Update status
        error = math.sqrt((pose.x - robot.x)**2 + (pose.y - robot.y)**2)
        elapsed = time.time() - state['start_time']

        status_text.set_text(
            f"Step: {state['step']:4d} | "
            f"Time: {elapsed:.1f}s | "
            f"Error: {error:.3f}m | "
            f"True: ({robot.x:.2f}, {robot.y:.2f}) | "
            f"Est: ({pose.x:.2f}, {pose.y:.2f})"
        )

        state['step'] += 1

        return (robot_marker, robot_path, robot_heading,
                lidar_scatter, map_img, est_marker, est_path, status_text)

    ani = FuncAnimation(fig, update, interval=50, blit=False)

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.08)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='SLAM Demo')
    parser.add_argument('--backend', choices=['auto', 'breezyslam', 'builtin'],
                        default='auto', help='SLAM backend to use')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization')
    args = parser.parse_args()

    if args.no_viz or not MATPLOTLIB_AVAILABLE:
        run_demo_console(args)
    else:
        run_demo_visual(args)


if __name__ == '__main__':
    main()
