#!/usr/bin/env python3
"""
Perception Demo

Demonstrates the perception module with simulated data:
- LiDAR processing and obstacle detection
- Sensor fusion
- Real-time visualization

This demo runs without hardware - perfect for testing before deployment.

Usage:
    python scripts/demo_perception.py
"""

import sys
import time
import math
import numpy as np

# Add src to path
sys.path.insert(0, '/home/user/Robocar/src')

from perception.lidar_processor import LidarProcessor, create_simulated_scan
from perception.obstacle_detector import ObstacleDetector, ObstacleType
from perception.sensor_fusion import SensorFusion
from perception.transforms import Pose2D, GPSConverter

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("matplotlib not installed. Install with: pip install matplotlib")


class SimulatedRobot:
    """Simulates a moving robot with sensors."""

    def __init__(self):
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0

        # World obstacles (x, y, radius)
        self.obstacles = [
            (3.0, 1.0, 0.4),
            (2.0, -1.5, 0.3),
            (-2.0, 2.0, 0.5),
            (4.0, 0.0, 0.3),
            (1.0, 3.0, 0.35),
        ]

        # World walls ((x1, y1), (x2, y2))
        self.walls = [
            ((5.0, -4.0), (5.0, 4.0)),    # Right wall
            ((-5.0, -4.0), (-5.0, 4.0)),  # Left wall
            ((-5.0, 4.0), (5.0, 4.0)),    # Top wall
            ((-5.0, -4.0), (5.0, -4.0)),  # Bottom wall
        ]

        # GPS origin
        self.gps_origin = (48.8566, 2.3522)
        self.gps_converter = GPSConverter(*self.gps_origin)

        # Time
        self.last_time = time.time()

    def update(self, velocity: float, angular_velocity: float):
        """Update robot state."""
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.velocity = velocity
        self.angular_velocity = angular_velocity

        # Update pose
        self.theta += angular_velocity * dt
        self.x += velocity * math.cos(self.theta) * dt
        self.y += velocity * math.sin(self.theta) * dt

    def get_lidar_scan(self):
        """Generate LiDAR scan from current position."""
        # Transform obstacles to robot frame
        robot_obstacles = []
        for ox, oy, r in self.obstacles:
            # Transform to robot frame
            dx = ox - self.x
            dy = oy - self.y
            rx = dx * math.cos(-self.theta) - dy * math.sin(-self.theta)
            ry = dx * math.sin(-self.theta) + dy * math.cos(-self.theta)
            robot_obstacles.append((rx, ry, r))

        # Transform walls to robot frame
        robot_walls = []
        for (x1, y1), (x2, y2) in self.walls:
            dx1, dy1 = x1 - self.x, y1 - self.y
            dx2, dy2 = x2 - self.x, y2 - self.y

            rx1 = dx1 * math.cos(-self.theta) - dy1 * math.sin(-self.theta)
            ry1 = dx1 * math.sin(-self.theta) + dy1 * math.cos(-self.theta)
            rx2 = dx2 * math.cos(-self.theta) - dy2 * math.sin(-self.theta)
            ry2 = dx2 * math.sin(-self.theta) + dy2 * math.cos(-self.theta)

            robot_walls.append(((rx1, ry1), (rx2, ry2)))

        return create_simulated_scan(robot_obstacles, robot_walls, noise_std=0.02)

    def get_gps_position(self):
        """Get GPS position with noise."""
        noise = np.random.normal(0, 0.02, 2)  # 2cm RTK noise
        lat, lon, _ = self.gps_converter.local_to_gps(
            self.x + noise[0],
            self.y + noise[1]
        )
        return lat, lon

    def get_odometry(self, dt: float):
        """Get odometry data."""
        distance = self.velocity * dt
        delta_theta = self.angular_velocity * dt
        return distance, delta_theta


def run_demo_console():
    """Run demo with console output only."""
    print("\n" + "=" * 60)
    print("PERCEPTION DEMO (Console Mode)")
    print("=" * 60)

    # Initialize
    robot = SimulatedRobot()
    lidar_processor = LidarProcessor()
    obstacle_detector = ObstacleDetector()
    fusion = SensorFusion()
    fusion.set_gps_origin(*robot.gps_origin)

    print("\nSimulating robot moving forward...")
    print("-" * 60)

    # Simulate 5 seconds
    for step in range(50):
        # Move robot
        robot.update(velocity=0.5, angular_velocity=0.1)

        # Get sensor data
        scan = robot.get_lidar_scan()
        lat, lon = robot.get_gps_position()
        distance, delta_theta = robot.get_odometry(0.1)

        # Process LiDAR
        processed = lidar_processor.process(scan)

        # Detect obstacles
        obstacles = obstacle_detector.detect(processed)

        # Update fusion
        fusion.update_odometry(distance, delta_theta, 0.1)
        fusion.update_gps(lat, lon, accuracy=0.02, quality=4)

        # Get state
        state = fusion.get_state()

        # Print every 10 steps
        if step % 10 == 0:
            nearest = obstacle_detector.get_nearest_obstacle(obstacles)
            nearest_dist = nearest.min_distance if nearest else float('inf')

            print(f"Step {step:3d}: "
                  f"pos=({state.pose.x:5.2f}, {state.pose.y:5.2f}) "
                  f"heading={math.degrees(state.pose.theta):6.1f}° "
                  f"obstacles={len(obstacles):2d} "
                  f"nearest={nearest_dist:.2f}m")

        time.sleep(0.05)

    # Final status
    print("-" * 60)
    print("\nFinal sensor status:")
    for sensor_type, status in fusion.get_sensor_status().items():
        print(f"  {sensor_type.name}: healthy={status.is_healthy}, "
              f"rate={status.update_rate:.1f}Hz")

    print(f"\nLocalized: {fusion.is_localized()}")
    print("\nDemo complete!")


def run_demo_visual():
    """Run demo with matplotlib visualization."""
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available, running console demo...")
        run_demo_console()
        return

    print("\n" + "=" * 60)
    print("PERCEPTION DEMO (Visual Mode)")
    print("=" * 60)
    print("\nClose the window to exit.")

    # Initialize
    robot = SimulatedRobot()
    lidar_processor = LidarProcessor()
    obstacle_detector = ObstacleDetector()
    fusion = SensorFusion()
    fusion.set_gps_origin(*robot.gps_origin)

    # Setup figure
    fig = plt.figure(figsize=(14, 6))
    fig.suptitle('Robocar Perception Demo', fontsize=14)

    # World view (left)
    ax_world = fig.add_subplot(121)
    ax_world.set_title('World View')
    ax_world.set_xlim(-6, 6)
    ax_world.set_ylim(-5, 5)
    ax_world.set_aspect('equal')
    ax_world.grid(True, alpha=0.3)

    # Draw static obstacles
    for ox, oy, r in robot.obstacles:
        circle = plt.Circle((ox, oy), r, color='red', alpha=0.5)
        ax_world.add_patch(circle)

    # Draw walls
    for (x1, y1), (x2, y2) in robot.walls:
        ax_world.plot([x1, x2], [y1, y2], 'k-', linewidth=2)

    # Robot marker
    robot_marker, = ax_world.plot([], [], 'b^', markersize=15)
    robot_trail, = ax_world.plot([], [], 'b-', alpha=0.3, linewidth=1)

    # Heading indicator
    heading_line, = ax_world.plot([], [], 'b-', linewidth=2)

    # Robot frame view (right)
    ax_robot = fig.add_subplot(122)
    ax_robot.set_title('Robot View (LiDAR)')
    ax_robot.set_xlim(-5, 5)
    ax_robot.set_ylim(-5, 5)
    ax_robot.set_aspect('equal')
    ax_robot.grid(True, alpha=0.3)

    # LiDAR points
    lidar_scatter, = ax_robot.plot([], [], 'g.', markersize=2)

    # Detected obstacles
    obstacle_patches = []

    # Robot at origin
    ax_robot.plot([0], [0], 'b^', markersize=15)
    ax_robot.arrow(0, 0, 0.5, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue')

    # Status text
    status_text = ax_robot.text(
        0.02, 0.98, '', transform=ax_robot.transAxes,
        verticalalignment='top', fontfamily='monospace',
        fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    )

    # Trail data
    trail_x = []
    trail_y = []

    # Animation state
    state = {'time': 0}

    def update(frame):
        # Move robot in a pattern
        t = state['time']
        velocity = 0.3
        angular_velocity = 0.2 * math.sin(t * 0.5)
        robot.update(velocity, angular_velocity)
        state['time'] += 0.1

        # Get sensor data
        scan = robot.get_lidar_scan()
        lat, lon = robot.get_gps_position()
        distance, delta_theta = robot.get_odometry(0.1)

        # Process
        processed = lidar_processor.process(scan)
        obstacles = obstacle_detector.detect(processed)

        # Update fusion
        fusion.update_odometry(distance, delta_theta, 0.1)
        fusion.update_gps(lat, lon, accuracy=0.02, quality=4)
        robot_state = fusion.get_state()

        # Update world view
        robot_marker.set_data([robot.x], [robot.y])

        trail_x.append(robot.x)
        trail_y.append(robot.y)
        if len(trail_x) > 200:
            trail_x.pop(0)
            trail_y.pop(0)
        robot_trail.set_data(trail_x, trail_y)

        # Heading line
        hx = robot.x + 0.5 * math.cos(robot.theta)
        hy = robot.y + 0.5 * math.sin(robot.theta)
        heading_line.set_data([robot.x, hx], [robot.y, hy])

        # Update robot view
        valid_points = lidar_processor.get_valid_points(processed)
        if len(valid_points) > 0:
            lidar_scatter.set_data(valid_points[:, 0], valid_points[:, 1])

        # Clear old obstacle patches
        for patch in obstacle_patches:
            patch.remove()
        obstacle_patches.clear()

        # Draw detected obstacles
        for obs in obstacles:
            color = {
                ObstacleType.STATIC: 'orange',
                ObstacleType.DYNAMIC: 'red',
                ObstacleType.WALL: 'gray',
                ObstacleType.POLE: 'brown',
                ObstacleType.UNKNOWN: 'purple'
            }.get(obs.obstacle_type, 'purple')

            rect = patches.Rectangle(
                (obs.bbox.x_min, obs.bbox.y_min),
                obs.bbox.width, obs.bbox.height,
                linewidth=2, edgecolor=color, facecolor='none'
            )
            ax_robot.add_patch(rect)
            obstacle_patches.append(rect)

        # Update status
        nearest = obstacle_detector.get_nearest_obstacle(obstacles)
        nearest_dist = nearest.min_distance if nearest else float('inf')

        risk, dangerous, ttc = obstacle_detector.check_collision_risk(obstacles)

        status = (
            f"Position: ({robot_state.pose.x:.2f}, {robot_state.pose.y:.2f})\n"
            f"Heading: {math.degrees(robot_state.pose.theta):.1f}°\n"
            f"Speed: {robot_state.speed:.2f} m/s\n"
            f"─────────────────────\n"
            f"Obstacles: {len(obstacles)}\n"
            f"Nearest: {nearest_dist:.2f} m\n"
            f"Collision risk: {'YES' if risk else 'No'}"
        )
        if risk and dangerous:
            status += f"\n  TTC: {ttc:.1f}s"

        status_text.set_text(status)

        return (robot_marker, robot_trail, heading_line, lidar_scatter,
                status_text, *obstacle_patches)

    ani = FuncAnimation(fig, update, interval=100, blit=False)

    plt.tight_layout()
    plt.show()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Perception Demo')
    parser.add_argument('--no-viz', action='store_true',
                        help='Run without visualization')
    args = parser.parse_args()

    if args.no_viz or not MATPLOTLIB_AVAILABLE:
        run_demo_console()
    else:
        run_demo_visual()


if __name__ == '__main__':
    main()
