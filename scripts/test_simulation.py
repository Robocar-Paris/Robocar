#!/usr/bin/env python3
"""
Robocar Simulation Test - NO HARDWARE REQUIRED

Ce script teste l'ensemble du système en simulation :
- Drivers simulés (LiDAR, GPS, Moteur)
- Perception et détection d'obstacles
- SLAM et cartographie
- Fusion de capteurs

Usage:
    python scripts/test_simulation.py              # Test complet avec visualisation
    python scripts/test_simulation.py --no-viz     # Test en mode console
    python scripts/test_simulation.py --module slam      # Tester uniquement SLAM
    python scripts/test_simulation.py --module perception  # Tester uniquement perception
"""

import sys
import os
import time
import math
import argparse
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Add src to path (works from any directory)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
SRC_DIR = os.path.join(PROJECT_ROOT, 'src')
sys.path.insert(0, SRC_DIR)


# =============================================================================
# Simulated Sensors
# =============================================================================

@dataclass
class SimulatedLidarPoint:
    """Single LiDAR measurement point."""
    angle: float  # radians
    distance: float  # meters
    intensity: int = 200
    valid: bool = True


@dataclass
class SimulatedLidarScan:
    """Complete LiDAR scan."""
    points: List[SimulatedLidarPoint]
    timestamp: float
    scan_frequency: float = 10.0  # Hz


@dataclass
class SimulatedGPSPosition:
    """GPS position data."""
    latitude: float
    longitude: float
    altitude: float = 0.0
    quality: int = 4  # RTK Fixed
    satellites: int = 12
    accuracy_h: float = 0.02  # 2cm RTK accuracy
    quality_string: str = "RTK_FIXED"


class SimulatedWorld:
    """Virtual environment with walls and obstacles."""

    def __init__(self):
        # Room boundaries (10m x 10m)
        self.walls = [
            ((-5, -5), (-5, 5)),   # Left wall
            ((-5, 5), (5, 5)),     # Top wall
            ((5, 5), (5, -5)),     # Right wall
            ((5, -5), (-5, -5)),   # Bottom wall
            # Inner walls (corridor)
            ((-5, 0), (-2, 0)),
            ((2, 0), (5, 0)),
        ]

        # Circular obstacles (x, y, radius)
        self.obstacles = [
            (-3, 3, 0.4),
            (3, 2, 0.5),
            (0, -3, 0.3),
            (-2, -2, 0.35),
            (4, -3, 0.45),
        ]

        # GPS origin (Paris coordinates)
        self.gps_origin = (48.8566, 2.3522)

    def ray_cast(self, x: float, y: float, angle: float, max_range: float = 12.0) -> float:
        """Cast a ray and return distance to nearest obstacle."""
        min_dist = max_range
        dx = math.cos(angle)
        dy = math.sin(angle)

        # Check walls
        for (x1, y1), (x2, y2) in self.walls:
            dist = self._ray_line_intersection(x, y, dx, dy, x1, y1, x2, y2)
            if dist and dist < min_dist:
                min_dist = dist

        # Check circular obstacles
        for ox, oy, r in self.obstacles:
            dist = self._ray_circle_intersection(x, y, dx, dy, ox, oy, r)
            if dist and dist < min_dist:
                min_dist = dist

        return min_dist

    def _ray_line_intersection(self, x, y, dx, dy, x1, y1, x2, y2) -> Optional[float]:
        """Ray-line segment intersection."""
        lx, ly = x2 - x1, y2 - y1
        denom = dx * ly - dy * lx

        if abs(denom) < 1e-10:
            return None

        t = ((x1 - x) * ly - (y1 - y) * lx) / denom
        s = ((x1 - x) * dy - (y1 - y) * dx) / denom

        if t > 0.01 and 0 <= s <= 1:
            return t
        return None

    def _ray_circle_intersection(self, x, y, dx, dy, cx, cy, r) -> Optional[float]:
        """Ray-circle intersection."""
        fx, fy = x - cx, y - cy
        a = dx*dx + dy*dy
        b = 2 * (fx*dx + fy*dy)
        c = fx*fx + fy*fy - r*r

        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return None

        t1 = (-b - math.sqrt(discriminant)) / (2*a)
        if t1 > 0.01:
            return t1

        t2 = (-b + math.sqrt(discriminant)) / (2*a)
        if t2 > 0.01:
            return t2

        return None

    def get_lidar_scan(self, x: float, y: float, theta: float,
                       num_points: int = 360, noise_std: float = 0.02) -> SimulatedLidarScan:
        """Generate LiDAR scan from position."""
        points = []
        for i in range(num_points):
            # Angles from 0 to 2*pi (compatible with SLAM builtin)
            angle = (2 * math.pi * i / num_points)
            world_angle = theta + angle
            distance = self.ray_cast(x, y, world_angle)

            # Add noise
            distance += np.random.normal(0, noise_std)
            distance = max(0.05, min(12.0, distance))

            points.append(SimulatedLidarPoint(
                angle=angle,
                distance=distance,
                valid=True
            ))

        return SimulatedLidarScan(points=points, timestamp=time.time())

    def get_gps_position(self, x: float, y: float, noise_std: float = 0.02) -> SimulatedGPSPosition:
        """Generate GPS position from local coordinates."""
        # Add RTK-level noise (2cm)
        noisy_x = x + np.random.normal(0, noise_std)
        noisy_y = y + np.random.normal(0, noise_std)

        # Convert to GPS coordinates (simple approximation)
        meters_per_deg_lat = 111320
        meters_per_deg_lon = 111320 * math.cos(math.radians(self.gps_origin[0]))

        lat = self.gps_origin[0] + noisy_y / meters_per_deg_lat
        lon = self.gps_origin[1] + noisy_x / meters_per_deg_lon

        return SimulatedGPSPosition(latitude=lat, longitude=lon)


class SimulatedRobot:
    """Simulated robot with kinematics."""

    def __init__(self, world: SimulatedWorld):
        self.world = world
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.path_history = [(0.0, 0.0)]

    def update(self, velocity: float, angular_velocity: float, dt: float = 0.1):
        """Update robot position."""
        self.velocity = velocity
        self.angular_velocity = angular_velocity

        # Simple differential drive model
        self.theta += angular_velocity * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += velocity * math.cos(self.theta) * dt
        self.y += velocity * math.sin(self.theta) * dt

        self.path_history.append((self.x, self.y))
        if len(self.path_history) > 500:
            self.path_history.pop(0)

    def get_lidar_scan(self) -> SimulatedLidarScan:
        """Get LiDAR scan from current position."""
        return self.world.get_lidar_scan(self.x, self.y, self.theta)

    def get_gps_position(self) -> SimulatedGPSPosition:
        """Get GPS position."""
        return self.world.get_gps_position(self.x, self.y)

    def get_odometry(self, dt: float) -> Tuple[float, float]:
        """Get odometry (distance, delta_theta)."""
        return self.velocity * dt, self.angular_velocity * dt


# =============================================================================
# Test Functions
# =============================================================================

def test_slam_module(visualize: bool = True):
    """Test SLAM module with simulation."""
    print("\n" + "=" * 60)
    print("TEST: SLAM Module")
    print("=" * 60)

    try:
        from slam.slam_core import SLAM, SLAMConfig
        print("[OK] SLAM module imported")
    except ImportError as e:
        print(f"[FAIL] Cannot import SLAM: {e}")
        return False

    # Create world and robot
    world = SimulatedWorld()
    robot = SimulatedRobot(world)

    # Create SLAM
    config = SLAMConfig(
        map_size_pixels=200,
        map_size_meters=15.0,
        scan_size=360
    )
    slam = SLAM(config)
    print("[OK] SLAM initialized")

    # Simulate exploration
    print("[INFO] Running simulation...")
    errors = []

    for step in range(100):
        # Control pattern: move in expanding spiral
        t = step * 0.1
        robot.update(
            velocity=0.3,
            angular_velocity=0.3 * math.sin(t * 0.5),
            dt=0.1
        )

        # Get sensor data
        scan = robot.get_lidar_scan()
        ranges_mm = [int(p.distance * 1000) for p in scan.points]

        # Update SLAM
        dx, dtheta = robot.get_odometry(0.1)
        pose = slam.update(ranges_mm, velocity=(dx * 1000, math.degrees(dtheta), 0.1))

        # Calculate error
        error = math.sqrt((pose.x - robot.x)**2 + (pose.y - robot.y)**2)
        errors.append(error)

        if step % 25 == 0:
            print(f"  Step {step:3d}: True=({robot.x:5.2f}, {robot.y:5.2f}) "
                  f"Est=({pose.x:5.2f}, {pose.y:5.2f}) Error={error:.3f}m")

    # Results
    avg_error = sum(errors) / len(errors)
    max_error = max(errors)

    print(f"\n[RESULT] Average error: {avg_error:.3f}m")
    print(f"[RESULT] Maximum error: {max_error:.3f}m")

    # Get map stats
    map_data = slam.get_map()
    free = np.sum(map_data < 100)
    occupied = np.sum(map_data > 150)
    total = map_data.size

    print(f"[RESULT] Map: {100*free/total:.1f}% free, {100*occupied/total:.1f}% occupied")

    success = avg_error < 0.5  # Less than 50cm average error
    print(f"\n[{'PASS' if success else 'FAIL'}] SLAM test {'passed' if success else 'failed'}")

    return success


def test_perception_module(visualize: bool = True):
    """Test perception module with simulation."""
    print("\n" + "=" * 60)
    print("TEST: Perception Module")
    print("=" * 60)

    try:
        from perception.lidar_processor import LidarProcessor
        from perception.obstacle_detector import ObstacleDetector
        print("[OK] Perception modules imported")
    except ImportError as e:
        print(f"[FAIL] Cannot import perception: {e}")
        return False

    # Create world and robot
    world = SimulatedWorld()
    robot = SimulatedRobot(world)

    # Create perception pipeline
    processor = LidarProcessor()
    detector = ObstacleDetector()
    print("[OK] Perception pipeline initialized")

    # Run test
    print("[INFO] Running simulation...")
    total_obstacles = 0
    detection_counts = []

    for step in range(50):
        # Move robot
        t = step * 0.1
        robot.update(velocity=0.2, angular_velocity=0.15 * math.sin(t), dt=0.1)

        # Get and process LiDAR
        scan = robot.get_lidar_scan()

        # Process the scan directly (SimulatedLidarScan is compatible with LidarScan)
        processed = processor.process(scan)
        obstacles = detector.detect(processed)

        detection_counts.append(len(obstacles))
        total_obstacles += len(obstacles)

        if step % 10 == 0:
            nearest = detector.get_nearest_obstacle(obstacles)
            nearest_dist = nearest.min_distance if nearest else float('inf')
            print(f"  Step {step:3d}: Detected {len(obstacles)} obstacles, "
                  f"nearest at {nearest_dist:.2f}m")

    # Results
    avg_detections = sum(detection_counts) / len(detection_counts)
    print(f"\n[RESULT] Average obstacles detected: {avg_detections:.1f}")
    print(f"[RESULT] Total detections: {total_obstacles}")

    # We expect to detect obstacles consistently
    success = avg_detections > 2  # Should detect at least some obstacles
    print(f"\n[{'PASS' if success else 'FAIL'}] Perception test {'passed' if success else 'failed'}")

    return success


def test_sensor_fusion(visualize: bool = True):
    """Test sensor fusion with simulation."""
    print("\n" + "=" * 60)
    print("TEST: Sensor Fusion Module")
    print("=" * 60)

    try:
        from perception.sensor_fusion import SensorFusion
        print("[OK] Sensor fusion module imported")
    except ImportError as e:
        print(f"[FAIL] Cannot import sensor fusion: {e}")
        return False

    # Create world and robot
    world = SimulatedWorld()
    robot = SimulatedRobot(world)

    # Create fusion
    fusion = SensorFusion()
    fusion.set_gps_origin(*world.gps_origin)
    print("[OK] Sensor fusion initialized")

    # Run test
    print("[INFO] Running simulation...")
    position_errors = []

    for step in range(50):
        # Move robot
        robot.update(velocity=0.3, angular_velocity=0.1, dt=0.1)

        # Get sensor data
        gps = robot.get_gps_position()
        dx, dtheta = robot.get_odometry(0.1)

        # Update fusion
        fusion.update_odometry(dx, dtheta, 0.1)
        fusion.update_gps(gps.latitude, gps.longitude,
                         accuracy=gps.accuracy_h, quality=gps.quality)

        # Get state
        state = fusion.get_state()

        # Calculate error
        error = math.sqrt(
            (state.pose.x - robot.x)**2 +
            (state.pose.y - robot.y)**2
        )
        position_errors.append(error)

        if step % 10 == 0:
            print(f"  Step {step:3d}: True=({robot.x:5.2f}, {robot.y:5.2f}) "
                  f"Fused=({state.pose.x:5.2f}, {state.pose.y:5.2f}) "
                  f"Error={error:.3f}m")

    # Results
    avg_error = sum(position_errors) / len(position_errors)
    max_error = max(position_errors)

    print(f"\n[RESULT] Average position error: {avg_error:.3f}m")
    print(f"[RESULT] Maximum position error: {max_error:.3f}m")

    # Check sensor status
    print("\n[RESULT] Sensor status:")
    for sensor_type, status in fusion.get_sensor_status().items():
        print(f"  {sensor_type.name}: healthy={status.is_healthy}")

    success = avg_error < 0.3  # Should be within 30cm with RTK GPS
    print(f"\n[{'PASS' if success else 'FAIL'}] Sensor fusion test {'passed' if success else 'failed'}")

    return success


def test_full_system(visualize: bool = True):
    """Test complete system integration."""
    print("\n" + "=" * 60)
    print("TEST: Full System Integration")
    print("=" * 60)

    results = {
        'SLAM': test_slam_module(visualize=False),
        'Perception': test_perception_module(visualize=False),
        'Sensor Fusion': test_sensor_fusion(visualize=False),
    }

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    all_passed = True
    for module, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {module:20s}: [{status}]")
        if not passed:
            all_passed = False

    print("=" * 60)
    if all_passed:
        print("[SUCCESS] All tests passed!")
    else:
        print("[FAILURE] Some tests failed")

    return all_passed


def run_visual_demo():
    """Run visual demo with matplotlib."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.patches import Circle
    except ImportError:
        print("[ERROR] matplotlib required for visualization")
        print("Install with: pip install matplotlib")
        return

    print("\n" + "=" * 60)
    print("VISUAL SIMULATION DEMO")
    print("=" * 60)
    print("Close the window to exit\n")

    # Import modules
    try:
        from slam.slam_core import SLAM, SLAMConfig
        from perception.lidar_processor import LidarProcessor
        from perception.obstacle_detector import ObstacleDetector
    except ImportError as e:
        print(f"[ERROR] Cannot import modules: {e}")
        return

    # Create world and robot
    world = SimulatedWorld()
    robot = SimulatedRobot(world)

    # Create processing pipeline
    slam_config = SLAMConfig(map_size_pixels=200, map_size_meters=15.0, scan_size=360)
    slam = SLAM(slam_config)
    processor = LidarProcessor()
    detector = ObstacleDetector()

    # Setup figure
    fig = plt.figure(figsize=(16, 6))
    fig.suptitle('Robocar Simulation - No Hardware Required', fontsize=14)

    # World view
    ax_world = fig.add_subplot(131)
    ax_world.set_title('World (Ground Truth)')
    ax_world.set_xlim(-6, 6)
    ax_world.set_ylim(-6, 6)
    ax_world.set_aspect('equal')
    ax_world.grid(True, alpha=0.3)

    # Draw walls
    for (x1, y1), (x2, y2) in world.walls:
        ax_world.plot([x1, x2], [y1, y2], 'k-', linewidth=2)

    # Draw obstacles
    for ox, oy, r in world.obstacles:
        circle = Circle((ox, oy), r, color='red', alpha=0.5)
        ax_world.add_patch(circle)

    robot_marker, = ax_world.plot([], [], 'b^', markersize=12)
    robot_path, = ax_world.plot([], [], 'b-', alpha=0.3)
    robot_heading, = ax_world.plot([], [], 'b-', linewidth=2)

    # LiDAR view
    ax_lidar = fig.add_subplot(132, projection='polar')
    ax_lidar.set_title('LiDAR Scan')
    ax_lidar.set_ylim(0, 6)
    lidar_scatter, = ax_lidar.plot([], [], 'g.', markersize=2)

    # SLAM map
    ax_map = fig.add_subplot(133)
    ax_map.set_title('SLAM Map')
    map_img = ax_map.imshow(
        np.ones((200, 200)) * 127,
        cmap='gray', vmin=0, vmax=255,
        extent=[-7.5, 7.5, -7.5, 7.5],
        origin='lower'
    )
    est_marker, = ax_map.plot([], [], 'r^', markersize=10)

    # Status
    status_text = fig.text(0.02, 0.02, '', fontfamily='monospace', fontsize=9)

    state = {'step': 0}

    def update(frame):
        t = state['step'] * 0.1

        # Move robot
        robot.update(
            velocity=0.3,
            angular_velocity=0.3 * math.sin(t * 0.3) + 0.1 * math.cos(t * 0.7),
            dt=0.1
        )

        # Get sensor data
        scan = robot.get_lidar_scan()
        ranges_mm = [int(p.distance * 1000) for p in scan.points]
        angles = [p.angle for p in scan.points]
        ranges_m = [p.distance for p in scan.points]

        # Update SLAM
        dx, dtheta = robot.get_odometry(0.1)
        pose = slam.update(ranges_mm, velocity=(dx * 1000, math.degrees(dtheta), 0.1))

        # Process for obstacles
        processed = processor.process(scan)
        obstacles = detector.detect(processed)

        # Update world view
        robot_marker.set_data([robot.x], [robot.y])
        path_x = [p[0] for p in robot.path_history]
        path_y = [p[1] for p in robot.path_history]
        robot_path.set_data(path_x, path_y)

        hx = robot.x + 0.6 * math.cos(robot.theta)
        hy = robot.y + 0.6 * math.sin(robot.theta)
        robot_heading.set_data([robot.x, hx], [robot.y, hy])

        # Update LiDAR
        lidar_scatter.set_data(angles, ranges_m)

        # Update map
        map_data = slam.get_map()
        map_img.set_data(np.flipud(map_data))
        est_marker.set_data([pose.x], [pose.y])

        # Status
        error = math.sqrt((pose.x - robot.x)**2 + (pose.y - robot.y)**2)
        nearest = detector.get_nearest_obstacle(obstacles)
        nearest_dist = nearest.min_distance if nearest else float('inf')

        status_text.set_text(
            f"Step: {state['step']:4d} | "
            f"Position error: {error:.3f}m | "
            f"Obstacles: {len(obstacles)} | "
            f"Nearest: {nearest_dist:.2f}m"
        )

        state['step'] += 1
        return robot_marker, robot_path, robot_heading, lidar_scatter, map_img, est_marker, status_text

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.08)
    plt.show()


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Test Robocar system in simulation (no hardware required)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python test_simulation.py              # Full test with visualization
    python test_simulation.py --no-viz     # Console only
    python test_simulation.py --module slam    # Test SLAM only
    python test_simulation.py --module perception  # Test perception only
        """
    )
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization')
    parser.add_argument('--module', choices=['slam', 'perception', 'fusion', 'all'],
                        default='all', help='Module to test (default: all)')

    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  ROBOCAR SIMULATION TEST")
    print("  No hardware required - Testing software components")
    print("=" * 60)

    if args.module == 'slam':
        test_slam_module(visualize=not args.no_viz)
    elif args.module == 'perception':
        test_perception_module(visualize=not args.no_viz)
    elif args.module == 'fusion':
        test_sensor_fusion(visualize=not args.no_viz)
    elif args.module == 'all':
        if args.no_viz:
            test_full_system(visualize=False)
        else:
            # Run tests first, then visual demo
            success = test_full_system(visualize=False)
            if success:
                print("\n[INFO] All tests passed! Launching visual demo...")
                run_visual_demo()
            else:
                print("\n[INFO] Some tests failed. Fix issues before visual demo.")


if __name__ == '__main__':
    main()