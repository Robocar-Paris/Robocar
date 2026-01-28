#!/usr/bin/env python3
"""
Mapping Mode

Drive the robot manually (gamepad) while building a map.
Save the map for later autonomous navigation.

Usage:
    python scripts/run_mapping.py
    python scripts/run_mapping.py --save-dir ~/maps
    python scripts/run_mapping.py --lidar-port /dev/ttyUSB0

Steps:
    1. Drive the robot around the environment
    2. Press 'S' to save the map
    3. Press 'Q' or BACK button to quit
"""

import sys
import os
import time
import math
import argparse
import signal

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from drivers.lidar_ld19 import LD19Driver
from drivers.vesc_motor import VESCController
from perception.lidar_processor import LidarProcessor
from slam.slam_core import SLAM, SLAMConfig
from slam.occupancy_grid import OccupancyGrid
from core.safety import SafetyMonitor, SafetyConfig


class MappingSession:
    """Manages a mapping session."""

    def __init__(self, lidar_port: str, vesc_port: str, save_dir: str):
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

        # Initialize components
        print("[INIT] Starting mapping session...")

        # Drivers
        self.lidar = LD19Driver(lidar_port)
        self.vesc = VESCController(vesc_port)

        # Perception
        self.lidar_processor = LidarProcessor()

        # SLAM
        self.slam = SLAM(SLAMConfig(
            map_size_pixels=800,
            map_size_meters=40.0,
            scan_size=360
        ))

        # Occupancy grid (higher resolution for saving)
        self.grid = OccupancyGrid(
            width=800, height=800,
            resolution=0.05
        )

        # Safety
        self.safety = SafetyMonitor(
            config=SafetyConfig(emergency_stop_distance=0.15),
            emergency_callback=self._emergency_stop
        )

        # State
        self.running = False
        self.total_distance = 0.0
        self.scan_count = 0

    def start(self):
        """Start all sensors."""
        print("[INIT] Starting sensors...")

        if not self.lidar.start():
            print("[ERROR] Failed to start LiDAR!")
            return False

        if not self.vesc.start():
            print("[WARNING] VESC not available - mapping only (no motor)")

        self.safety.start_watchdog()
        self.running = True

        print("[OK] Mapping session started!")
        print("[INFO] Drive the robot around to build the map")
        print("[INFO] Press Ctrl+C to stop and save")

        return True

    def update(self):
        """Main update loop iteration."""
        # Get LiDAR scan
        scan = self.lidar.get_latest_scan()
        if scan is None:
            return

        self.scan_count += 1

        # Process scan
        processed = self.lidar_processor.process(scan)

        # Get odometry
        total_dist, velocity = self.vesc.get_odometry()
        self.total_distance = total_dist

        # Update SLAM
        scan_mm = [int(p.distance * 1000) for p in scan.points]
        pose = self.slam.update(scan_mm)

        # Update occupancy grid
        import numpy as np
        valid = processed.valid_mask
        self.grid.update_from_scan(
            pose.x, pose.y, pose.theta,
            processed.distances[valid],
            processed.angles[valid]
        )

        # Safety check
        nearest_dist, _ = self.lidar_processor.find_nearest_obstacle(processed)
        self.safety.check_and_limit(0, nearest_dist)
        self.safety.update_sensor_time()

        # Print status periodically
        if self.scan_count % 50 == 0:
            map_data = self.grid.get_map_data()
            import numpy as np
            explored = np.sum(map_data != 0.5) / map_data.size * 100
            print(f"[MAP] Scans: {self.scan_count} | "
                  f"Pos: ({pose.x:.2f}, {pose.y:.2f}) | "
                  f"Dist: {self.total_distance:.1f}m | "
                  f"Explored: {explored:.1f}%")

    def save_map(self, name: str = "map"):
        """Save the current map."""
        img_path = os.path.join(self.save_dir, f"{name}.png")
        yaml_path = os.path.join(self.save_dir, f"{name}.yaml")

        self.grid.save(img_path, yaml_path)

        # Also save SLAM map
        slam_path = os.path.join(self.save_dir, f"{name}_slam.png")
        slam_map = self.slam.get_map()
        try:
            import cv2
            cv2.imwrite(slam_path, slam_map)
        except ImportError:
            pass

        print(f"[SAVED] Map saved to {img_path}")
        print(f"[SAVED] Metadata saved to {yaml_path}")

    def stop(self):
        """Stop session and cleanup."""
        self.running = False
        self.safety.stop_watchdog()

        if self.vesc.is_running:
            self.vesc.emergency_stop()
            self.vesc.stop()

        if self.lidar.is_running:
            self.lidar.stop()

        print("[STOP] Session ended")

    def _emergency_stop(self):
        """Emergency stop callback."""
        print("[EMERGENCY] Safety system triggered emergency stop!")
        if self.vesc.is_running:
            self.vesc.emergency_stop()

    def run(self):
        """Main loop."""
        if not self.start():
            return

        try:
            while self.running:
                self.update()
                time.sleep(0.05)  # 20 Hz

        except KeyboardInterrupt:
            print("\n[INFO] Stopping...")

        finally:
            # Auto-save map on exit
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.save_map(f"map_{timestamp}")
            self.stop()

            print(f"\n[SUMMARY]")
            print(f"  Total scans: {self.scan_count}")
            print(f"  Total distance: {self.total_distance:.1f}m")


def main():
    parser = argparse.ArgumentParser(description='Mapping Mode')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0')
    parser.add_argument('--vesc-port', default='/dev/ttyACM0')
    parser.add_argument('--save-dir', default='config/maps')
    args = parser.parse_args()

    print("=" * 50)
    print("ROBOCAR - MAPPING MODE")
    print("=" * 50)

    session = MappingSession(
        lidar_port=args.lidar_port,
        vesc_port=args.vesc_port,
        save_dir=args.save_dir
    )
    session.run()


if __name__ == '__main__':
    main()