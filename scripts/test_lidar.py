#!/usr/bin/env python3
"""
LiDAR LD19 Test Script

Tests the LiDAR driver and visualizes scan data in real-time.
Requires: matplotlib (pip install matplotlib)

Usage:
    python scripts/test_lidar.py [port]
    python scripts/test_lidar.py /dev/ttyUSB0
"""

import sys
import time
import math
import argparse

# Add src to path (works from any directory)
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from driver.lidar import LidarDriver, scan_to_cartesian

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("matplotlib not installed. Install with: pip install matplotlib")


def test_basic(port: str):
    """Basic test without visualization."""
    print(f"\n[TEST] Testing LiDAR on {port}")
    print("=" * 50)

    lidar = LidarDriver(port)

    if not lidar.start():
        print("[ERROR] Failed to start LiDAR")
        return False

    print("[OK] LiDAR started")
    print("[INFO] Reading scans...")

    try:
        for i in range(10):
            scan = lidar.get_scan(timeout=1.0)
            if scan:
                valid_points = sum(1 for p in scan.points if p.valid)
                if valid_points > 0:
                    distances = [p.distance for p in scan.points if p.valid]
                    min_dist = min(distances)
                    max_dist = max(distances)
                    avg_dist = sum(distances) / len(distances)

                    print(f"[SCAN {i+1}] Points: {valid_points}, "
                          f"Distance: min={min_dist:.2f}m, max={max_dist:.2f}m, avg={avg_dist:.2f}m")
                else:
                    print(f"[SCAN {i+1}] No valid points")
            else:
                print(f"[SCAN {i+1}] Timeout - no data received")

            time.sleep(0.1)

        print("\n[OK] Basic test completed")
        return True

    finally:
        lidar.stop()
        print("[OK] LiDAR stopped")


def test_visualization(port: str):
    """Test with real-time visualization."""
    if not MATPLOTLIB_AVAILABLE:
        print("[ERROR] matplotlib required for visualization")
        return False

    print(f"\n[TEST] LiDAR Visualization on {port}")
    print("=" * 50)
    print("[INFO] Close the plot window to exit")

    lidar = LidarDriver(port)

    if not lidar.start():
        print("[ERROR] Failed to start LiDAR")
        return False

    # Setup plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('LiDAR LD19 - Real-time Scan', fontsize=14)

    # Polar plot (left)
    ax1 = plt.subplot(121, projection='polar')
    ax1.set_title('Polar View')
    ax1.set_ylim(0, 5)  # 5 meters max
    scatter1, = ax1.plot([], [], 'b.', markersize=2)

    # Cartesian plot (right)
    ax2 = plt.subplot(122)
    ax2.set_title('Top-Down View')
    ax2.set_xlim(-5, 5)
    ax2.set_ylim(-5, 5)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linewidth=0.5)
    ax2.axvline(x=0, color='k', linewidth=0.5)
    scatter2, = ax2.plot([], [], 'b.', markersize=2)
    robot_marker, = ax2.plot([0], [0], 'r^', markersize=15)  # Robot position

    # Stats text
    stats_text = ax2.text(0.02, 0.98, '', transform=ax2.transAxes,
                          verticalalignment='top', fontfamily='monospace',
                          fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    def update(frame):
        scan = lidar.get_latest_scan()
        if scan is None:
            return scatter1, scatter2, stats_text

        angles = []
        distances = []
        for point in scan.points:
            if point.valid:
                angles.append(point.angle)
                distances.append(point.distance)

        if not angles:
            return scatter1, scatter2, stats_text

        # Update polar plot
        scatter1.set_data(angles, distances)

        # Update Cartesian plot
        x = [d * math.cos(a) for a, d in zip(angles, distances)]
        y = [d * math.sin(a) for a, d in zip(angles, distances)]
        scatter2.set_data(x, y)

        # Update stats
        min_dist = min(distances)
        max_dist = max(distances)
        avg_dist = sum(distances) / len(distances)

        # Find closest obstacle direction
        min_idx = distances.index(min_dist)
        min_angle = math.degrees(angles[min_idx])

        stats = (f"Points: {len(distances)}\n"
                 f"Min: {min_dist:.2f}m @ {min_angle:.0f}°\n"
                 f"Max: {max_dist:.2f}m\n"
                 f"Avg: {avg_dist:.2f}m")
        stats_text.set_text(stats)

        return scatter1, scatter2, stats_text

    try:
        ani = animation.FuncAnimation(fig, update, interval=100, blit=True)
        plt.tight_layout()
        plt.show()
        return True
    finally:
        lidar.stop()
        print("[OK] LiDAR stopped")


def main():
    parser = argparse.ArgumentParser(description='Test LiDAR LD19')
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization')
    args = parser.parse_args()

    if args.no_viz or not MATPLOTLIB_AVAILABLE:
        test_basic(args.port)
    else:
        test_visualization(args.port)


if __name__ == '__main__':
    main()