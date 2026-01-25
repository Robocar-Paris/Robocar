#!/usr/bin/env python3
"""
GPS RTK Test Script

Tests the GPS driver and displays position data.
Optionally plots position history.

Usage:
    python scripts/test_gps.py [port]
    python scripts/test_gps.py /dev/ttyUSB1
"""

import sys
import time
import argparse

# Add src to path (works from any directory)
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(PROJECT_ROOT, 'src'))

from drivers.gps_pointone import PointOneGPS, gps_to_local

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def quality_color(quality: int) -> str:
    """Get color based on GPS quality."""
    colors = {
        0: '\033[91m',  # Red - No fix
        1: '\033[93m',  # Yellow - GPS
        2: '\033[93m',  # Yellow - DGPS
        4: '\033[92m',  # Green - RTK Fixed
        5: '\033[96m',  # Cyan - RTK Float
    }
    return colors.get(quality, '\033[0m')


def test_basic(port: str, duration: int = 30):
    """Basic test with console output."""
    print(f"\n[TEST] Testing GPS on {port}")
    print("=" * 60)

    gps = PointOneGPS(port)

    if not gps.start():
        print("[ERROR] Failed to start GPS")
        return False

    print("[OK] GPS started")
    print("[INFO] Waiting for position data...")
    print()
    print("Quality Legend: NO_FIX | GPS | DGPS | RTK_FLOAT | RTK_FIXED")
    print("-" * 60)

    origin = None
    RESET = '\033[0m'

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            pos = gps.get_position()

            if pos:
                if origin is None:
                    origin = pos
                    print(f"[ORIGIN] Set origin at ({pos.latitude:.6f}, {pos.longitude:.6f})")

                # Calculate local position
                local_x, local_y = gps_to_local(pos, origin)

                color = quality_color(pos.quality)
                print(f"{color}[{pos.quality_string:10}]{RESET} "
                      f"({pos.latitude:.6f}, {pos.longitude:.6f}) "
                      f"alt={pos.altitude:6.1f}m "
                      f"acc={pos.accuracy_h:5.2f}m "
                      f"sats={pos.satellites:2d} "
                      f"local=({local_x:6.2f}, {local_y:6.2f})m")
            else:
                print("[WAITING] No position data...")

            time.sleep(1.0)

        print("\n[OK] Test completed")
        return True

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
        return True

    finally:
        gps.stop()
        print("[OK] GPS stopped")


def test_visualization(port: str):
    """Test with real-time position plot."""
    if not MATPLOTLIB_AVAILABLE:
        print("[ERROR] matplotlib required for visualization")
        return False

    print(f"\n[TEST] GPS Visualization on {port}")
    print("=" * 50)
    print("[INFO] Close the plot window to exit")

    gps = PointOneGPS(port)

    if not gps.start():
        print("[ERROR] Failed to start GPS")
        return False

    # Wait for initial position
    print("[INFO] Waiting for initial GPS fix...")
    origin = None
    for _ in range(30):
        pos = gps.get_position()
        if pos and pos.quality > 0:
            origin = pos
            print(f"[OK] Got initial position: ({pos.latitude:.6f}, {pos.longitude:.6f})")
            break
        time.sleep(1.0)

    if origin is None:
        print("[ERROR] Could not get initial position")
        gps.stop()
        return False

    # Setup plot
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title('GPS Position - Local Coordinates')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Initialize plot elements
    path_line, = ax.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Path')
    current_pos, = ax.plot([], [], 'ro', markersize=10, label='Current')
    origin_marker, = ax.plot([0], [0], 'g*', markersize=15, label='Origin')
    ax.legend()

    # Data storage
    x_data = []
    y_data = []

    # Stats text
    stats_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                         verticalalignment='top', fontfamily='monospace',
                         fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    def update(frame):
        pos = gps.get_position()
        if pos is None or pos.quality == 0:
            return path_line, current_pos, stats_text

        x, y = gps_to_local(pos, origin)
        x_data.append(x)
        y_data.append(y)

        # Update path
        path_line.set_data(x_data, y_data)

        # Update current position
        current_pos.set_data([x], [y])

        # Update axis limits
        if len(x_data) > 1:
            margin = 5
            ax.set_xlim(min(x_data) - margin, max(x_data) + margin)
            ax.set_ylim(min(y_data) - margin, max(y_data) + margin)

        # Calculate distance from origin
        distance = (x**2 + y**2)**0.5

        # Update stats
        stats = (f"Quality: {pos.quality_string}\n"
                 f"Lat: {pos.latitude:.6f}\n"
                 f"Lon: {pos.longitude:.6f}\n"
                 f"Alt: {pos.altitude:.1f}m\n"
                 f"Accuracy: {pos.accuracy_h:.2f}m\n"
                 f"Satellites: {pos.satellites}\n"
                 f"Distance: {distance:.2f}m")
        stats_text.set_text(stats)

        return path_line, current_pos, stats_text

    try:
        ani = animation.FuncAnimation(fig, update, interval=200, blit=False)
        plt.tight_layout()
        plt.show()
        return True
    finally:
        gps.stop()
        print("[OK] GPS stopped")


def main():
    parser = argparse.ArgumentParser(description='Test GPS RTK')
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB1',
                        help='Serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--duration', type=int, default=30,
                        help='Test duration in seconds (default: 30)')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization')
    args = parser.parse_args()

    if args.no_viz or not MATPLOTLIB_AVAILABLE:
        test_basic(args.port, args.duration)
    else:
        test_visualization(args.port)


if __name__ == '__main__':
    main()