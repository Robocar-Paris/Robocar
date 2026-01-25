#!/usr/bin/env python3
"""
Test All Sensors

Simultaneously tests LiDAR, GPS, and VESC to verify hardware setup.
Displays combined visualization of all sensor data.

Usage:
    python scripts/test_all_sensors.py
"""

import sys
import time
import math
import threading
import argparse

# Add src to path
sys.path.insert(0, '/home/user/Robocar/src')

from drivers.lidar_ld19 import LD19Driver
from drivers.gps_pointone import PointOneGPS, gps_to_local
from drivers.vesc_motor import VESCController

try:
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class SensorTester:
    """Combined sensor testing class."""

    def __init__(
        self,
        lidar_port: str = '/dev/ttyUSB0',
        gps_port: str = '/dev/ttyUSB1',
        vesc_port: str = '/dev/ttyACM0'
    ):
        self.lidar_port = lidar_port
        self.gps_port = gps_port
        self.vesc_port = vesc_port

        self.lidar = None
        self.gps = None
        self.vesc = None

        self.gps_origin = None
        self.running = False

    def start(self) -> dict:
        """
        Start all sensors.

        Returns:
            Dict with status of each sensor
        """
        status = {
            'lidar': False,
            'gps': False,
            'vesc': False
        }

        # Start LiDAR
        print(f"[LIDAR] Starting on {self.lidar_port}...")
        self.lidar = LD19Driver(self.lidar_port)
        if self.lidar.start():
            print("[LIDAR] OK")
            status['lidar'] = True
        else:
            print("[LIDAR] FAILED")

        # Start GPS
        print(f"[GPS] Starting on {self.gps_port}...")
        self.gps = PointOneGPS(self.gps_port)
        if self.gps.start():
            print("[GPS] OK")
            status['gps'] = True
        else:
            print("[GPS] FAILED")

        # Start VESC
        print(f"[VESC] Starting on {self.vesc_port}...")
        self.vesc = VESCController(self.vesc_port)
        if self.vesc.start():
            print("[VESC] OK")
            status['vesc'] = True
        else:
            print("[VESC] FAILED")

        self.running = True
        return status

    def stop(self):
        """Stop all sensors."""
        self.running = False

        if self.lidar:
            self.lidar.stop()
        if self.gps:
            self.gps.stop()
        if self.vesc:
            self.vesc.stop()

        print("[ALL] Sensors stopped")

    def get_status(self) -> dict:
        """Get current status of all sensors."""
        status = {}

        # LiDAR status
        if self.lidar and self.lidar.is_running:
            scan = self.lidar.get_latest_scan()
            if scan:
                valid_points = sum(1 for p in scan.points if p.valid)
                status['lidar'] = {
                    'ok': True,
                    'points': valid_points,
                    'timestamp': scan.timestamp
                }
            else:
                status['lidar'] = {'ok': False, 'error': 'No scan data'}
        else:
            status['lidar'] = {'ok': False, 'error': 'Not running'}

        # GPS status
        if self.gps and self.gps.is_running:
            pos = self.gps.get_position()
            if pos:
                status['gps'] = {
                    'ok': True,
                    'quality': pos.quality_string,
                    'lat': pos.latitude,
                    'lon': pos.longitude,
                    'accuracy': pos.accuracy_h,
                    'satellites': pos.satellites
                }
            else:
                status['gps'] = {'ok': False, 'error': 'No position'}
        else:
            status['gps'] = {'ok': False, 'error': 'Not running'}

        # VESC status
        if self.vesc and self.vesc.is_running:
            state = self.vesc.get_state()
            if state:
                status['vesc'] = {
                    'ok': True,
                    'voltage': state.voltage,
                    'current': state.current,
                    'rpm': state.rpm,
                    'temp': state.temp_mos
                }
            else:
                status['vesc'] = {'ok': True, 'warning': 'No state data'}
        else:
            status['vesc'] = {'ok': False, 'error': 'Not running'}

        return status


def test_console(tester: SensorTester, duration: int = 30):
    """Console-based test output."""
    print("\n" + "=" * 70)
    print("SENSOR STATUS MONITOR")
    print("=" * 70)
    print("Press Ctrl+C to stop\n")

    try:
        start_time = time.time()
        while time.time() - start_time < duration:
            status = tester.get_status()

            # Clear line and print status
            print("\033[2J\033[H")  # Clear screen
            print("=" * 70)
            print(f"ROBOCAR SENSOR STATUS - {time.strftime('%H:%M:%S')}")
            print("=" * 70)

            # LiDAR
            lidar = status.get('lidar', {})
            if lidar.get('ok'):
                print(f"[LIDAR]  OK    | Points: {lidar['points']:4d}")
            else:
                print(f"[LIDAR]  FAIL  | {lidar.get('error', 'Unknown')}")

            # GPS
            gps = status.get('gps', {})
            if gps.get('ok'):
                quality_color = '\033[92m' if gps['quality'] == 'RTK_FIXED' else '\033[93m'
                print(f"[GPS]    OK    | {quality_color}{gps['quality']:10}\033[0m | "
                      f"Acc: {gps['accuracy']:.2f}m | Sats: {gps['satellites']}")
            else:
                print(f"[GPS]    FAIL  | {gps.get('error', 'Unknown')}")

            # VESC
            vesc = status.get('vesc', {})
            if vesc.get('ok'):
                voltage = vesc.get('voltage', 0)
                voltage_color = '\033[92m' if voltage > 10 else '\033[91m'
                print(f"[VESC]   OK    | {voltage_color}Voltage: {voltage:.1f}V\033[0m | "
                      f"Temp: {vesc.get('temp', 0):.1f}C")
            else:
                print(f"[VESC]   FAIL  | {vesc.get('error', 'Unknown')}")

            print("-" * 70)
            print(f"Running for {int(time.time() - start_time)}s / {duration}s")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")


def test_visualization(tester: SensorTester):
    """Matplotlib visualization."""
    if not MATPLOTLIB_AVAILABLE:
        print("[ERROR] matplotlib required for visualization")
        return

    fig = plt.figure(figsize=(16, 9))
    fig.suptitle('Robocar Sensor Dashboard', fontsize=14)

    gs = GridSpec(2, 3, figure=fig)

    # LiDAR polar plot
    ax_lidar = fig.add_subplot(gs[0, 0], projection='polar')
    ax_lidar.set_title('LiDAR (Polar)')
    ax_lidar.set_ylim(0, 5)
    lidar_scatter, = ax_lidar.plot([], [], 'b.', markersize=2)

    # LiDAR cartesian plot
    ax_lidar2 = fig.add_subplot(gs[0, 1])
    ax_lidar2.set_title('LiDAR (Top-Down)')
    ax_lidar2.set_xlim(-5, 5)
    ax_lidar2.set_ylim(-5, 5)
    ax_lidar2.set_aspect('equal')
    ax_lidar2.grid(True, alpha=0.3)
    lidar_scatter2, = ax_lidar2.plot([], [], 'b.', markersize=2)
    ax_lidar2.plot([0], [0], 'r^', markersize=12)  # Robot

    # GPS plot
    ax_gps = fig.add_subplot(gs[0, 2])
    ax_gps.set_title('GPS Track')
    ax_gps.set_xlabel('East (m)')
    ax_gps.set_ylabel('North (m)')
    ax_gps.grid(True, alpha=0.3)
    ax_gps.set_aspect('equal')
    gps_path, = ax_gps.plot([], [], 'b-', linewidth=1, alpha=0.5)
    gps_current, = ax_gps.plot([], [], 'ro', markersize=10)
    gps_origin, = ax_gps.plot([0], [0], 'g*', markersize=15)

    # Status panel
    ax_status = fig.add_subplot(gs[1, :])
    ax_status.set_xlim(0, 1)
    ax_status.set_ylim(0, 1)
    ax_status.axis('off')
    status_text = ax_status.text(0.02, 0.95, '', transform=ax_status.transAxes,
                                  verticalalignment='top', fontfamily='monospace',
                                  fontsize=11, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Data storage
    gps_x_data = []
    gps_y_data = []
    gps_origin_pos = [None]

    def update(frame):
        status = tester.get_status()

        # Update LiDAR
        if tester.lidar and tester.lidar.is_running:
            scan = tester.lidar.get_latest_scan()
            if scan:
                angles = [p.angle for p in scan.points if p.valid]
                distances = [p.distance for p in scan.points if p.valid]
                lidar_scatter.set_data(angles, distances)

                x = [d * math.cos(a) for a, d in zip(angles, distances)]
                y = [d * math.sin(a) for a, d in zip(angles, distances)]
                lidar_scatter2.set_data(x, y)

        # Update GPS
        if tester.gps and tester.gps.is_running:
            pos = tester.gps.get_position()
            if pos and pos.quality > 0:
                if gps_origin_pos[0] is None:
                    gps_origin_pos[0] = pos

                x, y = gps_to_local(pos, gps_origin_pos[0])
                gps_x_data.append(x)
                gps_y_data.append(y)

                gps_path.set_data(gps_x_data, gps_y_data)
                gps_current.set_data([x], [y])

                if len(gps_x_data) > 1:
                    margin = 5
                    ax_gps.set_xlim(min(gps_x_data) - margin, max(gps_x_data) + margin)
                    ax_gps.set_ylim(min(gps_y_data) - margin, max(gps_y_data) + margin)

        # Build status text
        lines = []
        lines.append(f"{'='*60}")
        lines.append(f"ROBOCAR SENSOR STATUS - {time.strftime('%H:%M:%S')}")
        lines.append(f"{'='*60}")

        lidar_status = status.get('lidar', {})
        gps_status = status.get('gps', {})
        vesc_status = status.get('vesc', {})

        if lidar_status.get('ok'):
            lines.append(f"[LIDAR]  OK    | Points: {lidar_status['points']}")
        else:
            lines.append(f"[LIDAR]  FAIL  | {lidar_status.get('error', 'Unknown')}")

        if gps_status.get('ok'):
            lines.append(f"[GPS]    OK    | {gps_status['quality']:10} | "
                        f"Acc: {gps_status['accuracy']:.2f}m | Sats: {gps_status['satellites']}")
        else:
            lines.append(f"[GPS]    FAIL  | {gps_status.get('error', 'Unknown')}")

        if vesc_status.get('ok'):
            lines.append(f"[VESC]   OK    | Voltage: {vesc_status.get('voltage', 0):.1f}V | "
                        f"Temp: {vesc_status.get('temp', 0):.1f}C | RPM: {vesc_status.get('rpm', 0):.0f}")
        else:
            lines.append(f"[VESC]   FAIL  | {vesc_status.get('error', 'Unknown')}")

        status_text.set_text('\n'.join(lines))

        return lidar_scatter, lidar_scatter2, gps_path, gps_current, status_text

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Test all sensors')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0',
                        help='LiDAR port (default: /dev/ttyUSB0)')
    parser.add_argument('--gps-port', default='/dev/ttyUSB1',
                        help='GPS port (default: /dev/ttyUSB1)')
    parser.add_argument('--vesc-port', default='/dev/ttyACM0',
                        help='VESC port (default: /dev/ttyACM0)')
    parser.add_argument('--duration', type=int, default=60,
                        help='Test duration in seconds (default: 60)')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization')
    args = parser.parse_args()

    tester = SensorTester(
        lidar_port=args.lidar_port,
        gps_port=args.gps_port,
        vesc_port=args.vesc_port
    )

    print("\n" + "=" * 50)
    print("ROBOCAR SENSOR TEST")
    print("=" * 50)

    status = tester.start()

    working_count = sum(1 for v in status.values() if v)
    print(f"\n[RESULT] {working_count}/3 sensors working")

    if working_count == 0:
        print("[ERROR] No sensors available. Check connections.")
        return

    try:
        if args.no_viz or not MATPLOTLIB_AVAILABLE:
            test_console(tester, args.duration)
        else:
            test_visualization(tester)
    finally:
        tester.stop()


if __name__ == '__main__':
    main()
