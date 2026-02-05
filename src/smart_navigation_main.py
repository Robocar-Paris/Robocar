#!/usr/bin/env python3
"""
Smart Navigation Main Script for Robocar

Navigation from Point A to Point B using GPS + LiDAR (no camera).

Uses:
- GPS RTK for positioning and target navigation
- LiDAR LD19 for obstacle detection and avoidance
- VESC for motor control

Usage:
    python3 smart_navigation_main.py --target-lat 48.8157 --target-lon 2.3632
"""

import sys
import os
import time
import argparse
import signal

# Add src directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import drivers
from driver.vesc_motor import VESCController
from driver.gps_rtk import GPSRTKDriver, load_polaris_config
from driver.lidar import LidarDriver

# Import navigation
from navigation.smart_navigation import SmartNavigator, NavigationConfig

# Import safety
from core.safety import SafetyMonitor, SafetyConfig


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Robocar GPS Navigation (Point A to B)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Target GPS coordinates
    parser.add_argument('--target-lat', type=float, required=True,
                       help='Target latitude (degrees)')
    parser.add_argument('--target-lon', type=float, required=True,
                       help='Target longitude (degrees)')

    # Hardware ports
    parser.add_argument('--vesc-port', type=str, default='/dev/ttyACM0',
                       help='VESC serial port')
    parser.add_argument('--gps-port', type=str, default='/dev/ttyUSB0',
                       help='GPS serial port')
    parser.add_argument('--lidar-port', type=str, default='/dev/ttyUSB1',
                       help='LiDAR serial port')

    # Configuration
    parser.add_argument('--arrival-radius', type=float, default=1.0,
                       help='Arrival radius in meters')
    parser.add_argument('--cruise-speed', type=float, default=0.4,
                       help='Cruise speed (0-1)')
    parser.add_argument('--timeout', type=float, default=300.0,
                       help='Navigation timeout in seconds')

    # Polaris RTK
    parser.add_argument('--polaris-key', type=str, default=None,
                       help='Polaris API key for RTK corrections')
    parser.add_argument('--polaris-config', type=str,
                       default='config/gps.yaml',
                       help='Path to GPS config file with Polaris key')

    # Debug
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Verbose output')

    return parser.parse_args()


class RobocarNavigator:
    """
    Main Robocar navigation application.

    Uses GPS + LiDAR for navigation from point A to point B.
    """

    def __init__(self, args):
        """Initialize all components."""
        self.args = args
        self.running = False

        # Components
        self.vesc = None
        self.gps = None
        self.lidar = None
        self.safety = None
        self.navigator = None

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def start(self) -> bool:
        """Start all components."""
        print("=" * 60)
        print("       ROBOCAR GPS NAVIGATION")
        print("       (Point A to Point B)")
        print("=" * 60)
        print()

        # === 1. VESC Motor Controller ===
        print("[1/5] Initializing VESC motor controller...")
        self.vesc = VESCController(port=self.args.vesc_port)
        if not self.vesc.start():
            print("[ERROR] Failed to start VESC controller")
            return False
        print(f"      VESC connected on {self.args.vesc_port}")

        # === 2. GPS RTK Driver ===
        print("[2/5] Initializing GPS RTK driver...")

        # Load Polaris API key
        polaris_key = self.args.polaris_key
        if not polaris_key and os.path.exists(self.args.polaris_config):
            polaris_key = load_polaris_config(self.args.polaris_config)

        self.gps = GPSRTKDriver(
            port=self.args.gps_port,
            polaris_api_key=polaris_key
        )

        if not self.gps.start(auto_detect=True):
            print("[ERROR] Failed to start GPS driver")
            self._cleanup()
            return False

        # Wait for GPS fix
        print("      Waiting for GPS fix...")
        if not self.gps.wait_for_fix(timeout=30.0):
            print("[WARNING] No GPS fix after 30s, continuing anyway...")
        else:
            pos = self.gps.get_position()
            print(f"      GPS fix: {pos.quality_string} "
                  f"({pos.latitude:.6f}, {pos.longitude:.6f})")

        # === 3. LiDAR Driver ===
        print("[3/5] Initializing LiDAR LD19...")
        self.lidar = LidarDriver(port=self.args.lidar_port)
        if not self.lidar.start():
            print("[WARNING] LiDAR not available - obstacle avoidance disabled")
        else:
            print(f"      LiDAR connected on {self.args.lidar_port}")

        # === 4. Safety Monitor ===
        print("[4/5] Initializing safety monitor...")
        safety_config = SafetyConfig(
            emergency_stop_distance=0.15,   # 15cm hard stop
            critical_distance=0.3,          # 30cm stop
            warning_distance=0.5,           # 50cm slow
            watchdog_timeout=0.5
        )
        self.safety = SafetyMonitor(
            config=safety_config,
            emergency_callback=self._emergency_stop
        )
        self.safety.start_watchdog()
        print("      Safety monitor active (emergency stop at 30cm)")

        # === 5. Smart Navigator ===
        print("[5/5] Initializing GPS navigator...")
        nav_config = NavigationConfig(
            arrival_radius_m=self.args.arrival_radius,
            cruise_speed=self.args.cruise_speed,
            warning_distance_m=1.5,
            critical_distance_m=0.5
        )
        self.navigator = SmartNavigator(
            vesc=self.vesc,
            gps=self.gps,
            lidar=self.lidar,
            config=nav_config,
            safety_monitor=self.safety
        )
        self.navigator.set_arrival_callback(self._on_arrival)
        print(f"      Target: ({self.args.target_lat:.6f}, {self.args.target_lon:.6f})")
        print(f"      Arrival radius: {self.args.arrival_radius}m")

        print()
        print("=" * 60)
        print("       ALL SYSTEMS READY")
        print("=" * 60)
        print()

        self.running = True
        return True

    def navigate(self) -> bool:
        """Start navigation to target."""
        if not self.running:
            print("[ERROR] System not started")
            return False

        print(f"[NAV] Starting navigation to target...")
        print(f"      Target: ({self.args.target_lat:.6f}, {self.args.target_lon:.6f})")
        print()

        # Start navigation (blocking)
        success = self.navigator.start_navigation(
            target_lat=self.args.target_lat,
            target_lon=self.args.target_lon,
            blocking=True,
            timeout_s=self.args.timeout
        )

        return success

    def stop(self):
        """Stop all components."""
        print("\n[SHUTDOWN] Stopping all components...")
        self.running = False

        if self.navigator:
            self.navigator.stop_navigation()

        if self.safety:
            self.safety.stop_watchdog()

        if self.lidar:
            self.lidar.stop()

        if self.gps:
            self.gps.stop()

        if self.vesc:
            self.vesc.stop()

        print("[SHUTDOWN] Complete")

    def _cleanup(self):
        """Cleanup on error."""
        if self.vesc:
            self.vesc.stop()
        if self.gps:
            self.gps.stop()
        if self.lidar:
            self.lidar.stop()

    def _emergency_stop(self):
        """Emergency stop callback."""
        print("\n" + "!" * 60)
        print("       EMERGENCY STOP TRIGGERED")
        print("!" * 60)

        if self.vesc:
            self.vesc.brake(25.0)
            self.vesc.set_speed(0)

    def _on_arrival(self):
        """Callback when arrived at target."""
        print("\n" + "*" * 60)
        print("       DESTINATION REACHED!")
        print("*" * 60)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print(f"\n[SIGNAL] Received signal {signum}")
        self.stop()
        sys.exit(0)


def main():
    """Main entry point."""
    args = parse_args()

    print()
    print("ROBOCAR GPS Navigation")
    print("-" * 40)
    print(f"Target:    ({args.target_lat:.6f}, {args.target_lon:.6f})")
    print(f"Arrival:   {args.arrival_radius}m radius")
    print(f"Speed:     {args.cruise_speed}")
    print(f"Timeout:   {args.timeout}s")
    print("-" * 40)
    print()

    # Create and start navigator
    robocar = RobocarNavigator(args)

    if not robocar.start():
        print("[FATAL] Failed to initialize system")
        sys.exit(1)

    success = False
    try:
        # Run navigation
        success = robocar.navigate()

        if success:
            print("\n[RESULT] Navigation completed successfully!")
        else:
            print("\n[RESULT] Navigation failed or timed out")

    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        robocar.stop()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
