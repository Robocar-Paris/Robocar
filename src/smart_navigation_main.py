#!/usr/bin/env python3
"""
Smart Navigation Main Script for Robocar

This script demonstrates the complete autonomous navigation system with:
- OAK-D Lite camera for obstacle detection
- GPS RTK for positioning
- VESC motor control
- Flask web dashboard for monitoring

Usage:
    # On Jetson Nano:
    python3 smart_navigation_main.py --target-lat 48.8157 --target-lon 2.3632

    # Then open browser on PC:
    http://<jetson-ip>:5000

Requirements:
    - depthai (OAK-D camera)
    - flask (web dashboard)
    - opencv-python
    - numpy
    - pyserial
    - pyvesc
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
from driver.oak_d_camera import OakDCamera, CameraConfig

# Import navigation
from navigation.smart_navigation import SmartNavigator, NavigationConfig

# Import safety
from core.safety import SafetyMonitor, SafetyConfig

# Import visualization
from visualization.web_dashboard import WebDashboard


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Robocar Smart Navigation',
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

    # Configuration
    parser.add_argument('--arrival-radius', type=float, default=1.0,
                       help='Arrival radius in meters')
    parser.add_argument('--cruise-speed', type=float, default=0.4,
                       help='Cruise speed (0-1)')
    parser.add_argument('--timeout', type=float, default=300.0,
                       help='Navigation timeout in seconds')

    # Dashboard
    parser.add_argument('--dashboard-port', type=int, default=5000,
                       help='Web dashboard port')
    parser.add_argument('--no-dashboard', action='store_true',
                       help='Disable web dashboard')

    # Polaris RTK
    parser.add_argument('--polaris-key', type=str, default=None,
                       help='Polaris API key for RTK corrections')
    parser.add_argument('--polaris-config', type=str,
                       default='config/gps.yaml',
                       help='Path to GPS config file with Polaris key')

    # Debug
    parser.add_argument('--simulate', action='store_true',
                       help='Run in simulation mode (no hardware)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Verbose output')

    return parser.parse_args()


class RobocarNavigator:
    """
    Main Robocar navigation application.

    Integrates all components:
    - VESC motor controller
    - GPS RTK driver
    - OAK-D camera
    - Safety monitor
    - Smart navigator
    - Web dashboard
    """

    def __init__(self, args):
        """Initialize all components."""
        self.args = args
        self.running = False

        # Components (initialized in start())
        self.vesc = None
        self.gps = None
        self.camera = None
        self.safety = None
        self.navigator = None
        self.dashboard = None

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def start(self) -> bool:
        """
        Start all components.

        Returns:
            True if all components started successfully
        """
        print("=" * 60)
        print("       ROBOCAR SMART NAVIGATION SYSTEM")
        print("=" * 60)
        print()

        # === 1. VESC Motor Controller ===
        print("[1/6] Initializing VESC motor controller...")
        self.vesc = VESCController(port=self.args.vesc_port)
        if not self.args.simulate:
            if not self.vesc.start():
                print("[ERROR] Failed to start VESC controller")
                return False
        print(f"      VESC connected on {self.args.vesc_port}")

        # === 2. GPS RTK Driver ===
        print("[2/6] Initializing GPS RTK driver...")

        # Load Polaris API key
        polaris_key = self.args.polaris_key
        if not polaris_key and os.path.exists(self.args.polaris_config):
            polaris_key = load_polaris_config(self.args.polaris_config)

        self.gps = GPSRTKDriver(
            port=self.args.gps_port,
            polaris_api_key=polaris_key
        )

        if not self.args.simulate:
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

        # === 3. OAK-D Camera ===
        print("[3/6] Initializing OAK-D Lite camera...")
        camera_config = CameraConfig(
            fps=30,
            warning_distance_m=1.5,
            critical_distance_m=0.5
        )
        self.camera = OakDCamera(config=camera_config)

        if not self.camera.start():
            print("[WARNING] OAK-D camera not available, using simulation")
        else:
            print(f"      Camera: RGB {self.camera.rgb_width}x{self.camera.rgb_height}, "
                  f"Depth {self.camera.depth_width}x{self.camera.depth_height}")

        # === 4. Safety Monitor ===
        print("[4/6] Initializing safety monitor...")
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
        print("[5/6] Initializing smart navigator...")
        nav_config = NavigationConfig(
            arrival_radius_m=self.args.arrival_radius,
            cruise_speed=self.args.cruise_speed,
            warning_distance_m=1.5,
            critical_distance_m=0.5
        )
        self.navigator = SmartNavigator(
            vesc=self.vesc,
            gps=self.gps,
            camera=self.camera,
            config=nav_config,
            safety_monitor=self.safety
        )
        self.navigator.set_arrival_callback(self._on_arrival)
        print(f"      Target: ({self.args.target_lat:.6f}, {self.args.target_lon:.6f})")
        print(f"      Arrival radius: {self.args.arrival_radius}m")

        # === 6. Web Dashboard ===
        if not self.args.no_dashboard:
            print("[6/6] Starting web dashboard...")
            self.dashboard = WebDashboard(
                camera=self.camera,
                navigator=self.navigator
            )
            if self.dashboard.start(port=self.args.dashboard_port):
                print(f"      Dashboard: http://0.0.0.0:{self.args.dashboard_port}")
            else:
                print("[WARNING] Failed to start dashboard")
        else:
            print("[6/6] Web dashboard disabled")

        print()
        print("=" * 60)
        print("       ALL SYSTEMS READY")
        print("=" * 60)
        print()

        self.running = True
        return True

    def navigate(self) -> bool:
        """
        Start navigation to target.

        Returns:
            True if arrived successfully
        """
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

        if self.dashboard:
            self.dashboard.stop()

        if self.safety:
            self.safety.stop_watchdog()

        if self.camera:
            self.camera.stop()

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

    def _emergency_stop(self):
        """Emergency stop callback."""
        print("\n" + "!" * 60)
        print("       EMERGENCY STOP TRIGGERED")
        print("!" * 60)

        if self.vesc:
            self.vesc.brake(25.0)  # Strong brake
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
    print("ROBOCAR Smart Navigation")
    print("-" * 40)
    print(f"Target:    ({args.target_lat:.6f}, {args.target_lon:.6f})")
    print(f"Arrival:   {args.arrival_radius}m radius")
    print(f"Speed:     {args.cruise_speed}")
    print(f"Timeout:   {args.timeout}s")
    print(f"Dashboard: {'Disabled' if args.no_dashboard else f'Port {args.dashboard_port}'}")
    print(f"Simulate:  {'Yes' if args.simulate else 'No'}")
    print("-" * 40)
    print()

    # Create and start navigator
    robocar = RobocarNavigator(args)

    if not robocar.start():
        print("[FATAL] Failed to initialize system")
        sys.exit(1)

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
