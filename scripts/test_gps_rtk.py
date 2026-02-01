#!/usr/bin/env python3
"""
Test GPS RTK with Polaris Corrections

This script tests the complete GPS RTK setup:
1. Connection to Point One GPS receiver
2. Connection to Polaris for RTK corrections
3. Achievement of RTK Fixed solution

Usage:
    python scripts/test_gps_rtk.py --api-key YOUR_POLARIS_API_KEY
    python scripts/test_gps_rtk.py --config config/gps.yaml

Requirements:
    - Point One GPS connected to USB port
    - Valid Polaris API key from Point One Navigation
    - Antenna with clear sky view
"""

import sys
import time
import argparse
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from driver.gps_rtk import GPSRTKDriver


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    try:
        import yaml
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load config: {e}")
        return {}


def print_status_line(pos, stats, elapsed):
    """Print formatted status line."""
    if pos is None:
        print(f"\r[{elapsed:3.0f}s] Searching for satellites...          ", end='', flush=True)
        return

    # Color based on quality
    colors = {
        4: '\033[92m',  # Green for RTK Fixed
        5: '\033[93m',  # Yellow for RTK Float
        2: '\033[94m',  # Blue for DGPS
        1: '\033[0m',   # Normal for GPS
        0: '\033[91m',  # Red for no fix
    }
    reset = '\033[0m'
    color = colors.get(pos.quality, reset)

    # Polaris status
    polaris = "Polaris: "
    if stats['polaris_connected']:
        if stats['polaris_correction_age'] < 5:
            polaris += f"\033[92mOK ({stats['polaris_correction_age']:.1f}s)\033[0m"
        else:
            polaris += f"\033[93m{stats['polaris_correction_age']:.1f}s\033[0m"
    else:
        polaris += "\033[91mDisconnected\033[0m"

    print(f"\r[{elapsed:3.0f}s] {color}{pos.quality_string:10}{reset} | "
          f"({pos.latitude:11.7f}, {pos.longitude:11.7f}) | "
          f"Acc: {pos.accuracy_h:5.3f}m | "
          f"Sats: {pos.satellites:2d} | "
          f"{polaris}          ", end='', flush=True)


def main():
    parser = argparse.ArgumentParser(
        description='Test GPS RTK with Polaris corrections',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Using API key directly:
    python scripts/test_gps_rtk.py --api-key YOUR_API_KEY

    # Using config file:
    python scripts/test_gps_rtk.py --config config/gps.yaml

    # Custom port:
    python scripts/test_gps_rtk.py --port /dev/ttyUSB0 --api-key YOUR_API_KEY

Notes:
    - Ensure the antenna has a clear view of the sky
    - RTK Fixed typically takes 30-120 seconds to achieve
    - Get your Polaris API key from: https://app.pointonenav.com/
        """
    )
    parser.add_argument('--port', default='/dev/ttyUSB1',
                        help='GPS serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--api-key', dest='api_key',
                        help='Polaris API key')
    parser.add_argument('--config', default='config/gps.yaml',
                        help='Path to GPS config file (default: config/gps.yaml)')
    parser.add_argument('--duration', type=int, default=120,
                        help='Test duration in seconds (default: 120)')
    parser.add_argument('--no-polaris', action='store_true',
                        help='Run without Polaris (GPS only, no RTK)')
    args = parser.parse_args()

    # Load configuration
    config = {}
    if os.path.exists(args.config):
        config = load_config(args.config)
        print(f"[INFO] Loaded config from {args.config}")

    # Get API key
    api_key = args.api_key
    if not api_key and not args.no_polaris:
        api_key = config.get('polaris', {}).get('api_key')
        if api_key == "YOUR_POLARIS_API_KEY_HERE":
            api_key = None

    # Get port
    port = args.port
    if port == '/dev/ttyUSB1':
        port = config.get('gps', {}).get('port', port)

    print("=" * 70)
    print("GPS RTK TEST - Point One Navigation with Polaris Corrections")
    print("=" * 70)
    print(f"Port:     {port}")
    print(f"Polaris:  {'Enabled' if api_key else 'Disabled (no API key)'}")
    print(f"Duration: {args.duration}s")
    print()

    if not api_key and not args.no_polaris:
        print("\033[93m[WARNING] No Polaris API key provided!\033[0m")
        print("Without Polaris corrections, you will not get RTK Fixed precision.")
        print()
        print("To use RTK corrections:")
        print("  1. Get your API key from https://app.pointonenav.com/")
        print("  2. Run: python scripts/test_gps_rtk.py --api-key YOUR_KEY")
        print("  3. Or add your key to config/gps.yaml")
        print()
        print("Continuing without RTK corrections...")
        print()

    print("IMPORTANT: Ensure antenna has clear sky view!")
    print("=" * 70)
    print()

    # Create driver
    gps = GPSRTKDriver(
        port=port,
        polaris_api_key=api_key
    )

    # Set initial position if configured (helps Polaris)
    initial_pos = config.get('initial_position', {})
    if initial_pos and api_key:
        lat = initial_pos.get('latitude', 0)
        lon = initial_pos.get('longitude', 0)
        if lat and lon and gps._polaris:
            gps._polaris.set_position(lat, lon, initial_pos.get('altitude', 0))
            print(f"[INFO] Initial position set to ({lat:.4f}, {lon:.4f})")

    # Start driver
    print("[GPS] Starting...")
    if not gps.start():
        print("\033[91m[ERROR] Failed to start GPS driver\033[0m")
        print("Check that:")
        print(f"  - GPS is connected to {port}")
        print("  - You have permission to access the port")
        print("  - No other application is using the port")
        return 1

    print("[GPS] Started successfully")
    print("[GPS] Waiting for position...")
    print()

    # Statistics
    first_fix_time = None
    first_rtk_time = None
    rtk_fixed_count = 0
    best_accuracy = float('inf')

    try:
        start_time = time.time()

        while time.time() - start_time < args.duration:
            elapsed = time.time() - start_time
            pos = gps.get_position()
            stats = gps.get_stats()

            # Track statistics
            if pos:
                if first_fix_time is None and pos.is_valid:
                    first_fix_time = elapsed
                    print(f"\n[GPS] First fix achieved at {elapsed:.1f}s")

                if first_rtk_time is None and pos.is_rtk_fixed:
                    first_rtk_time = elapsed
                    print(f"\n\033[92m[GPS] RTK FIXED achieved at {elapsed:.1f}s!\033[0m")

                if pos.is_rtk_fixed:
                    rtk_fixed_count += 1
                    if pos.accuracy_h < best_accuracy:
                        best_accuracy = pos.accuracy_h

            # Print status
            print_status_line(pos, stats, elapsed)
            time.sleep(0.5)

        print()  # New line after loop

    except KeyboardInterrupt:
        print("\n\n[GPS] Test interrupted by user")

    finally:
        gps.stop()

    # Print summary
    print()
    print("=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)

    final_pos = gps.get_position()
    final_stats = gps.get_stats()

    if final_pos:
        print(f"Last Position:")
        print(f"  Latitude:  {final_pos.latitude:.7f}°")
        print(f"  Longitude: {final_pos.longitude:.7f}°")
        print(f"  Altitude:  {final_pos.altitude:.1f}m")
        print(f"  Quality:   {final_pos.quality_string}")
        print(f"  Accuracy:  {final_pos.accuracy_h:.3f}m horizontal")
        print(f"  Satellites: {final_pos.satellites}")
        print()

    print(f"Statistics:")
    print(f"  Positions received: {final_stats['positions_received']}")
    print(f"  RTK Fixed count:    {final_stats['rtk_fixed_count']}")
    print(f"  RTCM bytes sent:    {final_stats['rtcm_bytes_sent']}")

    if first_fix_time:
        print(f"  Time to first fix:  {first_fix_time:.1f}s")
    if first_rtk_time:
        print(f"  Time to RTK Fixed:  {first_rtk_time:.1f}s")
    if best_accuracy < float('inf'):
        print(f"  Best accuracy:      {best_accuracy:.3f}m")

    print()

    # Final assessment
    if final_pos and final_pos.is_rtk_fixed:
        print("\033[92m[SUCCESS] RTK FIXED - Centimeter-level accuracy achieved!\033[0m")
        return 0
    elif final_pos and final_pos.quality == 5:
        print("\033[93m[PARTIAL] RTK FLOAT - Decimeter accuracy\033[0m")
        print("RTK Float is good but not optimal. To achieve RTK Fixed:")
        print("  - Ensure clear sky view (no obstructions)")
        print("  - Wait longer (can take up to 2 minutes)")
        print("  - Check Polaris correction age (should be < 5s)")
        return 0
    elif final_pos and final_pos.is_valid:
        print("\033[93m[LIMITED] GPS Only - Meter-level accuracy\033[0m")
        if not api_key:
            print("Add Polaris API key for RTK precision")
        else:
            print("Check:")
            print("  - Antenna sky view")
            print("  - Polaris connection status")
            print("  - Correction data flow")
        return 0
    else:
        print("\033[91m[FAILED] No GPS fix achieved\033[0m")
        print("Check:")
        print("  - Antenna is connected and has sky view")
        print("  - GPS is properly powered")
        print("  - Serial port is correct")
        return 1


if __name__ == '__main__':
    sys.exit(main())