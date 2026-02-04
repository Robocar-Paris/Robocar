"""
GPS RTK Driver with Polaris Corrections

Complete driver for Point One Navigation RTK GPS with Polaris corrections.
This driver:
1. Connects to the GPS receiver via serial port
2. Connects to Polaris to receive RTCM corrections
3. Forwards corrections to the GPS for RTK positioning
4. Parses NMEA messages to get centimeter-level position

Usage:
    gps = GPSRTKDriver(
        port='/dev/ttyUSB1',
        polaris_api_key='your_api_key'
    )
    gps.start()

    pos = gps.get_position()
    if pos and pos.is_rtk_fixed:
        print(f"RTK Fixed: {pos.latitude}, {pos.longitude}")

    gps.stop()
"""

import serial
import threading
import time
import serial.tools.list_ports
from typing import Optional, Callable
from dataclasses import dataclass

from driver.polaris_client import PolarisClient, PolarisConfig


@dataclass
class GPSPosition:
    """GPS position data from Point One RTK."""
    timestamp: float         # Unix timestamp
    latitude: float          # Degrees (positive = North)
    longitude: float         # Degrees (positive = East)
    altitude: float          # Meters above sea level
    quality: int             # Fix quality (0, 1, 2, 4, 5)
    satellites: int          # Number of satellites
    accuracy_h: float        # Horizontal accuracy in meters
    accuracy_v: float        # Vertical accuracy in meters

    @property
    def quality_string(self) -> str:
        """Human-readable quality string."""
        quality_map = {
            0: "NO_FIX",
            1: "GPS",
            2: "DGPS",
            4: "RTK_FIXED",
            5: "RTK_FLOAT"
        }
        return quality_map.get(self.quality, "UNKNOWN")

    @property
    def is_rtk_fixed(self) -> bool:
        """Check if position has RTK fixed quality."""
        return self.quality == 4

    @property
    def is_rtk(self) -> bool:
        """Check if position has any RTK quality (fixed or float)."""
        return self.quality in (4, 5)

    @property
    def is_valid(self) -> bool:
        """Check if position is valid (has fix)."""
        return self.quality > 0


class GPSRTKDriver:
    """
    Complete GPS RTK driver with Polaris corrections.

    This driver combines:
    - Serial communication with Point One GPS receiver
    - Polaris client for RTCM corrections
    - NMEA parsing for position data

    The driver automatically:
    - Connects to Polaris and receives corrections
    - Forwards corrections to the GPS receiver
    - Updates Polaris with current position for optimal corrections
    - Parses NMEA GGA and GST messages for position and accuracy
    """

    def __init__(self,
                 port: str = '/dev/ttyUSB1',
                 baudrate: int = 460800,
                 polaris_api_key: Optional[str] = None,
                 polaris_config: Optional[PolarisConfig] = None):
        """
        Initialize GPS RTK driver.

        Args:
            port: Serial port for GPS receiver
            baudrate: Serial baudrate (460800 for Point One)
            polaris_api_key: API key for Polaris RTK corrections
            polaris_config: Optional Polaris configuration
        """
        self.port = port
        self.baudrate = baudrate

        # Serial connection
        self._serial: Optional[serial.Serial] = None

        # Polaris client for RTK corrections
        self._polaris: Optional[PolarisClient] = None
        if polaris_api_key:
            config = polaris_config or PolarisConfig(api_key=polaris_api_key)
            self._polaris = PolarisClient(polaris_api_key, config)

        # Threading
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._correction_thread: Optional[threading.Thread] = None

        # Position data
        self._latest_position: Optional[GPSPosition] = None
        self._position_lock = threading.Lock()

        # Accuracy from GST message
        self._current_accuracy_h = 0.0
        self._current_accuracy_v = 0.0

        # Callbacks
        self._position_callback: Optional[Callable[[GPSPosition], None]] = None
        self._rtk_callback: Optional[Callable[[bool], None]] = None

        # Statistics
        self._rtcm_bytes_sent = 0
        self._position_count = 0
        self._rtk_fixed_count = 0
        self._last_rtk_fixed_time = 0.0

    def start(self, auto_detect: bool = False) -> bool:
        """
        Start the GPS RTK driver.

        Args:
            auto_detect: If True and initial port fails, try other available ports

        Returns:
            True if successfully started
        """
        # Build list of ports to try
        ports_to_try = [self.port]

        if auto_detect:
            try:
                available = [p.device for p in serial.tools.list_ports.comports()
                            if '/dev/ttyUSB' in p.device and p.device != self.port]
                # Prioritize ttyUSB1 (typical GPS port), then by number
                available.sort(key=lambda x: (0 if 'USB1' in x else 1, x))
                ports_to_try.extend(available)
            except ImportError:
                pass

        # Try each port
        last_error = None
        for port in ports_to_try:
            try:
                self._serial = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=1.0
                )
                self.port = port  # Update to working port
                print(f"[GPS-RTK] Connected to {port} at {self.baudrate} baud")
                break
            except serial.SerialException as e:
                last_error = e
                if "Input/output error" in str(e):
                    print(f"[GPS-RTK] Port {port} has I/O error, trying next...")
                continue
        else:
            print(f"[GPS-RTK] Failed to open serial port: {last_error}")
            return False

        self._running = True

        # Start NMEA read thread
        self._read_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name="GPSRead"
        )
        self._read_thread.start()

        # Start Polaris and correction forwarding
        if self._polaris:
            # Set initial position callback
            self._polaris.set_callbacks(
                on_connected=self._on_polaris_connected,
                on_disconnected=self._on_polaris_disconnected
            )
            self._polaris.start()

            # Start correction forwarding thread
            self._correction_thread = threading.Thread(
                target=self._correction_loop,
                daemon=True,
                name="RTKCorrections"
            )
            self._correction_thread.start()
            print("[GPS-RTK] Polaris corrections enabled")
        else:
            print("[GPS-RTK] Running without RTK corrections (no Polaris API key)")

        return True

    def stop(self):
        """Stop the GPS RTK driver."""
        self._running = False

        # Stop threads
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
        if self._correction_thread:
            self._correction_thread.join(timeout=2.0)

        # Stop Polaris
        if self._polaris:
            self._polaris.stop()

        # Close serial port
        if self._serial:
            self._serial.close()
            self._serial = None

        print("[GPS-RTK] Stopped")

    def get_position(self) -> Optional[GPSPosition]:
        """
        Get the latest GPS position.

        Returns:
            GPSPosition object or None if no position available
        """
        with self._position_lock:
            return self._latest_position

    def wait_for_fix(self, timeout: float = 30.0) -> bool:
        """
        Wait for any GPS fix.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if fix achieved, False if timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            pos = self.get_position()
            if pos and pos.is_valid:
                return True
            time.sleep(0.1)
        return False

    def wait_for_rtk_fixed(self, timeout: float = 120.0) -> bool:
        """
        Wait for RTK fixed solution.

        RTK fixed typically takes 30-120 seconds after receiving corrections.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if RTK fixed achieved, False if timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            pos = self.get_position()
            if pos and pos.is_rtk_fixed:
                return True
            time.sleep(0.1)
        return False

    def set_callbacks(self,
                      on_position: Optional[Callable[[GPSPosition], None]] = None,
                      on_rtk_change: Optional[Callable[[bool], None]] = None):
        """
        Set callback functions.

        Args:
            on_position: Called when new position received
            on_rtk_change: Called when RTK status changes
        """
        self._position_callback = on_position
        self._rtk_callback = on_rtk_change

    @property
    def is_running(self) -> bool:
        """Check if driver is running."""
        return self._running

    @property
    def is_polaris_connected(self) -> bool:
        """Check if Polaris is connected."""
        return self._polaris is not None and self._polaris.is_connected()

    @property
    def rtcm_bytes_sent(self) -> int:
        """Total RTCM correction bytes sent to GPS."""
        return self._rtcm_bytes_sent

    @property
    def polaris_seconds_since_correction(self) -> float:
        """Seconds since last Polaris correction."""
        if self._polaris:
            return self._polaris.seconds_since_correction
        return float('inf')

    def _on_polaris_connected(self):
        """Called when Polaris connects."""
        print("[GPS-RTK] Polaris connected - receiving RTK corrections")

    def _on_polaris_disconnected(self):
        """Called when Polaris disconnects."""
        print("[GPS-RTK] Polaris disconnected - attempting reconnect...")

    def _correction_loop(self):
        """Forward RTCM corrections from Polaris to GPS (runs in thread)."""
        while self._running:
            if not self._polaris:
                time.sleep(1.0)
                continue

            # Get corrections from Polaris
            rtcm_data = self._polaris.get_corrections(timeout=0.1)

            if rtcm_data and self._serial and self._serial.is_open:
                try:
                    # Send corrections to GPS receiver
                    self._serial.write(rtcm_data)
                    self._rtcm_bytes_sent += len(rtcm_data)
                except serial.SerialException as e:
                    print(f"[GPS-RTK] Failed to send corrections: {e}")

            # Update Polaris with current position
            pos = self.get_position()
            if pos and pos.is_valid and self._polaris:
                self._polaris.set_position(
                    pos.latitude,
                    pos.longitude,
                    pos.altitude
                )

    def _read_loop(self):
        """Read and parse NMEA messages from GPS (runs in thread)."""
        buffer = ""

        while self._running:
            try:
                if self._serial and self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    buffer += data.decode('ascii', errors='ignore')

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('$'):
                            self._parse_nmea(line)
                else:
                    time.sleep(0.01)

            except serial.SerialException as e:
                print(f"[GPS-RTK] Serial error: {e}")
                time.sleep(0.1)
            except Exception as e:
                print(f"[GPS-RTK] Parse error: {e}")

    def _parse_nmea(self, sentence: str):
        """Parse NMEA sentence."""
        try:
            # Verify checksum
            if '*' in sentence:
                data, checksum = sentence[1:].split('*')
                calculated = 0
                for char in data:
                    calculated ^= ord(char)
                if calculated != int(checksum, 16):
                    return  # Invalid checksum
            else:
                data = sentence[1:]

            parts = data.split(',')
            msg_type = parts[0]

            if msg_type in ('GPGGA', 'GNGGA'):
                self._parse_gga(parts)
            elif msg_type in ('GPGST', 'GNGST'):
                self._parse_gst(parts)

        except (ValueError, IndexError):
            pass  # Ignore malformed sentences

    def _parse_gga(self, parts: list):
        """Parse GGA sentence for position and fix quality."""
        if len(parts) < 15:
            return

        quality = int(parts[6]) if parts[6] else 0
        if quality == 0:
            return  # No fix

        # Parse position
        lat = self._parse_coordinate(parts[2], parts[3])
        lon = self._parse_coordinate(parts[4], parts[5])

        if lat is None or lon is None:
            return

        satellites = int(parts[7]) if parts[7] else 0
        altitude = float(parts[9]) if parts[9] else 0.0

        # Track RTK status changes
        prev_pos = self._latest_position
        was_rtk_fixed = prev_pos and prev_pos.is_rtk_fixed
        is_rtk_fixed = quality == 4

        if is_rtk_fixed and not was_rtk_fixed:
            self._last_rtk_fixed_time = time.time()
            print("[GPS-RTK] RTK FIXED achieved!")
            if self._rtk_callback:
                self._rtk_callback(True)
        elif was_rtk_fixed and not is_rtk_fixed:
            print(f"[GPS-RTK] Lost RTK Fixed (now: {quality})")
            if self._rtk_callback:
                self._rtk_callback(False)

        if is_rtk_fixed:
            self._rtk_fixed_count += 1

        position = GPSPosition(
            timestamp=time.time(),
            latitude=lat,
            longitude=lon,
            altitude=altitude,
            quality=quality,
            satellites=satellites,
            accuracy_h=self._current_accuracy_h,
            accuracy_v=self._current_accuracy_v
        )

        with self._position_lock:
            self._latest_position = position
            self._position_count += 1

        if self._position_callback:
            self._position_callback(position)

    def _parse_gst(self, parts: list):
        """Parse GST sentence for accuracy statistics."""
        if len(parts) < 9:
            return

        try:
            lat_err = float(parts[6]) if parts[6] else 99.99
            lon_err = float(parts[7]) if parts[7] else 99.99
            alt_err_str = parts[8].split('*')[0] if parts[8] else ""
            alt_err = float(alt_err_str) if alt_err_str else 99.99

            import math
            self._current_accuracy_h = math.sqrt(lat_err**2 + lon_err**2)
            self._current_accuracy_v = alt_err
        except ValueError:
            pass

    def _parse_coordinate(self, value: str, direction: str) -> Optional[float]:
        """Parse NMEA coordinate to decimal degrees."""
        if not value or not direction:
            return None

        try:
            decimal_idx = value.find('.')
            if decimal_idx < 2:
                return None

            degrees = float(value[:decimal_idx-2])
            minutes = float(value[decimal_idx-2:])
            result = degrees + minutes / 60.0

            if direction in ('S', 'W'):
                result = -result

            return result
        except ValueError:
            return None

    def get_stats(self) -> dict:
        """Get driver statistics."""
        return {
            'positions_received': self._position_count,
            'rtk_fixed_count': self._rtk_fixed_count,
            'rtcm_bytes_sent': self._rtcm_bytes_sent,
            'polaris_connected': self.is_polaris_connected,
            'polaris_correction_age': self.polaris_seconds_since_correction,
            'last_rtk_fixed': self._last_rtk_fixed_time
        }


def load_polaris_config(config_path: str) -> Optional[str]:
    """
    Load Polaris API key from configuration file.

    Args:
        config_path: Path to YAML config file

    Returns:
        API key string or None
    """
    try:
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config.get('polaris', {}).get('api_key')
    except Exception as e:
        print(f"[GPS-RTK] Failed to load config: {e}")
        return None


if __name__ == '__main__':
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    api_key = sys.argv[2] if len(sys.argv) > 2 else None

    print(f"[GPS-RTK] Testing on {port}")
    if api_key:
        print(f"[GPS-RTK] Polaris API key provided")
    else:
        print("[GPS-RTK] No Polaris API key - running without RTK corrections")

    gps = GPSRTKDriver(
        port=port,
        polaris_api_key=api_key
    )

    if not gps.start():
        print("[GPS-RTK] Failed to start")
        sys.exit(1)

    try:
        print("[GPS-RTK] Waiting for position...")
        for i in range(60):
            pos = gps.get_position()
            if pos:
                status = f"\033[92m" if pos.is_rtk_fixed else ""
                status += pos.quality_string
                status += "\033[0m" if pos.is_rtk_fixed else ""

                print(f"[{i:3d}s] {status:15} | "
                      f"({pos.latitude:.7f}, {pos.longitude:.7f}) | "
                      f"acc={pos.accuracy_h:.3f}m | "
                      f"sats={pos.satellites}")
            else:
                print(f"[{i:3d}s] Waiting for fix...")

            stats = gps.get_stats()
            if i % 10 == 0 and stats['rtcm_bytes_sent'] > 0:
                print(f"      RTCM sent: {stats['rtcm_bytes_sent']} bytes, "
                      f"Polaris: {'connected' if stats['polaris_connected'] else 'disconnected'}")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[GPS-RTK] Interrupted")
    finally:
        gps.stop()
        print(f"[GPS-RTK] Stats: {gps.get_stats()}")
