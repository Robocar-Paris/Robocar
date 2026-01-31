"""
GPS RTK Driver for Point One Navigation

Simple driver to retrieve GPS coordinates from Point One RTK GPS.
Parses NMEA GGA (position) and GST (accuracy) sentences.

RTK Quality Levels:
- 0: No fix
- 1: GPS (autonomous) - ~2-5m accuracy
- 2: DGPS - ~0.5-2m accuracy
- 4: RTK Fixed - ~1-2cm accuracy
- 5: RTK Float - ~10-50cm accuracy
"""

import serial
import threading
import time
from typing import Optional, Callable
from dataclasses import dataclass


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
    def is_valid(self) -> bool:
        """Check if position is valid (has fix)."""
        return self.quality > 0


class GPSDriver:
    """
    Simple driver for Point One RTK GPS.

    Usage:
        gps = GPSDriver('/dev/ttyUSB1')
        gps.start()

        pos = gps.get_position()
        if pos and pos.is_valid:
            print(f"Lat: {pos.latitude}, Lon: {pos.longitude}")
            print(f"Quality: {pos.quality_string}")

        gps.stop()
    """

    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 460800):
        """
        Initialize GPS driver.

        Args:
            port: Serial port path
            baudrate: Serial baudrate (default 460800 for Point One)
        """
        self.port = port
        self.baudrate = baudrate

        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self._latest_position: Optional[GPSPosition] = None
        self._lock = threading.Lock()

        # Callback for new positions
        self._position_callback: Optional[Callable[[GPSPosition], None]] = None

        # Temporary parsing state for GST accuracy
        self._current_accuracy_h = 0.0
        self._current_accuracy_v = 0.0

    def start(self) -> bool:
        """
        Start the GPS driver.

        Returns:
            True if successfully started, False otherwise.
        """
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            return True
        except serial.SerialException as e:
            print(f"[GPS] Failed to open serial port: {e}")
            return False

    def stop(self):
        """Stop the GPS driver."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial:
            self._serial.close()
            self._serial = None

    def get_position(self) -> Optional[GPSPosition]:
        """
        Get the latest GPS position.

        Returns:
            GPSPosition object or None if no position available
        """
        with self._lock:
            return self._latest_position

    def wait_for_fix(self, timeout: float = 30.0) -> bool:
        """
        Wait for any GPS fix.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if fix achieved, False if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            pos = self.get_position()
            if pos and pos.is_valid:
                return True
            time.sleep(0.1)
        return False

    def wait_for_rtk_fixed(self, timeout: float = 60.0) -> bool:
        """
        Wait for RTK fixed solution.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if RTK fixed achieved, False if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            pos = self.get_position()
            if pos and pos.is_rtk_fixed:
                return True
            time.sleep(0.1)
        return False

    def set_position_callback(self, callback: Callable[[GPSPosition], None]):
        """Set callback for new positions."""
        self._position_callback = callback

    def _read_loop(self):
        """Main reading loop (runs in separate thread)."""
        buffer = ""

        while self._running:
            try:
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting).decode('ascii', errors='ignore')
                    buffer += data

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('$'):
                            self._parse_nmea(line)
                else:
                    time.sleep(0.01)

            except serial.SerialException as e:
                print(f"[GPS] Serial error: {e}")
                time.sleep(0.1)
            except Exception as e:
                print(f"[GPS] Parse error: {e}")

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
        """
        Parse GGA sentence (position and fix quality).

        Format: $GPGGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,alt,M,geoid,M,age,station*checksum
        """
        if len(parts) < 15:
            return

        # Parse quality first
        quality = int(parts[6]) if parts[6] else 0
        if quality == 0:
            return  # No fix

        # Parse position
        lat = self._parse_coordinate(parts[2], parts[3])
        lon = self._parse_coordinate(parts[4], parts[5])

        if lat is None or lon is None:
            return

        # Parse other fields
        satellites = int(parts[7]) if parts[7] else 0
        altitude = float(parts[9]) if parts[9] else 0.0

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

        with self._lock:
            self._latest_position = position

        if self._position_callback:
            self._position_callback(position)

    def _parse_gst(self, parts: list):
        """
        Parse GST sentence (accuracy statistics).

        Format: $GPGST,time,rms,maj,min,orient,lat_err,lon_err,alt_err*checksum
        """
        if len(parts) < 9:
            return

        try:
            lat_err = float(parts[6]) if parts[6] else 99.99
            lon_err = float(parts[7]) if parts[7] else 99.99
            alt_err_str = parts[8].split('*')[0] if parts[8] else ""
            alt_err = float(alt_err_str) if alt_err_str else 99.99

            # Horizontal accuracy is combination of lat/lon errors
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
            # NMEA format: DDDMM.MMMM
            if len(value) < 4:
                return None

            # Find decimal point to determine degree/minute split
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

    @property
    def is_running(self) -> bool:
        """Check if driver is running."""
        return self._running


if __name__ == '__main__':
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    print(f"[GPS] Testing Point One RTK on {port}...")

    gps = GPSDriver(port)
    if gps.start():
        print("[GPS] Started successfully")
        print("[GPS] Waiting for position...")

        try:
            for _ in range(30):
                pos = gps.get_position()
                if pos:
                    print(f"[GPS] {pos.quality_string}: "
                          f"({pos.latitude:.6f}, {pos.longitude:.6f}) "
                          f"alt={pos.altitude:.1f}m "
                          f"acc_h={pos.accuracy_h:.2f}m "
                          f"sats={pos.satellites}")
                else:
                    print("[GPS] No position yet...")
                time.sleep(1.0)
        finally:
            gps.stop()
    else:
        print("[GPS] Failed to start")
