"""
GPS RTK Driver for Point One Navigation

Point One provides centimeter-accurate RTK positioning.
This driver handles NMEA parsing and RTK status monitoring.

NMEA Sentences:
- GGA: Global Positioning System Fix Data (position + quality)
- RMC: Recommended Minimum Specific GPS Data (position + velocity)
- GST: GPS Pseudorange Noise Statistics (accuracy)

RTK Quality Levels:
- 0: No fix
- 1: GPS (autonomous) - ~2-5m accuracy
- 2: DGPS - ~0.5-2m accuracy
- 4: RTK Fixed - ~1-2cm accuracy
- 5: RTK Float - ~10-50cm accuracy

References:
- https://pointonenav.com/docs
- NMEA 0183 Standard
"""

import serial
import threading
import time
import math
from typing import Optional, Tuple, Callable
from dataclasses import dataclass, field
from datetime import datetime
from collections import deque


@dataclass
class GPSPosition:
    """GPS position data."""
    timestamp: float         # Unix timestamp
    latitude: float          # Degrees (positive = North)
    longitude: float         # Degrees (positive = East)
    altitude: float          # Meters above sea level
    quality: int             # Fix quality (0-5)
    satellites: int          # Number of satellites
    hdop: float              # Horizontal dilution of precision
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

    def distance_to(self, other: 'GPSPosition') -> float:
        """
        Calculate distance to another position using Haversine formula.

        Returns:
            Distance in meters
        """
        R = 6371000  # Earth radius in meters

        lat1 = math.radians(self.latitude)
        lat2 = math.radians(other.latitude)
        dlat = math.radians(other.latitude - self.latitude)
        dlon = math.radians(other.longitude - self.longitude)

        a = (math.sin(dlat/2)**2 +
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def bearing_to(self, other: 'GPSPosition') -> float:
        """
        Calculate bearing to another position.

        Returns:
            Bearing in radians (0 = North, pi/2 = East)
        """
        lat1 = math.radians(self.latitude)
        lat2 = math.radians(other.latitude)
        dlon = math.radians(other.longitude - self.longitude)

        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

        return math.atan2(x, y)


@dataclass
class GPSVelocity:
    """GPS velocity data."""
    timestamp: float      # Unix timestamp
    speed: float          # Speed in m/s
    heading: float        # Heading in radians (0 = North)
    valid: bool           # Whether velocity is valid


class GPSDriver:
    """
    Driver for Point One RTK GPS.

    Usage:
        gps = PointOneGPS('/dev/ttyUSB1')
        gps.start()

        while True:
            pos = gps.get_position()
            if pos:
                print(f"Position: {pos.latitude:.6f}, {pos.longitude:.6f}")
                print(f"Quality: {pos.quality_string}, Accuracy: {pos.accuracy_h:.2f}m")

        gps.stop()
    """

    def __init__(
        self,
        port: str = '/dev/ttyUSB1',
        baudrate: int = 115200,
        buffer_size: int = 100
    ):
        """
        Initialize GPS driver.

        Args:
            port: Serial port path
            baudrate: Serial baudrate
            buffer_size: Number of positions to buffer
        """
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size

        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self._position_buffer: deque = deque(maxlen=buffer_size)
        self._velocity_buffer: deque = deque(maxlen=buffer_size)
        self._latest_position: Optional[GPSPosition] = None
        self._latest_velocity: Optional[GPSVelocity] = None
        self._lock = threading.Lock()

        # Callbacks
        self._position_callback: Optional[Callable[[GPSPosition], None]] = None

        # Temporary parsing state
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

    def get_velocity(self) -> Optional[GPSVelocity]:
        """
        Get the latest GPS velocity.

        Returns:
            GPSVelocity object or None if no velocity available
        """
        with self._lock:
            return self._latest_velocity

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

    def get_position_history(self, count: int = 10) -> list:
        """Get recent position history."""
        with self._lock:
            return list(self._position_buffer)[-count:]

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
            elif msg_type in ('GPRMC', 'GNRMC'):
                self._parse_rmc(parts)
            elif msg_type in ('GPGST', 'GNGST'):
                self._parse_gst(parts)

        except (ValueError, IndexError) as e:
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
        hdop = float(parts[8]) if parts[8] else 99.99
        altitude = float(parts[9]) if parts[9] else 0.0

        position = GPSPosition(
            timestamp=time.time(),
            latitude=lat,
            longitude=lon,
            altitude=altitude,
            quality=quality,
            satellites=satellites,
            hdop=hdop,
            accuracy_h=self._current_accuracy_h,
            accuracy_v=self._current_accuracy_v
        )

        with self._lock:
            self._latest_position = position
            self._position_buffer.append(position)

        if self._position_callback:
            self._position_callback(position)

    def _parse_rmc(self, parts: list):
        """
        Parse RMC sentence (position and velocity).

        Format: $GPRMC,time,status,lat,N/S,lon,E/W,speed,heading,date,mag_var,E/W,mode*checksum
        """
        if len(parts) < 12:
            return

        # Check validity
        if parts[2] != 'A':
            return  # Invalid

        # Parse speed and heading
        speed_knots = float(parts[7]) if parts[7] else 0.0
        speed_ms = speed_knots * 0.514444  # Convert to m/s

        heading_deg = float(parts[8]) if parts[8] else 0.0
        heading_rad = math.radians(heading_deg)

        velocity = GPSVelocity(
            timestamp=time.time(),
            speed=speed_ms,
            heading=heading_rad,
            valid=True
        )

        with self._lock:
            self._latest_velocity = velocity
            self._velocity_buffer.append(velocity)

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
            alt_err = float(parts[8].split('*')[0]) if parts[8] else 99.99

            # Horizontal accuracy is combination of lat/lon errors
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


# Utility functions

def calculate_utm_zone(longitude: float) -> int:
    """Calculate UTM zone from longitude."""
    return int((longitude + 180) / 6) + 1


def gps_to_local(
    position: GPSPosition,
    origin: GPSPosition
) -> Tuple[float, float]:
    """
    Convert GPS position to local coordinates relative to origin.
    Uses flat-Earth approximation (valid for short distances).

    Args:
        position: Current GPS position
        origin: Origin GPS position

    Returns:
        (x, y) in meters where x=East, y=North
    """
    # Earth radius at equator
    R = 6378137.0

    # Latitude correction
    lat_rad = math.radians(origin.latitude)

    # Meters per degree
    m_per_deg_lat = R * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(lat_rad)

    dx = (position.longitude - origin.longitude) * m_per_deg_lon
    dy = (position.latitude - origin.latitude) * m_per_deg_lat

    return dx, dy


def local_to_gps(
    x: float,
    y: float,
    origin: GPSPosition
) -> Tuple[float, float]:
    """
    Convert local coordinates to GPS position.

    Args:
        x: East offset in meters
        y: North offset in meters
        origin: Origin GPS position

    Returns:
        (latitude, longitude) in degrees
    """
    R = 6378137.0
    lat_rad = math.radians(origin.latitude)

    m_per_deg_lat = R * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(lat_rad)

    lat = origin.latitude + y / m_per_deg_lat
    lon = origin.longitude + x / m_per_deg_lon

    return lat, lon


if __name__ == '__main__':
    # Simple test
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    print(f"[GPS] Testing GPS on {port}...")

    gps = GPSDriver(port)
    if gps.start():
        print("[GPS] Started successfully")
        print("[GPS] Waiting for RTK fixed (max 30s)...")

        try:
            for _ in range(30):
                pos = gps.get_position()
                if pos:
                    print(f"[GPS] {pos.quality_string}: "
                          f"({pos.latitude:.6f}, {pos.longitude:.6f}) "
                          f"alt={pos.altitude:.1f}m "
                          f"acc={pos.accuracy_h:.2f}m "
                          f"sats={pos.satellites}")
                else:
                    print("[GPS] Waiting for position...")
                time.sleep(1.0)
        finally:
            gps.stop()
    else:
        print("[GPS] Failed to start")