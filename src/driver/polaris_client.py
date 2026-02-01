"""
Polaris Client for Point One RTK Corrections

This module connects to Point One Navigation's Polaris service
to receive RTCM3 corrections for RTK positioning.

The corrections are sent to the GPS receiver to achieve centimeter-level accuracy.

Usage:
    client = PolarisClient(api_key="your_polaris_api_key")
    client.set_position(latitude, longitude, altitude)
    client.start()

    # Get corrections in your loop
    rtcm_data = client.get_corrections()
    if rtcm_data:
        serial_port.write(rtcm_data)
"""

import socket
import ssl
import threading
import time
import struct
from typing import Optional, Callable
from dataclasses import dataclass
from queue import Queue, Empty


@dataclass
class PolarisConfig:
    """Configuration for Polaris connection."""
    api_key: str
    host: str = "polaris.pointonenav.com"
    port: int = 2101
    reconnect_delay: float = 5.0
    position_update_interval: float = 10.0


class PolarisClient:
    """
    Client for Point One Polaris RTK correction service.

    Polaris provides real-time RTCM3 corrections for centimeter-level
    GPS positioning. This client:
    1. Connects to Polaris server via TLS
    2. Authenticates with API key
    3. Sends position updates (GGA) to receive corrections from nearest base
    4. Receives RTCM3 correction data stream

    Usage:
        client = PolarisClient("your_api_key")
        client.set_position(48.8566, 2.3522, 35.0)  # Paris coordinates
        client.start()

        while True:
            rtcm = client.get_corrections()
            if rtcm:
                gps_serial.write(rtcm)
    """

    # NTRIP protocol constants
    NTRIP_VERSION = "Ntrip/2.0"
    USER_AGENT = "Robocar/1.0"

    def __init__(self, api_key: str, config: Optional[PolarisConfig] = None):
        """
        Initialize Polaris client.

        Args:
            api_key: Polaris API key from Point One Navigation
            config: Optional configuration (uses defaults if not provided)
        """
        self.api_key = api_key
        self.config = config or PolarisConfig(api_key=api_key)

        # Connection state
        self._socket: Optional[ssl.SSLSocket] = None
        self._connected = False
        self._running = False

        # Position for GGA messages
        self._latitude = 0.0
        self._longitude = 0.0
        self._altitude = 0.0
        self._position_lock = threading.Lock()

        # Correction data queue
        self._correction_queue: Queue = Queue(maxsize=100)

        # Threads
        self._receive_thread: Optional[threading.Thread] = None
        self._position_thread: Optional[threading.Thread] = None

        # Statistics
        self._bytes_received = 0
        self._last_correction_time = 0.0
        self._connection_count = 0

        # Callbacks
        self._on_connected: Optional[Callable[[], None]] = None
        self._on_disconnected: Optional[Callable[[], None]] = None
        self._on_correction: Optional[Callable[[bytes], None]] = None

    def set_position(self, latitude: float, longitude: float, altitude: float = 0.0):
        """
        Set current position for RTCM corrections.

        Polaris uses this position to send corrections from the nearest
        base station. Update this regularly with GPS position.

        Args:
            latitude: Latitude in degrees (positive = North)
            longitude: Longitude in degrees (positive = East)
            altitude: Altitude in meters above sea level
        """
        with self._position_lock:
            self._latitude = latitude
            self._longitude = longitude
            self._altitude = altitude

    def start(self) -> bool:
        """
        Start the Polaris client.

        Returns:
            True if connection initiated successfully
        """
        if self._running:
            return True

        self._running = True

        # Start receive thread
        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True,
            name="PolarisReceive"
        )
        self._receive_thread.start()

        # Start position update thread
        self._position_thread = threading.Thread(
            target=self._position_loop,
            daemon=True,
            name="PolarisPosition"
        )
        self._position_thread.start()

        return True

    def stop(self):
        """Stop the Polaris client."""
        self._running = False

        if self._receive_thread:
            self._receive_thread.join(timeout=2.0)
        if self._position_thread:
            self._position_thread.join(timeout=2.0)

        self._disconnect()

    def get_corrections(self, timeout: float = 0.1) -> Optional[bytes]:
        """
        Get RTCM correction data from queue.

        Args:
            timeout: Maximum time to wait for data

        Returns:
            RTCM3 correction bytes or None if no data available
        """
        try:
            return self._correction_queue.get(timeout=timeout)
        except Empty:
            return None

    def is_connected(self) -> bool:
        """Check if connected to Polaris."""
        return self._connected

    @property
    def bytes_received(self) -> int:
        """Total bytes of corrections received."""
        return self._bytes_received

    @property
    def seconds_since_correction(self) -> float:
        """Seconds since last correction received."""
        if self._last_correction_time == 0:
            return float('inf')
        return time.time() - self._last_correction_time

    def set_callbacks(self,
                      on_connected: Optional[Callable[[], None]] = None,
                      on_disconnected: Optional[Callable[[], None]] = None,
                      on_correction: Optional[Callable[[bytes], None]] = None):
        """Set callback functions for events."""
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._on_correction = on_correction

    def _connect(self) -> bool:
        """
        Connect to Polaris server.

        Returns:
            True if connection successful
        """
        try:
            # Create TCP socket (no TLS - Polaris uses plain NTRIP)
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(10.0)

            # Connect directly without TLS
            print(f"[Polaris] Connecting to {self.config.host}:{self.config.port}...")
            self._socket.connect((self.config.host, self.config.port))

            # Send NTRIP request with authentication
            if not self._send_ntrip_request():
                self._disconnect()
                return False

            self._connected = True
            self._connection_count += 1
            print(f"[Polaris] Connected successfully (connection #{self._connection_count})")

            if self._on_connected:
                self._on_connected()

            return True

        except socket.timeout:
            print("[Polaris] Connection timeout")
            return False
        except socket.error as e:
            print(f"[Polaris] Socket error: {e}")
            return False
        except Exception as e:
            print(f"[Polaris] Connection failed: {e}")
            return False

    def _disconnect(self):
        """Disconnect from Polaris server."""
        was_connected = self._connected
        self._connected = False

        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None

        if was_connected:
            print("[Polaris] Disconnected")
            if self._on_disconnected:
                self._on_disconnected()

    def _send_ntrip_request(self) -> bool:
        """
        Send NTRIP request for RTCM corrections.

        Returns:
            True if request accepted
        """
        try:
            # Build NTRIP GET request
            # Polaris uses the API key as the mountpoint
            request = (
                f"GET /{self.api_key} HTTP/1.1\r\n"
                f"Host: {self.config.host}\r\n"
                f"Ntrip-Version: {self.NTRIP_VERSION}\r\n"
                f"User-Agent: {self.USER_AGENT}\r\n"
                f"Accept: */*\r\n"
                f"Connection: close\r\n"
                f"\r\n"
            )

            self._socket.sendall(request.encode('ascii'))

            # Read response
            response = b""
            while b"\r\n\r\n" not in response:
                chunk = self._socket.recv(1024)
                if not chunk:
                    print("[Polaris] No response from server")
                    return False
                response += chunk

            # Parse HTTP response
            header_end = response.find(b"\r\n\r\n")
            headers = response[:header_end].decode('ascii', errors='ignore')

            # Check for success
            first_line = headers.split('\r\n')[0]
            if '200' in first_line:
                print("[Polaris] Authentication successful")
                # Put any remaining data back in queue
                remaining = response[header_end + 4:]
                if remaining:
                    self._process_rtcm(remaining)
                return True
            elif '401' in first_line:
                print("[Polaris] Authentication failed - check API key")
                return False
            else:
                print(f"[Polaris] Unexpected response: {first_line}")
                return False

        except Exception as e:
            print(f"[Polaris] Request failed: {e}")
            return False

    def _receive_loop(self):
        """Main receive loop (runs in separate thread)."""
        while self._running:
            # Connect if not connected
            if not self._connected:
                if self._connect():
                    # Connection successful, continue receiving
                    pass
                else:
                    # Wait before retry
                    time.sleep(self.config.reconnect_delay)
                    continue

            # Receive data
            try:
                self._socket.settimeout(1.0)
                data = self._socket.recv(4096)

                if not data:
                    print("[Polaris] Connection closed by server")
                    self._disconnect()
                    continue

                self._process_rtcm(data)

            except socket.timeout:
                # Normal timeout, continue
                pass
            except ssl.SSLError as e:
                if 'timed out' not in str(e).lower():
                    print(f"[Polaris] SSL error: {e}")
                    self._disconnect()
            except socket.error as e:
                print(f"[Polaris] Socket error: {e}")
                self._disconnect()
            except Exception as e:
                print(f"[Polaris] Receive error: {e}")
                self._disconnect()

    def _process_rtcm(self, data: bytes):
        """Process received RTCM data."""
        self._bytes_received += len(data)
        self._last_correction_time = time.time()

        # Add to queue (non-blocking)
        try:
            self._correction_queue.put_nowait(data)
        except:
            # Queue full, drop oldest
            try:
                self._correction_queue.get_nowait()
                self._correction_queue.put_nowait(data)
            except:
                pass

        # Call callback if set
        if self._on_correction:
            self._on_correction(data)

    def _position_loop(self):
        """Send position updates to Polaris (runs in separate thread)."""
        while self._running:
            if self._connected:
                self._send_position_update()

            time.sleep(self.config.position_update_interval)

    def _send_position_update(self):
        """Send GGA position update to Polaris."""
        if not self._connected or not self._socket:
            return

        with self._position_lock:
            lat = self._latitude
            lon = self._longitude
            alt = self._altitude

        if lat == 0.0 and lon == 0.0:
            return  # No position set

        try:
            gga = self._format_gga(lat, lon, alt)
            self._socket.sendall(gga.encode('ascii'))
        except Exception as e:
            print(f"[Polaris] Failed to send position: {e}")

    def _format_gga(self, lat: float, lon: float, alt: float) -> str:
        """
        Format position as NMEA GGA sentence.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters

        Returns:
            NMEA GGA sentence string
        """
        # Convert decimal degrees to NMEA format (DDDMM.MMMM)
        lat_dir = 'N' if lat >= 0 else 'S'
        lon_dir = 'E' if lon >= 0 else 'W'

        lat = abs(lat)
        lon = abs(lon)

        lat_deg = int(lat)
        lat_min = (lat - lat_deg) * 60.0
        lon_deg = int(lon)
        lon_min = (lon - lon_deg) * 60.0

        lat_nmea = f"{lat_deg:02d}{lat_min:09.6f}"
        lon_nmea = f"{lon_deg:03d}{lon_min:09.6f}"

        # Current time in UTC
        now = time.gmtime()
        time_str = f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.00"

        # Build GGA sentence (without checksum)
        gga_data = (
            f"GPGGA,{time_str},{lat_nmea},{lat_dir},{lon_nmea},{lon_dir},"
            f"1,12,1.0,{alt:.1f},M,0.0,M,,"
        )

        # Calculate checksum
        checksum = 0
        for char in gga_data:
            checksum ^= ord(char)

        gga = f"${gga_data}*{checksum:02X}\r\n"
        return gga


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python polaris_client.py <api_key> [lat] [lon]")
        print("Example: python polaris_client.py my_api_key 48.8566 2.3522")
        sys.exit(1)

    api_key = sys.argv[1]
    lat = float(sys.argv[2]) if len(sys.argv) > 2 else 48.8566  # Paris default
    lon = float(sys.argv[3]) if len(sys.argv) > 3 else 2.3522

    print(f"[Test] Connecting to Polaris with position ({lat}, {lon})...")

    client = PolarisClient(api_key)
    client.set_position(lat, lon, 35.0)

    def on_correction(data):
        print(f"[Test] Received {len(data)} bytes of RTCM corrections")

    client.set_callbacks(on_correction=on_correction)
    client.start()

    try:
        start = time.time()
        while time.time() - start < 30:
            time.sleep(1)
            print(f"[Test] Connected: {client.is_connected()}, "
                  f"Bytes: {client.bytes_received}, "
                  f"Last correction: {client.seconds_since_correction:.1f}s ago")
    except KeyboardInterrupt:
        print("\n[Test] Interrupted")
    finally:
        client.stop()
        print(f"[Test] Total bytes received: {client.bytes_received}")