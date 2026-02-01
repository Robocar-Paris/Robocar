"""
NTRIP Client for RTK Corrections

Supports multiple free RTK correction services:
- Centipede RTK (French community network - FREE)
- RTK2GO (Global free service)
- Custom NTRIP casters

Usage:
    # Centipede (France - gratuit)
    client = NTRIPClient(
        host="caster.centipede.fr",
        port=2101,
        mountpoint="XXXX"  # Your nearest base station
    )

    # RTK2GO (mondial - gratuit)
    client = NTRIPClient(
        host="rtk2go.com",
        port=2101,
        mountpoint="XXXX"
    )
"""

import socket
import base64
import threading
import time
from typing import Optional, Callable
from dataclasses import dataclass
from queue import Queue, Empty


@dataclass
class NTRIPConfig:
    """Configuration for NTRIP connection."""
    host: str
    port: int = 2101
    mountpoint: str = ""
    username: str = ""
    password: str = ""
    send_gga: bool = True
    gga_interval: float = 10.0


# Pre-configured services
CENTIPEDE_CONFIG = NTRIPConfig(
    host="caster.centipede.fr",
    port=2101,
    mountpoint="",  # User must set nearest base
    username="centipede",
    password="centipede"
)

RTK2GO_CONFIG = NTRIPConfig(
    host="rtk2go.com",
    port=2101,
    mountpoint="",  # User must set nearest base
    username="",
    password=""
)


class NTRIPClient:
    """
    NTRIP client for receiving RTK corrections.

    NTRIP (Networked Transport of RTCM via Internet Protocol) is the
    standard protocol for streaming RTK corrections over the internet.

    Supports:
    - Centipede RTK (France) - FREE
    - RTK2GO - FREE
    - Any NTRIP caster

    Example:
        client = NTRIPClient.centipede("LIENSS")  # Nearest base station
        client.set_position(48.8566, 2.3522)
        client.start()

        while True:
            rtcm = client.get_corrections()
            if rtcm:
                gps_serial.write(rtcm)
    """

    NTRIP_VERSION = "Ntrip/2.0"
    USER_AGENT = "NTRIP Robocar/1.0"

    def __init__(self, config: NTRIPConfig):
        """
        Initialize NTRIP client.

        Args:
            config: NTRIP configuration
        """
        self.config = config

        # Connection state
        self._socket: Optional[socket.socket] = None
        self._connected = False
        self._running = False

        # Position for GGA
        self._latitude = 0.0
        self._longitude = 0.0
        self._altitude = 0.0
        self._position_lock = threading.Lock()

        # Correction queue
        self._correction_queue: Queue = Queue(maxsize=100)

        # Threads
        self._receive_thread: Optional[threading.Thread] = None
        self._gga_thread: Optional[threading.Thread] = None

        # Stats
        self._bytes_received = 0
        self._last_correction_time = 0.0

    @classmethod
    def centipede(cls, mountpoint: str) -> 'NTRIPClient':
        """
        Create client for Centipede RTK (France - FREE).

        Find your nearest base station at: https://centipede.fr/index.php/view/map

        Args:
            mountpoint: Name of nearest base station (e.g., "LIENSS", "CT69", "METZ")

        Returns:
            Configured NTRIPClient
        """
        config = NTRIPConfig(
            host="caster.centipede.fr",
            port=2101,
            mountpoint=mountpoint,
            username="centipede",
            password="centipede"
        )
        return cls(config)

    @classmethod
    def rtk2go(cls, mountpoint: str) -> 'NTRIPClient':
        """
        Create client for RTK2GO (Global - FREE).

        Find base stations at: http://rtk2go.com:2101

        Args:
            mountpoint: Name of base station

        Returns:
            Configured NTRIPClient
        """
        config = NTRIPConfig(
            host="rtk2go.com",
            port=2101,
            mountpoint=mountpoint,
            username="",
            password=""
        )
        return cls(config)

    def set_position(self, latitude: float, longitude: float, altitude: float = 0.0):
        """Set current position for GGA messages."""
        with self._position_lock:
            self._latitude = latitude
            self._longitude = longitude
            self._altitude = altitude

    def start(self) -> bool:
        """Start the NTRIP client."""
        if self._running:
            return True

        if not self.config.mountpoint:
            print("[NTRIP] ERROR: No mountpoint specified!")
            print("        Find your nearest base at https://centipede.fr/index.php/view/map")
            return False

        self._running = True

        # Start receive thread
        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True,
            name="NTRIPReceive"
        )
        self._receive_thread.start()

        # Start GGA thread
        if self.config.send_gga:
            self._gga_thread = threading.Thread(
                target=self._gga_loop,
                daemon=True,
                name="NTRIPGGA"
            )
            self._gga_thread.start()

        return True

    def stop(self):
        """Stop the NTRIP client."""
        self._running = False

        if self._receive_thread:
            self._receive_thread.join(timeout=2.0)
        if self._gga_thread:
            self._gga_thread.join(timeout=2.0)

        self._disconnect()

    def get_corrections(self, timeout: float = 0.1) -> Optional[bytes]:
        """Get RTCM correction data."""
        try:
            return self._correction_queue.get(timeout=timeout)
        except Empty:
            return None

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected

    @property
    def bytes_received(self) -> int:
        """Total bytes received."""
        return self._bytes_received

    @property
    def seconds_since_correction(self) -> float:
        """Seconds since last correction."""
        if self._last_correction_time == 0:
            return float('inf')
        return time.time() - self._last_correction_time

    def _connect(self) -> bool:
        """Connect to NTRIP caster."""
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(10.0)

            print(f"[NTRIP] Connecting to {self.config.host}:{self.config.port}...")
            self._socket.connect((self.config.host, self.config.port))

            # Send NTRIP request
            if not self._send_request():
                self._disconnect()
                return False

            self._connected = True
            print(f"[NTRIP] Connected to {self.config.mountpoint}")
            return True

        except socket.timeout:
            print("[NTRIP] Connection timeout")
            return False
        except socket.error as e:
            print(f"[NTRIP] Connection failed: {e}")
            return False

    def _disconnect(self):
        """Disconnect from caster."""
        self._connected = False
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None

    def _send_request(self) -> bool:
        """Send NTRIP request."""
        try:
            # Build request
            request = f"GET /{self.config.mountpoint} HTTP/1.1\r\n"
            request += f"Host: {self.config.host}\r\n"
            request += f"Ntrip-Version: {self.NTRIP_VERSION}\r\n"
            request += f"User-Agent: {self.USER_AGENT}\r\n"

            # Add authentication if provided
            if self.config.username:
                auth = base64.b64encode(
                    f"{self.config.username}:{self.config.password}".encode()
                ).decode()
                request += f"Authorization: Basic {auth}\r\n"

            request += "Accept: */*\r\n"
            request += "Connection: close\r\n"
            request += "\r\n"

            self._socket.sendall(request.encode('ascii'))

            # Read response
            response = b""
            while b"\r\n\r\n" not in response:
                chunk = self._socket.recv(1024)
                if not chunk:
                    return False
                response += chunk

            # Check response
            header_end = response.find(b"\r\n\r\n")
            headers = response[:header_end].decode('ascii', errors='ignore')
            first_line = headers.split('\r\n')[0]

            if '200' in first_line or 'ICY 200' in first_line:
                print("[NTRIP] Authenticated successfully")
                remaining = response[header_end + 4:]
                if remaining:
                    self._process_data(remaining)
                return True
            elif '401' in first_line:
                print("[NTRIP] Authentication failed")
                return False
            elif '404' in first_line:
                print(f"[NTRIP] Mountpoint '{self.config.mountpoint}' not found")
                return False
            else:
                print(f"[NTRIP] Unexpected response: {first_line}")
                return False

        except Exception as e:
            print(f"[NTRIP] Request failed: {e}")
            return False

    def _receive_loop(self):
        """Main receive loop."""
        while self._running:
            if not self._connected:
                if self._connect():
                    pass
                else:
                    time.sleep(5.0)
                    continue

            try:
                self._socket.settimeout(1.0)
                data = self._socket.recv(4096)

                if not data:
                    print("[NTRIP] Connection closed")
                    self._disconnect()
                    continue

                self._process_data(data)

            except socket.timeout:
                pass
            except socket.error as e:
                print(f"[NTRIP] Error: {e}")
                self._disconnect()

    def _process_data(self, data: bytes):
        """Process received RTCM data."""
        self._bytes_received += len(data)
        self._last_correction_time = time.time()

        try:
            self._correction_queue.put_nowait(data)
        except:
            try:
                self._correction_queue.get_nowait()
                self._correction_queue.put_nowait(data)
            except:
                pass

    def _gga_loop(self):
        """Send periodic GGA updates."""
        while self._running:
            if self._connected:
                self._send_gga()
            time.sleep(self.config.gga_interval)

    def _send_gga(self):
        """Send GGA position to caster."""
        if not self._connected or not self._socket:
            return

        with self._position_lock:
            lat = self._latitude
            lon = self._longitude
            alt = self._altitude

        if lat == 0.0 and lon == 0.0:
            return

        try:
            gga = self._format_gga(lat, lon, alt)
            self._socket.sendall(gga.encode('ascii'))
        except:
            pass

    def _format_gga(self, lat: float, lon: float, alt: float) -> str:
        """Format GGA sentence."""
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

        now = time.gmtime()
        time_str = f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.00"

        gga_data = (
            f"GPGGA,{time_str},{lat_nmea},{lat_dir},{lon_nmea},{lon_dir},"
            f"1,12,1.0,{alt:.1f},M,0.0,M,,"
        )

        checksum = 0
        for char in gga_data:
            checksum ^= ord(char)

        return f"${gga_data}*{checksum:02X}\r\n"

    @staticmethod
    def list_centipede_bases():
        """Print info about finding Centipede bases."""
        print("=" * 60)
        print("CENTIPEDE RTK - Réseau RTK communautaire français GRATUIT")
        print("=" * 60)
        print()
        print("Pour trouver la base la plus proche de ton école:")
        print("  1. Va sur https://centipede.fr/index.php/view/map")
        print("  2. Zoome sur ta position")
        print("  3. Clique sur la base la plus proche")
        print("  4. Note le nom du mountpoint (ex: LIENSS, CT69, METZ)")
        print()
        print("Ensuite utilise:")
        print('  client = NTRIPClient.centipede("NOM_DE_LA_BASE")')
        print()


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        NTRIPClient.list_centipede_bases()
        sys.exit(0)

    mountpoint = sys.argv[1]
    lat = float(sys.argv[2]) if len(sys.argv) > 2 else 48.8566
    lon = float(sys.argv[3]) if len(sys.argv) > 3 else 2.3522

    print(f"[Test] Connecting to Centipede base: {mountpoint}")

    client = NTRIPClient.centipede(mountpoint)
    client.set_position(lat, lon, 35.0)
    client.start()

    try:
        start = time.time()
        while time.time() - start < 30:
            time.sleep(1)
            print(f"[Test] Connected: {client.is_connected()}, "
                  f"Bytes: {client.bytes_received}, "
                  f"Last: {client.seconds_since_correction:.1f}s ago")
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        print(f"[Test] Total bytes: {client.bytes_received}")