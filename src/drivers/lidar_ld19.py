"""
LiDAR LD19 Driver for Robocar

The LD19 is a low-cost 360° 2D LiDAR from Ldrobot.
- Range: 0.02m - 12m
- Scan frequency: 5-13Hz (configurable)
- Angular resolution: 1°
- Interface: UART (230400 baud)

Protocol:
- Each packet: 47 bytes
- Header: 0x54 (start byte)
- Contains 12 measurement points per packet
- CRC8 checksum at end

References:
- https://github.com/ldrobotSensorTeam/ldlidar_stl_ros
- Datasheet: LD19 Datasheet V1.0
"""

import serial
import struct
import threading
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass
from collections import deque
import math


@dataclass
class LidarPoint:
    """Single LiDAR measurement point."""
    angle: float       # Angle in radians (0 to 2*pi)
    distance: float    # Distance in meters
    intensity: int     # Signal intensity (0-255)
    valid: bool        # Whether measurement is valid


@dataclass
class LidarScan:
    """Complete 360° LiDAR scan."""
    timestamp: float           # Unix timestamp
    points: List[LidarPoint]   # List of measurement points
    scan_frequency: float      # Scan frequency in Hz


class LD19Driver:
    """
    Driver for LD19 LiDAR sensor.

    Usage:
        lidar = LD19Driver('/dev/ttyUSB0')
        lidar.start()

        while True:
            scan = lidar.get_scan()
            if scan:
                for point in scan.points:
                    print(f"Angle: {point.angle:.2f}, Distance: {point.distance:.2f}m")

        lidar.stop()
    """

    # Protocol constants
    HEADER = 0x54
    PACKET_SIZE = 47
    POINTS_PER_PACKET = 12

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 230400,
        buffer_size: int = 10
    ):
        """
        Initialize LD19 driver.

        Args:
            port: Serial port path
            baudrate: Serial baudrate (default 230400 for LD19)
            buffer_size: Number of complete scans to buffer
        """
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size

        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._scan_buffer: deque = deque(maxlen=buffer_size)
        self._current_scan_points: List[LidarPoint] = []
        self._last_angle: float = 0.0
        self._lock = threading.Lock()

    def start(self) -> bool:
        """
        Start the LiDAR driver.

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
            print(f"[LD19] Failed to open serial port: {e}")
            return False

    def stop(self):
        """Stop the LiDAR driver."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial:
            self._serial.close()
            self._serial = None

    def get_scan(self, timeout: float = 0.5) -> Optional[LidarScan]:
        """
        Get the latest complete scan.

        Args:
            timeout: Maximum time to wait for a scan (seconds)

        Returns:
            LidarScan object or None if no scan available
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._lock:
                if self._scan_buffer:
                    return self._scan_buffer.popleft()
            time.sleep(0.01)
        return None

    def get_latest_scan(self) -> Optional[LidarScan]:
        """
        Get the most recent scan, discarding older ones.

        Returns:
            LidarScan object or None if no scan available
        """
        with self._lock:
            if self._scan_buffer:
                scan = self._scan_buffer[-1]
                self._scan_buffer.clear()
                return scan
        return None

    def _read_loop(self):
        """Main reading loop (runs in separate thread)."""
        buffer = bytearray()

        while self._running:
            try:
                # Read available data
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    buffer.extend(data)

                    # Process complete packets
                    while len(buffer) >= self.PACKET_SIZE:
                        # Find header
                        header_idx = buffer.find(bytes([self.HEADER]))
                        if header_idx == -1:
                            buffer.clear()
                            break
                        elif header_idx > 0:
                            # Discard bytes before header
                            buffer = buffer[header_idx:]

                        if len(buffer) < self.PACKET_SIZE:
                            break

                        # Extract and parse packet
                        packet = bytes(buffer[:self.PACKET_SIZE])
                        buffer = buffer[self.PACKET_SIZE:]

                        if self._verify_checksum(packet):
                            self._parse_packet(packet)
                else:
                    time.sleep(0.001)

            except serial.SerialException as e:
                print(f"[LD19] Serial error: {e}")
                time.sleep(0.1)

    def _verify_checksum(self, packet: bytes) -> bool:
        """
        Verify packet CRC8 checksum.

        Args:
            packet: Raw packet bytes

        Returns:
            True if checksum is valid
        """
        # CRC8 table for polynomial 0x4D
        crc_table = [
            0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
            0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
            0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
            0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
            0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
            0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
            0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
            0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
            0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
            0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
            0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
            0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
            0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
            0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
            0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
            0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
            0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
            0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
            0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
            0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
            0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
            0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
            0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
            0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
            0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
            0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
            0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
            0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
            0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
            0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
            0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
            0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
        ]

        crc = 0
        for byte in packet[:-1]:
            crc = crc_table[(crc ^ byte) & 0xFF]

        return crc == packet[-1]

    def _parse_packet(self, packet: bytes):
        """
        Parse a valid packet and extract measurement points.

        Packet structure (47 bytes):
        - Byte 0: Header (0x54)
        - Byte 1: VerLen (version + length)
        - Bytes 2-3: Speed (deg/s, little-endian)
        - Bytes 4-5: Start angle (0.01 deg, little-endian)
        - Bytes 6-41: 12 points, 3 bytes each (distance + intensity)
        - Bytes 42-43: End angle (0.01 deg, little-endian)
        - Bytes 44-45: Timestamp (ms, little-endian)
        - Byte 46: CRC8 checksum
        """
        # Extract angles
        start_angle = struct.unpack('<H', packet[4:6])[0] * 0.01  # degrees
        end_angle = struct.unpack('<H', packet[42:44])[0] * 0.01  # degrees

        # Handle angle wraparound
        if end_angle < start_angle:
            end_angle += 360.0

        # Calculate angle step
        angle_step = (end_angle - start_angle) / (self.POINTS_PER_PACKET - 1)

        # Parse each point
        for i in range(self.POINTS_PER_PACKET):
            offset = 6 + i * 3
            distance = struct.unpack('<H', packet[offset:offset+2])[0] / 1000.0  # mm to m
            intensity = packet[offset + 2]

            angle_deg = (start_angle + i * angle_step) % 360.0
            angle_rad = math.radians(angle_deg)

            # Validate measurement
            valid = 0.02 < distance < 12.0 and intensity > 0

            point = LidarPoint(
                angle=angle_rad,
                distance=distance,
                intensity=intensity,
                valid=valid
            )

            self._current_scan_points.append(point)

        # Check if we completed a full scan (angle wrapped around)
        current_angle = start_angle
        if current_angle < self._last_angle - 180:  # Wrapped around
            self._complete_scan()
        self._last_angle = current_angle

    def _complete_scan(self):
        """Finalize and buffer the current scan."""
        if len(self._current_scan_points) > 100:  # Minimum points for valid scan
            scan = LidarScan(
                timestamp=time.time(),
                points=self._current_scan_points.copy(),
                scan_frequency=len(self._current_scan_points) / 360.0 * 10  # Approximate
            )
            with self._lock:
                self._scan_buffer.append(scan)

        self._current_scan_points.clear()

    @property
    def is_running(self) -> bool:
        """Check if driver is running."""
        return self._running


# Convenience functions for testing
def get_scan_as_arrays(scan: LidarScan) -> Tuple[List[float], List[float]]:
    """
    Convert scan to arrays for plotting.

    Returns:
        Tuple of (angles, distances) for valid points only
    """
    angles = []
    distances = []
    for point in scan.points:
        if point.valid:
            angles.append(point.angle)
            distances.append(point.distance)
    return angles, distances


def scan_to_cartesian(scan: LidarScan) -> List[Tuple[float, float]]:
    """
    Convert scan to Cartesian coordinates (x, y).

    Returns:
        List of (x, y) tuples in meters
    """
    points = []
    for point in scan.points:
        if point.valid:
            x = point.distance * math.cos(point.angle)
            y = point.distance * math.sin(point.angle)
            points.append((x, y))
    return points


if __name__ == '__main__':
    # Simple test
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    print(f"[LD19] Testing LiDAR on {port}...")

    lidar = LD19Driver(port)
    if lidar.start():
        print("[LD19] Started successfully")
        try:
            for _ in range(10):
                scan = lidar.get_scan(timeout=1.0)
                if scan:
                    valid_points = sum(1 for p in scan.points if p.valid)
                    print(f"[LD19] Scan: {valid_points} valid points, "
                          f"freq={scan.scan_frequency:.1f}Hz")
                else:
                    print("[LD19] No scan received")
        finally:
            lidar.stop()
    else:
        print("[LD19] Failed to start")
