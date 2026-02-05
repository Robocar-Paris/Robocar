"""
Smart Navigation Module for Robocar

Provides autonomous navigation to GPS waypoints with smart obstacle avoidance
using OAK-D Lite depth sensing.

Features:
- GPS navigation with arrival detection
- Smart obstacle avoidance (depth sector analysis)
- Integration with VESC motor control
- Real-time status updates for visualization

Usage:
    from navigation.smart_navigation import SmartNavigator

    navigator = SmartNavigator(
        vesc=vesc_controller,
        gps=gps_driver,
        camera=oak_camera
    )

    # Navigate to target
    success = navigator.start_navigation(
        target_lat=48.8156,
        target_lon=2.3632
    )
"""

import math
import time
import threading
from typing import Optional, Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum


@dataclass
class NavigationConfig:
    """Configuration for smart navigation."""
    # Arrival
    arrival_radius_m: float = 1.0         # Distance to consider arrival (meters)

    # Speed control
    cruise_speed: float = 0.4             # Normal driving speed (0-1)
    slow_speed: float = 0.2               # Speed when obstacle detected
    min_speed: float = 0.1                # Minimum speed

    # Obstacle avoidance (using depth camera)
    warning_distance_m: float = 1.5       # Start avoiding at this distance
    critical_distance_m: float = 0.5      # Must stop/turn at this distance
    # Note: Emergency stop at 0.3m is handled by SafetyMonitor

    # Steering
    gps_steering_gain: float = 2.0        # Gain for GPS heading correction
    avoidance_steering: float = 0.6       # Steering intensity when avoiding
    max_steering: float = 0.8             # Maximum steering value

    # Timing
    loop_rate_hz: int = 20                # Control loop frequency

    # GPS reference (Epitech Paris default)
    origin_lat: float = 48.8156
    origin_lon: float = 2.3631


class NavigationAction(Enum):
    """Current navigation action."""
    NAVIGATING = "navigating"
    AVOIDING_LEFT = "avoiding_left"
    AVOIDING_RIGHT = "avoiding_right"
    STOPPING = "stopping"
    ARRIVED = "arrived"
    ERROR = "error"


@dataclass
class NavigationStatus:
    """Real-time navigation status for visualization."""
    # Position
    current_lat: float = 0.0
    current_lon: float = 0.0
    target_lat: float = 0.0
    target_lon: float = 0.0

    # Navigation
    distance_to_target_m: float = float('inf')
    bearing_to_target_deg: float = 0.0
    current_heading_deg: float = 0.0
    heading_error_deg: float = 0.0

    # Obstacle avoidance
    obstacle_detected: bool = False
    obstacle_distance_m: float = float('inf')
    left_sector_distance_m: float = float('inf')
    right_sector_distance_m: float = float('inf')

    # Control
    action: NavigationAction = NavigationAction.NAVIGATING
    speed: float = 0.0
    steering: float = 0.0

    # Status
    gps_quality: str = "NO_FIX"
    is_running: bool = False
    elapsed_time_s: float = 0.0
    timestamp: float = 0.0


class SmartNavigator:
    """
    Smart autonomous navigator with GPS targeting and obstacle avoidance.

    Integrates:
    - VESC motor control for speed/steering
    - GPS RTK for positioning
    - OAK-D Lite depth camera for obstacle avoidance
    - SafetyMonitor (external) for emergency stops
    """

    # Earth radius for GPS calculations
    EARTH_RADIUS_M = 6371000.0

    def __init__(
        self,
        vesc,          # VESCController instance
        gps,           # GPSRTKDriver instance
        camera,        # OakDCamera instance
        config: Optional[NavigationConfig] = None,
        safety_monitor=None  # Optional SafetyMonitor for emergency stops
    ):
        """
        Initialize smart navigator.

        Args:
            vesc: VESC motor controller
            gps: GPS RTK driver
            camera: OAK-D camera driver
            config: Navigation configuration
            safety_monitor: Safety monitor (optional, for emergency callbacks)
        """
        self.vesc = vesc
        self.gps = gps
        self.camera = camera
        self.config = config or NavigationConfig()
        self.safety_monitor = safety_monitor

        # Target
        self._target_lat = 0.0
        self._target_lon = 0.0

        # State
        self._running = False
        self._nav_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Status for visualization
        self._status = NavigationStatus()
        self._start_time = 0.0

        # Callbacks
        self._status_callback: Optional[Callable[[NavigationStatus], None]] = None
        self._arrival_callback: Optional[Callable[[], None]] = None

        # Heading estimation (from GPS movement)
        self._prev_lat = 0.0
        self._prev_lon = 0.0
        self._estimated_heading = 0.0

    def start_navigation(
        self,
        target_lat: float,
        target_lon: float,
        blocking: bool = True,
        timeout_s: float = 300.0
    ) -> bool:
        """
        Start navigation to target GPS coordinate.

        Main navigation function that:
        1. Gets current GPS position
        2. Checks arrival condition (< arrival_radius)
        3. Analyzes depth map for obstacles
        4. Decides: Navigate to GPS or Avoid obstacle
        5. Sends commands to VESC
        6. Updates dashboard

        Args:
            target_lat: Target latitude (degrees)
            target_lon: Target longitude (degrees)
            blocking: If True, wait until arrival/timeout
            timeout_s: Maximum navigation time in seconds

        Returns:
            True if arrived successfully, False otherwise
        """
        if self._running:
            print("[NAV] Navigation already in progress")
            return False

        self._target_lat = target_lat
        self._target_lon = target_lon
        self._start_time = time.time()

        # Verify GPS fix
        gps_pos = self.gps.get_position()
        if not gps_pos or not gps_pos.is_valid:
            print("[NAV] ERROR: No GPS fix available")
            return False

        print(f"[NAV] Starting navigation to ({target_lat:.6f}, {target_lon:.6f})")
        print(f"[NAV] Current position: ({gps_pos.latitude:.6f}, {gps_pos.longitude:.6f})")
        print(f"[NAV] GPS Quality: {gps_pos.quality_string}")

        # Calculate initial distance
        initial_dist = self._calculate_distance(
            gps_pos.latitude, gps_pos.longitude,
            target_lat, target_lon
        )
        print(f"[NAV] Distance to target: {initial_dist:.1f}m")

        # Start navigation thread
        self._running = True
        self._nav_thread = threading.Thread(
            target=self._navigation_loop,
            args=(timeout_s,),
            daemon=True,
            name="SmartNavigation"
        )
        self._nav_thread.start()

        if blocking:
            # Wait for completion
            self._nav_thread.join(timeout=timeout_s + 5.0)
            return self._status.action == NavigationAction.ARRIVED
        else:
            return True

    def stop_navigation(self):
        """Stop navigation and halt the car."""
        self._running = False

        # Stop motors
        self.vesc.set_speed(0)
        self.vesc.set_steering(0)
        self.vesc.brake(10.0)  # Light brake

        if self._nav_thread:
            self._nav_thread.join(timeout=2.0)

        print("[NAV] Navigation stopped")

    def _navigation_loop(self, timeout_s: float):
        """
        Main navigation control loop.

        Runs at configured rate (default 20 Hz).
        Loop: Get Data -> Check Arrival -> Check Obstacles -> Navigate/Avoid -> Update
        """
        loop_period = 1.0 / self.config.loop_rate_hz

        while self._running:
            loop_start = time.time()

            try:
                # === STEP 1: GET SENSOR DATA ===
                gps_pos = self.gps.get_position()
                depth_analysis = self.camera.get_depth_analysis()

                # Update elapsed time
                elapsed = time.time() - self._start_time

                # Check timeout
                if elapsed > timeout_s:
                    print(f"[NAV] Timeout after {elapsed:.1f}s")
                    self._update_status(NavigationAction.ERROR)
                    break

                # Skip if no GPS
                if not gps_pos or not gps_pos.is_valid:
                    print("[NAV] Lost GPS fix, stopping...")
                    self._update_status(NavigationAction.STOPPING)
                    self.vesc.set_speed(0)
                    time.sleep(loop_period)
                    continue

                # Calculate navigation metrics
                distance = self._calculate_distance(
                    gps_pos.latitude, gps_pos.longitude,
                    self._target_lat, self._target_lon
                )
                bearing = self._calculate_bearing(
                    gps_pos.latitude, gps_pos.longitude,
                    self._target_lat, self._target_lon
                )

                # Update heading estimate from GPS movement
                self._update_heading_estimate(gps_pos.latitude, gps_pos.longitude)

                # === STEP 2: CHECK ARRIVAL ===
                if distance < self.config.arrival_radius_m:
                    print(f"[NAV] ARRIVED! Distance: {distance:.2f}m")
                    self.vesc.set_speed(0)
                    self.vesc.brake(15.0)
                    self._update_status(NavigationAction.ARRIVED, gps_pos, distance, bearing)
                    if self._arrival_callback:
                        self._arrival_callback()
                    break

                # === STEP 3: CHECK OBSTACLES (Depth Analysis) ===
                action = NavigationAction.NAVIGATING
                speed = self.config.cruise_speed
                steering = 0.0

                if depth_analysis:
                    center_dist = depth_analysis.center_min_distance
                    left_dist = depth_analysis.left_avg_distance
                    right_dist = depth_analysis.right_avg_distance

                    # Critical obstacle in front - MUST avoid
                    if center_dist < self.config.critical_distance_m:
                        speed = self.config.slow_speed

                        # Decide left or right based on depth sectors
                        if left_dist > right_dist:
                            # More space on left - turn left
                            action = NavigationAction.AVOIDING_LEFT
                            steering = -self.config.avoidance_steering
                            print(f"[NAV] AVOIDING LEFT (L:{left_dist:.1f}m > R:{right_dist:.1f}m)")
                        else:
                            # More space on right - turn right
                            action = NavigationAction.AVOIDING_RIGHT
                            steering = self.config.avoidance_steering
                            print(f"[NAV] AVOIDING RIGHT (R:{right_dist:.1f}m > L:{left_dist:.1f}m)")

                    # Warning distance - slow down and consider avoidance
                    elif center_dist < self.config.warning_distance_m:
                        speed = self.config.slow_speed

                        # Blend GPS steering with obstacle consideration
                        heading_error = self._normalize_angle(bearing - self._estimated_heading)
                        gps_steering = self._clamp(
                            heading_error * self.config.gps_steering_gain / 90.0,
                            -self.config.max_steering,
                            self.config.max_steering
                        )

                        # Bias towards open space while still going towards target
                        if left_dist > right_dist + 0.5:
                            # Bias left
                            steering = gps_steering - 0.2
                            action = NavigationAction.AVOIDING_LEFT
                        elif right_dist > left_dist + 0.5:
                            # Bias right
                            steering = gps_steering + 0.2
                            action = NavigationAction.AVOIDING_RIGHT
                        else:
                            steering = gps_steering

                # === STEP 4: NORMAL GPS NAVIGATION ===
                if action == NavigationAction.NAVIGATING:
                    # Calculate steering based on heading error
                    heading_error = self._normalize_angle(bearing - self._estimated_heading)

                    steering = self._clamp(
                        heading_error * self.config.gps_steering_gain / 90.0,
                        -self.config.max_steering,
                        self.config.max_steering
                    )

                    # Slow down when making sharp turns
                    if abs(steering) > 0.5:
                        speed = self.config.slow_speed

                # === STEP 5: APPLY SAFETY CHECK ===
                # Note: Emergency stop at 0.3m is handled by SafetyMonitor
                if self.safety_monitor:
                    obstacle_dist = depth_analysis.center_min_distance if depth_analysis else float('inf')
                    vesc_state = self.vesc.get_state()
                    speed = self.safety_monitor.check_and_limit(
                        requested_velocity=speed,
                        nearest_obstacle_distance=obstacle_dist,
                        battery_voltage=vesc_state.voltage if vesc_state else 12.0,
                        motor_temp=vesc_state.temp_motor if vesc_state else 25.0,
                        mos_temp=vesc_state.temp_mos if vesc_state else 25.0
                    )

                # === STEP 6: SEND COMMANDS TO VESC ===
                self.vesc.set_speed(speed)
                self.vesc.set_steering(steering)

                # === STEP 7: UPDATE STATUS FOR DASHBOARD ===
                self._update_status(
                    action=action,
                    gps_pos=gps_pos,
                    distance=distance,
                    bearing=bearing,
                    depth_analysis=depth_analysis,
                    speed=speed,
                    steering=steering
                )

            except Exception as e:
                print(f"[NAV] Loop error: {e}")
                self.vesc.set_speed(0)

            # Maintain loop rate
            elapsed_loop = time.time() - loop_start
            sleep_time = loop_period - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Cleanup
        self._running = False
        self.vesc.set_speed(0)
        self.vesc.set_steering(0)

    def _update_status(
        self,
        action: NavigationAction,
        gps_pos=None,
        distance: float = 0.0,
        bearing: float = 0.0,
        depth_analysis=None,
        speed: float = 0.0,
        steering: float = 0.0
    ):
        """Update navigation status for visualization."""
        with self._lock:
            self._status.action = action
            self._status.speed = speed
            self._status.steering = steering
            self._status.is_running = self._running
            self._status.elapsed_time_s = time.time() - self._start_time
            self._status.timestamp = time.time()

            self._status.target_lat = self._target_lat
            self._status.target_lon = self._target_lon
            self._status.distance_to_target_m = distance
            self._status.bearing_to_target_deg = bearing

            if gps_pos:
                self._status.current_lat = gps_pos.latitude
                self._status.current_lon = gps_pos.longitude
                self._status.gps_quality = gps_pos.quality_string
                self._status.current_heading_deg = self._estimated_heading
                self._status.heading_error_deg = self._normalize_angle(bearing - self._estimated_heading)

            if depth_analysis:
                self._status.obstacle_detected = depth_analysis.obstacle_detected
                self._status.obstacle_distance_m = depth_analysis.center_min_distance
                self._status.left_sector_distance_m = depth_analysis.left_avg_distance
                self._status.right_sector_distance_m = depth_analysis.right_avg_distance

        # Call status callback if set
        if self._status_callback:
            self._status_callback(self._status)

    def get_status(self) -> NavigationStatus:
        """Get current navigation status."""
        with self._lock:
            return NavigationStatus(**self._status.__dict__)

    def set_status_callback(self, callback: Callable[[NavigationStatus], None]):
        """Set callback for status updates."""
        self._status_callback = callback

    def set_arrival_callback(self, callback: Callable[[], None]):
        """Set callback for arrival event."""
        self._arrival_callback = callback

    # === GPS CALCULATIONS ===

    def _calculate_distance(
        self,
        lat1: float, lon1: float,
        lat2: float, lon2: float
    ) -> float:
        """
        Calculate distance between two GPS coordinates using Haversine formula.

        Returns:
            Distance in meters
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return self.EARTH_RADIUS_M * c

    def _calculate_bearing(
        self,
        lat1: float, lon1: float,
        lat2: float, lon2: float
    ) -> float:
        """
        Calculate bearing from point 1 to point 2.

        Returns:
            Bearing in degrees (0-360, where 0=North, 90=East)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))

        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

    def _update_heading_estimate(self, lat: float, lon: float):
        """
        Estimate current heading from GPS movement.

        Uses position change between samples to estimate heading.
        """
        if self._prev_lat != 0.0 and self._prev_lon != 0.0:
            # Only update if moved significantly (reduces noise)
            dist = self._calculate_distance(
                self._prev_lat, self._prev_lon,
                lat, lon
            )
            if dist > 0.1:  # At least 10cm movement
                self._estimated_heading = self._calculate_bearing(
                    self._prev_lat, self._prev_lon,
                    lat, lon
                )

        self._prev_lat = lat
        self._prev_lon = lon

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 to 180 degrees."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(max_val, value))

    @property
    def is_running(self) -> bool:
        """Check if navigation is running."""
        return self._running

    @property
    def is_arrived(self) -> bool:
        """Check if arrived at target."""
        return self._status.action == NavigationAction.ARRIVED


# === UTILITY FUNCTIONS ===

def gps_to_local(
    lat: float, lon: float,
    origin_lat: float, origin_lon: float
) -> Tuple[float, float]:
    """
    Convert GPS coordinates to local X/Y (meters).

    Args:
        lat, lon: Target coordinates
        origin_lat, origin_lon: Reference origin

    Returns:
        Tuple of (x, y) in meters
    """
    EARTH_RADIUS = 6371000.0

    # Approximate conversion (good for small distances)
    lat_diff = math.radians(lat - origin_lat)
    lon_diff = math.radians(lon - origin_lon)
    avg_lat = math.radians((lat + origin_lat) / 2)

    x = lon_diff * EARTH_RADIUS * math.cos(avg_lat)
    y = lat_diff * EARTH_RADIUS

    return x, y


def local_to_gps(
    x: float, y: float,
    origin_lat: float, origin_lon: float
) -> Tuple[float, float]:
    """
    Convert local X/Y (meters) to GPS coordinates.

    Args:
        x, y: Local coordinates in meters
        origin_lat, origin_lon: Reference origin

    Returns:
        Tuple of (latitude, longitude)
    """
    EARTH_RADIUS = 6371000.0

    lat_diff = y / EARTH_RADIUS
    lon_diff = x / (EARTH_RADIUS * math.cos(math.radians(origin_lat)))

    lat = origin_lat + math.degrees(lat_diff)
    lon = origin_lon + math.degrees(lon_diff)

    return lat, lon


# Test code
if __name__ == '__main__':
    print("[NAV] Smart Navigation module loaded")
    print("[NAV] This module requires VESC, GPS, and Camera drivers to be initialized")
    print("[NAV] See start_navigation() for usage")
