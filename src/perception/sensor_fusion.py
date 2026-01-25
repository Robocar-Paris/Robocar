"""
Sensor Fusion Module

Fuses data from multiple sensors (LiDAR, GPS, odometry) to provide
unified state estimation for the robot.

Components:
- Data synchronization
- State estimation (position, velocity, orientation)
- Covariance tracking
- Sensor health monitoring
"""

import time
import math
import threading
from typing import Optional, Tuple, List, Dict, Callable
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import numpy as np

from .transforms import Pose2D, Transform2D, GPSConverter, normalize_angle


class SensorType(Enum):
    """Type of sensor."""
    LIDAR = 1
    GPS = 2
    ODOMETRY = 3
    IMU = 4


@dataclass
class SensorReading:
    """Generic sensor reading."""
    sensor_type: SensorType
    timestamp: float
    data: dict
    covariance: Optional[np.ndarray] = None


@dataclass
class RobotState:
    """Estimated robot state."""
    timestamp: float
    pose: Pose2D                    # (x, y, theta) in world frame
    velocity: Tuple[float, float]   # (vx, vy) in world frame
    angular_velocity: float         # omega (rad/s)
    covariance: np.ndarray          # 6x6 covariance matrix [x, y, theta, vx, vy, omega]

    @property
    def speed(self) -> float:
        """Linear speed."""
        return math.sqrt(self.velocity[0]**2 + self.velocity[1]**2)

    @property
    def position(self) -> Tuple[float, float]:
        """Position as tuple."""
        return (self.pose.x, self.pose.y)

    @property
    def heading(self) -> float:
        """Heading angle."""
        return self.pose.theta

    def predict(self, dt: float) -> 'RobotState':
        """Predict state after dt seconds."""
        # Simple motion model
        new_theta = normalize_angle(self.pose.theta + self.angular_velocity * dt)
        new_x = self.pose.x + self.velocity[0] * dt
        new_y = self.pose.y + self.velocity[1] * dt

        return RobotState(
            timestamp=self.timestamp + dt,
            pose=Pose2D(new_x, new_y, new_theta),
            velocity=self.velocity,
            angular_velocity=self.angular_velocity,
            covariance=self.covariance.copy()
        )


@dataclass
class SensorStatus:
    """Health status of a sensor."""
    sensor_type: SensorType
    is_healthy: bool
    last_update: float
    update_rate: float      # Hz
    error_count: int
    message: str = ""


class SensorFusion:
    """
    Fuses multiple sensors for state estimation.

    Uses an Extended Kalman Filter (EKF) approach to combine
    GPS, LiDAR-based localization, and odometry.

    Usage:
        fusion = SensorFusion()
        fusion.set_gps_origin(48.8566, 2.3522)

        # In main loop:
        fusion.update_odometry(distance, delta_theta)
        fusion.update_gps(lat, lon, accuracy)
        state = fusion.get_state()
    """

    def __init__(
        self,
        update_rate: float = 50.0,
        gps_timeout: float = 2.0,
        odom_timeout: float = 0.5,
        lidar_timeout: float = 0.5
    ):
        """
        Initialize sensor fusion.

        Args:
            update_rate: Target fusion update rate (Hz)
            gps_timeout: GPS timeout before marking unhealthy (s)
            odom_timeout: Odometry timeout (s)
            lidar_timeout: LiDAR timeout (s)
        """
        self.update_rate = update_rate
        self.gps_timeout = gps_timeout
        self.odom_timeout = odom_timeout
        self.lidar_timeout = lidar_timeout

        # State
        self._state: Optional[RobotState] = None
        self._initialized = False
        self._lock = threading.Lock()

        # Sensor data buffers
        self._gps_buffer: deque = deque(maxlen=10)
        self._odom_buffer: deque = deque(maxlen=100)
        self._lidar_buffer: deque = deque(maxlen=10)

        # Sensor status
        self._sensor_status: Dict[SensorType, SensorStatus] = {}
        self._init_sensor_status()

        # GPS converter
        self._gps_converter = GPSConverter()
        self._gps_origin_set = False

        # EKF parameters
        self._init_ekf_parameters()

        # Callbacks
        self._state_callbacks: List[Callable[[RobotState], None]] = []

    def _init_sensor_status(self):
        """Initialize sensor status tracking."""
        for sensor_type in SensorType:
            self._sensor_status[sensor_type] = SensorStatus(
                sensor_type=sensor_type,
                is_healthy=False,
                last_update=0.0,
                update_rate=0.0,
                error_count=0
            )

    def _init_ekf_parameters(self):
        """Initialize EKF parameters."""
        # Process noise (how much state changes between updates)
        self._Q = np.diag([
            0.1,    # x position variance
            0.1,    # y position variance
            0.05,   # theta variance
            0.5,    # vx variance
            0.5,    # vy variance
            0.2     # omega variance
        ])

        # Measurement noise for different sensors
        self._R_gps = np.diag([0.02, 0.02])  # GPS (RTK fixed ~2cm)
        self._R_gps_float = np.diag([0.5, 0.5])  # GPS (RTK float ~50cm)
        self._R_odom = np.diag([0.1, 0.05])  # Odometry (distance, angle)

        # Initial state covariance
        self._P0 = np.diag([1.0, 1.0, 0.5, 0.1, 0.1, 0.1])

    def set_gps_origin(self, lat: float, lon: float, alt: float = 0.0):
        """Set GPS origin for local coordinates."""
        self._gps_converter.set_origin(lat, lon, alt)
        self._gps_origin_set = True

    def update_gps(
        self,
        lat: float,
        lon: float,
        accuracy: float = 0.02,
        quality: int = 4
    ):
        """
        Update with GPS measurement.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            accuracy: Horizontal accuracy (meters)
            quality: GPS quality (4=RTK fixed, 5=RTK float)
        """
        timestamp = time.time()

        # Set origin if not set
        if not self._gps_origin_set:
            self.set_gps_origin(lat, lon)

        # Convert to local coordinates
        east, north, _ = self._gps_converter.gps_to_local(lat, lon)

        reading = SensorReading(
            sensor_type=SensorType.GPS,
            timestamp=timestamp,
            data={
                'x': east,
                'y': north,
                'accuracy': accuracy,
                'quality': quality
            }
        )

        with self._lock:
            self._gps_buffer.append(reading)
            self._update_sensor_status(SensorType.GPS, timestamp)

        self._process_update()

    def update_odometry(
        self,
        distance: float,
        delta_theta: float,
        dt: float
    ):
        """
        Update with odometry measurement.

        Args:
            distance: Distance traveled (meters)
            delta_theta: Change in heading (radians)
            dt: Time since last update (seconds)
        """
        timestamp = time.time()

        reading = SensorReading(
            sensor_type=SensorType.ODOMETRY,
            timestamp=timestamp,
            data={
                'distance': distance,
                'delta_theta': delta_theta,
                'dt': dt
            }
        )

        with self._lock:
            self._odom_buffer.append(reading)
            self._update_sensor_status(SensorType.ODOMETRY, timestamp)

        self._process_update()

    def update_lidar_pose(
        self,
        x: float,
        y: float,
        theta: float,
        covariance: Optional[np.ndarray] = None
    ):
        """
        Update with LiDAR-based pose estimate (from scan matching).

        Args:
            x: X position in world frame
            y: Y position in world frame
            theta: Heading in world frame
            covariance: 3x3 pose covariance matrix
        """
        timestamp = time.time()

        reading = SensorReading(
            sensor_type=SensorType.LIDAR,
            timestamp=timestamp,
            data={
                'x': x,
                'y': y,
                'theta': theta
            },
            covariance=covariance
        )

        with self._lock:
            self._lidar_buffer.append(reading)
            self._update_sensor_status(SensorType.LIDAR, timestamp)

        self._process_update()

    def _update_sensor_status(self, sensor_type: SensorType, timestamp: float):
        """Update sensor health status."""
        status = self._sensor_status[sensor_type]

        # Calculate update rate
        if status.last_update > 0:
            dt = timestamp - status.last_update
            if dt > 0:
                new_rate = 1.0 / dt
                status.update_rate = 0.9 * status.update_rate + 0.1 * new_rate

        status.last_update = timestamp
        status.is_healthy = True

    def _process_update(self):
        """Process sensor updates and fuse data."""
        with self._lock:
            if not self._initialized:
                self._try_initialize()
                return

            # Prediction step (using odometry)
            self._ekf_predict()

            # Correction steps
            self._ekf_correct_gps()
            self._ekf_correct_lidar()

            # Check sensor health
            self._check_sensor_health()

            # Notify callbacks
            if self._state:
                for callback in self._state_callbacks:
                    callback(self._state)

    def _try_initialize(self):
        """Try to initialize state from available sensors."""
        # Need at least GPS for initialization
        if not self._gps_buffer:
            return

        gps_reading = self._gps_buffer[-1]

        # Initialize state
        self._state = RobotState(
            timestamp=gps_reading.timestamp,
            pose=Pose2D(
                gps_reading.data['x'],
                gps_reading.data['y'],
                0.0  # Unknown heading initially
            ),
            velocity=(0.0, 0.0),
            angular_velocity=0.0,
            covariance=self._P0.copy()
        )

        self._initialized = True

    def _ekf_predict(self):
        """EKF prediction step using odometry."""
        if not self._odom_buffer or not self._state:
            return

        # Get latest odometry
        odom = self._odom_buffer[-1]
        dt = odom.data['dt']
        distance = odom.data['distance']
        delta_theta = odom.data['delta_theta']

        # Current state
        x, y, theta = self._state.pose.x, self._state.pose.y, self._state.pose.theta

        # Motion model
        theta_mid = theta + delta_theta / 2
        new_x = x + distance * math.cos(theta_mid)
        new_y = y + distance * math.sin(theta_mid)
        new_theta = normalize_angle(theta + delta_theta)

        # Velocity estimate
        vx = distance / dt * math.cos(theta_mid) if dt > 0 else 0
        vy = distance / dt * math.sin(theta_mid) if dt > 0 else 0
        omega = delta_theta / dt if dt > 0 else 0

        # Jacobian of motion model
        F = np.eye(6)
        F[0, 2] = -distance * math.sin(theta_mid)
        F[1, 2] = distance * math.cos(theta_mid)

        # Update state
        self._state = RobotState(
            timestamp=odom.timestamp,
            pose=Pose2D(new_x, new_y, new_theta),
            velocity=(vx, vy),
            angular_velocity=omega,
            covariance=F @ self._state.covariance @ F.T + self._Q * dt
        )

    def _ekf_correct_gps(self):
        """EKF correction step using GPS."""
        if not self._gps_buffer or not self._state:
            return

        gps = self._gps_buffer[-1]

        # Skip if GPS is old
        if self._state.timestamp - gps.timestamp > 0.5:
            return

        # Measurement
        z = np.array([gps.data['x'], gps.data['y']])

        # Predicted measurement
        h = np.array([self._state.pose.x, self._state.pose.y])

        # Innovation
        y = z - h

        # Measurement Jacobian (H)
        H = np.zeros((2, 6))
        H[0, 0] = 1  # dx/dx
        H[1, 1] = 1  # dy/dy

        # Measurement noise
        R = self._R_gps if gps.data['quality'] == 4 else self._R_gps_float

        # Adjust for reported accuracy
        accuracy = gps.data['accuracy']
        R = R * (accuracy / 0.02)  # Scale relative to nominal 2cm

        # Kalman gain
        P = self._state.covariance
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        # State update
        state_vec = np.array([
            self._state.pose.x,
            self._state.pose.y,
            self._state.pose.theta,
            self._state.velocity[0],
            self._state.velocity[1],
            self._state.angular_velocity
        ])

        state_vec = state_vec + K @ y

        # Covariance update
        P = (np.eye(6) - K @ H) @ P

        # Update state
        self._state = RobotState(
            timestamp=self._state.timestamp,
            pose=Pose2D(state_vec[0], state_vec[1], normalize_angle(state_vec[2])),
            velocity=(state_vec[3], state_vec[4]),
            angular_velocity=state_vec[5],
            covariance=P
        )

    def _ekf_correct_lidar(self):
        """EKF correction step using LiDAR pose."""
        if not self._lidar_buffer or not self._state:
            return

        lidar = self._lidar_buffer[-1]

        # Skip if LiDAR is old
        if self._state.timestamp - lidar.timestamp > 0.5:
            return

        # Measurement
        z = np.array([lidar.data['x'], lidar.data['y'], lidar.data['theta']])

        # Predicted measurement
        h = np.array([
            self._state.pose.x,
            self._state.pose.y,
            self._state.pose.theta
        ])

        # Innovation
        y = z - h
        y[2] = normalize_angle(y[2])  # Normalize angle difference

        # Measurement Jacobian
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1

        # Measurement noise
        if lidar.covariance is not None:
            R = lidar.covariance
        else:
            R = np.diag([0.05, 0.05, 0.02])

        # Kalman gain
        P = self._state.covariance
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        # State update
        state_vec = np.array([
            self._state.pose.x,
            self._state.pose.y,
            self._state.pose.theta,
            self._state.velocity[0],
            self._state.velocity[1],
            self._state.angular_velocity
        ])

        state_vec = state_vec + K @ y

        # Covariance update
        P = (np.eye(6) - K @ H) @ P

        # Update state
        self._state = RobotState(
            timestamp=self._state.timestamp,
            pose=Pose2D(state_vec[0], state_vec[1], normalize_angle(state_vec[2])),
            velocity=(state_vec[3], state_vec[4]),
            angular_velocity=state_vec[5],
            covariance=P
        )

    def _check_sensor_health(self):
        """Check health of all sensors."""
        now = time.time()

        for sensor_type, status in self._sensor_status.items():
            timeout = {
                SensorType.GPS: self.gps_timeout,
                SensorType.ODOMETRY: self.odom_timeout,
                SensorType.LIDAR: self.lidar_timeout,
                SensorType.IMU: 0.5
            }.get(sensor_type, 1.0)

            if now - status.last_update > timeout:
                status.is_healthy = False
                status.message = f"Timeout ({now - status.last_update:.1f}s)"

    def get_state(self) -> Optional[RobotState]:
        """Get current robot state."""
        with self._lock:
            return self._state

    def get_pose(self) -> Optional[Pose2D]:
        """Get current pose."""
        state = self.get_state()
        return state.pose if state else None

    def get_position(self) -> Optional[Tuple[float, float]]:
        """Get current position."""
        state = self.get_state()
        return state.position if state else None

    def get_velocity(self) -> Optional[Tuple[float, float]]:
        """Get current velocity."""
        state = self.get_state()
        return state.velocity if state else None

    def get_sensor_status(self) -> Dict[SensorType, SensorStatus]:
        """Get status of all sensors."""
        return self._sensor_status.copy()

    def is_localized(self) -> bool:
        """Check if robot is well localized."""
        if not self._initialized or not self._state:
            return False

        # Check position uncertainty
        P = self._state.covariance
        position_variance = P[0, 0] + P[1, 1]

        return position_variance < 1.0  # Less than 1m² total variance

    def add_state_callback(self, callback: Callable[[RobotState], None]):
        """Add callback for state updates."""
        self._state_callbacks.append(callback)

    def reset(self):
        """Reset fusion state."""
        with self._lock:
            self._state = None
            self._initialized = False
            self._gps_buffer.clear()
            self._odom_buffer.clear()
            self._lidar_buffer.clear()


if __name__ == '__main__':
    # Test sensor fusion
    print("Testing Sensor Fusion...")

    fusion = SensorFusion()

    # Set origin
    fusion.set_gps_origin(48.8566, 2.3522)

    # Simulate some data
    print("\nSimulating sensor data...")

    # Initial GPS fix
    fusion.update_gps(48.8566, 2.3522, accuracy=0.02, quality=4)
    state = fusion.get_state()
    print(f"Initial state: ({state.pose.x:.2f}, {state.pose.y:.2f})")

    # Simulate forward motion
    for i in range(10):
        # Odometry: 0.1m forward
        fusion.update_odometry(0.1, 0.0, 0.1)

        # GPS update (with some noise)
        noise = np.random.normal(0, 0.01, 2)
        lat = 48.8566 + (i + 1) * 0.1 / fusion._gps_converter._m_per_deg_lat + noise[0] / fusion._gps_converter._m_per_deg_lat
        lon = 2.3522 + noise[1] / fusion._gps_converter._m_per_deg_lon
        fusion.update_gps(lat, lon, accuracy=0.02, quality=4)

        state = fusion.get_state()
        if state:
            pos_std = math.sqrt(state.covariance[0, 0] + state.covariance[1, 1])
            print(f"Step {i+1}: pos=({state.pose.x:.2f}, {state.pose.y:.2f}), "
                  f"vel={state.speed:.2f}m/s, uncertainty={pos_std:.3f}m")

        time.sleep(0.01)

    # Check sensor status
    print("\nSensor status:")
    for sensor_type, status in fusion.get_sensor_status().items():
        print(f"  {sensor_type.name}: healthy={status.is_healthy}, "
              f"rate={status.update_rate:.1f}Hz")

    print(f"\nIs localized: {fusion.is_localized()}")
    print("\nTest complete!")
