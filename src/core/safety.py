"""
Safety Module

Ensures the robot operates safely at all times.

Features:
- Emergency stop on obstacle too close
- Watchdog timer (stops if control loop hangs)
- Battery voltage monitoring
- Motor temperature monitoring
- Sensor health monitoring
- Maximum speed enforcement
"""

import time
import threading
from typing import Optional, Callable, List
from dataclasses import dataclass
from enum import Enum


class SafetyLevel(Enum):
    """Safety alert levels."""
    OK = 0
    WARNING = 1
    CRITICAL = 2
    EMERGENCY = 3


@dataclass
class SafetyAlert:
    """A safety alert."""
    level: SafetyLevel
    source: str
    message: str
    timestamp: float
    value: float = 0.0


@dataclass
class SafetyConfig:
    """Safety parameters."""
    # Obstacle distance
    emergency_stop_distance: float = 0.15   # meters - HARD STOP
    warning_distance: float = 0.5           # meters - slow down
    critical_distance: float = 0.3          # meters - stop

    # Battery
    battery_warning_voltage: float = 10.5   # volts
    battery_critical_voltage: float = 9.5   # volts

    # Temperature
    motor_temp_warning: float = 60.0        # celsius
    motor_temp_critical: float = 80.0       # celsius
    mos_temp_warning: float = 60.0
    mos_temp_critical: float = 80.0

    # Watchdog
    watchdog_timeout: float = 0.5           # seconds - max time without update
    sensor_timeout: float = 2.0             # seconds - max time without sensor data

    # Speed limits
    max_speed: float = 1.0                  # m/s absolute maximum
    max_speed_near_obstacle: float = 0.3    # m/s when obstacle nearby


class SafetyMonitor:
    """
    Monitors robot safety and triggers emergency stops.

    Must be called every control loop iteration.
    Will trigger emergency stop callback if conditions are unsafe.

    Usage:
        safety = SafetyMonitor(emergency_callback=vesc.emergency_stop)

        # In control loop (MUST be called every iteration):
        velocity = safety.check_and_limit(
            requested_velocity=0.5,
            nearest_obstacle_distance=1.2,
            battery_voltage=11.5,
            motor_temp=45.0
        )
    """

    def __init__(
        self,
        config: Optional[SafetyConfig] = None,
        emergency_callback: Optional[Callable] = None
    ):
        self.config = config or SafetyConfig()
        self._emergency_callback = emergency_callback

        # State
        self._is_emergency = False
        self._alerts: List[SafetyAlert] = []
        self._last_update_time = time.time()
        self._last_sensor_time = time.time()

        # Watchdog
        self._watchdog_thread: Optional[threading.Thread] = None
        self._watchdog_running = False

    def start_watchdog(self):
        """Start watchdog timer."""
        self._watchdog_running = True
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def stop_watchdog(self):
        """Stop watchdog timer."""
        self._watchdog_running = False
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2.0)

    def check_and_limit(
        self,
        requested_velocity: float,
        nearest_obstacle_distance: float = float('inf'),
        battery_voltage: float = 12.0,
        motor_temp: float = 25.0,
        mos_temp: float = 25.0
    ) -> float:
        """
        Check safety conditions and limit velocity.

        This MUST be called every control loop iteration.

        Args:
            requested_velocity: Desired velocity (m/s)
            nearest_obstacle_distance: Distance to closest obstacle (meters)
            battery_voltage: Battery voltage (volts)
            motor_temp: Motor temperature (celsius)
            mos_temp: MOSFET temperature (celsius)

        Returns:
            Safe velocity (may be reduced or zero)
        """
        self._last_update_time = time.time()
        self._alerts.clear()

        if self._is_emergency:
            return 0.0

        safe_velocity = min(abs(requested_velocity), self.config.max_speed)

        # Check obstacle distance
        safe_velocity = self._check_obstacle(safe_velocity, nearest_obstacle_distance)

        # Check battery
        safe_velocity = self._check_battery(safe_velocity, battery_voltage)

        # Check temperature
        safe_velocity = self._check_temperature(safe_velocity, motor_temp, mos_temp)

        # Preserve sign
        if requested_velocity < 0:
            safe_velocity = -safe_velocity

        return safe_velocity

    def _check_obstacle(self, velocity: float, distance: float) -> float:
        """Check obstacle distance safety."""
        if distance <= self.config.emergency_stop_distance:
            self._trigger_alert(SafetyLevel.EMERGENCY, "obstacle",
                              f"Object at {distance:.2f}m - EMERGENCY STOP",
                              distance)
            self._trigger_emergency()
            return 0.0

        if distance <= self.config.critical_distance:
            self._trigger_alert(SafetyLevel.CRITICAL, "obstacle",
                              f"Object at {distance:.2f}m - stopping", distance)
            return 0.0

        if distance <= self.config.warning_distance:
            self._trigger_alert(SafetyLevel.WARNING, "obstacle",
                              f"Object at {distance:.2f}m - slowing", distance)
            return min(velocity, self.config.max_speed_near_obstacle)

        return velocity

    def _check_battery(self, velocity: float, voltage: float) -> float:
        """Check battery voltage."""
        if voltage <= self.config.battery_critical_voltage:
            self._trigger_alert(SafetyLevel.CRITICAL, "battery",
                              f"Battery {voltage:.1f}V - critically low", voltage)
            return 0.0

        if voltage <= self.config.battery_warning_voltage:
            self._trigger_alert(SafetyLevel.WARNING, "battery",
                              f"Battery {voltage:.1f}V - low", voltage)

        return velocity

    def _check_temperature(self, velocity: float,
                           motor_temp: float, mos_temp: float) -> float:
        """Check temperatures."""
        if motor_temp >= self.config.motor_temp_critical or \
           mos_temp >= self.config.mos_temp_critical:
            self._trigger_alert(SafetyLevel.CRITICAL, "temperature",
                              f"Motor: {motor_temp:.0f}C, MOS: {mos_temp:.0f}C",
                              max(motor_temp, mos_temp))
            return 0.0

        if motor_temp >= self.config.motor_temp_warning or \
           mos_temp >= self.config.mos_temp_warning:
            self._trigger_alert(SafetyLevel.WARNING, "temperature",
                              f"Motor: {motor_temp:.0f}C, MOS: {mos_temp:.0f}C",
                              max(motor_temp, mos_temp))

        return velocity

    def update_sensor_time(self):
        """Call when sensor data is received."""
        self._last_sensor_time = time.time()

    def _trigger_emergency(self):
        """Trigger emergency stop."""
        self._is_emergency = True
        if self._emergency_callback:
            self._emergency_callback()

    def _trigger_alert(self, level: SafetyLevel, source: str,
                       message: str, value: float = 0.0):
        """Add a safety alert."""
        self._alerts.append(SafetyAlert(
            level=level, source=source, message=message,
            timestamp=time.time(), value=value
        ))

    def _watchdog_loop(self):
        """Watchdog thread - stops robot if control loop hangs."""
        while self._watchdog_running:
            time.sleep(0.1)

            # Check control loop
            elapsed = time.time() - self._last_update_time
            if elapsed > self.config.watchdog_timeout:
                print(f"[SAFETY] WATCHDOG: No update for {elapsed:.2f}s!")
                self._trigger_emergency()

            # Check sensor data
            sensor_elapsed = time.time() - self._last_sensor_time
            if sensor_elapsed > self.config.sensor_timeout:
                print(f"[SAFETY] No sensor data for {sensor_elapsed:.2f}s!")

    def reset_emergency(self):
        """Reset emergency state (requires manual action)."""
        self._is_emergency = False
        self._alerts.clear()

    @property
    def is_emergency(self) -> bool:
        return self._is_emergency

    @property
    def is_safe(self) -> bool:
        return not self._is_emergency and not any(
            a.level >= SafetyLevel.CRITICAL for a in self._alerts
        )

    @property
    def alerts(self) -> List[SafetyAlert]:
        return self._alerts.copy()

    @property
    def highest_alert_level(self) -> SafetyLevel:
        if self._is_emergency:
            return SafetyLevel.EMERGENCY
        if not self._alerts:
            return SafetyLevel.OK
        return max(a.level for a in self._alerts)

    def get_status(self) -> dict:
        """Get safety status."""
        return {
            "safe": self.is_safe,
            "emergency": self._is_emergency,
            "level": self.highest_alert_level.name,
            "alerts": [f"[{a.level.name}] {a.source}: {a.message}" for a in self._alerts]
        }