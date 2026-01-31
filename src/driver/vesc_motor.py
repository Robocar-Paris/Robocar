"""
VESC Motor Controller Driver

Refactored from the original control_moteur.py for better integration
with the autonomous navigation stack.

Features:
- Duty cycle control with safety limits
- Servo/steering control
- Odometry reading from VESC
- Smooth ramping for acceleration
"""

import serial
import time
import threading
from typing import Optional, Tuple
from dataclasses import dataclass
import struct


try:
    import pyvesc
    from pyvesc import GetValues, SetDutyCycle, SetServoPos
    PYVESC_AVAILABLE = True
except ImportError:
    PYVESC_AVAILABLE = False
    print("[VESC] Warning: pyvesc not installed. Motor control disabled.")


@dataclass
class VESCState:
    """Current state of the VESC controller."""
    timestamp: float
    duty_cycle: float       # Current duty cycle (-1.0 to 1.0)
    rpm: float              # Motor RPM
    current: float          # Motor current (A)
    voltage: float          # Battery voltage (V)
    tachometer: int         # Tachometer value (counts)
    temp_motor: float       # Motor temperature (C)
    temp_mos: float         # MOSFET temperature (C)


class VESCController:
    """
    VESC motor controller interface.

    Usage:
        vesc = VESCController('/dev/ttyACM0')
        vesc.start()

        # Set speed (duty cycle)
        vesc.set_duty(0.1)  # 10% forward

        # Set steering
        vesc.set_servo(0.5)  # Center
        vesc.set_servo(0.0)  # Full left
        vesc.set_servo(1.0)  # Full right

        # Get state
        state = vesc.get_state()
        print(f"RPM: {state.rpm}, Voltage: {state.voltage}V")

        vesc.stop()
    """

    # Safety limits
    MAX_DUTY_CYCLE = 0.2     # Maximum duty cycle
    MAX_STEERING = 0.3       # Maximum steering angle (radians)
    RAMP_STEP = 0.02         # Duty cycle change per ramp step
    RAMP_INTERVAL = 0.02     # Seconds between ramp steps

    def __init__(
        self,
        port: str = '/dev/ttyACM0',
        baudrate: int = 115200
    ):
        """
        Initialize VESC controller.

        Args:
            port: Serial port path
            baudrate: Serial baudrate
        """
        self.port = port
        self.baudrate = baudrate

        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Current state
        self._current_duty = 0.0
        self._target_duty = 0.0
        self._current_servo = 0.5
        self._latest_state: Optional[VESCState] = None

        # Odometry
        self._last_tachometer = 0
        self._total_distance = 0.0
        self._wheel_circumference = 0.3  # meters (adjust for your wheel)
        self._motor_poles = 14           # VESC motor poles
        self._gear_ratio = 1.0           # Gear ratio

    def start(self) -> bool:
        """
        Start the VESC controller.

        Returns:
            True if successfully started
        """
        if not PYVESC_AVAILABLE:
            print("[VESC] pyvesc not available")
            return False

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()

            # Initialize to safe state
            self._set_duty_raw(0.0)
            self._set_servo_raw(0.5)

            return True
        except serial.SerialException as e:
            print(f"[VESC] Failed to open serial port: {e}")
            return False

    def stop(self):
        """Stop the VESC controller safely."""
        # Ramp down duty cycle
        self._target_duty = 0.0
        time.sleep(0.5)  # Allow time for ramp down

        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

        # Final safety stop
        if self._serial:
            try:
                self._set_duty_raw(0.0)
                self._set_servo_raw(0.5)
            except:
                pass
            self._serial.close()
            self._serial = None

    def set_duty(self, duty: float):
        """
        Set target duty cycle with safety limits and ramping.

        Args:
            duty: Duty cycle (-1.0 to 1.0), will be clamped to MAX_DUTY_CYCLE
        """
        # Apply safety limits
        duty = max(-self.MAX_DUTY_CYCLE, min(self.MAX_DUTY_CYCLE, duty))
        self._target_duty = duty

    def set_servo(self, position: float):
        """
        Set servo position.

        Args:
            position: Servo position (0.0=left, 0.5=center, 1.0=right)
        """
        position = max(0.0, min(1.0, position))
        self._current_servo = position
        self._set_servo_raw(position)

    def set_steering_angle(self, angle: float):
        """
        Set steering angle in radians.

        Args:
            angle: Steering angle (-MAX_STEERING to +MAX_STEERING)
        """
        # Clamp angle
        angle = max(-self.MAX_STEERING, min(self.MAX_STEERING, angle))

        # Convert to servo position (0.0 to 1.0)
        # Assuming linear mapping: -MAX_STEERING -> 0.0, +MAX_STEERING -> 1.0
        position = (angle / self.MAX_STEERING + 1.0) / 2.0
        self.set_servo(position)

    def get_state(self) -> Optional[VESCState]:
        """
        Get current VESC state.

        Returns:
            VESCState object or None
        """
        with self._lock:
            return self._latest_state

    def get_odometry(self) -> Tuple[float, float]:
        """
        Get odometry data.

        Returns:
            Tuple of (distance_traveled_meters, current_velocity_ms)
        """
        state = self.get_state()
        if state is None:
            return 0.0, 0.0

        # Calculate distance from tachometer
        tach_diff = state.tachometer - self._last_tachometer
        self._last_tachometer = state.tachometer

        # Convert tachometer counts to distance
        # tachometer = electrical rotations * 6
        rotations = tach_diff / (6 * self._motor_poles * self._gear_ratio)
        distance = rotations * self._wheel_circumference
        self._total_distance += abs(distance)

        # Calculate velocity from RPM
        rps = state.rpm / 60.0 / self._gear_ratio
        velocity = rps * self._wheel_circumference

        return self._total_distance, velocity

    def reset_odometry(self):
        """Reset odometry counters."""
        self._total_distance = 0.0
        state = self.get_state()
        if state:
            self._last_tachometer = state.tachometer

    def emergency_stop(self):
        """Immediately stop the motor (no ramping)."""
        self._target_duty = 0.0
        self._current_duty = 0.0
        self._set_duty_raw(0.0)

    def _update_loop(self):
        """Main update loop (runs in separate thread)."""
        last_ramp_time = time.time()

        while self._running:
            try:
                # Ramp duty cycle
                current_time = time.time()
                if current_time - last_ramp_time >= self.RAMP_INTERVAL:
                    self._ramp_duty()
                    last_ramp_time = current_time

                # Read state
                self._read_state()

                time.sleep(0.01)

            except Exception as e:
                print(f"[VESC] Update error: {e}")
                time.sleep(0.1)

    def _ramp_duty(self):
        """Smoothly ramp duty cycle towards target."""
        if abs(self._target_duty - self._current_duty) < 0.001:
            return

        if self._target_duty > self._current_duty:
            self._current_duty = min(
                self._target_duty,
                self._current_duty + self.RAMP_STEP
            )
        else:
            self._current_duty = max(
                self._target_duty,
                self._current_duty - self.RAMP_STEP
            )

        self._set_duty_raw(self._current_duty)

    def _set_duty_raw(self, duty: float):
        """Send duty cycle command directly to VESC."""
        if not self._serial or not PYVESC_AVAILABLE:
            return

        try:
            self._serial.write(pyvesc.encode(SetDutyCycle(duty)))
        except Exception as e:
            print(f"[VESC] Duty write error: {e}")

    def _set_servo_raw(self, position: float):
        """Send servo command directly to VESC."""
        if not self._serial or not PYVESC_AVAILABLE:
            return

        try:
            self._serial.write(pyvesc.encode(SetServoPos(position)))
        except Exception as e:
            print(f"[VESC] Servo write error: {e}")

    def _read_state(self):
        """Read state from VESC."""
        if not self._serial or not PYVESC_AVAILABLE:
            return

        try:
            # Request values
            self._serial.write(pyvesc.encode_request(GetValues))

            # Read response
            buffer = b''
            while self._serial.in_waiting > 0:
                buffer += self._serial.read(self._serial.in_waiting)
                time.sleep(0.001)

            if buffer:
                response, _ = pyvesc.decode(buffer)
                if response:
                    state = VESCState(
                        timestamp=time.time(),
                        duty_cycle=getattr(response, 'duty_now', 0.0),
                        rpm=getattr(response, 'rpm', 0.0),
                        current=getattr(response, 'avg_motor_current', 0.0),
                        voltage=getattr(response, 'v_in', 0.0),
                        tachometer=getattr(response, 'tachometer', 0),
                        temp_motor=getattr(response, 'temp_motor', 0.0),
                        temp_mos=getattr(response, 'temp_mos', 0.0)
                    )
                    with self._lock:
                        self._latest_state = state

        except Exception as e:
            pass  # Ignore read errors

    @property
    def is_running(self) -> bool:
        """Check if controller is running."""
        return self._running

    @property
    def current_duty(self) -> float:
        """Get current duty cycle."""
        return self._current_duty

    @property
    def current_steering(self) -> float:
        """Get current servo position."""
        return self._current_servo


if __name__ == '__main__':
    # Simple test
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    print(f"[VESC] Testing VESC on {port}...")

    vesc = VESCController(port)
    if vesc.start():
        print("[VESC] Started successfully")
        try:
            # Test servo
            print("[VESC] Testing servo...")
            for pos in [0.3, 0.5, 0.7, 0.5]:
                vesc.set_servo(pos)
                time.sleep(0.5)

            # Read state
            state = vesc.get_state()
            if state:
                print(f"[VESC] Voltage: {state.voltage:.1f}V, Temp: {state.temp_mos:.1f}C")

            print("[VESC] Test complete")
        finally:
            vesc.stop()
    else:
        print("[VESC] Failed to start")
