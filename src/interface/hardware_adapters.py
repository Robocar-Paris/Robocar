"""
Adaptateurs pour le materiel reel (Jetson Nano).

Ces classes adaptent les drivers reels aux interfaces abstraites.
"""

from typing import Optional
import math

from .sensor_interface import (
    ILidarSensor, IGPSSensor, IMotorController,
    LidarData, LidarPoint, GPSData
)


class RealLidarAdapter(ILidarSensor):
    """Adaptateur pour le LiDAR LD19 reel."""

    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 230400):
        self.port = port
        self.baudrate = baudrate
        self._driver = None
        self._running = False

    def start(self) -> bool:
        try:
            from driver import LidarDriver
            self._driver = LidarDriver(self.port, self.baudrate)
            result = self._driver.start()
            self._running = result
            return result
        except Exception as e:
            print(f"[LiDAR] Erreur demarrage: {e}")
            return False

    def stop(self):
        if self._driver:
            self._driver.stop()
        self._running = False

    def get_scan(self) -> Optional[LidarData]:
        if not self._driver:
            return None

        scan = self._driver.get_latest_scan()
        if not scan:
            return None

        # Convertir au format interface
        points = []
        for p in scan.points:
            points.append(LidarPoint(
                angle=p.angle,
                distance=p.distance,
                intensity=p.intensity,
                valid=p.valid
            ))

        return LidarData(
            points=points,
            timestamp=scan.timestamp,
            scan_frequency=scan.scan_frequency
        )

    def is_running(self) -> bool:
        return self._running


class RealGPSAdapter(IGPSSensor):
    """Adaptateur pour le GPS RTK Point One reel."""

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 460800,
                 polaris_api_key: Optional[str] = None):
        self.port = port
        self.baudrate = baudrate
        self.polaris_api_key = polaris_api_key
        self._driver = None
        self._running = False

    def start(self) -> bool:
        try:
            if self.polaris_api_key:
                from driver import GPSRTKDriver
                self._driver = GPSRTKDriver(
                    port=self.port,
                    baudrate=self.baudrate,
                    polaris_api_key=self.polaris_api_key
                )
            else:
                from driver import GPSDriver
                self._driver = GPSDriver(self.port, self.baudrate)

            result = self._driver.start()
            self._running = result
            return result
        except Exception as e:
            print(f"[GPS] Erreur demarrage: {e}")
            return False

    def stop(self):
        if self._driver:
            self._driver.stop()
        self._running = False

    def get_position(self) -> Optional[GPSData]:
        if not self._driver:
            return None

        pos = self._driver.get_position()
        if not pos:
            return None

        return GPSData(
            latitude=pos.latitude,
            longitude=pos.longitude,
            altitude=getattr(pos, 'altitude', 0.0),
            accuracy_h=getattr(pos, 'accuracy_h', 100.0),
            accuracy_v=getattr(pos, 'accuracy_v', 100.0),
            satellites=getattr(pos, 'satellites', 0),
            quality=pos.quality,
            timestamp=pos.timestamp,
            is_valid=pos.is_valid
        )

    def is_running(self) -> bool:
        return self._running


class RealMotorAdapter(IMotorController):
    """Adaptateur pour le controleur VESC reel."""

    def __init__(self, port: str = '/dev/ttyACM0'):
        self.port = port
        self._controller = None
        self._running = False

    def start(self) -> bool:
        try:
            from driver import VESCController
            self._controller = VESCController(self.port)
            result = self._controller.start()
            self._running = result
            return result
        except Exception as e:
            print(f"[VESC] Erreur demarrage: {e}")
            return False

    def stop(self):
        if self._controller:
            self.emergency_stop()
            self._controller.stop()
        self._running = False

    def set_speed(self, speed: float):
        if self._controller:
            # Convertir [-1, 1] en duty cycle VESC
            duty = max(-1.0, min(1.0, speed)) * 0.15  # Limite a 15% duty
            self._controller.set_duty_cycle(duty)

    def set_steering(self, steering: float):
        if self._controller:
            # Convertir [-1, 1] en position servo (0.0 = gauche, 0.5 = centre, 1.0 = droite)
            servo_pos = 0.5 - (steering * 0.5)
            self._controller.set_servo(servo_pos)

    def emergency_stop(self):
        if self._controller:
            self._controller.set_duty_cycle(0)

    def is_running(self) -> bool:
        return self._running
