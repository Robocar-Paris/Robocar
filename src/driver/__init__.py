"""
Drivers for Robocar - Voiture RC autonome

Composants:
- GPS Point One RTK (gps.py)
- LiDAR LD19 (lidar.py)
- VESC Motor Controller (vesc_motor.py)

Optionnel (corrections RTK):
- Polaris Client (polaris_client.py)
- NTRIP Client Centipede/RTK2GO (ntrip_client.py)
"""

from .lidar import LidarDriver, LidarScan, LidarPoint
from .gps import GPSDriver, GPSPosition
from .vesc_motor import VESCController, VESCState

# Optional RTK components
try:
    from .gps_rtk import GPSRTKDriver
    from .polaris_client import PolarisClient, PolarisConfig
    from .ntrip_client import NTRIPClient, NTRIPConfig
    RTK_AVAILABLE = True
except ImportError:
    RTK_AVAILABLE = False

__all__ = [
    'LidarDriver',
    'LidarScan',
    'LidarPoint',
    'GPSDriver',
    'GPSPosition',
    'VESCController',
    'VESCState',
    'GPSRTKDriver',
    'PolarisClient',
    'PolarisConfig',
    'NTRIPClient',
    'NTRIPConfig',
    'RTK_AVAILABLE'
]