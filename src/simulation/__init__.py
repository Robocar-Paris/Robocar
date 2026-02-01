"""
Module de simulation pour tester sur PC sans hardware.

Composants:
- LidarSimulator: Simule un LiDAR LD19 avec environnement virtuel
- GPSSimulator: Simule un GPS avec trajectoire
- Environment: Environnement virtuel avec obstacles
"""

from .lidar_simulator import LidarSimulator, LidarSimulatorConfig
from .gps_simulator import GPSSimulator
from .environment import Environment, Obstacle, create_parking_env, create_corridor_env

__all__ = [
    'LidarSimulator',
    'LidarSimulatorConfig',
    'GPSSimulator',
    'Environment',
    'Obstacle',
    'create_parking_env',
    'create_corridor_env',
]
