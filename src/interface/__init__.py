"""
Interfaces abstraites pour les capteurs et actionneurs.

Ces interfaces permettent d'utiliser le meme code de navigation
avec les vrais capteurs (Jetson) ou les simulateurs (PC).
"""

from .sensor_interface import (
    ILidarSensor,
    IGPSSensor,
    IMotorController,
    LidarData,
    LidarPoint,
    GPSData,
)

from .hardware_adapters import (
    RealLidarAdapter,
    RealGPSAdapter,
    RealMotorAdapter,
)

from .simulation_adapters import (
    SimulatedLidarAdapter,
    SimulatedGPSAdapter,
    SimulatedMotorAdapter,
)

__all__ = [
    # Interfaces
    'ILidarSensor',
    'IGPSSensor',
    'IMotorController',
    'LidarData',
    'LidarPoint',
    'GPSData',
    # Hardware adapters
    'RealLidarAdapter',
    'RealGPSAdapter',
    'RealMotorAdapter',
    # Simulation adapters
    'SimulatedLidarAdapter',
    'SimulatedGPSAdapter',
    'SimulatedMotorAdapter',
]
