"""
Perception module for sensor data processing.

Components:
- LidarProcessor: Process and filter LiDAR scans
- ObstacleDetector: Detect and track obstacles
- SensorFusion: Fuse multiple sensors for state estimation
- Transforms: Coordinate transformations
"""

from .lidar_processor import (
    LidarProcessor,
    ProcessedScan,
    ScanSegment,
    create_simulated_scan
)

from .obstacle_detector import (
    ObstacleDetector,
    Obstacle,
    ObstacleType,
    BoundingBox
)

from .sensor_fusion import (
    SensorFusion,
    RobotState,
    SensorReading,
    SensorType,
    SensorStatus
)

from .transforms import (
    Pose2D,
    Transform2D,
    FrameManager,
    GPSConverter,
    normalize_angle,
    angle_difference
)

__all__ = [
    # LiDAR
    'LidarProcessor',
    'ProcessedScan',
    'ScanSegment',
    'create_simulated_scan',

    # Obstacles
    'ObstacleDetector',
    'Obstacle',
    'ObstacleType',
    'BoundingBox',

    # Fusion
    'SensorFusion',
    'RobotState',
    'SensorReading',
    'SensorType',
    'SensorStatus',

    # Transforms
    'Pose2D',
    'Transform2D',
    'FrameManager',
    'GPSConverter',
    'normalize_angle',
    'angle_difference'
]
