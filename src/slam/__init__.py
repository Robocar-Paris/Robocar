"""
SLAM Module

Provides SLAM (Simultaneous Localization and Mapping) capabilities
using proven algorithms from:
- BreezySLAM (https://github.com/simondlevy/BreezySLAM)
- F1Tenth particle filter (https://github.com/f1tenth/particle_filter)

Components:
- SLAM: Main SLAM interface (wraps BreezySLAM or built-in)
- ParticleFilter: Monte Carlo localization
- OccupancyGrid: 2D occupancy grid map

Usage:
    from slam import SLAM, ParticleFilter, OccupancyGrid

    # Create SLAM instance
    slam = SLAM()

    # Update with LiDAR scans
    pose = slam.update(scan_mm, velocity=(dx_mm, dtheta_deg, dt))

    # Get map
    map_img = slam.get_map()
"""

from .slam_core import (
    SLAM,
    SLAMConfig,
    SLAMPose,
    SLAMBase,
    BreezySLAMWrapper,
    BuiltinSLAM,
    create_slam,
    BREEZYSLAM_AVAILABLE
)

from .particle_filter import (
    ParticleFilter,
    ParticleFilterConfig,
    Particle,
    OccupancyMap
)

from .occupancy_grid import (
    OccupancyGrid,
    MapMetadata
)

__all__ = [
    # Main interfaces
    'SLAM',
    'ParticleFilter',
    'OccupancyGrid',

    # Configuration
    'SLAMConfig',
    'ParticleFilterConfig',
    'MapMetadata',

    # Data classes
    'SLAMPose',
    'Particle',
    'OccupancyMap',

    # Advanced
    'SLAMBase',
    'BreezySLAMWrapper',
    'BuiltinSLAM',
    'create_slam',
    'BREEZYSLAM_AVAILABLE',
]
