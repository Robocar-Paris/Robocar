"""
Navigation module for autonomous path planning and following.

Components:
- GlobalPlanner: A* shortest path on occupancy grid
- DWAPlanner: Dynamic Window Approach for obstacle avoidance
- PurePursuitFollower: Smooth path following for Ackermann vehicles
- StanleyFollower: Alternative controller (DARPA Grand Challenge)
- WaypointManager: Multi-waypoint mission management
"""

from .global_planner import GlobalPlanner, PlannerConfig, simplify_path
from .local_planner import DWAPlanner, DWAConfig, VelocityCommand
from .path_follower import PurePursuitFollower, StanleyFollower, FollowerConfig, ControlCommand
from .waypoint_manager import WaypointManager, Waypoint, Mission, MissionStatus