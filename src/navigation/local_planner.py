"""
Local Path Planner - Dynamic Window Approach (DWA)

Avoids obstacles in real-time while following the global path.

DWA is the standard local planner used in:
- ROS Navigation Stack
- Nav2 (ROS2)
- F1Tenth autonomous racing

The idea:
1. Sample many possible velocities (linear + angular)
2. Simulate each one for a short time
3. Score each trajectory (closeness to goal, obstacle clearance, speed)
4. Pick the best one

References:
- "The Dynamic Window Approach to Collision Avoidance" (Fox, Burgard, Thrun, 1997)
- Nav2 DWB Controller
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class DWAConfig:
    """DWA planner configuration."""
    # Velocity limits
    max_vel_x: float = 0.8             # m/s forward
    min_vel_x: float = 0.0             # m/s (0 = no reverse)
    max_vel_theta: float = 1.5         # rad/s

    # Acceleration limits
    max_acc_x: float = 0.5             # m/s²
    max_acc_theta: float = 2.0         # rad/s²

    # Simulation
    sim_time: float = 2.0              # seconds to simulate forward
    sim_granularity: float = 0.1       # time step for simulation
    vx_samples: int = 10               # number of linear velocity samples
    vtheta_samples: int = 20           # number of angular velocity samples

    # Scoring weights
    path_distance_weight: float = 1.0  # Closeness to global path
    goal_distance_weight: float = 2.0  # Closeness to goal
    obstacle_weight: float = 0.5       # Distance from obstacles
    speed_weight: float = 0.3          # Prefer faster trajectories

    # Safety
    min_obstacle_distance: float = 0.2 # meters - reject if closer
    robot_radius: float = 0.15         # meters

    # Goal tolerance
    xy_goal_tolerance: float = 0.3     # meters
    yaw_goal_tolerance: float = 0.2    # radians


@dataclass
class Trajectory:
    """A simulated trajectory."""
    vel_x: float                        # Linear velocity
    vel_theta: float                    # Angular velocity
    poses: List[Tuple[float, float, float]]  # [(x, y, theta), ...]
    score: float = 0.0
    min_obstacle_dist: float = float('inf')

    @property
    def endpoint(self) -> Tuple[float, float, float]:
        return self.poses[-1] if self.poses else (0, 0, 0)


@dataclass
class VelocityCommand:
    """Output velocity command."""
    linear: float       # m/s
    angular: float      # rad/s
    is_valid: bool = True


class DWAPlanner:
    """
    Dynamic Window Approach local planner.

    Generates velocity commands that:
    - Follow the global path
    - Avoid obstacles in real-time
    - Respect kinematic constraints (acceleration limits)

    Usage:
        dwa = DWAPlanner()

        # In control loop:
        cmd = dwa.compute_velocity(
            current_pose=(x, y, theta),
            current_vel=(vx, vtheta),
            global_path=path,
            obstacles=obstacle_list
        )

        motor.set_velocity(cmd.linear)
        motor.set_steering(cmd.angular)
    """

    def __init__(self, config: Optional[DWAConfig] = None):
        self.config = config or DWAConfig()

    def compute_velocity(
        self,
        current_pose: Tuple[float, float, float],
        current_vel: Tuple[float, float],
        global_path: List[Tuple[float, float]],
        obstacles: List[Tuple[float, float]],
        goal: Optional[Tuple[float, float]] = None
    ) -> VelocityCommand:
        """
        Compute best velocity command.

        Args:
            current_pose: (x, y, theta) current robot pose
            current_vel: (vx, vtheta) current velocities
            global_path: List of (x, y) waypoints to follow
            obstacles: List of (x, y) obstacle positions
            goal: Optional (x, y) final goal

        Returns:
            VelocityCommand with best (linear, angular) velocities
        """
        if not global_path:
            return VelocityCommand(0.0, 0.0, is_valid=False)

        # Determine local goal
        if goal is None:
            goal = global_path[-1]

        # Check if we reached the goal
        dist_to_goal = math.sqrt(
            (goal[0] - current_pose[0])**2 +
            (goal[1] - current_pose[1])**2
        )
        if dist_to_goal < self.config.xy_goal_tolerance:
            return VelocityCommand(0.0, 0.0)

        # Calculate dynamic window
        vx_min, vx_max, vt_min, vt_max = self._dynamic_window(current_vel)

        # Sample velocities
        vx_range = np.linspace(vx_min, vx_max, self.config.vx_samples)
        vt_range = np.linspace(vt_min, vt_max, self.config.vtheta_samples)

        # Evaluate all trajectories
        best_trajectory = None
        best_score = -float('inf')

        obstacle_array = np.array(obstacles) if obstacles else np.empty((0, 2))

        for vx in vx_range:
            for vt in vt_range:
                # Simulate trajectory
                traj = self._simulate(current_pose, vx, vt)

                # Check obstacle clearance
                min_dist = self._min_obstacle_distance(traj, obstacle_array)
                traj.min_obstacle_dist = min_dist

                if min_dist < self.config.min_obstacle_distance:
                    continue  # Too close to obstacle, reject

                # Score trajectory
                score = self._score_trajectory(
                    traj, global_path, goal, obstacle_array
                )
                traj.score = score

                if score > best_score:
                    best_score = score
                    best_trajectory = traj

        if best_trajectory is None:
            # No valid trajectory - emergency stop
            return VelocityCommand(0.0, 0.0, is_valid=False)

        return VelocityCommand(
            linear=best_trajectory.vel_x,
            angular=best_trajectory.vel_theta,
            is_valid=True
        )

    def _dynamic_window(self, current_vel: Tuple[float, float]) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window of achievable velocities.

        Based on current velocity and acceleration limits.
        """
        vx, vt = current_vel
        dt = self.config.sim_granularity

        # Velocity limits from dynamics
        vx_min = max(self.config.min_vel_x, vx - self.config.max_acc_x * dt)
        vx_max = min(self.config.max_vel_x, vx + self.config.max_acc_x * dt)
        vt_min = max(-self.config.max_vel_theta, vt - self.config.max_acc_theta * dt)
        vt_max = min(self.config.max_vel_theta, vt + self.config.max_acc_theta * dt)

        return vx_min, vx_max, vt_min, vt_max

    def _simulate(self, pose: Tuple[float, float, float],
                  vx: float, vt: float) -> Trajectory:
        """Simulate trajectory for sim_time seconds."""
        x, y, theta = pose
        dt = self.config.sim_granularity
        n_steps = int(self.config.sim_time / dt)

        poses = [(x, y, theta)]

        for _ in range(n_steps):
            theta += vt * dt
            x += vx * math.cos(theta) * dt
            y += vx * math.sin(theta) * dt
            poses.append((x, y, theta))

        return Trajectory(vel_x=vx, vel_theta=vt, poses=poses)

    def _min_obstacle_distance(self, traj: Trajectory,
                               obstacles: np.ndarray) -> float:
        """Find minimum distance from trajectory to any obstacle."""
        if len(obstacles) == 0:
            return float('inf')

        min_dist = float('inf')

        for x, y, _ in traj.poses:
            point = np.array([x, y])
            dists = np.linalg.norm(obstacles - point, axis=1)
            d = np.min(dists)
            if d < min_dist:
                min_dist = d

        return min_dist - self.config.robot_radius

    def _score_trajectory(
        self,
        traj: Trajectory,
        global_path: List[Tuple[float, float]],
        goal: Tuple[float, float],
        obstacles: np.ndarray
    ) -> float:
        """
        Score a trajectory (higher = better).

        Combines:
        - Path distance: closeness to global path
        - Goal distance: closeness to goal
        - Obstacle clearance: distance from obstacles
        - Speed: prefer faster trajectories
        """
        endpoint = traj.endpoint

        # 1. Path distance score (how close endpoint is to nearest path point)
        path_array = np.array(global_path)
        dists = np.linalg.norm(
            path_array - np.array([endpoint[0], endpoint[1]]), axis=1
        )
        path_score = 1.0 / (1.0 + np.min(dists))

        # 2. Goal distance score
        goal_dist = math.sqrt(
            (goal[0] - endpoint[0])**2 +
            (goal[1] - endpoint[1])**2
        )
        goal_score = 1.0 / (1.0 + goal_dist)

        # 3. Obstacle clearance score
        obstacle_score = min(traj.min_obstacle_dist, 2.0) / 2.0

        # 4. Speed score (prefer moving)
        speed_score = abs(traj.vel_x) / self.config.max_vel_x

        # Weighted sum
        score = (
            self.config.path_distance_weight * path_score +
            self.config.goal_distance_weight * goal_score +
            self.config.obstacle_weight * obstacle_score +
            self.config.speed_weight * speed_score
        )

        return score

    def get_trajectories_for_viz(
        self,
        current_pose: Tuple[float, float, float],
        current_vel: Tuple[float, float],
        obstacles: List[Tuple[float, float]]
    ) -> List[Trajectory]:
        """Generate all candidate trajectories for visualization."""
        vx_min, vx_max, vt_min, vt_max = self._dynamic_window(current_vel)

        vx_range = np.linspace(vx_min, vx_max, self.config.vx_samples)
        vt_range = np.linspace(vt_min, vt_max, self.config.vtheta_samples)

        obstacle_array = np.array(obstacles) if obstacles else np.empty((0, 2))

        trajectories = []
        for vx in vx_range:
            for vt in vt_range:
                traj = self._simulate(current_pose, vx, vt)
                traj.min_obstacle_dist = self._min_obstacle_distance(traj, obstacle_array)
                trajectories.append(traj)

        return trajectories