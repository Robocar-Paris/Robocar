"""
Path Following Controllers

Converts a planned path into steering and velocity commands.

Implements:
- Pure Pursuit: Simple, robust, widely used for Ackermann vehicles
- Stanley: More precise, used in Stanford's DARPA Grand Challenge winner

References:
- Pure Pursuit: "Implementation of the Pure Pursuit Path Tracking Algorithm"
  (R. Craig Coulter, CMU, 1992)
- Stanley: "Stanley: The Robot that Won the DARPA Grand Challenge"
  (Thrun et al., 2006)
- F1Tenth uses Pure Pursuit as the default controller
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class FollowerConfig:
    """Path follower configuration."""
    # Pure Pursuit parameters
    lookahead_distance: float = 0.6     # meters
    min_lookahead: float = 0.3          # minimum lookahead
    max_lookahead: float = 1.5          # maximum lookahead
    lookahead_ratio: float = 2.0        # lookahead = ratio * velocity

    # Vehicle parameters (Ackermann)
    wheelbase: float = 0.26             # meters (distance front-rear axle)
    max_steering_angle: float = 0.3     # radians (~17 degrees)
    max_steering_rate: float = 1.0      # rad/s (how fast steering can change)

    # Speed control
    max_speed: float = 0.8              # m/s
    min_speed: float = 0.1              # m/s
    speed_lookahead: float = 1.0        # how far ahead to check curvature

    # Goal
    goal_tolerance: float = 0.3         # meters
    slow_down_distance: float = 1.0     # start slowing at this distance


@dataclass
class ControlCommand:
    """Output control command for the vehicle."""
    steering_angle: float   # radians (positive = left)
    velocity: float         # m/s
    reached_goal: bool = False


class PurePursuitFollower:
    """
    Pure Pursuit path following controller.

    How it works:
    1. Find a "lookahead point" on the path ahead of the robot
    2. Calculate the arc that connects the robot to that point
    3. Convert arc radius to steering angle using Ackermann geometry

    This is the most popular controller for autonomous vehicles because:
    - Simple to implement
    - Smooth steering
    - Naturally handles curves
    - Works well at any speed

    Usage:
        follower = PurePursuitFollower()

        # In control loop (20 Hz):
        cmd = follower.compute(
            pose=(x, y, theta),
            velocity=current_speed,
            path=global_path
        )
        vesc.set_steering_angle(cmd.steering_angle)
        vesc.set_duty(cmd.velocity)
    """

    def __init__(self, config: Optional[FollowerConfig] = None):
        self.config = config or FollowerConfig()
        self._closest_idx = 0  # Track closest path point for efficiency

    def compute(
        self,
        pose: Tuple[float, float, float],
        velocity: float,
        path: List[Tuple[float, float]]
    ) -> ControlCommand:
        """
        Compute steering and velocity commands.

        Args:
            pose: (x, y, theta) robot pose in world frame
            velocity: Current forward velocity (m/s)
            path: List of (x, y) waypoints

        Returns:
            ControlCommand with steering angle and velocity
        """
        if not path or len(path) < 2:
            return ControlCommand(0.0, 0.0, reached_goal=True)

        x, y, theta = pose

        # Check if goal reached
        goal = path[-1]
        dist_to_goal = math.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)
        if dist_to_goal < self.config.goal_tolerance:
            return ControlCommand(0.0, 0.0, reached_goal=True)

        # Find closest point on path
        closest_idx = self._find_closest_point(x, y, path)
        self._closest_idx = closest_idx

        # Calculate adaptive lookahead
        lookahead = self.config.lookahead_ratio * abs(velocity)
        lookahead = max(self.config.min_lookahead,
                       min(self.config.max_lookahead, lookahead))

        # Find lookahead point on path
        lookahead_point = self._find_lookahead_point(x, y, path, closest_idx, lookahead)

        if lookahead_point is None:
            # No lookahead point found, aim for end of path
            lookahead_point = path[-1]

        # Calculate steering angle
        steering = self._compute_steering(x, y, theta, lookahead_point, lookahead)

        # Calculate velocity
        target_velocity = self._compute_velocity(
            steering, dist_to_goal, path, closest_idx
        )

        return ControlCommand(
            steering_angle=steering,
            velocity=target_velocity,
            reached_goal=False
        )

    def _find_closest_point(self, x: float, y: float,
                            path: List[Tuple[float, float]]) -> int:
        """Find index of closest point on path."""
        # Start search from last known closest point for efficiency
        start = max(0, self._closest_idx - 5)
        end = min(len(path), self._closest_idx + 50)

        min_dist = float('inf')
        min_idx = self._closest_idx

        for i in range(start, end):
            dx = path[i][0] - x
            dy = path[i][1] - y
            dist = dx*dx + dy*dy  # No sqrt needed for comparison
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        return min_idx

    def _find_lookahead_point(
        self,
        x: float, y: float,
        path: List[Tuple[float, float]],
        start_idx: int,
        lookahead: float
    ) -> Optional[Tuple[float, float]]:
        """
        Find the point on path at lookahead distance.

        Interpolates between path points for smooth tracking.
        """
        for i in range(start_idx, len(path) - 1):
            # Segment start and end
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            robot = np.array([x, y])

            # Find intersection of circle (center=robot, radius=lookahead)
            # with line segment p1-p2
            d = p2 - p1
            f = p1 - robot

            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - lookahead**2

            discriminant = b*b - 4*a*c

            if discriminant < 0:
                continue

            sqrt_disc = math.sqrt(discriminant)
            t1 = (-b - sqrt_disc) / (2*a)
            t2 = (-b + sqrt_disc) / (2*a)

            # We want the furthest intersection that's on the segment
            for t in [t2, t1]:
                if 0 <= t <= 1:
                    point = p1 + t * d
                    # Make sure it's ahead of us (not behind)
                    if i >= start_idx:
                        return (float(point[0]), float(point[1]))

        # Fallback: return last point
        if start_idx < len(path):
            return path[-1]
        return None

    def _compute_steering(
        self,
        x: float, y: float, theta: float,
        target: Tuple[float, float],
        lookahead: float
    ) -> float:
        """
        Compute steering angle using Pure Pursuit geometry.

        The key formula:
        steering_angle = atan(2 * L * sin(alpha) / lookahead_distance)

        Where:
        - L = wheelbase
        - alpha = angle from heading to target
        """
        # Transform target to robot frame
        dx = target[0] - x
        dy = target[1] - y

        # Angle to target in world frame
        target_angle = math.atan2(dy, dx)

        # Angle difference (alpha)
        alpha = target_angle - theta
        # Normalize to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Actual distance to target
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            return 0.0

        # Pure Pursuit steering formula (Ackermann)
        # curvature = 2 * sin(alpha) / distance
        # steering = atan(wheelbase * curvature)
        curvature = 2.0 * math.sin(alpha) / max(dist, lookahead)
        steering = math.atan(self.config.wheelbase * curvature)

        # Clamp to max steering
        steering = max(-self.config.max_steering_angle,
                      min(self.config.max_steering_angle, steering))

        return steering

    def _compute_velocity(
        self,
        steering: float,
        dist_to_goal: float,
        path: List[Tuple[float, float]],
        closest_idx: int
    ) -> float:
        """Compute target velocity based on curvature and distance."""
        # Slow down for sharp turns
        curvature_factor = 1.0 - 0.6 * abs(steering) / self.config.max_steering_angle

        # Slow down near goal
        goal_factor = min(1.0, dist_to_goal / self.config.slow_down_distance)

        # Compute path curvature ahead
        path_curvature = self._path_curvature_ahead(path, closest_idx)
        path_factor = 1.0 - 0.5 * min(1.0, path_curvature)

        # Target velocity
        velocity = self.config.max_speed * curvature_factor * goal_factor * path_factor
        velocity = max(self.config.min_speed, velocity)

        return velocity

    def _path_curvature_ahead(
        self,
        path: List[Tuple[float, float]],
        start_idx: int
    ) -> float:
        """Estimate path curvature ahead for speed planning."""
        lookahead_pts = 10
        end_idx = min(len(path), start_idx + lookahead_pts)

        if end_idx - start_idx < 3:
            return 0.0

        # Calculate total angle change
        total_angle_change = 0.0
        total_distance = 0.0

        for i in range(start_idx + 1, end_idx - 1):
            dx1 = path[i][0] - path[i-1][0]
            dy1 = path[i][1] - path[i-1][1]
            dx2 = path[i+1][0] - path[i][0]
            dy2 = path[i+1][1] - path[i][1]

            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)

            diff = abs(math.atan2(math.sin(angle2 - angle1), math.cos(angle2 - angle1)))
            total_angle_change += diff

            dist = math.sqrt(dx1*dx1 + dy1*dy1)
            total_distance += dist

        if total_distance < 0.01:
            return 0.0

        return total_angle_change / total_distance

    @property
    def closest_point_index(self) -> int:
        """Get index of closest point on path."""
        return self._closest_idx

    def reset(self):
        """Reset state."""
        self._closest_idx = 0


class StanleyFollower:
    """
    Stanley path following controller.

    Used by Stanford's "Stanley" which won the DARPA Grand Challenge.
    More precise than Pure Pursuit at low speeds, uses front axle.

    Key difference from Pure Pursuit:
    - Uses cross-track error (distance from path) directly
    - Uses heading error
    - Better at following straight lines

    Steering = heading_error + atan(k * cross_track_error / velocity)
    """

    def __init__(self, config: Optional[FollowerConfig] = None):
        self.config = config or FollowerConfig()
        self._k = 2.5  # Cross-track error gain
        self._closest_idx = 0

    def compute(
        self,
        pose: Tuple[float, float, float],
        velocity: float,
        path: List[Tuple[float, float]]
    ) -> ControlCommand:
        """Compute steering using Stanley controller."""
        if not path or len(path) < 2:
            return ControlCommand(0.0, 0.0, reached_goal=True)

        x, y, theta = pose

        # Goal check
        goal = path[-1]
        dist_to_goal = math.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)
        if dist_to_goal < self.config.goal_tolerance:
            return ControlCommand(0.0, 0.0, reached_goal=True)

        # Front axle position
        fx = x + self.config.wheelbase * math.cos(theta)
        fy = y + self.config.wheelbase * math.sin(theta)

        # Find closest point to front axle
        closest_idx = self._find_closest(fx, fy, path)
        self._closest_idx = closest_idx

        # Heading error
        if closest_idx < len(path) - 1:
            path_dx = path[closest_idx + 1][0] - path[closest_idx][0]
            path_dy = path[closest_idx + 1][1] - path[closest_idx][1]
        else:
            path_dx = path[closest_idx][0] - path[closest_idx - 1][0]
            path_dy = path[closest_idx][1] - path[closest_idx - 1][1]

        path_heading = math.atan2(path_dy, path_dx)
        heading_error = path_heading - theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Cross-track error
        dx = fx - path[closest_idx][0]
        dy = fy - path[closest_idx][1]
        cross_track = dx * math.sin(path_heading) - dy * math.cos(path_heading)

        # Stanley formula
        vel_clamp = max(abs(velocity), 0.1)
        steering = heading_error + math.atan2(self._k * cross_track, vel_clamp)

        # Clamp
        steering = max(-self.config.max_steering_angle,
                      min(self.config.max_steering_angle, steering))

        # Velocity
        curvature_factor = 1.0 - 0.6 * abs(steering) / self.config.max_steering_angle
        goal_factor = min(1.0, dist_to_goal / self.config.slow_down_distance)
        target_vel = self.config.max_speed * curvature_factor * goal_factor
        target_vel = max(self.config.min_speed, target_vel)

        return ControlCommand(steering, target_vel)

    def _find_closest(self, x: float, y: float,
                      path: List[Tuple[float, float]]) -> int:
        """Find closest point index."""
        start = max(0, self._closest_idx - 5)
        end = min(len(path), self._closest_idx + 50)

        min_dist = float('inf')
        min_idx = self._closest_idx

        for i in range(start, end):
            d = (path[i][0] - x)**2 + (path[i][1] - y)**2
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def reset(self):
        self._closest_idx = 0