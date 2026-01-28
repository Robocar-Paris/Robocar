#!/usr/bin/env python3
"""
Navigation Demo

Full navigation demo with simulated environment:
- A* path planning
- DWA obstacle avoidance
- Pure Pursuit path following
- Real-time visualization

Usage:
    python scripts/demo_navigation.py
    python scripts/demo_navigation.py --no-viz
"""

import sys
import os
import time
import math
import argparse
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from slam.occupancy_grid import OccupancyGrid
from navigation.global_planner import GlobalPlanner, PlannerConfig, simplify_path
from navigation.local_planner import DWAPlanner, DWAConfig
from navigation.path_follower import PurePursuitFollower, FollowerConfig
from navigation.waypoint_manager import WaypointManager

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


def create_test_map():
    """Create a test map with obstacles and walls."""
    grid = OccupancyGrid(width=200, height=200, resolution=0.1,
                         origin_x=-10, origin_y=-10)

    # Build walls
    for i in range(200):
        grid.set_occupied(i, 0)          # Bottom
        grid.set_occupied(i, 199)        # Top
        grid.set_occupied(0, i)          # Left
        grid.set_occupied(199, i)        # Right

    # Interior walls
    for i in range(60, 140):
        grid.set_occupied(80, i)         # Vertical wall with gap

    for i in range(60, 80):
        grid.set_occupied(i, 100)        # Horizontal wall

    for i in range(120, 140):
        grid.set_occupied(i, 100)        # Horizontal wall

    # Obstacles (blocks)
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            grid.set_occupied(40 + dx, 60 + dy)
            grid.set_occupied(140 + dx, 50 + dy)
            grid.set_occupied(60 + dx, 150 + dy)
            grid.set_occupied(150 + dx, 140 + dy)

    return grid


class SimRobot:
    """Simple simulated robot for demo."""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.trail_x = [x]
        self.trail_y = [y]

    def update(self, dt=0.05):
        self.theta += self.angular_velocity * dt
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt
        self.trail_x.append(self.x)
        self.trail_y.append(self.y)
        if len(self.trail_x) > 2000:
            self.trail_x.pop(0)
            self.trail_y.pop(0)


def run_demo_console():
    """Console-only demo."""
    print("=" * 60)
    print("NAVIGATION DEMO (Console)")
    print("=" * 60)

    # Setup
    grid = create_test_map()
    planner = GlobalPlanner(PlannerConfig(robot_radius=0.2, safety_margin=0.1))
    planner.set_map(grid, inflate=True)

    follower = PurePursuitFollower(FollowerConfig(
        wheelbase=0.26, max_speed=0.5, lookahead_distance=0.6
    ))

    robot = SimRobot(x=-5.0, y=-5.0, theta=0)
    goal = (5.0, 5.0)

    # Plan path
    print(f"\nPlanning path from ({robot.x}, {robot.y}) to {goal}...")
    path = planner.plan((robot.x, robot.y), goal)

    if path is None:
        print("No path found!")
        return

    path = simplify_path(path, tolerance=0.2)
    length = planner.path_length(path)
    print(f"Path found: {len(path)} waypoints, {length:.1f}m")

    # Follow path
    print("\nFollowing path...")
    for step in range(500):
        cmd = follower.compute(
            pose=(robot.x, robot.y, robot.theta),
            velocity=robot.velocity,
            path=path
        )

        if cmd.reached_goal:
            print(f"\nGoal reached at step {step}!")
            break

        # Apply commands
        robot.velocity = cmd.velocity
        # Convert steering angle to angular velocity
        if abs(cmd.velocity) > 0.01:
            robot.angular_velocity = cmd.velocity * math.tan(cmd.steering_angle) / 0.26
        else:
            robot.angular_velocity = 0

        robot.update()

        if step % 50 == 0:
            dist = math.sqrt((goal[0] - robot.x)**2 + (goal[1] - robot.y)**2)
            print(f"  Step {step}: pos=({robot.x:.2f}, {robot.y:.2f}) "
                  f"speed={robot.velocity:.2f}m/s dist_to_goal={dist:.2f}m")

    final_dist = math.sqrt((goal[0] - robot.x)**2 + (goal[1] - robot.y)**2)
    print(f"\nFinal distance to goal: {final_dist:.3f}m")
    print("Demo complete!")


def run_demo_visual():
    """Visual demo with matplotlib."""
    if not MATPLOTLIB_AVAILABLE:
        run_demo_console()
        return

    print("=" * 60)
    print("NAVIGATION DEMO (Visual)")
    print("=" * 60)
    print("Click on the map to set a new goal!")

    # Setup
    grid = create_test_map()
    planner = GlobalPlanner(PlannerConfig(robot_radius=0.2, safety_margin=0.1))
    planner.set_map(grid, inflate=True)

    dwa = DWAPlanner(DWAConfig(
        max_vel_x=0.5, min_vel_x=0.0, max_vel_theta=1.0,
        sim_time=1.5, vx_samples=6, vtheta_samples=12
    ))

    follower = PurePursuitFollower(FollowerConfig(
        wheelbase=0.26, max_speed=0.5
    ))

    robot = SimRobot(x=-5.0, y=-5.0, theta=0.5)

    # State
    state = {
        'goal': (5.0, 5.0),
        'path': None,
        'step': 0,
        'status': 'Planning...'
    }

    # Initial plan
    state['path'] = planner.plan((robot.x, robot.y), state['goal'])
    if state['path']:
        state['path'] = simplify_path(state['path'], 0.15)
        state['status'] = f"Path: {len(state['path'])} pts"

    # Setup figure
    fig, (ax_map, ax_info) = plt.subplots(1, 2, figsize=(14, 7),
                                           gridspec_kw={'width_ratios': [3, 1]})
    fig.suptitle('Robocar Navigation Demo - Click to set goal', fontsize=14)

    # Map view
    map_img = grid.get_map_rgb()
    ax_map.imshow(map_img, extent=[-10, 10, -10, 10], origin='lower', alpha=0.8)
    ax_map.set_xlabel('X (meters)')
    ax_map.set_ylabel('Y (meters)')
    ax_map.grid(True, alpha=0.2)

    # Plot elements
    path_line, = ax_map.plot([], [], 'g-', linewidth=2, alpha=0.7, label='Path')
    trail_line, = ax_map.plot([], [], 'b-', linewidth=1, alpha=0.4, label='Trail')
    robot_marker, = ax_map.plot([], [], 'b^', markersize=15, label='Robot')
    heading_line, = ax_map.plot([], [], 'b-', linewidth=2)
    goal_marker, = ax_map.plot([state['goal'][0]], [state['goal'][1]], 'r*',
                                markersize=20, label='Goal')
    ax_map.legend(loc='upper left')

    # Info panel
    ax_info.axis('off')
    info_text = ax_info.text(0.05, 0.95, '', transform=ax_info.transAxes,
                              verticalalignment='top', fontfamily='monospace',
                              fontsize=10)

    # Click handler
    def on_click(event):
        if event.inaxes != ax_map:
            return
        state['goal'] = (event.xdata, event.ydata)
        goal_marker.set_data([event.xdata], [event.ydata])

        # Replan
        new_path = planner.plan((robot.x, robot.y), state['goal'])
        if new_path:
            state['path'] = simplify_path(new_path, 0.15)
            state['status'] = f"New path: {len(state['path'])} pts"
            follower.reset()
        else:
            state['status'] = "No path to clicked point!"

    fig.canvas.mpl_connect('button_press_event', on_click)

    def update(frame):
        path = state['path']
        goal = state['goal']

        if path and len(path) > 1:
            # Pure Pursuit
            cmd = follower.compute(
                pose=(robot.x, robot.y, robot.theta),
                velocity=robot.velocity,
                path=path
            )

            if cmd.reached_goal:
                robot.velocity = 0
                robot.angular_velocity = 0
                state['status'] = "Goal reached!"
            else:
                robot.velocity = cmd.velocity
                if abs(cmd.velocity) > 0.01:
                    robot.angular_velocity = (cmd.velocity *
                        math.tan(cmd.steering_angle) / 0.26)
                else:
                    robot.angular_velocity = 0

        robot.update()
        state['step'] += 1

        # Replan periodically
        if state['step'] % 200 == 0 and path:
            new_path = planner.plan((robot.x, robot.y), goal)
            if new_path:
                state['path'] = simplify_path(new_path, 0.15)

        # Update visuals
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            path_line.set_data(px, py)

        trail_line.set_data(robot.trail_x, robot.trail_y)
        robot_marker.set_data([robot.x], [robot.y])

        hx = robot.x + 0.5 * math.cos(robot.theta)
        hy = robot.y + 0.5 * math.sin(robot.theta)
        heading_line.set_data([robot.x, hx], [robot.y, hy])

        # Info
        dist = math.sqrt((goal[0] - robot.x)**2 + (goal[1] - robot.y)**2)
        info = (
            f"ROBOCAR STATUS\n"
            f"{'─'*25}\n"
            f"Position:\n"
            f"  X: {robot.x:7.2f} m\n"
            f"  Y: {robot.y:7.2f} m\n"
            f"  θ: {math.degrees(robot.theta):7.1f}°\n"
            f"\nMotion:\n"
            f"  Speed:   {robot.velocity:.2f} m/s\n"
            f"  Omega:   {robot.angular_velocity:.2f} r/s\n"
            f"\nNavigation:\n"
            f"  Goal: ({goal[0]:.1f}, {goal[1]:.1f})\n"
            f"  Dist: {dist:.2f} m\n"
            f"  Path: {len(path) if path else 0} pts\n"
            f"\nStatus: {state['status']}\n"
            f"Step: {state['step']}"
        )
        info_text.set_text(info)

        return path_line, trail_line, robot_marker, heading_line, info_text

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Navigation Demo')
    parser.add_argument('--no-viz', action='store_true')
    args = parser.parse_args()

    if args.no_viz or not MATPLOTLIB_AVAILABLE:
        run_demo_console()
    else:
        run_demo_visual()


if __name__ == '__main__':
    main()