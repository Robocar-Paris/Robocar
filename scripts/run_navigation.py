#!/usr/bin/env python3
"""
Autonomous Navigation Mode

The main script that makes the robot drive autonomously from A to B.
This assembles ALL modules: drivers, perception, SLAM, navigation, safety.

Usage:
    # Navigate to a local coordinate
    python scripts/run_navigation.py --goal 5.0 3.0

    # Navigate to GPS coordinate
    python scripts/run_navigation.py --gps-goal 48.8567 2.3523

    # Multi-waypoint mission
    python scripts/run_navigation.py --mission mission.yaml

    # Use a pre-built map
    python scripts/run_navigation.py --map config/maps/map.yaml --goal 5.0 3.0

Workflow:
    1. Load map (or build one)
    2. Localize robot on map
    3. Plan path to goal
    4. Follow path while avoiding obstacles
    5. Arrive at goal
"""

import sys
import os
import time
import math
import argparse
import signal

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from drivers.lidar_ld19 import LD19Driver
from drivers.gps_pointone import PointOneGPS
from drivers.vesc_motor import VESCController
from perception.lidar_processor import LidarProcessor
from perception.obstacle_detector import ObstacleDetector
from perception.sensor_fusion import SensorFusion
from slam.slam_core import SLAM, SLAMConfig
from slam.occupancy_grid import OccupancyGrid
from navigation.global_planner import GlobalPlanner, PlannerConfig
from navigation.local_planner import DWAPlanner, DWAConfig
from navigation.path_follower import PurePursuitFollower, FollowerConfig
from navigation.waypoint_manager import WaypointManager
from core.state_machine import StateMachine, RobotState, StateEvent
from core.safety import SafetyMonitor, SafetyConfig


class AutonomousNavigator:
    """
    Main autonomous navigation controller.

    Assembles all modules and runs the control loop.
    """

    def __init__(self, args):
        self.args = args

        # === PHASE 0: DRIVERS ===
        self.lidar = LD19Driver(args.lidar_port)
        self.gps = PointOneGPS(args.gps_port)
        self.vesc = VESCController(args.vesc_port)

        # === PHASE 1: PERCEPTION ===
        self.lidar_processor = LidarProcessor()
        self.obstacle_detector = ObstacleDetector()
        self.sensor_fusion = SensorFusion()

        # === PHASE 2: SLAM ===
        self.slam = SLAM(SLAMConfig(
            map_size_pixels=800,
            map_size_meters=40.0,
            scan_size=360
        ))
        self.occupancy_grid = OccupancyGrid(800, 800, 0.05)

        # === PHASE 3: NAVIGATION ===
        self.global_planner = GlobalPlanner(PlannerConfig(
            robot_radius=0.15,
            safety_margin=0.10,
        ))
        self.local_planner = DWAPlanner(DWAConfig(
            max_vel_x=0.5,
            min_vel_x=0.0,
            max_vel_theta=1.0,
        ))
        self.path_follower = PurePursuitFollower(FollowerConfig(
            wheelbase=0.26,
            max_speed=0.5,
            max_steering_angle=0.3,
        ))
        self.waypoint_manager = WaypointManager()

        # === PHASE 4: INTEGRATION ===
        self.state_machine = StateMachine()
        self.safety = SafetyMonitor(
            config=SafetyConfig(
                emergency_stop_distance=0.15,
                warning_distance=0.5,
                max_speed=0.5,
            ),
            emergency_callback=self._emergency_stop
        )

        # Internal state
        self.global_path = None
        self.running = False
        self.loop_count = 0
        self.loop_hz = 0.0

        # Register state machine callbacks
        self._setup_state_callbacks()

    def _setup_state_callbacks(self):
        """Setup state machine callbacks."""
        sm = self.state_machine

        sm.on_enter(RobotState.NAVIGATING, self._on_enter_navigating)
        sm.on_exit(RobotState.NAVIGATING, self._on_exit_navigating)
        sm.on_enter(RobotState.EMERGENCY, self._on_enter_emergency)

        sm.on_transition(self._on_any_transition)

    def _on_any_transition(self, old, event, new):
        print(f"[STATE] {old.name} --{event.name}--> {new.name}")

    def _on_enter_navigating(self):
        print("[NAV] Starting autonomous navigation")
        self.path_follower.reset()

    def _on_exit_navigating(self):
        print("[NAV] Stopping navigation")
        self.vesc.set_duty(0)
        self.vesc.set_servo(0.5)

    def _on_enter_emergency(self):
        print("[EMERGENCY] EMERGENCY STOP ACTIVATED")
        self._emergency_stop()

    def _emergency_stop(self):
        """Immediately stop the robot."""
        if self.vesc.is_running:
            self.vesc.emergency_stop()

    def start(self) -> bool:
        """Initialize and start all systems."""
        print("=" * 60)
        print("ROBOCAR - AUTONOMOUS NAVIGATION")
        print("=" * 60)

        # Start drivers
        print("\n[INIT] Starting drivers...")

        lidar_ok = self.lidar.start()
        print(f"  LiDAR: {'OK' if lidar_ok else 'FAILED'}")

        gps_ok = self.gps.start()
        print(f"  GPS:   {'OK' if gps_ok else 'FAILED'}")

        vesc_ok = self.vesc.start()
        print(f"  VESC:  {'OK' if vesc_ok else 'FAILED'}")

        if not lidar_ok:
            print("[ERROR] LiDAR is required for navigation!")
            return False

        if not vesc_ok:
            print("[ERROR] VESC is required for navigation!")
            return False

        # Setup GPS origin
        if gps_ok:
            pos = self.gps.get_position()
            if pos and pos.quality > 0:
                self.sensor_fusion.set_gps_origin(pos.latitude, pos.longitude)
                self.waypoint_manager.set_gps_origin(pos.latitude, pos.longitude)
                print(f"  GPS Origin: ({pos.latitude:.6f}, {pos.longitude:.6f})")

        # Load map if provided
        if self.args.map:
            print(f"\n[INIT] Loading map from {self.args.map}...")
            self.occupancy_grid = OccupancyGrid.load(
                self.args.map.replace('.yaml', '.png'),
                self.args.map
            )
            self.global_planner.set_map(self.occupancy_grid)
            print("  Map loaded!")

        # Setup waypoints
        self._setup_waypoints()

        # Start safety
        self.safety.start_watchdog()

        self.running = True
        print("\n[OK] All systems initialized!")
        return True

    def _setup_waypoints(self):
        """Setup navigation waypoints from args."""
        self.waypoint_manager.create_mission("Navigation")

        if self.args.goal:
            x, y = self.args.goal
            self.waypoint_manager.add_waypoint(x, y, name="Goal")
            print(f"  Goal: ({x}, {y})")

        elif self.args.gps_goal:
            lat, lon = self.args.gps_goal
            self.waypoint_manager.add_waypoint_gps(lat, lon, name="GPS Goal")
            print(f"  GPS Goal: ({lat}, {lon})")

    def run(self):
        """Main control loop."""
        if not self.start():
            return

        # Start navigation
        self.state_machine.handle_event(StateEvent.START_NAVIGATION)
        # Skip localization for now, go straight to navigating
        self.state_machine.handle_event(StateEvent.LOCALIZED)
        self.waypoint_manager.start()

        print("\n[RUN] Control loop started (20 Hz)")
        print("[RUN] Press Ctrl+C to stop\n")

        try:
            while self.running:
                loop_start = time.time()
                self._control_loop()
                self.loop_count += 1

                # Timing
                elapsed = time.time() - loop_start
                sleep_time = max(0, 0.05 - elapsed)  # Target 20 Hz
                time.sleep(sleep_time)

                if elapsed > 0:
                    self.loop_hz = 0.9 * self.loop_hz + 0.1 * (1.0 / (elapsed + sleep_time))

                # Check mission completion
                if self.waypoint_manager.is_mission_complete:
                    print("\n[DONE] Mission complete!")
                    self.state_machine.handle_event(StateEvent.GOAL_REACHED)
                    break

        except KeyboardInterrupt:
            print("\n[STOP] User interrupt")

        finally:
            self._shutdown()

    def _control_loop(self):
        """Single iteration of the control loop."""

        # ─── 1. READ SENSORS ───
        scan = self.lidar.get_latest_scan()
        gps_pos = self.gps.get_position() if self.gps.is_running else None

        if scan is None:
            return

        # ─── 2. PERCEPTION ───
        processed = self.lidar_processor.process(scan)
        obstacles = self.obstacle_detector.detect(processed)

        # Obstacle positions for DWA
        obstacle_points = []
        for obs in obstacles:
            obstacle_points.extend([
                (p[0], p[1]) for p in obs.points
            ])

        # Nearest obstacle
        nearest = self.obstacle_detector.get_nearest_obstacle(obstacles)
        nearest_dist = nearest.min_distance if nearest else float('inf')

        # ─── 3. SLAM & LOCALIZATION ───
        scan_mm = [int(p.distance * 1000) for p in scan.points]
        total_dist, velocity = self.vesc.get_odometry()
        slam_pose = self.slam.update(scan_mm)

        # Sensor fusion
        if gps_pos and gps_pos.quality > 0:
            self.sensor_fusion.update_gps(
                gps_pos.latitude, gps_pos.longitude,
                gps_pos.accuracy_h, gps_pos.quality
            )

        robot_x = slam_pose.x
        robot_y = slam_pose.y
        robot_theta = slam_pose.theta

        # Update occupancy grid
        import numpy as np
        valid = processed.valid_mask
        self.occupancy_grid.update_from_scan(
            robot_x, robot_y, robot_theta,
            processed.distances[valid],
            processed.angles[valid]
        )

        # ─── 4. SAFETY CHECK ───
        vesc_state = self.vesc.get_state()
        battery_v = vesc_state.voltage if vesc_state else 12.0
        motor_temp = vesc_state.temp_motor if vesc_state else 25.0

        # Safety is checked BEFORE sending any motor command
        self.safety.update_sensor_time()

        # ─── 5. NAVIGATION ───
        if self.state_machine.state == RobotState.NAVIGATING:
            # Update waypoint progress
            self.waypoint_manager.update_position(robot_x, robot_y)
            goal = self.waypoint_manager.get_current_goal()

            if goal is None:
                self.vesc.set_duty(0)
                return

            # Replan global path periodically or if not set
            if self.global_path is None or self.loop_count % 100 == 0:
                self.global_planner.set_map(self.occupancy_grid, inflate=True)
                self.global_path = self.global_planner.plan(
                    (robot_x, robot_y), goal
                )
                if self.global_path:
                    dist = self.global_planner.path_length(self.global_path)
                    print(f"[PATH] Planned path: {len(self.global_path)} points, {dist:.1f}m")
                else:
                    print("[PATH] No path found! Waiting...")
                    self.vesc.set_duty(0)
                    return

            # Local planning (DWA) for obstacle avoidance
            dwa_cmd = self.local_planner.compute_velocity(
                current_pose=(robot_x, robot_y, robot_theta),
                current_vel=(velocity, 0),
                global_path=self.global_path,
                obstacles=obstacle_points,
                goal=goal
            )

            if not dwa_cmd.is_valid:
                # DWA couldn't find a safe trajectory
                print("[NAV] No safe trajectory - stopping")
                self.vesc.set_duty(0)
                self.state_machine.handle_event(StateEvent.OBSTACLE_BLOCKED)
                return

            # Path following for smooth steering
            follower_cmd = self.path_follower.compute(
                pose=(robot_x, robot_y, robot_theta),
                velocity=velocity,
                path=self.global_path
            )

            if follower_cmd.reached_goal:
                print("[NAV] Path follower reached goal")
                self.vesc.set_duty(0)
                return

            # Blend DWA and Pure Pursuit
            # DWA for speed, Pure Pursuit for steering
            target_velocity = min(dwa_cmd.linear, follower_cmd.velocity)
            target_steering = follower_cmd.steering_angle

            # Apply speed limit from waypoint manager
            max_speed = self.waypoint_manager.get_current_max_speed()
            target_velocity = min(target_velocity, max_speed)

            # ─── 6. SAFETY LIMIT ───
            safe_velocity = self.safety.check_and_limit(
                requested_velocity=target_velocity,
                nearest_obstacle_distance=nearest_dist,
                battery_voltage=battery_v,
                motor_temp=motor_temp
            )

            if self.safety.is_emergency:
                self.state_machine.handle_event(StateEvent.EMERGENCY_STOP)
                return

            # ─── 7. SEND COMMANDS ───
            # Convert velocity to duty cycle (simple proportional)
            duty = safe_velocity / 2.0  # max 1 m/s ≈ 0.5 duty
            duty = max(-0.2, min(0.2, duty))

            self.vesc.set_duty(duty)
            self.vesc.set_steering_angle(target_steering)

        elif self.state_machine.state == RobotState.PAUSED:
            self.vesc.set_duty(0)

            # Auto-resume after 3 seconds if path is clear
            if nearest_dist > 0.5 and self.state_machine.time_in_state > 3.0:
                print("[NAV] Path clear, resuming...")
                self.global_path = None  # Force replan
                self.state_machine.handle_event(StateEvent.RESUME)

        else:
            self.vesc.set_duty(0)

        # ─── 8. STATUS ───
        if self.loop_count % 40 == 0:  # Every 2 seconds
            mission = self.waypoint_manager.get_mission_status()
            safety_status = self.safety.get_status()

            print(f"[STATUS] State={self.state_machine.state.name} | "
                  f"Pos=({robot_x:.2f},{robot_y:.2f}) | "
                  f"Speed={velocity:.2f}m/s | "
                  f"Nearest={nearest_dist:.2f}m | "
                  f"Mission={mission.get('progress', 'N/A')} | "
                  f"Safety={safety_status['level']} | "
                  f"Loop={self.loop_hz:.0f}Hz")

    def _shutdown(self):
        """Shutdown all systems."""
        print("\n[SHUTDOWN] Stopping all systems...")

        self.safety.stop_watchdog()

        if self.vesc.is_running:
            self.vesc.emergency_stop()
            self.vesc.stop()

        if self.lidar.is_running:
            self.lidar.stop()

        if self.gps.is_running:
            self.gps.stop()

        # Save final map
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        map_dir = "config/maps"
        os.makedirs(map_dir, exist_ok=True)
        self.occupancy_grid.save(
            os.path.join(map_dir, f"nav_{timestamp}.png"),
            os.path.join(map_dir, f"nav_{timestamp}.yaml")
        )

        print("[SHUTDOWN] Complete")
        print(f"[SUMMARY] Total loops: {self.loop_count}")


def main():
    parser = argparse.ArgumentParser(description='Autonomous Navigation')

    # Ports
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0')
    parser.add_argument('--gps-port', default='/dev/ttyUSB1')
    parser.add_argument('--vesc-port', default='/dev/ttyACM0')

    # Goal
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'),
                        help='Goal in local coordinates (meters)')
    parser.add_argument('--gps-goal', nargs=2, type=float, metavar=('LAT', 'LON'),
                        help='Goal in GPS coordinates')
    parser.add_argument('--mission', type=str,
                        help='Mission YAML file')

    # Map
    parser.add_argument('--map', type=str,
                        help='Pre-built map YAML file')

    args = parser.parse_args()

    if not args.goal and not args.gps_goal and not args.mission:
        parser.error("Please specify a goal: --goal X Y, --gps-goal LAT LON, or --mission FILE")

    navigator = AutonomousNavigator(args)
    navigator.run()


if __name__ == '__main__':
    main()