#!/usr/bin/env python3
"""
Simulation graphique de la voiture RC.

Affiche une visualisation en temps reel avec matplotlib:
- Position du robot
- Scan LiDAR
- Obstacles detectes
- Waypoints et trajectoire

Usage:
    python scripts/run_simulation_gui.py
    python scripts/run_simulation_gui.py --env corridor
"""

import sys
import time
import argparse
import math
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyArrow, Polygon
from matplotlib.collections import LineCollection
import matplotlib.animation as animation

from simulation import (
    LidarSimulator, LidarSimulatorConfig,
    GPSSimulator,
    Environment, Obstacle, ObstacleType,
    create_parking_env, create_corridor_env
)
from perception import LidarProcessor, ObstacleDetector


class GraphicalSimulation:
    """
    Simulation graphique de la voiture RC.
    """

    # Parametres navigation
    WAYPOINT_REACHED_DIST = 0.5
    MAX_SPEED = 0.5
    MIN_SPEED = 0.1
    OBSTACLE_STOP_DIST = 0.5
    OBSTACLE_SLOW_DIST = 1.5
    STEERING_GAIN = 2.0

    def __init__(self, environment: Environment):
        self.env = environment

        # Simulateurs
        self.lidar = LidarSimulator(environment)
        self.gps = GPSSimulator(rtk_mode=True)

        # Etat robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 0.0

        # Waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.trajectory = []  # Historique des positions

        # Scan LiDAR actuel
        self.scan_points = []

        # Stats
        self.total_distance = 0.0
        self.start_time = 0.0
        self.running = True
        self.paused = False

        # Setup graphique
        self._setup_plot()

    def _setup_plot(self):
        """Configure le graphique matplotlib."""
        plt.ion()  # Mode interactif

        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('Simulation Voiture RC')

        # Limites basees sur l'environnement
        margin = 1.0
        self.ax.set_xlim(-self.env.width/2 - margin, self.env.width/2 + margin)
        self.ax.set_ylim(-self.env.height/2 - margin, self.env.height/2 + margin)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (metres)')
        self.ax.set_ylabel('Y (metres)')

        # Elements graphiques (seront mis a jour)
        self.obstacle_patches = []
        self.waypoint_markers = []
        self.robot_patch = None
        self.robot_arrow = None
        self.lidar_scatter = None
        self.trajectory_line = None
        self.info_text = None

        # Dessiner les obstacles statiques
        self._draw_obstacles()

        # Connecter les evenements clavier
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('close_event', self._on_close)

    def _draw_obstacles(self):
        """Dessine les obstacles de l'environnement."""
        for obs in self.env.obstacles:
            if obs.obstacle_type == ObstacleType.WALL:
                self.ax.plot([obs.x, obs.x2], [obs.y, obs.y2],
                           'k-', linewidth=2)

            elif obs.obstacle_type == ObstacleType.BOX:
                # Rectangle avec rotation
                w, h = obs.width, obs.height
                corners = np.array([
                    [-w/2, -h/2],
                    [w/2, -h/2],
                    [w/2, h/2],
                    [-w/2, h/2]
                ])
                # Rotation
                cos_r = np.cos(obs.rotation)
                sin_r = np.sin(obs.rotation)
                rot_matrix = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
                corners = corners @ rot_matrix.T
                # Translation
                corners += np.array([obs.x, obs.y])

                patch = Polygon(corners, closed=True,
                              facecolor='gray', edgecolor='black',
                              alpha=0.7)
                self.ax.add_patch(patch)
                self.obstacle_patches.append(patch)

            elif obs.obstacle_type in [ObstacleType.CYLINDER, ObstacleType.CIRCLE]:
                circle = Circle((obs.x, obs.y), obs.radius,
                               facecolor='darkgray', edgecolor='black',
                               alpha=0.7)
                self.ax.add_patch(circle)
                self.obstacle_patches.append(circle)

    def _draw_waypoints(self):
        """Dessine les waypoints."""
        for marker in self.waypoint_markers:
            marker.remove()
        self.waypoint_markers.clear()

        for i, (wx, wy) in enumerate(self.waypoints):
            if i < self.current_waypoint_idx:
                # Waypoint atteint
                color = 'green'
                marker = 'o'
                size = 80
            elif i == self.current_waypoint_idx:
                # Waypoint actuel
                color = 'red'
                marker = 'X'
                size = 150
            else:
                # Waypoint futur
                color = 'orange'
                marker = 'o'
                size = 100

            scatter = self.ax.scatter(wx, wy, c=color, marker=marker,
                                     s=size, zorder=5, edgecolors='black')
            self.waypoint_markers.append(scatter)

            # Numero du waypoint
            text = self.ax.text(wx + 0.3, wy + 0.3, str(i+1),
                              fontsize=10, fontweight='bold')
            self.waypoint_markers.append(text)

    def _draw_robot(self):
        """Dessine le robot."""
        # Supprimer ancien robot
        if self.robot_patch:
            self.robot_patch.remove()
        if self.robot_arrow:
            self.robot_arrow.remove()

        # Corps du robot (rectangle)
        robot_length = 0.4
        robot_width = 0.25

        corners = np.array([
            [-robot_length/2, -robot_width/2],
            [robot_length/2, -robot_width/2],
            [robot_length/2, robot_width/2],
            [-robot_length/2, robot_width/2]
        ])

        # Rotation
        cos_t = np.cos(self.theta)
        sin_t = np.sin(self.theta)
        rot_matrix = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        corners = corners @ rot_matrix.T
        corners += np.array([self.x, self.y])

        self.robot_patch = Polygon(corners, closed=True,
                                  facecolor='blue', edgecolor='darkblue',
                                  alpha=0.8, zorder=10)
        self.ax.add_patch(self.robot_patch)

        # Fleche de direction
        arrow_length = 0.5
        dx = arrow_length * cos_t
        dy = arrow_length * sin_t
        self.robot_arrow = self.ax.arrow(self.x, self.y, dx, dy,
                                        head_width=0.15, head_length=0.1,
                                        fc='yellow', ec='black', zorder=11)

    def _draw_lidar(self):
        """Dessine le scan LiDAR."""
        if self.lidar_scatter:
            self.lidar_scatter.remove()

        if not self.scan_points:
            return

        points = np.array(self.scan_points)
        if len(points) > 0:
            # Points proches en rouge, loin en vert
            distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            colors = plt.cm.RdYlGn(distances / 12.0)

            self.lidar_scatter = self.ax.scatter(
                points[:, 0], points[:, 1],
                c=distances, cmap='RdYlGn', s=5, alpha=0.6, zorder=3
            )

    def _draw_trajectory(self):
        """Dessine la trajectoire parcourue."""
        if self.trajectory_line:
            self.trajectory_line.remove()

        if len(self.trajectory) > 1:
            traj = np.array(self.trajectory)
            self.trajectory_line, = self.ax.plot(
                traj[:, 0], traj[:, 1],
                'b-', alpha=0.5, linewidth=2, zorder=2
            )

    def _draw_info(self):
        """Affiche les informations."""
        if self.info_text:
            self.info_text.remove()

        elapsed = time.time() - self.start_time if self.start_time > 0 else 0

        if self.current_waypoint_idx < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_idx]
            dist = self.distance_to_waypoint(target[0], target[1])
            wp_info = f"Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: {dist:.1f}m"
        else:
            wp_info = "TERMINE!"

        info = (
            f"Position: ({self.x:.2f}, {self.y:.2f})\n"
            f"Heading: {math.degrees(self.theta):.0f}°\n"
            f"Vitesse: {self.speed:.2f} m/s\n"
            f"{wp_info}\n"
            f"Distance totale: {self.total_distance:.1f}m\n"
            f"Temps: {elapsed:.1f}s\n"
            f"\n[ESPACE] Pause | [Q] Quitter | [R] Reset"
        )

        self.info_text = self.ax.text(
            0.02, 0.98, info,
            transform=self.ax.transAxes,
            verticalalignment='top',
            fontfamily='monospace',
            fontsize=10,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

    def _on_key(self, event):
        """Gestion des touches clavier."""
        if event.key == ' ':
            self.paused = not self.paused
        elif event.key == 'q':
            self.running = False
            plt.close()
        elif event.key == 'r':
            self._reset()

    def _on_close(self, event):
        """Fermeture de la fenetre."""
        self.running = False

    def _reset(self):
        """Reset la simulation."""
        self.x = self.start_x
        self.y = self.start_y
        self.theta = self.start_theta
        self.current_waypoint_idx = 0
        self.trajectory = []
        self.total_distance = 0.0
        self.start_time = time.time()

    def set_start_position(self, x: float, y: float, theta: float = 0.0):
        """Definit la position de depart."""
        self.x = x
        self.y = y
        self.theta = theta
        self.start_x = x
        self.start_y = y
        self.start_theta = theta

        self.lidar.set_robot_pose(x, y, theta)
        self.gps.set_local_position(x, y, theta)

    def add_waypoint(self, x: float, y: float):
        """Ajoute un waypoint."""
        self.waypoints.append((x, y))

    def distance_to_waypoint(self, target_x: float, target_y: float) -> float:
        dx = target_x - self.x
        dy = target_y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def bearing_to_waypoint(self, target_x: float, target_y: float) -> float:
        dx = target_x - self.x
        dy = target_y - self.y
        return math.atan2(dy, dx)

    def check_obstacles(self) -> tuple:
        """Verifie les obstacles via LiDAR."""
        scan = self.lidar.get_single_scan()

        # Convertir en coordonnees cartesiennes pour affichage
        self.scan_points = []
        min_dist_front = float('inf')
        min_dist_left = float('inf')
        min_dist_right = float('inf')

        for point in scan.points:
            if not point.valid:
                continue

            # Coordonnees dans le repere monde
            angle_world = point.angle + self.theta
            px = self.x + point.distance * math.cos(angle_world)
            py = self.y + point.distance * math.sin(angle_world)
            self.scan_points.append([px, py])

            angle_deg = math.degrees(point.angle)

            if -45 < angle_deg < 45:
                if point.distance < min_dist_front:
                    min_dist_front = point.distance

            if 45 < angle_deg < 90:
                if point.distance < min_dist_left:
                    min_dist_left = point.distance

            if -90 < angle_deg < -45:
                if point.distance < min_dist_right:
                    min_dist_right = point.distance

        steering = 0.0
        if min_dist_front < self.OBSTACLE_SLOW_DIST:
            if min_dist_left > min_dist_right:
                steering = 0.5
            else:
                steering = -0.5

        obstacle = min_dist_front < self.OBSTACLE_STOP_DIST

        return obstacle, min_dist_front, steering

    def compute_control(self, target_x: float, target_y: float) -> tuple:
        """Calcule les commandes."""
        distance = self.distance_to_waypoint(target_x, target_y)
        target_bearing = self.bearing_to_waypoint(target_x, target_y)

        heading_error = target_bearing - self.theta
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        steering = self.STEERING_GAIN * heading_error
        steering = max(-1.0, min(1.0, steering))

        speed = self.MAX_SPEED
        if distance < 2.0:
            speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * (distance / 2.0)

        return speed, steering

    def update_physics(self, speed: float, steering: float, dt: float):
        """Met a jour la physique."""
        wheelbase = 0.26

        v = speed
        delta = steering * 0.5

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += (v / wheelbase) * math.tan(delta) * dt

        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        self.total_distance += abs(v) * dt
        self.speed = v

        self.lidar.set_robot_pose(self.x, self.y, self.theta)
        self.gps.set_local_position(self.x, self.y, self.theta)

        # Ajouter a la trajectoire
        self.trajectory.append([self.x, self.y])

    def update_display(self):
        """Met a jour l'affichage."""
        self._draw_waypoints()
        self._draw_trajectory()
        self._draw_lidar()
        self._draw_robot()
        self._draw_info()

        self.ax.set_title(
            f'Simulation Voiture RC - {"PAUSE" if self.paused else "EN COURS"}',
            fontsize=14, fontweight='bold'
        )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        """Boucle principale."""
        if not self.waypoints:
            print("Aucun waypoint!")
            return

        print("=" * 50)
        print("   SIMULATION GRAPHIQUE VOITURE RC")
        print("=" * 50)
        print(f"Waypoints: {len(self.waypoints)}")
        print("Controles:")
        print("  [ESPACE] - Pause/Resume")
        print("  [R] - Reset")
        print("  [Q] - Quitter")
        print("=" * 50)

        self.lidar.start()
        self.gps.start()
        self.start_time = time.time()

        dt = 0.05
        last_update = time.time()

        try:
            while self.running:
                if not plt.fignum_exists(self.fig.number):
                    break

                if self.paused:
                    self.update_display()
                    plt.pause(0.1)
                    continue

                # Fin de navigation?
                if self.current_waypoint_idx >= len(self.waypoints):
                    self.update_display()
                    print("\n[TERMINE] Tous les waypoints atteints!")
                    plt.pause(0.1)
                    continue

                target_x, target_y = self.waypoints[self.current_waypoint_idx]
                distance = self.distance_to_waypoint(target_x, target_y)

                # Waypoint atteint?
                if distance < self.WAYPOINT_REACHED_DIST:
                    print(f"\n[OK] Waypoint {self.current_waypoint_idx + 1} atteint!")
                    self.current_waypoint_idx += 1
                    continue

                # Obstacles
                obstacle, obs_dist, avoid_steering = self.check_obstacles()

                if obstacle:
                    self.speed = 0
                    self.update_physics(0.1, avoid_steering * 2, dt)
                else:
                    speed, steering = self.compute_control(target_x, target_y)

                    if obs_dist < self.OBSTACLE_SLOW_DIST:
                        speed *= 0.5
                        steering += avoid_steering * 0.5

                    self.update_physics(speed, steering, dt)

                # Mise a jour affichage
                current_time = time.time()
                if current_time - last_update > 0.05:
                    self.update_display()
                    last_update = current_time

                plt.pause(dt)

        except KeyboardInterrupt:
            print("\nArret par l'utilisateur")

        finally:
            self.lidar.stop()
            self.gps.stop()

            elapsed = time.time() - self.start_time
            print(f"\n=== RESUME ===")
            print(f"Temps: {elapsed:.1f}s")
            print(f"Distance: {self.total_distance:.1f}m")
            print(f"Waypoints: {self.current_waypoint_idx}/{len(self.waypoints)}")

            plt.ioff()
            plt.show()


def main():
    parser = argparse.ArgumentParser(description='Simulation graphique')
    parser.add_argument('--env', choices=['parking', 'corridor'], default='parking')
    parser.add_argument('--start-x', type=float, default=None)
    parser.add_argument('--start-y', type=float, default=None)

    args = parser.parse_args()

    # Creer environnement
    if args.env == 'parking':
        env = create_parking_env()
        waypoints = [(-3, 0), (0, 0), (3, 0), (7, 0)]
        start_x, start_y = -7, 0
    else:
        env = create_corridor_env()
        waypoints = [(-5, 0), (-2, 0), (0, 0), (2, 0), (5, 0)]
        start_x, start_y = -6, 0

    if args.start_x is not None:
        start_x = args.start_x
    if args.start_y is not None:
        start_y = args.start_y

    # Creer et lancer simulation
    sim = GraphicalSimulation(env)
    sim.set_start_position(start_x, start_y, 0)

    for x, y in waypoints:
        sim.add_waypoint(x, y)

    sim.run()


if __name__ == '__main__':
    main()
