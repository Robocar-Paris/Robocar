#!/usr/bin/env python3
"""
Test d'integration VISUEL pour Robocar.

Ce script lance une simulation complete avec visualisation en temps reel:
- Robot naviguant de point A a point B
- Evitement d'obstacles
- Carte SLAM construite en temps reel
- Points LiDAR
- Trajectoire suivie

Utiliser ce script pour valider visuellement que tout fonctionne
AVANT de deployer sur la vraie voiture.

Usage:
    python scripts/test_integration_visual.py              # Scenario par defaut
    python scripts/test_integration_visual.py --env epitech
    python scripts/test_integration_visual.py --env parking
    python scripts/test_integration_visual.py --no-slam    # Sans SLAM
"""

import sys
import os
import math
import time
import argparse

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle
from matplotlib.collections import LineCollection
import matplotlib.gridspec as gridspec

from simulation.environment import (
    Environment, Obstacle, ObstacleType,
    create_parking_env, create_corridor_env, create_epitech_env, create_empty_env
)
from interface import (
    SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
)
from navigation_controller import NavigationController


class VisualIntegrationTest:
    """Test d'integration avec visualisation complete."""

    LOOP_RATE = 20  # Hz

    def __init__(self, environment: Environment, with_slam: bool = True):
        self.environment = environment
        self.with_slam = with_slam

        # SLAM
        self.slam = None
        if with_slam:
            try:
                from slam.slam_core import SLAM, SLAMConfig
                config = SLAMConfig(
                    map_size_pixels=400,
                    map_size_meters=40.0
                )
                self.slam = SLAM(config)
                print("[INFO] SLAM active")
            except ImportError as e:
                print(f"[WARN] SLAM non disponible: {e}")
                self.with_slam = False

        # Adaptateurs
        self.lidar = SimulatedLidarAdapter(environment)
        self.gps = SimulatedGPSAdapter()
        self.motor = SimulatedMotorAdapter()

        # Controleur
        self.controller = NavigationController(
            lidar=self.lidar,
            gps=self.gps,
            motor=self.motor,
            mode='simulation'
        )

        # Etat
        self.running = False
        self.paused = False

        # Historique
        self.trajectory = []
        self.slam_trajectory = []
        self.collision_points = []

        # Metriques
        self.start_time = 0.0
        self.total_distance = 0.0
        self.min_obstacle_dist = float('inf')
        self.collisions = 0

    def setup_scenario(self, start_pose, waypoints):
        """Configure le scenario de test."""
        x, y, theta = start_pose
        self.controller.set_initial_pose(x, y, theta)

        for wx, wy in waypoints:
            self.controller.add_waypoint_local(wx, wy)

        self.start_pose = start_pose
        self.waypoints = waypoints

    def check_collision(self, x: float, y: float) -> bool:
        """Verifie collision avec les obstacles."""
        robot_radius = 0.2

        for obs in self.environment.obstacles:
            if obs.obstacle_type == ObstacleType.CYLINDER or obs.obstacle_type == ObstacleType.CIRCLE:
                dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2) - obs.radius
                if dist < robot_radius:
                    return True
            elif obs.obstacle_type == ObstacleType.BOX:
                # Simplification: check center distance
                cos_r = math.cos(-obs.rotation)
                sin_r = math.sin(-obs.rotation)
                lx = cos_r * (x - obs.x) - sin_r * (y - obs.y)
                ly = sin_r * (x - obs.x) + cos_r * (y - obs.y)
                if abs(lx) < obs.width/2 + robot_radius and abs(ly) < obs.height/2 + robot_radius:
                    return True

        return False

    def draw_environment(self, ax):
        """Dessine l'environnement sur l'axe."""
        ax.clear()

        # Dessiner les obstacles
        for obs in self.environment.obstacles:
            if obs.obstacle_type == ObstacleType.WALL:
                ax.plot([obs.x, obs.x2], [obs.y, obs.y2], 'k-', linewidth=2)

            elif obs.obstacle_type == ObstacleType.CYLINDER or obs.obstacle_type == ObstacleType.CIRCLE:
                circle = Circle((obs.x, obs.y), obs.radius,
                               facecolor='gray', edgecolor='black', alpha=0.8)
                ax.add_patch(circle)

            elif obs.obstacle_type == ObstacleType.BOX:
                w, h = obs.width, obs.height
                corners = np.array([
                    [-w/2, -h/2], [w/2, -h/2], [w/2, h/2], [-w/2, h/2]
                ])
                cos_r = np.cos(obs.rotation)
                sin_r = np.sin(obs.rotation)
                rot_matrix = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
                corners = corners @ rot_matrix.T + np.array([obs.x, obs.y])

                polygon = Polygon(corners, facecolor='brown',
                                 edgecolor='black', alpha=0.7)
                ax.add_patch(polygon)

    def run(self):
        """Execute le test avec visualisation."""
        if not self.controller.init_sensors():
            print("[ERREUR] Impossible d'initialiser les capteurs")
            return False

        # Configuration matplotlib
        plt.ion()

        if self.with_slam:
            fig = plt.figure(figsize=(16, 8))
            gs = gridspec.GridSpec(1, 2, width_ratios=[1.2, 1])
            ax_main = fig.add_subplot(gs[0])
            ax_slam = fig.add_subplot(gs[1])
        else:
            fig, ax_main = plt.subplots(1, 1, figsize=(12, 10))
            ax_slam = None

        fig.canvas.manager.set_window_title('Robocar - Test Integration Visuel')

        # Callbacks
        def on_key(event):
            if event.key == ' ':
                self.paused = not self.paused
            elif event.key == 'q':
                self.running = False

        def on_close(event):
            self.running = False

        fig.canvas.mpl_connect('key_press_event', on_key)
        fig.canvas.mpl_connect('close_event', on_close)

        # Limites de l'environnement
        margin = 5
        ax_main.set_xlim(-self.environment.width/2 - margin,
                         self.environment.width/2 + margin)
        ax_main.set_ylim(-self.environment.height/2 - margin,
                         self.environment.height/2 + margin)
        ax_main.set_aspect('equal')

        self.running = True
        self.start_time = time.time()
        dt = 1.0 / self.LOOP_RATE

        prev_x, prev_y = self.controller.state.x, self.controller.state.y
        prev_theta = self.controller.state.theta
        iteration = 0

        print("\n" + "="*50)
        print("  TEST D'INTEGRATION VISUEL")
        print("="*50)
        print(f"  Waypoints: {len(self.waypoints)}")
        print(f"  SLAM: {'Actif' if self.with_slam else 'Desactive'}")
        print("\n  Controles:")
        print("    [ESPACE] Pause/Resume")
        print("    [Q] Quitter")
        print("="*50 + "\n")

        try:
            while self.running and plt.fignum_exists(fig.number):
                if self.paused:
                    plt.pause(0.1)
                    continue

                iteration += 1

                # Mise a jour physique
                self.controller.update_state_from_motor(dt)

                x = self.controller.state.x
                y = self.controller.state.y
                theta = self.controller.state.theta

                # Mettre a jour trajectoire
                self.trajectory.append((x, y))

                # Calculer distance parcourue
                dx = x - prev_x
                dy = y - prev_y
                self.total_distance += math.sqrt(dx*dx + dy*dy)

                # Verifier collision
                if self.check_collision(x, y):
                    self.collisions += 1
                    self.collision_points.append((x, y))

                # Mettre a jour min obstacle distance
                if self.controller.state.obstacle_distance < self.min_obstacle_dist:
                    self.min_obstacle_dist = self.controller.state.obstacle_distance

                # Mise a jour SLAM
                if self.slam:
                    scan = self.lidar.get_scan()
                    if scan:
                        scan_mm = [
                            int(p.distance * 1000) if p.valid else 12000
                            for p in scan.points
                        ]
                        dxy_mm = math.sqrt(dx*dx + dy*dy) * 1000
                        dtheta_deg = math.degrees(theta - prev_theta)
                        self.slam.update(scan_mm, velocity=(dxy_mm, dtheta_deg, dt))

                        slam_pose = self.slam.get_pose()
                        self.slam_trajectory.append((slam_pose.x, slam_pose.y))

                prev_x, prev_y, prev_theta = x, y, theta

                # Navigation
                nav_continue = self.controller.navigation_step()

                # === MISE A JOUR GRAPHIQUE ===
                if iteration % 2 == 0:  # Mise a jour tous les 2 frames
                    ax_main.clear()

                    # Dessiner environnement
                    self.draw_environment(ax_main)

                    # Dessiner trajectoire
                    if len(self.trajectory) > 1:
                        traj = np.array(self.trajectory)
                        ax_main.plot(traj[:, 0], traj[:, 1], 'b-',
                                    alpha=0.6, linewidth=2, label='Trajectoire')

                    # Dessiner points LiDAR
                    if self.controller.scan_points:
                        points = np.array(self.controller.scan_points)
                        ax_main.scatter(points[:, 0], points[:, 1],
                                       c='red', s=3, alpha=0.5, label='LiDAR')

                    # Dessiner waypoints
                    for i, (wx, wy) in enumerate(self.waypoints):
                        if i < self.controller.current_waypoint_idx:
                            color, size = 'green', 100
                        elif i == self.controller.current_waypoint_idx:
                            color, size = 'red', 150
                        else:
                            color, size = 'orange', 100

                        ax_main.scatter(wx, wy, c=color, s=size,
                                       marker='X', edgecolors='black', zorder=5)
                        ax_main.text(wx + 0.5, wy + 0.5, str(i+1),
                                    fontsize=10, fontweight='bold')

                    # Dessiner robot
                    robot_length, robot_width = 0.4, 0.25
                    corners = np.array([
                        [-robot_length/2, -robot_width/2],
                        [robot_length/2, -robot_width/2],
                        [robot_length/2, robot_width/2],
                        [-robot_length/2, robot_width/2]
                    ])
                    cos_t = np.cos(theta)
                    sin_t = np.sin(theta)
                    rot = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
                    corners = corners @ rot.T + np.array([x, y])

                    robot = Polygon(corners, facecolor='blue',
                                   edgecolor='darkblue', alpha=0.8, zorder=10)
                    ax_main.add_patch(robot)

                    # Fleche de direction
                    ax_main.arrow(x, y, 0.5*cos_t, 0.5*sin_t,
                                 head_width=0.15, head_length=0.1,
                                 fc='yellow', ec='black', zorder=11)

                    # Dessiner collisions
                    if self.collision_points:
                        coll = np.array(self.collision_points)
                        ax_main.scatter(coll[:, 0], coll[:, 1],
                                       c='red', marker='x', s=200,
                                       linewidths=3, label='Collisions')

                    # Informations
                    elapsed = time.time() - self.start_time
                    wp_idx = min(self.controller.current_waypoint_idx, len(self.waypoints))
                    wp_total = len(self.waypoints)

                    info_text = (
                        f"Position: ({x:.2f}, {y:.2f})\n"
                        f"Heading: {math.degrees(theta):.0f} deg\n"
                        f"Vitesse: {self.controller.state.speed:.2f} m/s\n"
                        f"Waypoint: {wp_idx}/{wp_total}\n"
                        f"Distance: {self.controller.state.distance_to_waypoint:.2f}m\n"
                        f"Obstacle: {self.controller.state.obstacle_distance:.2f}m\n"
                        f"Min Obstacle: {self.min_obstacle_dist:.2f}m\n"
                        f"Total dist: {self.total_distance:.1f}m\n"
                        f"Collisions: {self.collisions}\n"
                        f"Temps: {elapsed:.1f}s\n"
                        f"\n[ESPACE] Pause | [Q] Quitter"
                    )

                    ax_main.text(0.02, 0.98, info_text,
                                transform=ax_main.transAxes,
                                verticalalignment='top',
                                fontfamily='monospace', fontsize=9,
                                bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

                    ax_main.set_xlim(-self.environment.width/2 - margin,
                                    self.environment.width/2 + margin)
                    ax_main.set_ylim(-self.environment.height/2 - margin,
                                    self.environment.height/2 + margin)
                    ax_main.set_aspect('equal')
                    ax_main.grid(True, alpha=0.3)
                    ax_main.set_xlabel('X (metres)')
                    ax_main.set_ylabel('Y (metres)')

                    status = "PAUSE" if self.paused else "EN COURS"
                    if not nav_continue:
                        status = "TERMINE" if wp_idx >= wp_total else "BLOQUE"

                    ax_main.set_title(f'Navigation - {status}', fontsize=14, fontweight='bold')

                    # Mise a jour carte SLAM
                    if ax_slam and self.slam:
                        ax_slam.clear()
                        map_img = self.slam.get_map()

                        # Extent dynamique base sur la config SLAM (40m par defaut)
                        half_size = 40.0  # map_size_meters / 2
                        ax_slam.imshow(map_img, cmap='gray', origin='lower',
                                      extent=[-half_size, half_size, -half_size, half_size])

                        # Position robot sur la carte
                        slam_pose = self.slam.get_pose()
                        ax_slam.plot(slam_pose.x, slam_pose.y, 'ro', markersize=8)
                        ax_slam.arrow(slam_pose.x, slam_pose.y,
                                     2*math.cos(slam_pose.theta),
                                     2*math.sin(slam_pose.theta),
                                     head_width=0.5, color='red')

                        # Trajectoire SLAM
                        if len(self.slam_trajectory) > 1:
                            st = np.array(self.slam_trajectory)
                            ax_slam.plot(st[:, 0], st[:, 1], 'g-', alpha=0.5)

                        # Limites dynamiques basees sur l'environnement
                        ax_slam.set_xlim(-half_size, half_size)
                        ax_slam.set_ylim(-half_size, half_size)
                        ax_slam.set_aspect('equal')
                        ax_slam.grid(True, alpha=0.3)
                        ax_slam.set_title(f'Carte SLAM - Robot: ({slam_pose.x:.1f}, {slam_pose.y:.1f})',
                                         fontsize=12, fontweight='bold')
                        ax_slam.set_xlabel('X (metres)')
                        ax_slam.set_ylabel('Y (metres)')

                    fig.canvas.draw()
                    fig.canvas.flush_events()

                if not nav_continue:
                    # Attendre avant de fermer
                    print("\n[INFO] Navigation terminee!")
                    print(f"  Waypoints atteints: {self.controller.current_waypoint_idx}/{len(self.waypoints)}")
                    print(f"  Distance parcourue: {self.total_distance:.1f}m")
                    print(f"  Collisions: {self.collisions}")
                    print(f"  Temps: {time.time() - self.start_time:.1f}s")

                    while self.running and plt.fignum_exists(fig.number):
                        plt.pause(0.5)
                    break

                plt.pause(dt)

        except KeyboardInterrupt:
            print("\n[INFO] Interruption utilisateur")

        finally:
            self.controller.shutdown()
            plt.ioff()

        # Resultat
        success = (self.controller.current_waypoint_idx >= len(self.waypoints)
                   and self.collisions == 0)

        print("\n" + "="*50)
        if success:
            print("  RESULTAT: SUCCES")
            print("  Le code est pret pour le deploiement!")
        else:
            print("  RESULTAT: ECHEC")
            if self.collisions > 0:
                print(f"  - {self.collisions} collision(s) detectee(s)")
            if self.controller.current_waypoint_idx < len(self.waypoints):
                print(f"  - Seulement {self.controller.current_waypoint_idx}/{len(self.waypoints)} waypoints atteints")
        print("="*50)

        return success


def create_scenario(env_name: str):
    """Cree un scenario selon le nom de l'environnement."""

    if env_name == 'empty':
        env = create_empty_env(20, 20)
        start = (0.0, -5.0, math.pi/2)
        waypoints = [(0.0, 5.0)]
        return env, start, waypoints

    elif env_name == 'obstacle':
        env = create_empty_env(20, 20)
        env.add_cylinder(3.0, 0.0, 0.5)
        start = (0.0, 0.0, 0.0)
        waypoints = [(6.0, 0.0)]
        return env, start, waypoints

    elif env_name == 'corridor':
        env = create_corridor_env()
        start = (-6.0, 0.0, 0.0)
        waypoints = [(6.0, 0.0)]
        return env, start, waypoints

    elif env_name == 'parking':
        env = create_parking_env()
        start = (-2.0, -5.0, math.pi/2)
        waypoints = [(0.0, 0.0), (2.0, 5.0)]
        return env, start, waypoints

    elif env_name == 'epitech':
        env = create_epitech_env()
        start = (0.0, -12.0, math.pi/2)
        # Obstacles a eviter:
        # - Lampadaires: (5,5), (0,10), (-5,5), (10,15), (-8,12)
        # - Poubelles: (3,2), (3.8,2)
        # - Borne: (8,8), Banc: (-3,8)
        # Parcours qui reste dans les zones libres
        waypoints = [
            (-2.0, -5.0),   # Zone libre ouest
            (-2.0, 3.0),    # Monte en evitant poubelles
            (7.0, 3.0),     # Traverse vers l'est (sous le lampadaire 5,5)
            (7.0, 12.0),    # Monte vers le nord-est
        ]
        return env, start, waypoints

    elif env_name == 'slalom':
        env = create_empty_env(25, 10)
        env.add_cylinder(3.0, -1.5, 0.4)
        env.add_cylinder(6.0, 1.5, 0.4)
        env.add_cylinder(9.0, -1.5, 0.4)
        env.add_cylinder(12.0, 1.5, 0.4)
        start = (-10.0, 0.0, 0.0)
        waypoints = [(0.0, 0.0), (15.0, 0.0)]
        return env, start, waypoints

    elif env_name == 'full':
        # Parcours complet dans Epitech - boucle evitant tous obstacles
        env = create_epitech_env()
        start = (-3.0, -12.0, math.pi/2)
        waypoints = [
            (-3.0, -5.0),  # Zone libre
            (-3.0, 3.0),   # Monte cote ouest
            (7.0, 3.0),    # Traverse vers l'est
            (7.0, 12.0),   # Nord-est
            (-3.0, -5.0),  # Retour
        ]
        return env, start, waypoints

    else:
        print(f"[WARN] Environnement '{env_name}' inconnu, utilisation de 'parking'")
        return create_scenario('parking')


def main():
    parser = argparse.ArgumentParser(
        description="Test d'integration visuel Robocar"
    )

    parser.add_argument(
        '--env', '-e',
        type=str,
        default='parking',
        choices=['empty', 'obstacle', 'corridor', 'parking', 'epitech', 'slalom', 'full'],
        help='Environnement de test (defaut: parking)'
    )
    parser.add_argument(
        '--no-slam',
        action='store_true',
        help='Desactiver le SLAM'
    )
    parser.add_argument(
        '--start',
        type=str,
        help='Position de depart: x,y,theta (ex: "0,0,1.57")'
    )
    parser.add_argument(
        '--waypoints', '-w',
        type=str,
        help='Waypoints: "x1,y1;x2,y2;..." (ex: "5,0;5,5;0,5")'
    )

    args = parser.parse_args()

    # Charger le scenario
    env, start, waypoints = create_scenario(args.env)

    # Override si specifie
    if args.start:
        parts = [float(x) for x in args.start.split(',')]
        if len(parts) == 3:
            start = tuple(parts)

    if args.waypoints:
        waypoints = []
        for wp in args.waypoints.split(';'):
            x, y = [float(v) for v in wp.split(',')]
            waypoints.append((x, y))

    # Creer et lancer le test
    test = VisualIntegrationTest(env, with_slam=not args.no_slam)
    test.setup_scenario(start, waypoints)

    success = test.run()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
