#!/usr/bin/env python3
"""
Test d'integration complet pour Robocar.

Ce script teste l'ensemble des fonctionnalites:
- Navigation autonome de point A a point B
- Evitement d'obstacles
- SLAM (cartographie et localisation)
- Detection LiDAR

Execute tous les tests en simulation pour valider le code
AVANT de le deployer sur la vraie voiture.

Usage:
    python scripts/test_integration.py                    # Tous les tests
    python scripts/test_integration.py --scenario parking # Un scenario
    python scripts/test_integration.py --gui              # Avec visualisation
    python scripts/test_integration.py --verbose          # Mode verbeux
"""

import sys
import os
import math
import time
import argparse
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from enum import Enum

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from simulation.environment import (
    Environment, Obstacle, ObstacleType,
    create_parking_env, create_corridor_env, create_epitech_env, create_empty_env
)
from interface import (
    SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
)
from navigation_controller import NavigationController, NavigationState


class TestResult(Enum):
    """Resultat d'un test."""
    PASSED = "PASSED"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    COLLISION = "COLLISION"


@dataclass
class TestMetrics:
    """Metriques collectees pendant un test."""
    # Temps
    start_time: float = 0.0
    end_time: float = 0.0
    elapsed_time: float = 0.0

    # Distance
    total_distance: float = 0.0
    direct_distance: float = 0.0  # Distance directe A->B
    efficiency: float = 0.0       # direct/total (1.0 = parfait)

    # Waypoints
    waypoints_total: int = 0
    waypoints_reached: int = 0

    # Obstacles
    min_obstacle_distance: float = float('inf')
    obstacle_stops: int = 0       # Nombre d'arrets pour obstacle
    collision_count: int = 0      # Nombre de collisions

    # Position
    final_distance_to_goal: float = 0.0
    max_deviation: float = 0.0    # Deviation max par rapport au chemin direct

    # Vitesse
    avg_speed: float = 0.0
    max_speed: float = 0.0


@dataclass
class TestScenario:
    """Definition d'un scenario de test."""
    name: str
    description: str
    environment: Environment
    start_pose: Tuple[float, float, float]  # x, y, theta
    waypoints: List[Tuple[float, float]]    # Liste de (x, y)
    timeout: float = 120.0                  # Timeout en secondes
    min_success_distance: float = 0.5       # Distance max pour considerer arrive
    expected_result: TestResult = TestResult.PASSED


@dataclass
class TestReport:
    """Rapport d'un test."""
    scenario: TestScenario
    result: TestResult
    metrics: TestMetrics
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def __str__(self):
        status = "OK" if self.result == TestResult.PASSED else "ECHEC"
        return (
            f"[{status}] {self.scenario.name}\n"
            f"  Temps: {self.metrics.elapsed_time:.1f}s\n"
            f"  Distance: {self.metrics.total_distance:.1f}m\n"
            f"  Waypoints: {self.metrics.waypoints_reached}/{self.metrics.waypoints_total}\n"
            f"  Min obstacle: {self.metrics.min_obstacle_distance:.2f}m\n"
            f"  Collisions: {self.metrics.collision_count}"
        )


class CollisionChecker:
    """Verifie les collisions avec l'environnement."""

    ROBOT_RADIUS = 0.2  # Rayon du robot en metres

    def __init__(self, environment: Environment):
        self.environment = environment
        self.collision_count = 0
        self.min_distance = float('inf')

    def check_collision(self, x: float, y: float) -> bool:
        """
        Verifie si le robot est en collision a la position donnee.

        Returns:
            True si collision detectee
        """
        # Verifier tous les obstacles
        for obs in self.environment.obstacles:
            dist = self._distance_to_obstacle(x, y, obs)

            if dist < self.min_distance:
                self.min_distance = dist

            if dist < self.ROBOT_RADIUS:
                self.collision_count += 1
                return True

        return False

    def _distance_to_obstacle(self, x: float, y: float, obs: Obstacle) -> float:
        """Calcule la distance minimale a un obstacle."""
        if obs.obstacle_type == ObstacleType.WALL:
            return self._distance_to_wall(x, y, obs)
        elif obs.obstacle_type == ObstacleType.CYLINDER or obs.obstacle_type == ObstacleType.CIRCLE:
            return self._distance_to_circle(x, y, obs)
        elif obs.obstacle_type == ObstacleType.BOX:
            return self._distance_to_box(x, y, obs)
        return float('inf')

    def _distance_to_wall(self, x: float, y: float, obs: Obstacle) -> float:
        """Distance a un segment de mur."""
        x1, y1, x2, y2 = obs.x, obs.y, obs.x2, obs.y2

        # Vecteur du segment
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy

        if length_sq == 0:
            return math.sqrt((x - x1)**2 + (y - y1)**2)

        # Projection sur le segment
        t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / length_sq))

        # Point le plus proche
        px = x1 + t * dx
        py = y1 + t * dy

        return math.sqrt((x - px)**2 + (y - py)**2)

    def _distance_to_circle(self, x: float, y: float, obs: Obstacle) -> float:
        """Distance a un cercle/cylindre."""
        dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
        return dist - obs.radius

    def _distance_to_box(self, x: float, y: float, obs: Obstacle) -> float:
        """Distance a une boite (approximation)."""
        # Transformer en repere local de la boite
        cos_r = math.cos(-obs.rotation)
        sin_r = math.sin(-obs.rotation)

        lx = cos_r * (x - obs.x) - sin_r * (y - obs.y)
        ly = sin_r * (x - obs.x) + cos_r * (y - obs.y)

        # Distance au rectangle
        half_w = obs.width / 2
        half_h = obs.height / 2

        dx = max(abs(lx) - half_w, 0)
        dy = max(abs(ly) - half_h, 0)

        return math.sqrt(dx * dx + dy * dy)


class IntegrationTestRunner:
    """Execute les tests d'integration."""

    LOOP_RATE = 20  # Hz
    COLLISION_CHECK_RATE = 10  # Hz

    def __init__(self, gui: bool = False, verbose: bool = False):
        self.gui = gui
        self.verbose = verbose
        self.reports: List[TestReport] = []

    def run_scenario(self, scenario: TestScenario) -> TestReport:
        """Execute un scenario de test."""
        if self.verbose:
            print(f"\n{'='*60}")
            print(f"TEST: {scenario.name}")
            print(f"Description: {scenario.description}")
            print(f"{'='*60}")

        # Initialiser les metriques
        metrics = TestMetrics()
        errors = []
        warnings = []

        # Creer les adaptateurs
        env = scenario.environment
        lidar = SimulatedLidarAdapter(env)
        gps = SimulatedGPSAdapter()
        motor = SimulatedMotorAdapter()

        # Creer le controleur
        controller = NavigationController(
            lidar=lidar,
            gps=gps,
            motor=motor,
            mode='simulation'
        )

        # Configurer la position initiale
        start_x, start_y, start_theta = scenario.start_pose
        controller.set_initial_pose(start_x, start_y, start_theta)

        # Ajouter les waypoints
        for wp_x, wp_y in scenario.waypoints:
            controller.add_waypoint_local(wp_x, wp_y)

        metrics.waypoints_total = len(scenario.waypoints)

        # Calculer la distance directe
        if scenario.waypoints:
            goal_x, goal_y = scenario.waypoints[-1]
            metrics.direct_distance = math.sqrt(
                (goal_x - start_x)**2 + (goal_y - start_y)**2
            )

        # Initialiser le verificateur de collision
        collision_checker = CollisionChecker(env)

        # Demarrer les capteurs
        if not controller.init_sensors():
            errors.append("Echec initialisation capteurs")
            return TestReport(
                scenario=scenario,
                result=TestResult.FAILED,
                metrics=metrics,
                errors=errors
            )

        # Variables de suivi
        result = TestResult.PASSED
        dt = 1.0 / self.LOOP_RATE
        collision_check_interval = int(self.LOOP_RATE / self.COLLISION_CHECK_RATE)
        iteration = 0

        metrics.start_time = time.time()
        prev_x, prev_y = start_x, start_y

        try:
            while True:
                iteration += 1

                # Verifier timeout
                elapsed = time.time() - metrics.start_time
                if elapsed > scenario.timeout:
                    result = TestResult.TIMEOUT
                    errors.append(f"Timeout apres {elapsed:.1f}s")
                    break

                # Mise a jour physique
                controller.update_state_from_motor(dt)

                # Verifier collision (moins frequemment)
                if iteration % collision_check_interval == 0:
                    if collision_checker.check_collision(
                        controller.state.x,
                        controller.state.y
                    ):
                        result = TestResult.COLLISION
                        errors.append(
                            f"Collision a ({controller.state.x:.2f}, {controller.state.y:.2f})"
                        )
                        break

                # Mettre a jour metriques
                dx = controller.state.x - prev_x
                dy = controller.state.y - prev_y
                metrics.total_distance += math.sqrt(dx*dx + dy*dy)
                prev_x, prev_y = controller.state.x, controller.state.y

                if controller.state.speed > metrics.max_speed:
                    metrics.max_speed = controller.state.speed

                if controller.state.obstacle_detected:
                    metrics.obstacle_stops += 1

                if controller.state.obstacle_distance < metrics.min_obstacle_distance:
                    metrics.min_obstacle_distance = controller.state.obstacle_distance

                # Iteration de navigation
                if not controller.navigation_step():
                    # Navigation terminee
                    metrics.waypoints_reached = controller.current_waypoint_idx
                    break

                # Affichage verbeux
                if self.verbose and iteration % 20 == 0:
                    wp_idx = min(controller.current_waypoint_idx, len(scenario.waypoints) - 1)
                    print(
                        f"\r  [{elapsed:5.1f}s] "
                        f"pos=({controller.state.x:6.2f}, {controller.state.y:6.2f}) "
                        f"WP {wp_idx+1}/{len(scenario.waypoints)} "
                        f"dist={controller.state.distance_to_waypoint:5.2f}m "
                        f"obs={controller.state.obstacle_distance:5.2f}m",
                        end=""
                    )

                time.sleep(dt)

        except KeyboardInterrupt:
            errors.append("Interruption utilisateur")
            result = TestResult.FAILED

        finally:
            controller.shutdown()

        # Finaliser metriques
        metrics.end_time = time.time()
        metrics.elapsed_time = metrics.end_time - metrics.start_time
        metrics.waypoints_reached = controller.current_waypoint_idx
        metrics.collision_count = collision_checker.collision_count
        metrics.min_obstacle_distance = collision_checker.min_distance

        if metrics.elapsed_time > 0:
            metrics.avg_speed = metrics.total_distance / metrics.elapsed_time

        if metrics.total_distance > 0:
            metrics.efficiency = metrics.direct_distance / metrics.total_distance

        # Calculer distance finale au goal
        if scenario.waypoints:
            goal_x, goal_y = scenario.waypoints[-1]
            metrics.final_distance_to_goal = math.sqrt(
                (controller.state.x - goal_x)**2 +
                (controller.state.y - goal_y)**2
            )

            # Verifier si arrive
            if metrics.waypoints_reached >= len(scenario.waypoints):
                if result == TestResult.PASSED:
                    result = TestResult.PASSED
            elif result == TestResult.PASSED:
                result = TestResult.FAILED
                errors.append(
                    f"N'a atteint que {metrics.waypoints_reached}/{metrics.waypoints_total} waypoints"
                )

        # Warnings
        if metrics.efficiency < 0.5:
            warnings.append(f"Efficacite faible: {metrics.efficiency:.1%}")

        if metrics.obstacle_stops > 10:
            warnings.append(f"Beaucoup d'arrets obstacles: {metrics.obstacle_stops}")

        if self.verbose:
            print()  # Nouvelle ligne apres le \r

        report = TestReport(
            scenario=scenario,
            result=result,
            metrics=metrics,
            errors=errors,
            warnings=warnings
        )

        self.reports.append(report)
        return report

    def run_scenario_with_slam(self, scenario: TestScenario) -> TestReport:
        """Execute un scenario avec SLAM actif."""
        if self.verbose:
            print(f"\n{'='*60}")
            print(f"TEST SLAM: {scenario.name}")
            print(f"{'='*60}")

        # Importer SLAM
        try:
            from slam.slam_core import SLAM, SLAMConfig
            slam_available = True
        except ImportError as e:
            if self.verbose:
                print(f"[WARN] SLAM non disponible: {e}")
            slam_available = False

        # Initialiser les metriques
        metrics = TestMetrics()
        errors = []
        warnings = []

        # Creer les adaptateurs
        env = scenario.environment
        lidar = SimulatedLidarAdapter(env)
        gps = SimulatedGPSAdapter()
        motor = SimulatedMotorAdapter()

        # Creer SLAM si disponible
        slam = None
        if slam_available:
            config = SLAMConfig(
                map_size_pixels=400,
                map_size_meters=30.0
            )
            slam = SLAM(config)

        # Creer le controleur
        controller = NavigationController(
            lidar=lidar,
            gps=gps,
            motor=motor,
            mode='simulation'
        )

        # Configurer
        start_x, start_y, start_theta = scenario.start_pose
        controller.set_initial_pose(start_x, start_y, start_theta)

        for wp_x, wp_y in scenario.waypoints:
            controller.add_waypoint_local(wp_x, wp_y)

        metrics.waypoints_total = len(scenario.waypoints)

        # Collision checker
        collision_checker = CollisionChecker(env)

        # Demarrer
        if not controller.init_sensors():
            errors.append("Echec initialisation capteurs")
            return TestReport(
                scenario=scenario,
                result=TestResult.FAILED,
                metrics=metrics,
                errors=errors
            )

        # Variables
        result = TestResult.PASSED
        dt = 1.0 / self.LOOP_RATE
        iteration = 0
        prev_x, prev_y = start_x, start_y
        prev_theta = start_theta

        metrics.start_time = time.time()

        try:
            while True:
                iteration += 1
                elapsed = time.time() - metrics.start_time

                if elapsed > scenario.timeout:
                    result = TestResult.TIMEOUT
                    errors.append(f"Timeout apres {elapsed:.1f}s")
                    break

                # Mise a jour physique
                controller.update_state_from_motor(dt)

                # Mise a jour SLAM
                if slam:
                    scan = lidar.get_scan()
                    if scan:
                        # Convertir scan en mm
                        scan_mm = [
                            int(p.distance * 1000) if p.valid else 12000
                            for p in scan.points
                        ]

                        # Calculer odometrie
                        dx = controller.state.x - prev_x
                        dy = controller.state.y - prev_y
                        dxy_mm = math.sqrt(dx*dx + dy*dy) * 1000
                        dtheta_deg = math.degrees(controller.state.theta - prev_theta)

                        # Mettre a jour SLAM
                        slam.update(scan_mm, velocity=(dxy_mm, dtheta_deg, dt))

                        prev_x, prev_y = controller.state.x, controller.state.y
                        prev_theta = controller.state.theta

                # Verifier collision
                if collision_checker.check_collision(
                    controller.state.x,
                    controller.state.y
                ):
                    result = TestResult.COLLISION
                    errors.append(
                        f"Collision a ({controller.state.x:.2f}, {controller.state.y:.2f})"
                    )
                    break

                # Metriques
                if controller.state.obstacle_distance < metrics.min_obstacle_distance:
                    metrics.min_obstacle_distance = controller.state.obstacle_distance

                # Navigation
                if not controller.navigation_step():
                    metrics.waypoints_reached = controller.current_waypoint_idx
                    break

                if self.verbose and iteration % 40 == 0:
                    slam_info = ""
                    if slam:
                        slam_pose = slam.get_pose()
                        slam_info = f" SLAM=({slam_pose.x:.2f},{slam_pose.y:.2f})"

                    print(
                        f"\r  [{elapsed:5.1f}s] "
                        f"pos=({controller.state.x:5.2f},{controller.state.y:5.2f})"
                        f"{slam_info}",
                        end=""
                    )

                time.sleep(dt)

        except KeyboardInterrupt:
            errors.append("Interruption utilisateur")
            result = TestResult.FAILED

        finally:
            controller.shutdown()

            # Sauvegarder la carte SLAM
            if slam and self.verbose:
                try:
                    import numpy as np
                    map_img = slam.get_map()
                    # Calculer couverture
                    coverage = np.sum(map_img != 127) / map_img.size * 100
                    print(f"\n  SLAM: Couverture carte = {coverage:.1f}%")
                except Exception as e:
                    warnings.append(f"Erreur sauvegarde carte: {e}")

        # Finaliser
        metrics.end_time = time.time()
        metrics.elapsed_time = metrics.end_time - metrics.start_time
        metrics.waypoints_reached = controller.current_waypoint_idx
        metrics.collision_count = collision_checker.collision_count
        metrics.min_obstacle_distance = collision_checker.min_distance

        if metrics.waypoints_reached < metrics.waypoints_total and result == TestResult.PASSED:
            result = TestResult.FAILED
            errors.append(f"N'a atteint que {metrics.waypoints_reached}/{metrics.waypoints_total} waypoints")

        if self.verbose:
            print()

        report = TestReport(
            scenario=scenario,
            result=result,
            metrics=metrics,
            errors=errors,
            warnings=warnings
        )

        self.reports.append(report)
        return report

    def print_summary(self):
        """Affiche le resume de tous les tests."""
        print("\n" + "="*70)
        print("                    RAPPORT DE TEST D'INTEGRATION")
        print("="*70)

        passed = sum(1 for r in self.reports if r.result == TestResult.PASSED)
        failed = sum(1 for r in self.reports if r.result == TestResult.FAILED)
        timeout = sum(1 for r in self.reports if r.result == TestResult.TIMEOUT)
        collision = sum(1 for r in self.reports if r.result == TestResult.COLLISION)

        print(f"\nResultats: {passed} PASSED / {failed} FAILED / {timeout} TIMEOUT / {collision} COLLISION")
        print(f"Total: {len(self.reports)} tests\n")

        for report in self.reports:
            # Couleur selon resultat
            if report.result == TestResult.PASSED:
                status = "[OK]    "
            elif report.result == TestResult.FAILED:
                status = "[ECHEC] "
            elif report.result == TestResult.TIMEOUT:
                status = "[TIMEOUT]"
            else:
                status = "[CRASH] "

            print(f"{status} {report.scenario.name}")
            print(f"         Temps: {report.metrics.elapsed_time:.1f}s | "
                  f"Distance: {report.metrics.total_distance:.1f}m | "
                  f"Waypoints: {report.metrics.waypoints_reached}/{report.metrics.waypoints_total} | "
                  f"Efficacite: {report.metrics.efficiency:.0%}")

            if report.errors:
                for err in report.errors:
                    print(f"         ! {err}")

            if report.warnings:
                for warn in report.warnings:
                    print(f"         ? {warn}")

            print()

        print("="*70)

        # Verdict final
        if passed == len(self.reports):
            print("VERDICT: TOUS LES TESTS ONT REUSSI")
            print("Le code est pret pour le deploiement sur la voiture!")
        else:
            print(f"VERDICT: {len(self.reports) - passed} TESTS ONT ECHOUE")
            print("Corrigez les problemes avant le deploiement.")

        print("="*70)

        return passed == len(self.reports)


def create_test_scenarios() -> List[TestScenario]:
    """Cree tous les scenarios de test."""
    scenarios = []

    # ============================================
    # TEST 1: Navigation simple (environnement vide)
    # ============================================
    scenarios.append(TestScenario(
        name="Navigation Simple",
        description="Aller en ligne droite de (0,0) a (5,0) sans obstacles",
        environment=create_empty_env(20, 20),
        start_pose=(0.0, 0.0, 0.0),
        waypoints=[(5.0, 0.0)],
        timeout=30.0
    ))

    # ============================================
    # TEST 2: Navigation avec virage
    # ============================================
    scenarios.append(TestScenario(
        name="Navigation avec Virages",
        description="Parcours en L avec 3 waypoints",
        environment=create_empty_env(20, 20),
        start_pose=(0.0, 0.0, 0.0),
        waypoints=[(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)],
        timeout=60.0
    ))

    # ============================================
    # TEST 3: Evitement d'obstacle simple
    # ============================================
    env_obstacle = create_empty_env(20, 20)
    env_obstacle.add_cylinder(3.0, 0.0, 0.5)  # Obstacle sur le chemin

    scenarios.append(TestScenario(
        name="Evitement Obstacle Simple",
        description="Aller de (0,0) a (6,0) avec un obstacle au milieu",
        environment=env_obstacle,
        start_pose=(0.0, 0.0, 0.0),
        waypoints=[(6.0, 0.0)],
        timeout=45.0
    ))

    # ============================================
    # TEST 4: Couloir avec obstacles
    # ============================================
    scenarios.append(TestScenario(
        name="Couloir avec Obstacles",
        description="Traverser un couloir avec plusieurs obstacles",
        environment=create_corridor_env(),
        start_pose=(-6.0, 0.0, 0.0),
        waypoints=[(6.0, 0.0)],
        timeout=60.0
    ))

    # ============================================
    # TEST 5: Parking
    # ============================================
    scenarios.append(TestScenario(
        name="Navigation Parking",
        description="Naviguer dans un parking avec voitures garees",
        environment=create_parking_env(),
        start_pose=(-2.0, -5.0, math.pi/2),
        waypoints=[(0.0, 0.0), (2.0, 5.0)],
        timeout=90.0
    ))

    # ============================================
    # TEST 6: Environnement Epitech (complexe)
    # ============================================
    # Obstacles: lampadaires (5,5), (0,10), (-5,5); poubelles (3,2)
    scenarios.append(TestScenario(
        name="Epitech Navigation",
        description="Navigation complexe dans l'environnement Epitech simule",
        environment=create_epitech_env(),
        start_pose=(0.0, -10.0, math.pi/2),
        waypoints=[(-2.0, -5.0), (-2.0, 3.0), (7.0, 3.0), (7.0, 12.0)],
        timeout=120.0
    ))

    # ============================================
    # TEST 7: Slalom entre obstacles
    # ============================================
    env_slalom = create_empty_env(25, 10)
    # Ligne d'obstacles alternees
    env_slalom.add_cylinder(3.0, -1.5, 0.4)
    env_slalom.add_cylinder(6.0, 1.5, 0.4)
    env_slalom.add_cylinder(9.0, -1.5, 0.4)
    env_slalom.add_cylinder(12.0, 1.5, 0.4)
    env_slalom.add_cylinder(15.0, -1.5, 0.4)

    scenarios.append(TestScenario(
        name="Slalom",
        description="Slalom entre des obstacles alternes",
        environment=env_slalom,
        start_pose=(-10.0, 0.0, 0.0),
        waypoints=[(0.0, 0.0), (18.0, 0.0)],
        timeout=90.0
    ))

    # ============================================
    # TEST 8: Cul-de-sac (doit rebrousser chemin)
    # ============================================
    env_deadend = create_empty_env(15, 15)
    # Creer un cul-de-sac
    env_deadend.add_wall(-2, 3, 2, 3)    # Mur du fond
    env_deadend.add_wall(-2, 0, -2, 3)   # Mur gauche
    env_deadend.add_wall(2, 0, 2, 3)     # Mur droit

    scenarios.append(TestScenario(
        name="Evitement Cul-de-sac",
        description="Detecter le cul-de-sac et contourner",
        environment=env_deadend,
        start_pose=(0.0, -3.0, math.pi/2),
        waypoints=[(0.0, 5.0)],  # Derriere le cul-de-sac
        timeout=90.0
    ))

    # ============================================
    # TEST 9: Point tres proche
    # ============================================
    scenarios.append(TestScenario(
        name="Precision Arrivee",
        description="Atteindre un point tres proche avec precision",
        environment=create_empty_env(10, 10),
        start_pose=(0.0, 0.0, 0.0),
        waypoints=[(1.0, 0.5)],
        timeout=20.0,
        min_success_distance=0.3
    ))

    # ============================================
    # TEST 10: Parcours complet multi-waypoints
    # ============================================
    # Parcours en boucle evitant tous les obstacles
    scenarios.append(TestScenario(
        name="Parcours Complet",
        description="Parcours avec 5 waypoints dans environnement Epitech",
        environment=create_epitech_env(),
        start_pose=(-3.0, -12.0, math.pi/2),
        waypoints=[
            (-3.0, -5.0),  # Point 1 - zone libre
            (-3.0, 3.0),   # Point 2 - monte cote ouest
            (7.0, 3.0),    # Point 3 - traverse vers l'est
            (7.0, 12.0),   # Point 4 - nord-est
            (-3.0, -5.0),  # Retour au depart
        ],
        timeout=180.0
    ))

    return scenarios


def main():
    parser = argparse.ArgumentParser(
        description="Tests d'integration Robocar",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  python scripts/test_integration.py                    # Tous les tests
  python scripts/test_integration.py --scenario parking # Un scenario specifique
  python scripts/test_integration.py --verbose          # Mode verbeux
  python scripts/test_integration.py --slam             # Avec tests SLAM
        """
    )

    parser.add_argument(
        '--scenario', '-s',
        type=str,
        help='Nom du scenario a tester (defaut: tous)'
    )
    parser.add_argument(
        '--gui', '-g',
        action='store_true',
        help='Activer la visualisation graphique'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Mode verbeux'
    )
    parser.add_argument(
        '--slam',
        action='store_true',
        help='Activer les tests SLAM'
    )
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='Lister les scenarios disponibles'
    )
    parser.add_argument(
        '--quick', '-q',
        action='store_true',
        help='Tests rapides (premiers scenarios uniquement)'
    )

    args = parser.parse_args()

    # Creer les scenarios
    scenarios = create_test_scenarios()

    # Lister les scenarios
    if args.list:
        print("\nScenarios disponibles:")
        print("-" * 50)
        for i, s in enumerate(scenarios, 1):
            print(f"  {i}. {s.name}")
            print(f"     {s.description}")
        print()
        return 0

    # Filtrer les scenarios
    if args.scenario:
        filtered = [s for s in scenarios if args.scenario.lower() in s.name.lower()]
        if not filtered:
            print(f"Scenario '{args.scenario}' non trouve.")
            print("Utilisez --list pour voir les scenarios disponibles.")
            return 1
        scenarios = filtered

    if args.quick:
        scenarios = scenarios[:3]  # Seulement les 3 premiers

    # Creer le runner
    runner = IntegrationTestRunner(gui=args.gui, verbose=args.verbose)

    print("\n" + "="*70)
    print("          ROBOCAR - TESTS D'INTEGRATION AUTOMATISES")
    print("="*70)
    print(f"\nNombre de tests: {len(scenarios)}")
    print(f"Mode SLAM: {'Oui' if args.slam else 'Non'}")
    print(f"Mode verbose: {'Oui' if args.verbose else 'Non'}")
    print()

    # Executer les tests
    for scenario in scenarios:
        if args.slam:
            runner.run_scenario_with_slam(scenario)
        else:
            runner.run_scenario(scenario)

    # Afficher le resume
    success = runner.print_summary()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
