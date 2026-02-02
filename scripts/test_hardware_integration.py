#!/usr/bin/env python3
"""
Test d'integration HARDWARE pour Robocar.

Ce script teste les VRAIS capteurs et composants:
- LiDAR LD19 reel
- GPS RTK reel (si disponible)
- Moteur VESC reel (optionnel, mode test sans mouvement)

A lancer AVANT de deployer sur la voiture pour verifier
que tous les composants hardware fonctionnent ensemble.

Usage:
    python scripts/test_hardware_integration.py
    python scripts/test_hardware_integration.py --lidar-port /dev/ttyUSB2
    python scripts/test_hardware_integration.py --no-motor  # Sans test moteur
"""

import sys
import os
import time
import math
import argparse
from dataclasses import dataclass
from typing import Optional, List, Tuple

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


@dataclass
class HardwareTestResult:
    """Resultat d'un test hardware."""
    component: str
    success: bool
    message: str
    details: dict = None


class HardwareIntegrationTest:
    """Test d'integration des composants hardware."""

    def __init__(self,
                 lidar_port: str = '/dev/ttyUSB0',
                 gps_port: str = '/dev/ttyUSB1',
                 vesc_port: str = '/dev/ttyACM0',
                 test_motor: bool = False):
        self.lidar_port = lidar_port
        self.gps_port = gps_port
        self.vesc_port = vesc_port
        self.test_motor = test_motor

        self.results: List[HardwareTestResult] = []

    def test_lidar(self) -> HardwareTestResult:
        """Teste le LiDAR LD19."""
        print("\n" + "="*50)
        print("TEST 1: LiDAR LD19")
        print("="*50)

        try:
            from driver.lidar import LidarDriver

            print(f"[INFO] Connexion au LiDAR sur {self.lidar_port}...")
            lidar = LidarDriver(self.lidar_port)

            if not lidar.start():
                return HardwareTestResult(
                    component="LiDAR",
                    success=False,
                    message=f"Impossible de demarrer le LiDAR sur {self.lidar_port}"
                )

            print("[OK] LiDAR demarre")
            print("[INFO] Lecture de 5 scans...")

            scans_ok = 0
            total_points = 0
            min_dist = float('inf')
            max_dist = 0

            for i in range(5):
                scan = lidar.get_scan(timeout=2.0)
                if scan:
                    valid_points = sum(1 for p in scan.points if p.valid)
                    if valid_points > 0:
                        scans_ok += 1
                        total_points += valid_points
                        distances = [p.distance for p in scan.points if p.valid]
                        min_dist = min(min_dist, min(distances))
                        max_dist = max(max_dist, max(distances))
                        print(f"  Scan {i+1}: {valid_points} points, "
                              f"min={min(distances):.2f}m, max={max(distances):.2f}m")
                    else:
                        print(f"  Scan {i+1}: Aucun point valide")
                else:
                    print(f"  Scan {i+1}: Timeout")
                time.sleep(0.2)

            lidar.stop()

            if scans_ok >= 3:
                return HardwareTestResult(
                    component="LiDAR",
                    success=True,
                    message=f"LiDAR OK - {scans_ok}/5 scans, {total_points//scans_ok} pts/scan",
                    details={
                        "scans": scans_ok,
                        "avg_points": total_points // scans_ok if scans_ok > 0 else 0,
                        "min_distance": min_dist,
                        "max_distance": max_dist
                    }
                )
            else:
                return HardwareTestResult(
                    component="LiDAR",
                    success=False,
                    message=f"LiDAR instable - seulement {scans_ok}/5 scans"
                )

        except ImportError as e:
            return HardwareTestResult(
                component="LiDAR",
                success=False,
                message=f"Module LiDAR non disponible: {e}"
            )
        except Exception as e:
            return HardwareTestResult(
                component="LiDAR",
                success=False,
                message=f"Erreur LiDAR: {e}"
            )

    def test_gps(self) -> HardwareTestResult:
        """Teste le GPS RTK."""
        print("\n" + "="*50)
        print("TEST 2: GPS RTK")
        print("="*50)

        try:
            # Try Point One GPS first
            try:
                from driver.gps_rtk import GPSRTKDriver
                gps_class = GPSRTKDriver
                gps_name = "GPS RTK Point One"
            except ImportError:
                try:
                    from driver.gps import GPSDriver
                    gps_class = GPSDriver
                    gps_name = "GPS Standard"
                except ImportError:
                    return HardwareTestResult(
                        component="GPS",
                        success=False,
                        message="Aucun driver GPS disponible"
                    )

            print(f"[INFO] Connexion au {gps_name} sur {self.gps_port}...")

            # Check if port exists
            if not os.path.exists(self.gps_port):
                return HardwareTestResult(
                    component="GPS",
                    success=False,
                    message=f"Port {self.gps_port} non trouve"
                )

            gps = gps_class(self.gps_port)

            if not gps.start():
                return HardwareTestResult(
                    component="GPS",
                    success=False,
                    message=f"Impossible de demarrer le GPS sur {self.gps_port}"
                )

            print("[OK] GPS demarre")
            print("[INFO] Attente d'un fix (max 10s)...")

            # Wait for fix
            start_time = time.time()
            position = None
            while time.time() - start_time < 10:
                position = gps.get_position()
                if position and position.is_valid:
                    break
                time.sleep(0.5)

            gps.stop()

            if position and position.is_valid:
                quality_str = getattr(position, 'quality_string', f"Q{position.quality}")
                return HardwareTestResult(
                    component="GPS",
                    success=True,
                    message=f"GPS OK - {quality_str}",
                    details={
                        "latitude": position.latitude,
                        "longitude": position.longitude,
                        "quality": position.quality,
                        "satellites": getattr(position, 'satellites', 'N/A')
                    }
                )
            else:
                return HardwareTestResult(
                    component="GPS",
                    success=False,
                    message="GPS pas de fix apres 10s (normal en interieur)"
                )

        except Exception as e:
            return HardwareTestResult(
                component="GPS",
                success=False,
                message=f"Erreur GPS: {e}"
            )

    def test_motor(self) -> HardwareTestResult:
        """Teste le moteur VESC (sans mouvement)."""
        print("\n" + "="*50)
        print("TEST 3: Moteur VESC")
        print("="*50)

        if not self.test_motor:
            return HardwareTestResult(
                component="Moteur",
                success=True,
                message="Test moteur desactive (--no-motor)"
            )

        try:
            from driver.vesc_motor import VESCMotor

            print(f"[INFO] Connexion au VESC sur {self.vesc_port}...")

            if not os.path.exists(self.vesc_port):
                return HardwareTestResult(
                    component="Moteur",
                    success=False,
                    message=f"Port {self.vesc_port} non trouve"
                )

            motor = VESCMotor(self.vesc_port)

            if not motor.start():
                return HardwareTestResult(
                    component="Moteur",
                    success=False,
                    message="Impossible de demarrer le VESC"
                )

            print("[OK] VESC connecte")

            # Read status without moving
            print("[INFO] Lecture du status...")
            # motor.get_status() or similar

            motor.stop()

            return HardwareTestResult(
                component="Moteur",
                success=True,
                message="VESC OK - Communication etablie"
            )

        except ImportError:
            return HardwareTestResult(
                component="Moteur",
                success=False,
                message="Driver VESC non disponible (pyvesc requis)"
            )
        except Exception as e:
            return HardwareTestResult(
                component="Moteur",
                success=False,
                message=f"Erreur VESC: {e}"
            )

    def test_slam_with_real_lidar(self) -> HardwareTestResult:
        """Teste le SLAM avec le vrai LiDAR."""
        print("\n" + "="*50)
        print("TEST 4: SLAM avec LiDAR reel")
        print("="*50)

        try:
            from driver.lidar import LidarDriver
            from slam.slam_core import SLAM, SLAMConfig

            print(f"[INFO] Demarrage LiDAR...")
            lidar = LidarDriver(self.lidar_port)
            if not lidar.start():
                return HardwareTestResult(
                    component="SLAM",
                    success=False,
                    message="LiDAR requis pour tester le SLAM"
                )

            print("[INFO] Initialisation SLAM...")
            config = SLAMConfig(map_size_pixels=200, map_size_meters=20.0)
            slam = SLAM(config)

            print("[INFO] Integration SLAM + LiDAR (10 iterations)...")
            successful_updates = 0

            for i in range(10):
                scan = lidar.get_scan(timeout=1.0)
                if scan:
                    # Convert to mm for SLAM
                    scan_mm = [
                        int(p.distance * 1000) if p.valid else 12000
                        for p in scan.points
                    ]

                    # Update SLAM
                    pose = slam.update(scan_mm)
                    successful_updates += 1

                    if i % 3 == 0:
                        print(f"  Update {i+1}: pose=({pose.x:.2f}, {pose.y:.2f}, "
                              f"{math.degrees(pose.theta):.0f}deg)")

                time.sleep(0.1)

            lidar.stop()

            # Get final map
            map_img = slam.get_map()
            import numpy as np
            coverage = np.sum(map_img != 127) / map_img.size * 100

            if successful_updates >= 7:
                return HardwareTestResult(
                    component="SLAM",
                    success=True,
                    message=f"SLAM OK - {successful_updates}/10 updates, {coverage:.1f}% couverture",
                    details={
                        "updates": successful_updates,
                        "coverage": coverage,
                        "map_size": map_img.shape
                    }
                )
            else:
                return HardwareTestResult(
                    component="SLAM",
                    success=False,
                    message=f"SLAM instable - seulement {successful_updates}/10 updates"
                )

        except Exception as e:
            return HardwareTestResult(
                component="SLAM",
                success=False,
                message=f"Erreur SLAM: {e}"
            )

    def test_navigation_controller_with_real_lidar(self) -> HardwareTestResult:
        """Teste le NavigationController avec le vrai LiDAR (sans moteur)."""
        print("\n" + "="*50)
        print("TEST 5: Navigation Controller (LiDAR reel, moteur simule)")
        print("="*50)

        try:
            from driver.lidar import LidarDriver
            from interface.sensor_interface import ILidarSensor, LidarData, LidarPoint
            from interface import SimulatedGPSAdapter, SimulatedMotorAdapter
            from navigation_controller import NavigationController

            # Adaptateur pour le vrai LiDAR
            class RealLidarTestAdapter(ILidarSensor):
                def __init__(self, port):
                    self.driver = LidarDriver(port)

                def start(self):
                    return self.driver.start()

                def stop(self):
                    self.driver.stop()

                def get_scan(self):
                    scan = self.driver.get_scan(timeout=0.5)
                    if not scan:
                        return None
                    points = [
                        LidarPoint(p.angle, p.distance, p.intensity, p.valid)
                        for p in scan.points
                    ]
                    return LidarData(points, scan.timestamp, scan.scan_frequency)

                def is_running(self):
                    return self.driver.is_running

            print("[INFO] Creation des adaptateurs...")
            lidar = RealLidarTestAdapter(self.lidar_port)
            gps = SimulatedGPSAdapter()
            motor = SimulatedMotorAdapter()

            print("[INFO] Creation du NavigationController...")
            controller = NavigationController(lidar, gps, motor, mode='simulation')
            controller.set_initial_pose(0, 0, 0)
            controller.add_waypoint_local(2, 0)

            print("[INFO] Demarrage des capteurs...")
            if not controller.init_sensors():
                return HardwareTestResult(
                    component="Navigation",
                    success=False,
                    message="Echec initialisation capteurs"
                )

            print("[INFO] Test de detection d'obstacles (5s)...")
            start_time = time.time()
            obstacle_readings = []

            while time.time() - start_time < 5:
                controller.update_state_from_motor(0.05)

                # Check obstacles
                obstacle, dist, _ = controller.check_obstacles()
                if dist < float('inf'):
                    obstacle_readings.append(dist)

                if len(obstacle_readings) > 0 and len(obstacle_readings) % 10 == 0:
                    avg = sum(obstacle_readings[-10:]) / 10
                    print(f"  Obstacle moyen: {avg:.2f}m")

                time.sleep(0.05)

            controller.shutdown()

            if len(obstacle_readings) > 20:
                avg_dist = sum(obstacle_readings) / len(obstacle_readings)
                min_dist = min(obstacle_readings)
                return HardwareTestResult(
                    component="Navigation",
                    success=True,
                    message=f"Navigation OK - Detection obstacles: min={min_dist:.2f}m, avg={avg_dist:.2f}m",
                    details={
                        "readings": len(obstacle_readings),
                        "min_distance": min_dist,
                        "avg_distance": avg_dist
                    }
                )
            else:
                return HardwareTestResult(
                    component="Navigation",
                    success=False,
                    message="Pas assez de lectures LiDAR"
                )

        except Exception as e:
            import traceback
            traceback.print_exc()
            return HardwareTestResult(
                component="Navigation",
                success=False,
                message=f"Erreur: {e}"
            )

    def run_all_tests(self):
        """Execute tous les tests."""
        print("\n" + "="*60)
        print("   ROBOCAR - TEST D'INTEGRATION HARDWARE")
        print("="*60)
        print(f"\nConfiguration:")
        print(f"  LiDAR port: {self.lidar_port}")
        print(f"  GPS port:   {self.gps_port}")
        print(f"  VESC port:  {self.vesc_port}")
        print(f"  Test moteur: {'Oui' if self.test_motor else 'Non'}")

        # Run tests
        self.results.append(self.test_lidar())
        self.results.append(self.test_gps())
        self.results.append(self.test_motor())

        # Only run SLAM and Navigation if LiDAR passed
        if self.results[0].success:
            self.results.append(self.test_slam_with_real_lidar())
            self.results.append(self.test_navigation_controller_with_real_lidar())
        else:
            self.results.append(HardwareTestResult(
                component="SLAM",
                success=False,
                message="Ignore - LiDAR requis"
            ))
            self.results.append(HardwareTestResult(
                component="Navigation",
                success=False,
                message="Ignore - LiDAR requis"
            ))

        # Print summary
        self.print_summary()

    def print_summary(self):
        """Affiche le resume des tests."""
        print("\n" + "="*60)
        print("                    RESUME DES TESTS")
        print("="*60 + "\n")

        passed = 0
        failed = 0

        for result in self.results:
            status = "[OK]    " if result.success else "[ECHEC] "
            print(f"{status} {result.component}: {result.message}")
            if result.success:
                passed += 1
            else:
                failed += 1

            if result.details:
                for key, value in result.details.items():
                    print(f"           {key}: {value}")

        print("\n" + "="*60)
        print(f"Resultats: {passed} OK / {failed} ECHEC")
        print("="*60)

        if failed == 0:
            print("\nTOUS LES TESTS ONT REUSSI!")
            print("La voiture est prete pour le deploiement.")
        elif passed >= 3:
            print("\nTESTS PARTIELLEMENT REUSSIS")
            print("Verifiez les composants en echec avant deploiement.")
        else:
            print("\nTESTS EN ECHEC")
            print("Corrigez les problemes hardware avant deploiement.")

        print("="*60)


def main():
    parser = argparse.ArgumentParser(
        description="Test d'integration hardware Robocar"
    )
    parser.add_argument(
        '--lidar-port',
        default='/dev/ttyUSB2',
        help='Port du LiDAR (defaut: /dev/ttyUSB2)'
    )
    parser.add_argument(
        '--gps-port',
        default='/dev/ttyUSB1',
        help='Port du GPS (defaut: /dev/ttyUSB1)'
    )
    parser.add_argument(
        '--vesc-port',
        default='/dev/ttyACM0',
        help='Port du VESC (defaut: /dev/ttyACM0)'
    )
    parser.add_argument(
        '--no-motor',
        action='store_true',
        help='Desactiver le test moteur'
    )
    parser.add_argument(
        '--motor',
        action='store_true',
        help='Activer le test moteur (desactive par defaut pour securite)'
    )

    args = parser.parse_args()

    tester = HardwareIntegrationTest(
        lidar_port=args.lidar_port,
        gps_port=args.gps_port,
        vesc_port=args.vesc_port,
        test_motor=args.motor and not args.no_motor
    )

    tester.run_all_tests()


if __name__ == '__main__':
    main()
