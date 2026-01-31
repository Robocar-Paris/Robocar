#!/usr/bin/env python3
"""
Script de lancement LiDAR LD19 - Robocar
=========================================

Usage:
    python run_lidar.py              # Mode simulation (par defaut)
    python run_lidar.py --real       # Mode reel avec LiDAR
    python run_lidar.py --test       # Lancer les tests unitaires
    python run_lidar.py --port /dev/ttyUSB0  # Specifier le port

Options:
    --simulate, -s    Mode simulation (defaut)
    --real, -r        Mode reel avec LiDAR physique
    --port, -p        Port serie (defaut: /dev/ttyUSB0)
    --interactive, -i Mode interactif
    --test, -t        Lancer les tests unitaires
"""

import sys
import os
import argparse
from pathlib import Path

# Configuration des chemins
PROJECT_ROOT = Path(__file__).parent
LIDAR_TEST_SRC = PROJECT_ROOT / 'lidar_test' / 'src'
SRC_DIR = PROJECT_ROOT / 'src'

# Ajouter les chemins au PYTHONPATH
sys.path.insert(0, str(LIDAR_TEST_SRC))
sys.path.insert(0, str(SRC_DIR))


def run_simulation(interactive: bool = False):
    """Lance le test LiDAR en mode simulation."""
    print("=" * 60)
    print("    LIDAR LD19 - MODE SIMULATION")
    print("=" * 60)

    from lidar_main import LidarTestSuite

    suite = LidarTestSuite(use_simulator=True)

    if interactive:
        suite.interactive_mode()
    else:
        suite.run_all_tests()


def run_real(port: str, interactive: bool = False):
    """Lance le test LiDAR avec le hardware reel."""
    print("=" * 60)
    print(f"    LIDAR LD19 - MODE REEL ({port})")
    print("=" * 60)

    from lidar_main import LidarTestSuite

    suite = LidarTestSuite(port=port, use_simulator=False)

    if interactive:
        suite.interactive_mode()
    else:
        suite.run_all_tests()


def run_tests():
    """Lance les tests unitaires."""
    print("=" * 60)
    print("    LIDAR LD19 - TESTS UNITAIRES")
    print("=" * 60)

    import subprocess
    result = subprocess.run(
        [sys.executable, str(PROJECT_ROOT / 'lidar_test' / 'test_lidar.py')],
        cwd=str(PROJECT_ROOT / 'lidar_test')
    )
    return result.returncode


def run_quick_demo():
    """Demo rapide du LiDAR en simulation."""
    print("=" * 60)
    print("    LIDAR LD19 - DEMO RAPIDE")
    print("=" * 60)
    print()

    from ld19_parser import LD19Parser, create_test_packet
    from lidar_simulator import LidarSimulator, create_room_with_furniture
    from obstacle_detector import ObstacleDetector, AlertLevel

    # 1. Test du parser
    print("[1/3] Test du parser LD19...")
    parser = LD19Parser()
    test_packet = create_test_packet()
    packets = parser.add_data(test_packet)
    if packets:
        print(f"      Parser OK - {len(packets[0].measurements)} mesures parsees")
    else:
        print("      ERREUR: Parser a echoue")
        return 1

    # 2. Test du simulateur
    print("[2/3] Test du simulateur...")
    env = create_room_with_furniture()
    simulator = LidarSimulator(env)
    scan = simulator.generate_scan(480)
    valid_points = scan.get_valid_measurements()
    print(f"      Simulateur OK - {len(valid_points)} points valides generes")

    # 3. Test de la detection d'obstacles
    print("[3/3] Test de la detection d'obstacles...")
    detector = ObstacleDetector()
    result = detector.detect(scan)
    print(f"      Detection OK - Niveau alerte: {result.max_alert_level.name}")
    print(f"      Obstacles detectes: {len(result.obstacles)}")

    print()
    print("=" * 60)
    print("    TOUS LES TESTS SONT OK!")
    print("=" * 60)
    print()
    print("Prochaines etapes:")
    print("  - python run_lidar.py --simulate     # Tests complets en simulation")
    print("  - python run_lidar.py --real         # Avec le vrai LiDAR")
    print("  - python run_lidar.py --test         # Tests unitaires")

    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Script de lancement LiDAR LD19',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--simulate', '-s', action='store_true',
                           help='Mode simulation (defaut)')
    mode_group.add_argument('--real', '-r', action='store_true',
                           help='Mode reel avec LiDAR physique')
    mode_group.add_argument('--test', '-t', action='store_true',
                           help='Lancer les tests unitaires')
    mode_group.add_argument('--demo', '-d', action='store_true',
                           help='Demo rapide')

    parser.add_argument('--port', '-p', type=str, default='/dev/ttyUSB0',
                       help='Port serie (defaut: /dev/ttyUSB0)')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Mode interactif')

    args = parser.parse_args()

    try:
        if args.test:
            return run_tests()
        elif args.real:
            run_real(args.port, args.interactive)
        elif args.demo:
            return run_quick_demo()
        else:
            # Par defaut: demo rapide si aucun argument
            if len(sys.argv) == 1:
                return run_quick_demo()
            run_simulation(args.interactive)

        return 0

    except KeyboardInterrupt:
        print("\n\nInterrompu par l'utilisateur")
        return 0
    except ImportError as e:
        print(f"\nErreur d'import: {e}")
        print("\nVerifiez que vous avez installe les dependances:")
        print("  pip install numpy pyserial")
        return 1
    except Exception as e:
        print(f"\nErreur: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
