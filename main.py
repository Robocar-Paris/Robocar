#!/usr/bin/env python3
"""
ROBOCAR - Voiture RC Autonome
=============================

Point d'entree principal avec deux modes de fonctionnement:

1. MODE SIMULATION (PC):
   python main.py --mode simulation
   - Visualisation graphique avec matplotlib
   - Environnements virtuels (parking, corridor, epitech)
   - Ideal pour tester l'algorithme de navigation

2. MODE REEL (Jetson Nano):
   python main.py --mode car
   - Pilotage des vrais capteurs (LiDAR LD19, GPS RTK Point One)
   - Controle moteur via VESC
   - Navigation autonome reelle

Coordonnees de reference: Epitech Paris (Le Kremlin-Bicetre)
- Latitude:  48.8156
- Longitude: 2.3631

Usage:
    python main.py --mode simulation --env epitech
    python main.py --mode simulation --env parking --no-gui
    python main.py --mode car --waypoints waypoints.txt
    python main.py --mode car --target-lat 48.8158 --target-lon 2.3635

Auteur: Projet Epitech Robocar
"""

import sys
import argparse
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent / 'src'))


def run_simulation_mode(args):
    """Execute le mode simulation avec visualisation graphique."""
    from navigation_controller import NavigationController
    from simulation import (
        Environment, create_parking_env, create_corridor_env, create_epitech_env
    )
    from interface import (
        SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
    )

    print("=" * 60)
    print("   ROBOCAR - MODE SIMULATION")
    print("=" * 60)

    # Creer l'environnement
    if args.env == 'parking':
        env = create_parking_env()
        waypoints_local = [(-3, 0), (0, 0), (3, 0), (7, 0)]
        start_x, start_y = -7, 0
    elif args.env == 'corridor':
        env = create_corridor_env()
        waypoints_local = [(-5, 0), (-2, 0), (0, 0), (2, 0), (5, 0)]
        start_x, start_y = -6, 0
    elif args.env == 'epitech':
        env = create_epitech_env()
        waypoints_local = [
            (5, 0),    # Point intermediaire
            (10, 5),   # Vers le nord-est
            (15, 10),  # Destination
        ]
        start_x, start_y = 0, 0
    else:
        print(f"[ERREUR] Environnement inconnu: {args.env}")
        return 1

    # Surcharger position de depart si specifiee
    if args.start_x is not None:
        start_x = args.start_x
    if args.start_y is not None:
        start_y = args.start_y

    print(f"Environnement: {args.env}")
    print(f"Position depart: ({start_x}, {start_y})")
    print(f"Waypoints: {len(waypoints_local)}")

    # Creer les adaptateurs simules
    lidar = SimulatedLidarAdapter(env)
    gps = SimulatedGPSAdapter(
        origin_lat=args.origin_lat or 48.8156,
        origin_lon=args.origin_lon or 2.3631
    )
    motor = SimulatedMotorAdapter(wheelbase=0.26)

    # Creer le controleur de navigation
    controller = NavigationController(
        lidar=lidar,
        gps=gps,
        motor=motor,
        mode='simulation'
    )

    # Configurer position initiale
    controller.set_initial_pose(start_x, start_y, 0.0)

    # Ajouter les waypoints
    for wx, wy in waypoints_local:
        controller.add_waypoint_local(wx, wy)

    # Lancer la navigation
    if args.no_gui:
        # Mode console sans visualisation
        controller.run_headless()
    else:
        # Mode graphique avec matplotlib
        controller.run_with_gui()

    return 0


def run_car_mode(args):
    """Execute le mode reel sur Jetson Nano."""
    from navigation_controller import NavigationController
    from interface import (
        RealLidarAdapter, RealGPSAdapter, RealMotorAdapter
    )

    print("=" * 60)
    print("   ROBOCAR - MODE REEL (Jetson Nano)")
    print("=" * 60)

    # Verifier qu'on a une destination
    waypoints = []

    if args.waypoints:
        # Charger depuis fichier
        with open(args.waypoints, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split(',')
                    if len(parts) >= 2:
                        lat = float(parts[0])
                        lon = float(parts[1])
                        waypoints.append((lat, lon))
        print(f"Waypoints charges depuis {args.waypoints}: {len(waypoints)}")

    elif args.target_lat and args.target_lon:
        waypoints.append((args.target_lat, args.target_lon))
        print(f"Destination: ({args.target_lat}, {args.target_lon})")

    else:
        print("[ERREUR] Aucune destination specifiee!")
        print("Utilisez --waypoints fichier.txt ou --target-lat/--target-lon")
        return 1

    print(f"Ports: GPS={args.gps_port}, LiDAR={args.lidar_port}, VESC={args.vesc_port}")

    # Creer les adaptateurs reels
    lidar = RealLidarAdapter(args.lidar_port)
    gps = RealGPSAdapter(args.gps_port, polaris_api_key=args.polaris_key)
    motor = RealMotorAdapter(args.vesc_port)

    # Creer le controleur de navigation
    controller = NavigationController(
        lidar=lidar,
        gps=gps,
        motor=motor,
        mode='car'
    )

    # Ajouter les waypoints GPS
    for lat, lon in waypoints:
        controller.add_waypoint_gps(lat, lon)

    # Lancer la navigation
    try:
        controller.run_real()
    except KeyboardInterrupt:
        print("\n[ARRET] Interruption utilisateur")
    finally:
        controller.shutdown()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description='ROBOCAR - Voiture RC Autonome',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  Mode simulation:
    python main.py --mode simulation --env parking
    python main.py --mode simulation --env epitech
    python main.py --mode simulation --env corridor --no-gui

  Mode reel (Jetson):
    python main.py --mode car --target-lat 48.8158 --target-lon 2.3635
    python main.py --mode car --waypoints mission.txt --polaris-key YOUR_KEY
"""
    )

    # Mode principal
    parser.add_argument(
        '--mode', '-m',
        choices=['simulation', 'car'],
        required=True,
        help='Mode: simulation (PC) ou car (Jetson Nano)'
    )

    # Options simulation
    sim_group = parser.add_argument_group('Options Simulation')
    sim_group.add_argument(
        '--env', '-e',
        choices=['parking', 'corridor', 'epitech'],
        default='epitech',
        help='Environnement de simulation (defaut: epitech)'
    )
    sim_group.add_argument(
        '--no-gui',
        action='store_true',
        help='Mode console sans visualisation graphique'
    )
    sim_group.add_argument(
        '--start-x',
        type=float,
        help='Position X de depart (metres)'
    )
    sim_group.add_argument(
        '--start-y',
        type=float,
        help='Position Y de depart (metres)'
    )
    sim_group.add_argument(
        '--origin-lat',
        type=float,
        default=48.8156,
        help='Latitude origine GPS (defaut: Epitech Paris)'
    )
    sim_group.add_argument(
        '--origin-lon',
        type=float,
        default=2.3631,
        help='Longitude origine GPS (defaut: Epitech Paris)'
    )

    # Options mode reel
    car_group = parser.add_argument_group('Options Mode Reel')
    car_group.add_argument(
        '--target-lat',
        type=float,
        help='Latitude de destination'
    )
    car_group.add_argument(
        '--target-lon',
        type=float,
        help='Longitude de destination'
    )
    car_group.add_argument(
        '--waypoints', '-w',
        type=str,
        help='Fichier de waypoints (lat,lon par ligne)'
    )
    car_group.add_argument(
        '--gps-port',
        default='/dev/ttyUSB0',
        help='Port serie GPS (defaut: /dev/ttyUSB0)'
    )
    car_group.add_argument(
        '--lidar-port',
        default='/dev/ttyUSB1',
        help='Port serie LiDAR (defaut: /dev/ttyUSB1)'
    )
    car_group.add_argument(
        '--vesc-port',
        default='/dev/ttyACM0',
        help='Port serie VESC (defaut: /dev/ttyACM0)'
    )
    car_group.add_argument(
        '--polaris-key',
        type=str,
        help='Cle API Polaris pour corrections RTK'
    )

    args = parser.parse_args()

    # Executer le mode choisi
    if args.mode == 'simulation':
        return run_simulation_mode(args)
    else:
        return run_car_mode(args)


if __name__ == '__main__':
    sys.exit(main())
