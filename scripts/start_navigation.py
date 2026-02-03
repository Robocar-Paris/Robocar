#!/usr/bin/env python3
"""
ROBOCAR - Script de lancement de la navigation autonome
========================================================

Ce script lance le systeme de navigation autonome complet.
La voiture se deplace d'un point A vers un point B en utilisant:
- GPS RTK Point One pour la localisation precise
- LiDAR LD19 pour la detection et l'evitement d'obstacles
- VESC pour le controle moteur

Le script effectue automatiquement:
1. Verification des composants (optionnel avec --skip-check)
2. Initialisation des capteurs
3. Attente du fix GPS
4. Navigation autonome vers la destination
5. Evitement d'obstacles en temps reel

Usage:
    # Navigation vers une destination GPS
    python scripts/start_navigation.py \\
        --target-lat 48.8970 --target-lon 2.2195

    # Navigation avec fichier de waypoints
    python scripts/start_navigation.py --waypoints mission.txt

    # Mode simulation (pour test sans hardware)
    python scripts/start_navigation.py --simulation --env epitech

    # Avec clé Polaris pour RTK
    python scripts/start_navigation.py \\
        --target-lat 48.8970 --target-lon 2.2195 \\
        --polaris-key YOUR_API_KEY

Exemple de fichier waypoints (mission.txt):
    # Latitude, Longitude
    48.8156, 2.3631
    48.8158, 2.3635
    48.8160, 2.3638

Codes de retour:
    0 = Navigation terminee avec succes
    1 = Erreur (composants manquants, pas de fix GPS, etc.)
    2 = Interruption utilisateur (Ctrl+C)
"""

import sys
import os
import time
import signal
import argparse
from typing import List, Tuple, Optional

# Ajouter src au path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


def print_banner():
    print(f"""
{Colors.BOLD}{Colors.BLUE}
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║     ██████╗  ██████╗ ██████╗  ██████╗  ██████╗ █████╗ ██████╗  ║
║     ██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗██╔════╝██╔══██╗██╔══██╗ ║
║     ██████╔╝██║   ██║██████╔╝██║   ██║██║     ███████║██████╔╝ ║
║     ██╔══██╗██║   ██║██╔══██╗██║   ██║██║     ██╔══██║██╔══██╗ ║
║     ██║  ██║╚██████╔╝██████╔╝╚██████╔╝╚██████╗██║  ██║██║  ██║ ║
║     ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝  ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝ ║
║                                                           ║
║              NAVIGATION AUTONOME GPS + LIDAR              ║
╚═══════════════════════════════════════════════════════════╝
{Colors.RESET}""")


def log_info(msg: str):
    print(f"{Colors.CYAN}[INFO]{Colors.RESET} {msg}")


def log_ok(msg: str):
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {msg}")


def log_error(msg: str):
    print(f"{Colors.RED}[ERREUR]{Colors.RESET} {msg}")


def log_warn(msg: str):
    print(f"{Colors.YELLOW}[WARN]{Colors.RESET} {msg}")


def log_nav(msg: str):
    print(f"{Colors.BLUE}[NAV]{Colors.RESET} {msg}")


class NavigationLauncher:
    """Lanceur de navigation autonome."""

    def __init__(self, args):
        self.args = args
        self.controller = None
        self.running = False

        # Configuration des ports
        self.lidar_port = args.lidar_port
        self.gps_port = args.gps_port
        self.vesc_port = args.vesc_port

        # Waypoints
        self.waypoints: List[Tuple[float, float]] = []

        # Signal handler pour arret propre
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Gere les signaux d'arret."""
        print(f"\n{Colors.YELLOW}[ARRET]{Colors.RESET} Signal recu, arret en cours...")
        self.running = False
        if self.controller:
            self.controller.running = False

    def check_components(self) -> bool:
        """Verifie que tous les composants sont disponibles."""
        log_info("Verification des composants...")

        # Importer le checker
        from check_components import ComponentChecker

        checker = ComponentChecker(verbose=False)

        # Verifier dependances
        if not checker.check_dependencies():
            return False

        # Verifier ports
        available = checker.check_ports()
        usb_ports = available['usb']
        acm_ports = available['acm']

        # Detecter composants
        lidar_ok, lidar_port = checker.detect_lidar(
            [self.lidar_port] if self.lidar_port else usb_ports
        )
        if lidar_ok and not self.lidar_port:
            self.lidar_port = lidar_port

        gps_ok, gps_port = checker.detect_gps(
            [self.gps_port] if self.gps_port else usb_ports
        )
        if gps_ok and not self.gps_port:
            self.gps_port = gps_port

        vesc_ok, vesc_port = checker.detect_vesc(
            [self.vesc_port] if self.vesc_port else acm_ports
        )
        if vesc_ok and not self.vesc_port:
            self.vesc_port = vesc_port

        all_ok = lidar_ok and gps_ok and vesc_ok

        if not all_ok:
            log_error("Certains composants ne sont pas disponibles")
            return False

        log_ok("Tous les composants sont disponibles")
        return True

    def load_waypoints(self) -> bool:
        """Charge les waypoints depuis les arguments ou un fichier."""
        if self.args.waypoints:
            # Charger depuis fichier
            try:
                with open(self.args.waypoints, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith('#'):
                            parts = line.split(',')
                            if len(parts) >= 2:
                                lat = float(parts[0].strip())
                                lon = float(parts[1].strip())
                                self.waypoints.append((lat, lon))
                log_ok(f"{len(self.waypoints)} waypoints charges depuis {self.args.waypoints}")
            except Exception as e:
                log_error(f"Erreur lecture fichier waypoints: {e}")
                return False

        elif self.args.target_lat and self.args.target_lon:
            # Destination unique
            self.waypoints.append((self.args.target_lat, self.args.target_lon))
            log_ok(f"Destination: ({self.args.target_lat:.6f}, {self.args.target_lon:.6f})")

        else:
            log_error("Aucune destination specifiee!")
            log_info("Utilisez --target-lat/--target-lon ou --waypoints fichier.txt")
            return False

        return True

    def run_simulation(self) -> int:
        """Execute en mode simulation."""
        from navigation_controller import NavigationController
        from simulation import (
            create_parking_env, create_corridor_env, create_epitech_env
        )
        from interface import (
            SimulatedLidarAdapter, SimulatedGPSAdapter, SimulatedMotorAdapter
        )

        log_info(f"Mode simulation - environnement: {self.args.env}")

        # Creer environnement
        if self.args.env == 'parking':
            env = create_parking_env()
            start_x, start_y = -7, 0
            waypoints_local = [(-3, 0), (0, 0), (3, 0), (7, 0)]
        elif self.args.env == 'corridor':
            env = create_corridor_env()
            start_x, start_y = -6, 0
            waypoints_local = [(-5, 0), (-2, 0), (0, 0), (2, 0), (5, 0)]
        else:  # epitech
            env = create_epitech_env()
            start_x, start_y = 0, 0
            waypoints_local = [(5, 0), (10, 5), (15, 10)]

        # Creer adaptateurs simules
        lidar = SimulatedLidarAdapter(env)
        gps = SimulatedGPSAdapter(origin_lat=48.8156, origin_lon=2.3631)
        motor = SimulatedMotorAdapter(wheelbase=0.26)

        # Creer controleur
        self.controller = NavigationController(lidar, gps, motor, mode='simulation')
        self.controller.set_initial_pose(start_x, start_y, 0.0)

        # Ajouter waypoints
        for wx, wy in waypoints_local:
            self.controller.add_waypoint_local(wx, wy)

        log_nav(f"Depart: ({start_x}, {start_y})")
        log_nav(f"Waypoints: {len(waypoints_local)}")

        # Lancer navigation
        if self.args.no_gui:
            self.controller.run_headless()
        else:
            self.controller.run_with_gui()

        return 0

    def run_real(self) -> int:
        """Execute en mode reel sur la voiture."""
        from navigation_controller import NavigationController
        from interface import RealLidarAdapter, RealGPSAdapter, RealMotorAdapter

        log_info("Mode reel - Initialisation des capteurs...")

        # Creer adaptateurs reels
        try:
            log_info(f"LiDAR sur {self.lidar_port}...")
            lidar = RealLidarAdapter(self.lidar_port)

            log_info(f"GPS sur {self.gps_port}...")
            gps = RealGPSAdapter(
                self.gps_port,
                polaris_api_key=self.args.polaris_key
            )

            log_info(f"VESC sur {self.vesc_port}...")
            motor = RealMotorAdapter(self.vesc_port)

        except Exception as e:
            log_error(f"Erreur initialisation: {e}")
            return 1

        # Creer controleur
        self.controller = NavigationController(lidar, gps, motor, mode='car')

        # Ajouter waypoints GPS
        for lat, lon in self.waypoints:
            self.controller.add_waypoint_gps(lat, lon)

        # Afficher resume mission
        print(f"\n{Colors.BOLD}=== MISSION ==={Colors.RESET}")
        print(f"Destination(s):")
        for i, (lat, lon) in enumerate(self.waypoints, 1):
            print(f"  {i}. ({lat:.6f}, {lon:.6f})")

        if self.args.polaris_key:
            log_ok("RTK Polaris active")
        else:
            log_warn("RTK Polaris desactive (precision GPS reduite)")

        print()

        # Confirmation avant demarrage (si demande)
        if not self.args.no_confirm:
            print(f"{Colors.YELLOW}La voiture va demarrer la navigation autonome.{Colors.RESET}")
            print(f"{Colors.YELLOW}Assurez-vous que la zone est degagee!{Colors.RESET}")
            response = input(f"\nDemarrer? [o/N] ").strip().lower()
            if response not in ('o', 'oui', 'y', 'yes'):
                log_info("Navigation annulee")
                return 0

        # Lancer navigation
        log_nav("Demarrage de la navigation...")
        self.running = True

        try:
            self.controller.run_real()
            return 0
        except KeyboardInterrupt:
            log_warn("Interruption utilisateur")
            return 2
        except Exception as e:
            log_error(f"Erreur navigation: {e}")
            return 1
        finally:
            if self.controller:
                self.controller.shutdown()

    def run(self) -> int:
        """Execute la navigation."""
        print_banner()

        # Mode simulation
        if self.args.simulation:
            return self.run_simulation()

        # Mode reel
        # 1. Verifier composants (sauf si skip)
        if not self.args.skip_check:
            if not self.check_components():
                return 1
        else:
            log_warn("Verification des composants ignoree (--skip-check)")

        # 2. Charger waypoints
        if not self.load_waypoints():
            return 1

        # 3. Lancer navigation
        return self.run_real()


def main():
    parser = argparse.ArgumentParser(
        description='Robocar - Navigation Autonome GPS + LiDAR',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Navigation vers un point GPS
  python start_navigation.py --target-lat 48.8970 --target-lon 2.2195

  # Navigation avec plusieurs waypoints
  python start_navigation.py --waypoints mission.txt

  # Mode simulation
  python start_navigation.py --simulation --env epitech

  # Avec RTK Polaris
  python start_navigation.py --target-lat 48.8970 --target-lon 2.2195 \\
      --polaris-key YOUR_API_KEY
        """
    )

    # Mode
    mode_group = parser.add_argument_group('Mode')
    mode_group.add_argument(
        '--simulation', '-s',
        action='store_true',
        help='Mode simulation (sans hardware)'
    )
    mode_group.add_argument(
        '--env',
        choices=['parking', 'corridor', 'epitech'],
        default='epitech',
        help='Environnement de simulation (defaut: epitech)'
    )
    mode_group.add_argument(
        '--no-gui',
        action='store_true',
        help='Mode sans interface graphique'
    )

    # Destination
    dest_group = parser.add_argument_group('Destination')
    dest_group.add_argument(
        '--target-lat',
        type=float,
        help='Latitude de destination'
    )
    dest_group.add_argument(
        '--target-lon',
        type=float,
        help='Longitude de destination'
    )
    dest_group.add_argument(
        '--waypoints', '-w',
        type=str,
        help='Fichier de waypoints (lat,lon par ligne)'
    )

    # Ports hardware
    hw_group = parser.add_argument_group('Hardware')
    hw_group.add_argument(
        '--lidar-port',
        default='/dev/ttyUSB0',
        help='Port LiDAR (defaut: /dev/ttyUSB0)'
    )
    hw_group.add_argument(
        '--gps-port',
        default='/dev/ttyUSB1',
        help='Port GPS (defaut: /dev/ttyUSB1)'
    )
    hw_group.add_argument(
        '--vesc-port',
        default='/dev/ttyACM0',
        help='Port VESC (defaut: /dev/ttyACM0)'
    )
    hw_group.add_argument(
        '--polaris-key',
        type=str,
        help='Cle API Polaris pour RTK'
    )

    # Options
    opt_group = parser.add_argument_group('Options')
    opt_group.add_argument(
        '--skip-check',
        action='store_true',
        help='Ignorer la verification des composants'
    )
    opt_group.add_argument(
        '--no-confirm',
        action='store_true',
        help='Ne pas demander de confirmation avant demarrage'
    )
    opt_group.add_argument(
        '--max-speed',
        type=float,
        default=0.5,
        help='Vitesse maximale (0.0-1.0, defaut: 0.5)'
    )
    opt_group.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Mode verbeux'
    )

    args = parser.parse_args()

    # Valider arguments
    if not args.simulation:
        if not args.waypoints and not (args.target_lat and args.target_lon):
            parser.error("En mode reel, specifiez --target-lat/--target-lon ou --waypoints")

    # Lancer
    launcher = NavigationLauncher(args)
    return launcher.run()


if __name__ == '__main__':
    sys.exit(main())
