#!/usr/bin/env python3
"""
ROBOCAR - Script de lancement de la navigation autonome
========================================================

Ce script lance le systeme de navigation autonome.
La voiture avance tout droit en utilisant:
- LiDAR LD19 pour la detection et l'evitement d'obstacles
- VESC pour le controle moteur

Usage:
    # Mode reel
    python scripts/start_navigation.py \\
        --lidar-port /dev/ttyUSB0 --vesc-port /dev/ttyACM0

    # Mode simulation (pour test sans hardware)
    python scripts/start_navigation.py --simulation --env epitech

Codes de retour:
    0 = Navigation terminee avec succes
    1 = Erreur (composants manquants, etc.)
    2 = Interruption utilisateur (Ctrl+C)
"""

import sys
import os
import time
import signal
import argparse

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
║              NAVIGATION AUTONOME LIDAR                    ║
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
        self.vesc_port = args.vesc_port

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

        vesc_ok, vesc_port = checker.detect_vesc(
            [self.vesc_port] if self.vesc_port else acm_ports
        )
        if vesc_ok and not self.vesc_port:
            self.vesc_port = vesc_port

        all_ok = lidar_ok and vesc_ok

        if not all_ok:
            log_error("Certains composants ne sont pas disponibles")
            return False

        log_ok("Tous les composants sont disponibles")
        return True

    def run_simulation(self) -> int:
        """Execute en mode simulation."""
        from navigation_controller import NavigationController
        from simulation import (
            create_parking_env, create_corridor_env, create_epitech_env
        )
        from interface import SimulatedLidarAdapter, SimulatedMotorAdapter

        log_info(f"Mode simulation - environnement: {self.args.env}")

        # Creer environnement
        if self.args.env == 'parking':
            env = create_parking_env()
            start_x, start_y = -7, 0
        elif self.args.env == 'corridor':
            env = create_corridor_env()
            start_x, start_y = -6, 0
        else:  # epitech
            env = create_epitech_env()
            start_x, start_y = 0, 0

        # Creer adaptateurs simules
        lidar = SimulatedLidarAdapter(env)
        motor = SimulatedMotorAdapter(wheelbase=0.26)

        # Creer controleur
        self.controller = NavigationController(lidar, motor, mode='simulation')
        self.controller.set_initial_pose(start_x, start_y, 0.0)

        log_nav(f"Depart: ({start_x}, {start_y})")

        # Lancer navigation
        if self.args.no_gui:
            self.controller.run_headless()
        else:
            self.controller.run_with_gui()

        return 0

    def run_slam(self) -> int:
        """Execute en mode SLAM avance."""
        from navigation_controller_slam import SLAMNavigationController
        from interface import RealLidarAdapter, RealMotorAdapter

        log_info("Mode SLAM avance - Initialisation...")
        log_info("  - Cartographie en temps reel")
        log_info("  - Planification globale A*")
        log_info("  - Evitement local DWA")

        try:
            log_info(f"LiDAR sur {self.lidar_port}...")
            lidar = RealLidarAdapter(self.lidar_port)

            log_info(f"VESC sur {self.vesc_port}...")
            motor = RealMotorAdapter(self.vesc_port)

        except Exception as e:
            log_error(f"Erreur initialisation: {e}")
            return 1

        # Creer controleur SLAM
        self.controller = SLAMNavigationController(lidar, None, motor)

        # Afficher resume mission
        print(f"\n{Colors.BOLD}=== MISSION SLAM ==={Colors.RESET}")
        print(f"Mode: Navigation SLAM LiDAR (avancer + evitement)")
        print()

        # Confirmation
        if not self.args.no_confirm:
            print(f"{Colors.YELLOW}La voiture va construire une carte et naviguer.{Colors.RESET}")
            print(f"{Colors.YELLOW}Assurez-vous que la zone est degagee!{Colors.RESET}")
            response = input(f"\nDemarrer? [o/N] ").strip().lower()
            if response not in ('o', 'oui', 'y', 'yes'):
                log_info("Navigation annulee")
                return 0

        # Lancer navigation SLAM
        log_nav("Demarrage de la navigation SLAM...")
        self.running = True

        try:
            if self.args.no_gui:
                self.controller.run()
            else:
                self.controller.run_with_visualization()
            return 0
        except KeyboardInterrupt:
            log_warn("Interruption utilisateur")
            return 2
        except Exception as e:
            log_error(f"Erreur navigation: {e}")
            import traceback
            traceback.print_exc()
            return 1
        finally:
            if self.controller:
                self.controller.shutdown()
                # Sauvegarder la carte
                try:
                    self.controller.save_map('/tmp/robocar_map.png')
                except:
                    pass

    def run_real(self) -> int:
        """Execute en mode reel sur la voiture."""
        from navigation_controller import NavigationController
        from interface import RealLidarAdapter, RealMotorAdapter

        log_info("Mode reel - Initialisation des capteurs...")

        # Creer adaptateurs reels
        try:
            log_info(f"LiDAR sur {self.lidar_port}...")
            lidar = RealLidarAdapter(self.lidar_port)

            log_info(f"VESC sur {self.vesc_port}...")
            motor = RealMotorAdapter(self.vesc_port)

        except Exception as e:
            log_error(f"Erreur initialisation: {e}")
            return 1

        # Creer controleur
        self.controller = NavigationController(lidar, motor, mode='car')

        # Afficher resume mission
        print(f"\n{Colors.BOLD}=== MISSION ==={Colors.RESET}")
        print(f"Mode: Avancer tout droit + evitement LiDAR")
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

        # 2. Lancer navigation (SLAM ou basique)
        if self.args.slam:
            return self.run_slam()
        else:
            return self.run_real()


def main():
    parser = argparse.ArgumentParser(
        description='Robocar - Navigation Autonome LiDAR',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Mode reel (avancer + evitement LiDAR)
  python start_navigation.py --lidar-port /dev/ttyUSB0 --vesc-port /dev/ttyACM0

  # Mode simulation
  python start_navigation.py --simulation --env epitech
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
        '--slam',
        action='store_true',
        help='Utiliser le controleur SLAM avance (cartographie + planification A* + DWA)'
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

    # Ports hardware
    hw_group = parser.add_argument_group('Hardware')
    hw_group.add_argument(
        '--lidar-port',
        default='/dev/ttyUSB0',
        help='Port LiDAR (defaut: /dev/ttyUSB0)'
    )
    hw_group.add_argument(
        '--vesc-port',
        default='/dev/ttyACM0',
        help='Port VESC (defaut: /dev/ttyACM0)'
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

    # Lancer
    launcher = NavigationLauncher(args)
    return launcher.run()


if __name__ == '__main__':
    sys.exit(main())
