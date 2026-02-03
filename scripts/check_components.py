#!/usr/bin/env python3
"""
ROBOCAR - Verification rapide des composants
=============================================

Ce script verifie rapidement si tous les composants hardware
sont detectes et fonctionnels avant de lancer la navigation.

Tests effectues:
1. Detection des ports serie (LiDAR, GPS, VESC)
2. Test de connexion aux composants
3. Verification des dependances Python
4. Resume avec status OK/ERREUR

Usage:
    python scripts/check_components.py              # Check rapide
    python scripts/check_components.py --verbose    # Mode verbeux
    python scripts/check_components.py --test       # Test avec lecture de donnees
    python scripts/check_components.py --auto-detect # Auto-detection des ports

Codes de retour:
    0 = Tous les composants OK
    1 = Un ou plusieurs composants en erreur
"""

import sys
import os
import time
import argparse
from typing import Dict, List, Optional, Tuple

# Ajouter src au path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


class Colors:
    """Couleurs ANSI pour le terminal."""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

    @staticmethod
    def disable():
        Colors.GREEN = ''
        Colors.RED = ''
        Colors.YELLOW = ''
        Colors.BLUE = ''
        Colors.CYAN = ''
        Colors.BOLD = ''
        Colors.RESET = ''


def print_banner():
    """Affiche la banniere du script."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}")
    print("=" * 60)
    print("      ROBOCAR - VERIFICATION DES COMPOSANTS")
    print("=" * 60)
    print(f"{Colors.RESET}")


def ok(msg: str = ""):
    """Affiche un message OK."""
    suffix = f" - {msg}" if msg else ""
    print(f"  {Colors.GREEN}[OK]{Colors.RESET}{suffix}")


def fail(msg: str = ""):
    """Affiche un message ERREUR."""
    suffix = f" - {msg}" if msg else ""
    print(f"  {Colors.RED}[ERREUR]{Colors.RESET}{suffix}")


def warn(msg: str = ""):
    """Affiche un avertissement."""
    suffix = f" - {msg}" if msg else ""
    print(f"  {Colors.YELLOW}[WARN]{Colors.RESET}{suffix}")


def info(msg: str):
    """Affiche une information."""
    print(f"  {Colors.CYAN}[INFO]{Colors.RESET} {msg}")


def section(title: str):
    """Affiche un titre de section."""
    print(f"\n{Colors.BOLD}{Colors.CYAN}>> {title}{Colors.RESET}")


class ComponentChecker:
    """Verificateur de composants."""

    DEFAULT_PORTS = {
        'lidar': ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2'],
        'gps': ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2'],
        'vesc': ['/dev/ttyACM0', '/dev/ttyACM1'],
    }

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.results: Dict[str, bool] = {}
        self.detected_ports: Dict[str, str] = {}

    def check_dependencies(self) -> bool:
        """Verifie les dependances Python."""
        section("1. DEPENDANCES PYTHON")

        required = {
            'numpy': 'numpy',
            'pyserial': 'serial',
            'pyyaml': 'yaml',
        }

        all_ok = True
        for name, module in required.items():
            try:
                __import__(module)
                if self.verbose:
                    ok(name)
            except ImportError:
                fail(f"{name} non installe")
                all_ok = False

        if all_ok:
            ok("Toutes les dependances requises sont installees")

        # Optionnelles
        optional = {
            'pyvesc': 'pyvesc',
            'matplotlib': 'matplotlib',
            'breezyslam': 'breezyslam',
        }

        for name, module in optional.items():
            try:
                __import__(module)
                if self.verbose:
                    ok(f"{name} (optionnel)")
            except ImportError:
                if self.verbose:
                    warn(f"{name} non installe (optionnel)")

        self.results['dependencies'] = all_ok
        return all_ok

    def check_ports(self) -> Dict[str, List[str]]:
        """Liste tous les ports serie disponibles."""
        section("2. PORTS SERIE DISPONIBLES")

        available_ports: Dict[str, List[str]] = {
            'usb': [],
            'acm': [],
        }

        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())

            if not ports:
                warn("Aucun port serie detecte")
                info("Verifiez que les appareils sont connectes en USB")
                return available_ports

            for port in ports:
                desc = port.description or "Inconnu"
                hwid = port.hwid or ""

                if '/dev/ttyUSB' in port.device:
                    available_ports['usb'].append(port.device)
                elif '/dev/ttyACM' in port.device:
                    available_ports['acm'].append(port.device)

                ok(f"{port.device} ({desc})")

                if self.verbose and hwid:
                    info(f"  HWID: {hwid}")

        except ImportError:
            fail("pyserial non installe")

        return available_ports

    def detect_lidar(self, ports: List[str], test_read: bool = False) -> Tuple[bool, Optional[str]]:
        """Detecte le LiDAR LD19 sur les ports disponibles."""
        section("3. LIDAR LD19")

        if not ports:
            fail("Aucun port USB disponible pour le LiDAR")
            return False, None

        try:
            from driver.lidar import LidarDriver
        except ImportError as e:
            fail(f"Driver LiDAR non disponible: {e}")
            return False, None

        for port in ports:
            info(f"Test sur {port}...")
            try:
                lidar = LidarDriver(port)
                if lidar.start():
                    ok(f"LiDAR detecte sur {port}")

                    if test_read:
                        info("Lecture de donnees...")
                        scan = lidar.get_scan(timeout=3.0)
                        if scan:
                            valid = sum(1 for p in scan.points if p.valid)
                            ok(f"Scan recu: {valid} points valides")
                        else:
                            warn("Pas de scan recu (timeout)")

                    lidar.stop()
                    self.detected_ports['lidar'] = port
                    self.results['lidar'] = True
                    return True, port

                lidar.stop()

            except Exception as e:
                if self.verbose:
                    warn(f"{port}: {e}")

        fail("LiDAR non detecte")
        info("Verifiez:")
        info("  - Connexion USB")
        info("  - Alimentation du LiDAR")
        info("  - Permissions (sudo chmod 666 /dev/ttyUSB*)")
        self.results['lidar'] = False
        return False, None

    def detect_gps(self, ports: List[str], test_read: bool = False) -> Tuple[bool, Optional[str]]:
        """Detecte le GPS RTK sur les ports disponibles."""
        section("4. GPS RTK POINT ONE")

        if not ports:
            fail("Aucun port USB disponible pour le GPS")
            return False, None

        # Exclure le port du LiDAR si deja detecte
        lidar_port = self.detected_ports.get('lidar')
        available = [p for p in ports if p != lidar_port]

        if not available:
            fail("Aucun port disponible (tous utilises)")
            return False, None

        try:
            from driver.gps_rtk import GPSRTKDriver
        except ImportError:
            try:
                from driver.gps import GPSDriver as GPSRTKDriver
            except ImportError as e:
                fail(f"Driver GPS non disponible: {e}")
                return False, None

        for port in available:
            info(f"Test sur {port}...")
            try:
                gps = GPSRTKDriver(port)
                if gps.start():
                    ok(f"GPS detecte sur {port}")

                    if test_read:
                        info("Attente d'un fix GPS (max 10s)...")
                        start = time.time()
                        while time.time() - start < 10:
                            pos = gps.get_position()
                            if pos and pos.is_valid:
                                ok(f"Position: ({pos.latitude:.6f}, {pos.longitude:.6f})")
                                ok(f"Qualite: {pos.quality_string}, Satellites: {pos.satellites}")
                                break
                            time.sleep(0.5)
                        else:
                            warn("Pas de fix GPS (normal en interieur)")

                    gps.stop()
                    self.detected_ports['gps'] = port
                    self.results['gps'] = True
                    return True, port

            except Exception as e:
                if self.verbose:
                    warn(f"{port}: {e}")

        fail("GPS non detecte")
        info("Verifiez:")
        info("  - Connexion USB")
        info("  - Antenne GPS connectee")
        info("  - Le GPS est dehors ou pres d'une fenetre")
        self.results['gps'] = False
        return False, None

    def detect_vesc(self, ports: List[str], test_read: bool = False) -> Tuple[bool, Optional[str]]:
        """Detecte le VESC sur les ports disponibles."""
        section("5. MOTEUR VESC")

        if not ports:
            fail("Aucun port ACM disponible pour le VESC")
            return False, None

        try:
            from driver.vesc_motor import VESCController, PYVESC_AVAILABLE
            if not PYVESC_AVAILABLE:
                fail("pyvesc non installe")
                info("Installez avec: pip install pyvesc")
                return False, None
        except ImportError as e:
            fail(f"Driver VESC non disponible: {e}")
            return False, None

        for port in ports:
            info(f"Test sur {port}...")
            try:
                vesc = VESCController(port)
                if vesc.start():
                    ok(f"VESC detecte sur {port}")

                    if test_read:
                        info("Lecture de l'etat...")
                        time.sleep(0.5)
                        state = vesc.get_state()
                        if state:
                            ok(f"Tension: {state.voltage:.1f}V")
                            ok(f"Temperature: {state.temp_mos:.1f}C")
                            if state.voltage < 10.0:
                                warn("Tension batterie faible!")
                        else:
                            warn("Pas de donnees d'etat recues")

                    vesc.stop()
                    self.detected_ports['vesc'] = port
                    self.results['vesc'] = True
                    return True, port

            except Exception as e:
                if self.verbose:
                    warn(f"{port}: {e}")

        fail("VESC non detecte")
        info("Verifiez:")
        info("  - Connexion USB")
        info("  - Alimentation du VESC (batterie)")
        info("  - Permissions (sudo chmod 666 /dev/ttyACM*)")
        self.results['vesc'] = False
        return False, None

    def print_summary(self):
        """Affiche le resume des verifications."""
        section("RESUME")

        print(f"\n{Colors.BOLD}  Composant        Status         Port{Colors.RESET}")
        print("  " + "-" * 45)

        components = [
            ('Dependances', 'dependencies', None),
            ('LiDAR LD19', 'lidar', self.detected_ports.get('lidar', '-')),
            ('GPS RTK', 'gps', self.detected_ports.get('gps', '-')),
            ('Moteur VESC', 'vesc', self.detected_ports.get('vesc', '-')),
        ]

        all_ok = True
        for name, key, port in components:
            success = self.results.get(key, False)
            status = f"{Colors.GREEN}OK{Colors.RESET}" if success else f"{Colors.RED}ERREUR{Colors.RESET}"
            port_str = port if port else ""
            print(f"  {name:16} {status:20} {port_str}")
            if not success:
                all_ok = False

        print()

        if all_ok:
            print(f"{Colors.GREEN}{Colors.BOLD}")
            print("  TOUS LES COMPOSANTS SONT OPERATIONNELS!")
            print(f"{Colors.RESET}")
            print(f"  Lancez la navigation avec:")
            print(f"  {Colors.CYAN}python scripts/start_navigation.py \\")
            print(f"      --lidar-port {self.detected_ports.get('lidar', '/dev/ttyUSB0')} \\")
            print(f"      --gps-port {self.detected_ports.get('gps', '/dev/ttyUSB1')} \\")
            print(f"      --vesc-port {self.detected_ports.get('vesc', '/dev/ttyACM0')} \\")
            print(f"      --target-lat <latitude> --target-lon <longitude>{Colors.RESET}")
        else:
            print(f"{Colors.YELLOW}{Colors.BOLD}")
            print("  CERTAINS COMPOSANTS NE SONT PAS DETECTES")
            print(f"{Colors.RESET}")
            print("  Verifiez les connexions et relancez le script.")

        return all_ok

    def generate_config(self) -> str:
        """Genere une ligne de configuration pour start_navigation.py."""
        parts = []
        if self.detected_ports.get('lidar'):
            parts.append(f"--lidar-port {self.detected_ports['lidar']}")
        if self.detected_ports.get('gps'):
            parts.append(f"--gps-port {self.detected_ports['gps']}")
        if self.detected_ports.get('vesc'):
            parts.append(f"--vesc-port {self.detected_ports['vesc']}")
        return " ".join(parts)


def auto_detect_ports() -> Dict[str, str]:
    """Auto-detection intelligente des ports."""
    detected = {}

    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())

        for port in ports:
            desc = port.description.lower() if port.description else ""
            hwid = port.hwid.lower() if port.hwid else ""

            # Detection basee sur les identifiants connus
            if 'cp210' in desc or 'cp210' in hwid:
                # CP2102 souvent utilise pour LiDAR ou GPS
                if 'lidar' not in detected:
                    detected['lidar'] = port.device
                elif 'gps' not in detected:
                    detected['gps'] = port.device
            elif 'ch340' in desc or 'ch340' in hwid:
                # CH340 aussi utilise pour LiDAR ou GPS
                if 'lidar' not in detected:
                    detected['lidar'] = port.device
                elif 'gps' not in detected:
                    detected['gps'] = port.device
            elif 'stm' in desc or 'vesc' in desc:
                detected['vesc'] = port.device
            elif 'acm' in port.device.lower():
                if 'vesc' not in detected:
                    detected['vesc'] = port.device

    except ImportError:
        pass

    return detected


def main():
    parser = argparse.ArgumentParser(
        description='Verification rapide des composants Robocar',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  python check_components.py              # Verification rapide
  python check_components.py --verbose    # Mode detaille
  python check_components.py --test       # Avec test de lecture
  python check_components.py --no-color   # Sans couleurs
        """
    )

    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Mode verbeux avec plus de details')
    parser.add_argument('--test', '-t', action='store_true',
                        help='Tester la lecture de donnees sur chaque composant')
    parser.add_argument('--no-color', action='store_true',
                        help='Desactiver les couleurs')
    parser.add_argument('--auto-detect', action='store_true',
                        help='Afficher les ports auto-detectes')
    parser.add_argument('--lidar-port', type=str,
                        help='Port specifique pour le LiDAR')
    parser.add_argument('--gps-port', type=str,
                        help='Port specifique pour le GPS')
    parser.add_argument('--vesc-port', type=str,
                        help='Port specifique pour le VESC')

    args = parser.parse_args()

    if args.no_color:
        Colors.disable()

    print_banner()

    checker = ComponentChecker(verbose=args.verbose)

    # 1. Verifier dependances
    checker.check_dependencies()

    # 2. Lister ports disponibles
    available = checker.check_ports()
    usb_ports = available['usb']
    acm_ports = available['acm']

    # Utiliser les ports specifies ou les disponibles
    lidar_ports = [args.lidar_port] if args.lidar_port else usb_ports
    gps_ports = [args.gps_port] if args.gps_port else usb_ports
    vesc_ports = [args.vesc_port] if args.vesc_port else acm_ports

    # 3. Detecter LiDAR
    checker.detect_lidar(lidar_ports, test_read=args.test)

    # 4. Detecter GPS
    checker.detect_gps(gps_ports, test_read=args.test)

    # 5. Detecter VESC
    checker.detect_vesc(vesc_ports, test_read=args.test)

    # 6. Resume
    all_ok = checker.print_summary()

    # Auto-detect info
    if args.auto_detect:
        print(f"\n{Colors.CYAN}Configuration detectee:{Colors.RESET}")
        print(f"  {checker.generate_config()}")

    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
