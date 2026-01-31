#!/usr/bin/env python3
"""
Test Diagnostic Complet pour Robocar

Script de diagnostic pour vérifier que tous les composants
du système de navigation fonctionnent correctement.

Tests effectués:
1. Vérification des dépendances Python
2. Vérification des ports série disponibles
3. Test des imports des drivers
4. Test des capteurs (LiDAR, GPS, VESC) si connectés
5. Test de simulation LiDAR (sans matériel)

Usage:
    python scripts/test_diagnostic.py              # Mode diagnostic complet
    python scripts/test_diagnostic.py --quick      # Mode rapide (dépendances seulement)
    python scripts/test_diagnostic.py --hardware   # Test avec matériel réel
    python scripts/test_diagnostic.py --simulate   # Test simulation LiDAR
"""

import sys
import os
import time
import argparse
from typing import Dict, List, Tuple, Optional

# Ajouter src au path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Couleurs pour l'affichage terminal
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


def print_header(title: str):
    """Affiche un en-tête de section."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{title:^60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.RESET}\n")


def print_result(name: str, success: bool, message: str = ""):
    """Affiche le résultat d'un test."""
    if success:
        status = f"{Colors.GREEN}[OK]{Colors.RESET}"
    else:
        status = f"{Colors.RED}[FAIL]{Colors.RESET}"

    msg = f" - {message}" if message else ""
    print(f"  {status} {name}{msg}")


def print_warning(message: str):
    """Affiche un avertissement."""
    print(f"  {Colors.YELLOW}[WARN]{Colors.RESET} {message}")


def print_info(message: str):
    """Affiche une information."""
    print(f"  {Colors.CYAN}[INFO]{Colors.RESET} {message}")


class DiagnosticTester:
    """Classe principale de diagnostic."""

    def __init__(self):
        self.results: Dict[str, bool] = {}
        self.messages: Dict[str, str] = {}

    def test_python_dependencies(self) -> Dict[str, bool]:
        """Test des dépendances Python requises."""
        print_header("1. DEPENDANCES PYTHON")

        dependencies = {
            'numpy': 'numpy',
            'pyserial': 'serial',
            'pyvesc': 'pyvesc',
            'pyyaml': 'yaml',
            'matplotlib': 'matplotlib',
        }

        results = {}

        for name, module in dependencies.items():
            try:
                __import__(module)
                results[name] = True
                version = ""
                try:
                    mod = __import__(module)
                    version = getattr(mod, '__version__', '')
                    if version:
                        version = f"v{version}"
                except:
                    pass
                print_result(name, True, version)
            except ImportError as e:
                results[name] = False
                print_result(name, False, str(e))

        # Dépendances optionnelles
        print("\n  Dépendances optionnelles:")
        optional = {
            'breezyslam': 'breezyslam',
            'torch': 'torch',
            'cv2': 'cv2',
        }

        for name, module in optional.items():
            try:
                __import__(module)
                print_result(name, True, "(optionnel)")
            except ImportError:
                print_warning(f"{name} non installé (optionnel)")

        return results

    def test_serial_ports(self) -> List[str]:
        """Test des ports série disponibles."""
        print_header("2. PORTS SERIE DISPONIBLES")

        ports = []

        try:
            import serial.tools.list_ports
            available_ports = list(serial.tools.list_ports.comports())

            if available_ports:
                for port in available_ports:
                    ports.append(port.device)
                    desc = port.description or "Unknown"
                    print_result(port.device, True, desc)
            else:
                print_warning("Aucun port série détecté")
                print_info("Vérifiez que les appareils sont connectés")
        except ImportError:
            print_result("pyserial", False, "Module non installé")

        # Vérifier les ports typiques
        print("\n  Ports attendus:")
        expected_ports = {
            '/dev/ttyUSB0': 'LiDAR (typique)',
            '/dev/ttyUSB1': 'GPS (typique)',
            '/dev/ttyACM0': 'VESC (typique)',
        }

        for port, desc in expected_ports.items():
            exists = os.path.exists(port)
            if exists:
                print_result(port, True, desc)
            else:
                print_warning(f"{port} non trouvé ({desc})")

        return ports

    def test_driver_imports(self) -> Dict[str, bool]:
        """Test des imports des drivers."""
        print_header("3. IMPORTS DES DRIVERS")

        results = {}

        # Test LiDAR driver
        try:
            from driver.lidar import LidarDriver, LidarScan, LidarPoint
            results['lidar'] = True
            print_result("LidarDriver", True)
        except Exception as e:
            results['lidar'] = False
            print_result("LidarDriver", False, str(e))

        # Test GPS driver
        try:
            from driver.gps import GPSDriver, GPSPosition
            results['gps'] = True
            print_result("GPSDriver", True)
        except Exception as e:
            results['gps'] = False
            print_result("GPSDriver", False, str(e))

        # Test VESC driver
        try:
            from driver.vesc_motor import VESCController, VESCState
            results['vesc'] = True
            print_result("VESCController", True)
        except Exception as e:
            results['vesc'] = False
            print_result("VESCController", False, str(e))

        # Test autres modules
        print("\n  Modules additionnels:")

        modules = [
            ('navigation', 'Navigation'),
            ('perception', 'Perception'),
            ('slam', 'SLAM'),
        ]

        for module_name, display_name in modules:
            try:
                __import__(f'{module_name}')
                print_result(display_name, True)
            except ImportError:
                print_warning(f"{display_name} non disponible")
            except Exception as e:
                print_warning(f"{display_name}: {e}")

        return results

    def test_config_files(self) -> Dict[str, bool]:
        """Test des fichiers de configuration."""
        print_header("4. FICHIERS DE CONFIGURATION")

        results = {}
        config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')

        config_files = [
            'sensors.yaml',
            'robot.yaml',
        ]

        for filename in config_files:
            filepath = os.path.join(config_dir, filename)
            if os.path.exists(filepath):
                results[filename] = True
                try:
                    import yaml
                    with open(filepath, 'r') as f:
                        yaml.safe_load(f)
                    print_result(filename, True, "Valide")
                except Exception as e:
                    print_result(filename, False, f"Erreur parsing: {e}")
                    results[filename] = False
            else:
                results[filename] = False
                print_warning(f"{filename} non trouvé")

        return results

    def test_hardware_lidar(self, port: str = '/dev/ttyUSB0', timeout: float = 5.0) -> bool:
        """Test du LiDAR avec matériel réel."""
        print_header("5. TEST LIDAR (Hardware)")

        try:
            from driver.lidar import LidarDriver

            print_info(f"Connexion au LiDAR sur {port}...")
            lidar = LidarDriver(port)

            if lidar.start():
                print_result("Connexion", True)

                # Attendre un scan
                print_info("Attente d'un scan...")
                scan = lidar.get_scan(timeout=timeout)

                if scan:
                    valid_points = sum(1 for p in scan.points if p.valid)
                    distances = [p.distance for p in scan.points if p.valid]

                    print_result("Reception scan", True, f"{valid_points} points valides")

                    if distances:
                        min_dist = min(distances)
                        max_dist = max(distances)
                        avg_dist = sum(distances) / len(distances)
                        print_info(f"Distances: min={min_dist:.2f}m, max={max_dist:.2f}m, avg={avg_dist:.2f}m")

                    lidar.stop()
                    return True
                else:
                    print_result("Reception scan", False, "Timeout")
                    lidar.stop()
                    return False
            else:
                print_result("Connexion", False)
                return False

        except Exception as e:
            print_result("LiDAR", False, str(e))
            return False

    def test_hardware_gps(self, port: str = '/dev/ttyUSB1', timeout: float = 10.0) -> bool:
        """Test du GPS avec matériel réel."""
        print_header("6. TEST GPS (Hardware)")

        try:
            from driver.gps import GPSDriver

            print_info(f"Connexion au GPS sur {port}...")
            gps = GPSDriver(port)

            if gps.start():
                print_result("Connexion", True)

                # Attendre une position
                print_info(f"Attente d'une position (max {timeout}s)...")
                start_time = time.time()

                while time.time() - start_time < timeout:
                    pos = gps.get_position()
                    if pos:
                        print_result("Reception position", True)
                        print_info(f"Position: ({pos.latitude:.6f}, {pos.longitude:.6f})")
                        print_info(f"Qualité: {pos.quality_string}")
                        print_info(f"Précision H: {pos.accuracy_h:.2f}m")
                        print_info(f"Satellites: {pos.satellites}")

                        gps.stop()
                        return True
                    time.sleep(0.5)

                print_result("Reception position", False, "Timeout")
                gps.stop()
                return False
            else:
                print_result("Connexion", False)
                return False

        except Exception as e:
            print_result("GPS", False, str(e))
            return False

    def test_hardware_vesc(self, port: str = '/dev/ttyACM0') -> bool:
        """Test du VESC avec matériel réel."""
        print_header("7. TEST VESC (Hardware)")

        try:
            from driver.vesc_motor import VESCController, PYVESC_AVAILABLE

            if not PYVESC_AVAILABLE:
                print_result("pyvesc", False, "Module non disponible")
                return False

            print_info(f"Connexion au VESC sur {port}...")
            vesc = VESCController(port)

            if vesc.start():
                print_result("Connexion", True)

                # Lire l'état
                time.sleep(0.5)  # Attendre la première lecture
                state = vesc.get_state()

                if state:
                    print_result("Lecture état", True)
                    print_info(f"Tension batterie: {state.voltage:.1f}V")
                    print_info(f"Température MOSFET: {state.temp_mos:.1f}°C")
                    print_info(f"RPM: {state.rpm:.0f}")

                    # Vérifications de sécurité
                    if state.voltage < 10.0:
                        print_warning("Tension batterie faible!")
                    if state.temp_mos > 60.0:
                        print_warning("Température élevée!")
                else:
                    print_warning("Aucune donnée d'état reçue")

                # Test servo (sans moteur)
                print_info("Test servo direction...")
                vesc.set_servo(0.5)  # Centre
                time.sleep(0.2)
                print_result("Servo", True, "Position centre")

                vesc.stop()
                return True
            else:
                print_result("Connexion", False)
                return False

        except Exception as e:
            print_result("VESC", False, str(e))
            return False

    def test_simulation(self) -> bool:
        """Test du système de simulation LiDAR."""
        print_header("8. TEST SIMULATION")

        # Vérifier si le module de simulation existe
        lidar_test_path = os.path.join(os.path.dirname(__file__), '..', 'lidar_test')

        if not os.path.exists(lidar_test_path):
            print_warning("Dossier lidar_test non trouvé")
            return False

        try:
            sys.path.insert(0, lidar_test_path)
            sys.path.insert(0, os.path.join(lidar_test_path, 'src'))

            # Test import simulateur
            try:
                from lidar_simulator import LidarSimulator, create_room_with_furniture
                print_result("Import LidarSimulator", True)
            except ImportError:
                # Essayer un autre chemin
                try:
                    from src.lidar_simulator import LidarSimulator, create_room_with_furniture
                    print_result("Import LidarSimulator", True)
                except ImportError as e:
                    print_result("Import LidarSimulator", False, str(e))
                    return False

            # Créer un environnement de test
            print_info("Création environnement simulé...")
            env = create_room_with_furniture(6.0, 6.0)
            simulator = LidarSimulator(env)

            # Générer un scan
            scan = simulator.generate_scan(x=3.0, y=3.0, heading=0.0)

            if scan and len(scan.measurements) > 0:
                valid_count = sum(1 for m in scan.measurements if m.is_valid)
                print_result("Génération scan", True, f"{valid_count}/{len(scan.measurements)} points valides")

                # Statistiques
                distances = [m.distance for m in scan.measurements if m.is_valid]
                if distances:
                    print_info(f"Distance min: {min(distances):.2f}m")
                    print_info(f"Distance max: {max(distances):.2f}m")

                return True
            else:
                print_result("Génération scan", False, "Aucun point généré")
                return False

        except Exception as e:
            print_result("Simulation", False, str(e))
            return False

    def run_diagnostic(self, quick: bool = False, hardware: bool = False, simulate: bool = False,
                       lidar_port: str = '/dev/ttyUSB0', gps_port: str = '/dev/ttyUSB1',
                       vesc_port: str = '/dev/ttyACM0') -> Dict[str, bool]:
        """Exécute le diagnostic complet."""

        print(f"\n{Colors.BOLD}{Colors.CYAN}")
        print("=" * 60)
        print("   DIAGNOSTIC SYSTEME ROBOCAR")
        print("=" * 60)
        print(f"{Colors.RESET}")
        print(f"  Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  Python: {sys.version.split()[0]}")
        print(f"  Plateforme: {sys.platform}")

        all_results = {}

        # 1. Dépendances Python
        dep_results = self.test_python_dependencies()
        all_results['dependencies'] = all(dep_results.values())

        if quick:
            return all_results

        # 2. Ports série
        ports = self.test_serial_ports()
        all_results['serial_ports'] = len(ports) > 0

        # 3. Imports drivers
        import_results = self.test_driver_imports()
        all_results['driver_imports'] = all(import_results.values())

        # 4. Fichiers de configuration
        config_results = self.test_config_files()
        all_results['config_files'] = all(config_results.values())

        # 5. Tests hardware (si demandé)
        if hardware:
            all_results['lidar_hardware'] = self.test_hardware_lidar(lidar_port)
            all_results['gps_hardware'] = self.test_hardware_gps(gps_port)
            all_results['vesc_hardware'] = self.test_hardware_vesc(vesc_port)

        # 6. Test simulation (si demandé)
        if simulate:
            all_results['simulation'] = self.test_simulation()

        # Résumé final
        self.print_summary(all_results)

        return all_results

    def print_summary(self, results: Dict[str, bool]):
        """Affiche le résumé du diagnostic."""
        print_header("RESUME DU DIAGNOSTIC")

        passed = sum(1 for v in results.values() if v)
        total = len(results)

        for test_name, success in results.items():
            display_name = test_name.replace('_', ' ').title()
            print_result(display_name, success)

        print(f"\n{Colors.BOLD}Résultat: {passed}/{total} tests réussis{Colors.RESET}")

        if passed == total:
            print(f"\n{Colors.GREEN}{Colors.BOLD}Tous les tests ont réussi!{Colors.RESET}")
        else:
            print(f"\n{Colors.YELLOW}{Colors.BOLD}Certains tests ont échoué.{Colors.RESET}")
            print(f"{Colors.YELLOW}Vérifiez les erreurs ci-dessus.{Colors.RESET}")

        # Recommandations
        print(f"\n{Colors.CYAN}Prochaines étapes:{Colors.RESET}")
        if not results.get('dependencies', True):
            print(f"  - Installer les dépendances: pip install -r requirements.txt")
        if not results.get('serial_ports', True):
            print(f"  - Connecter les appareils USB (LiDAR, GPS, VESC)")
        if not results.get('driver_imports', True):
            print(f"  - Vérifier les fichiers dans src/driver/")
        if 'lidar_hardware' in results and not results['lidar_hardware']:
            print(f"  - Vérifier la connexion du LiDAR")
        if 'gps_hardware' in results and not results['gps_hardware']:
            print(f"  - Vérifier la connexion GPS et l'antenne")
        if 'vesc_hardware' in results and not results['vesc_hardware']:
            print(f"  - Vérifier la connexion USB du VESC")


def main():
    parser = argparse.ArgumentParser(
        description='Diagnostic complet du système Robocar',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  python test_diagnostic.py              # Diagnostic de base
  python test_diagnostic.py --quick      # Vérification rapide des dépendances
  python test_diagnostic.py --hardware   # Test avec matériel connecté
  python test_diagnostic.py --simulate   # Test du simulateur LiDAR
  python test_diagnostic.py --hardware --simulate  # Tests complets
        """
    )

    parser.add_argument('--quick', action='store_true',
                        help='Mode rapide (dépendances uniquement)')
    parser.add_argument('--hardware', action='store_true',
                        help='Tester le matériel connecté')
    parser.add_argument('--simulate', action='store_true',
                        help='Tester le simulateur LiDAR')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0',
                        help='Port du LiDAR (défaut: /dev/ttyUSB0)')
    parser.add_argument('--gps-port', default='/dev/ttyUSB1',
                        help='Port du GPS (défaut: /dev/ttyUSB1)')
    parser.add_argument('--vesc-port', default='/dev/ttyACM0',
                        help='Port du VESC (défaut: /dev/ttyACM0)')

    args = parser.parse_args()

    tester = DiagnosticTester()
    results = tester.run_diagnostic(
        quick=args.quick,
        hardware=args.hardware,
        simulate=args.simulate,
        lidar_port=args.lidar_port,
        gps_port=args.gps_port,
        vesc_port=args.vesc_port
    )

    # Code de retour
    if all(results.values()):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
