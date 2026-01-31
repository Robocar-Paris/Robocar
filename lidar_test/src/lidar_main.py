#!/usr/bin/env python3
"""
Script Principal de Test LiDAR LD19
====================================
Script interactif pour tester le LiDAR LD19 avec:
- Connexion au vrai LiDAR via port série
- Mode simulation pour tester sans hardware
- Tests de perception d'obstacles
- Visualisation en temps réel
"""

import argparse
import sys
import time
import signal
from pathlib import Path

# Ajouter le chemin des modules
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("⚠️  pyserial non installé. Mode simulation uniquement.")

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.colors import LinearSegmentedColormap
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️  matplotlib non installé. Visualisation non disponible.")

from ld19_parser import LD19Parser, LidarScan, LidarPacket
from obstacle_detector import ObstacleDetector, AlertLevel, DetectionResult
from lidar_simulator import (
    LidarSimulator, 
    create_corridor_environment,
    create_parking_environment,
    create_room_with_furniture,
    create_obstacle_course,
    save_scan_to_file
)


class LidarTestSuite:
    """Suite de tests pour le LiDAR LD19."""
    
    def __init__(self, port: str = None, use_simulator: bool = False):
        """
        Initialise la suite de tests.
        
        Args:
            port: Port série du LiDAR (ex: /dev/ttyUSB0)
            use_simulator: Utiliser le simulateur au lieu du vrai LiDAR
        """
        self.port = port
        self.use_simulator = use_simulator or not SERIAL_AVAILABLE or port is None
        
        self.parser = LD19Parser()
        self.detector = ObstacleDetector()
        self.simulator = None
        self.serial_conn = None
        
        self._running = False
        self._scans_collected = []
        
        if self.use_simulator:
            print("🔧 Mode SIMULATION activé")
            self.simulator = LidarSimulator(create_room_with_furniture())
        else:
            print(f"🔌 Mode RÉEL - Port: {self.port}")
            
    def connect(self) -> bool:
        """Connecte au LiDAR réel."""
        if self.use_simulator:
            return True
            
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=230400,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            print(f"✅ Connecté au LiDAR sur {self.port}")
            return True
        except Exception as e:
            print(f"❌ Erreur de connexion: {e}")
            return False
    
    def disconnect(self):
        """Déconnecte du LiDAR."""
        if self.serial_conn:
            self.serial_conn.close()
            print("🔌 Déconnecté du LiDAR")
            
    def get_scan(self, timeout: float = 2.0) -> LidarScan:
        """
        Récupère un scan complet (360°).
        
        Args:
            timeout: Timeout en secondes
            
        Returns:
            LidarScan complet
        """
        if self.use_simulator:
            return self.simulator.generate_scan(480)
        
        scan = LidarScan()
        start_time = time.time()
        self.parser.reset()
        
        while time.time() - start_time < timeout:
            if self.serial_conn.in_waiting:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                packets = self.parser.add_data(data)
                
                for packet in packets:
                    scan.add_packet(packet)
                    
                if scan.point_count >= 400:  # Environ 360°
                    return scan
        
        return scan
    
    def test_connection(self) -> bool:
        """Test 1: Vérifie la connexion et la réception de données."""
        print("\n" + "="*60)
        print("TEST 1: Connexion et réception de données")
        print("="*60)
        
        if not self.connect():
            print("❌ ÉCHEC: Impossible de se connecter")
            return False
        
        scan = self.get_scan(timeout=3.0)
        
        if scan.point_count == 0:
            print("❌ ÉCHEC: Aucun point reçu")
            return False
        
        print(f"✅ SUCCÈS: {scan.point_count} points reçus")
        print(f"   Statistiques parser: {self.parser.stats}")
        return True
    
    def test_data_quality(self) -> bool:
        """Test 2: Vérifie la qualité des données."""
        print("\n" + "="*60)
        print("TEST 2: Qualité des données")
        print("="*60)
        
        scan = self.get_scan()
        
        if scan.point_count == 0:
            print("❌ ÉCHEC: Pas de données")
            return False
        
        angles, distances, intensities = scan.to_polar_arrays()
        
        # Vérifier la couverture angulaire
        angle_coverage = max(angles) - min(angles)
        print(f"   Couverture angulaire: {angle_coverage:.1f}°")
        
        # Vérifier les distances
        valid_distances = distances[distances > 0]
        if len(valid_distances) == 0:
            print("❌ ÉCHEC: Toutes les distances sont nulles")
            return False
        
        print(f"   Points valides: {len(valid_distances)}/{len(distances)}")
        print(f"   Distance min: {valid_distances.min()}mm")
        print(f"   Distance max: {valid_distances.max()}mm")
        print(f"   Distance moyenne: {valid_distances.mean():.1f}mm")
        
        # Vérifier les intensités
        valid_intensities = intensities[distances > 0]
        print(f"   Intensité moyenne: {valid_intensities.mean():.1f}")
        print(f"   Intensité min/max: {valid_intensities.min()}/{valid_intensities.max()}")
        
        # Critères de réussite
        success = (
            angle_coverage > 300 and  # Au moins 300° de couverture
            len(valid_distances) > 300 and  # Au moins 300 points valides
            valid_intensities.mean() > 20  # Intensité moyenne raisonnable
        )
        
        if success:
            print("✅ SUCCÈS: Qualité des données OK")
        else:
            print("⚠️  ATTENTION: Qualité des données dégradée")
        
        return success
    
    def test_obstacle_detection(self) -> bool:
        """Test 3: Vérifie la détection d'obstacles."""
        print("\n" + "="*60)
        print("TEST 3: Détection d'obstacles")
        print("="*60)
        
        scan = self.get_scan()
        
        if scan.point_count == 0:
            print("❌ ÉCHEC: Pas de données")
            return False
        
        # Effectuer la détection
        result = self.detector.detect(scan)
        
        print(f"   Temps de traitement: {result.processing_time_ms:.2f}ms")
        print(f"   Niveau d'alerte max: {result.max_alert_level.name}")
        print(f"   Obstacles détectés: {len(result.obstacles)}")
        
        print("\n   Statut des zones:")
        for zone_name, level in result.zones_status.items():
            status_icon = {
                AlertLevel.NONE: "🟢",
                AlertLevel.WARNING: "🟡",
                AlertLevel.DANGER: "🟠",
                AlertLevel.CRITICAL: "🔴"
            }[level]
            print(f"      {status_icon} {zone_name}: {level.name}")
        
        if result.obstacles:
            print("\n   Détails des obstacles:")
            for i, obs in enumerate(result.obstacles):
                print(f"      Obstacle {i+1}:")
                print(f"         Zone: {obs.zone.name}")
                print(f"         Distance: {obs.min_distance:.2f}m")
                print(f"         Angle: {obs.center_angle:.1f}°")
                print(f"         Points: {len(obs.points)}")
        
        print("✅ SUCCÈS: Détection d'obstacles fonctionnelle")
        return True
    
    def test_performance(self, duration: float = 5.0) -> bool:
        """Test 4: Mesure les performances (FPS, latence)."""
        print("\n" + "="*60)
        print(f"TEST 4: Performance ({duration}s)")
        print("="*60)
        
        scan_count = 0
        detection_times = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            scan = self.get_scan(timeout=0.5)
            
            if scan.point_count > 0:
                scan_count += 1
                
                # Mesurer le temps de détection
                t0 = time.time()
                result = self.detector.detect(scan)
                detection_times.append((time.time() - t0) * 1000)
        
        elapsed = time.time() - start_time
        fps = scan_count / elapsed
        
        print(f"   Durée totale: {elapsed:.1f}s")
        print(f"   Scans collectés: {scan_count}")
        print(f"   Fréquence: {fps:.1f} Hz")
        
        if detection_times:
            print(f"   Latence détection:")
            print(f"      Min: {min(detection_times):.2f}ms")
            print(f"      Max: {max(detection_times):.2f}ms")
            print(f"      Moyenne: {np.mean(detection_times):.2f}ms")
        
        # Critères: au moins 5 Hz pour la navigation
        if fps >= 5:
            print("✅ SUCCÈS: Performance suffisante pour la navigation")
            return True
        else:
            print("⚠️  ATTENTION: Performance insuffisante")
            return False
    
    def test_zones_coverage(self) -> bool:
        """Test 5: Vérifie la couverture de toutes les zones."""
        print("\n" + "="*60)
        print("TEST 5: Couverture des zones de sécurité")
        print("="*60)
        
        # Collecter plusieurs scans
        scans = []
        for i in range(5):
            scan = self.get_scan()
            if scan.point_count > 0:
                scans.append(scan)
            time.sleep(0.1)
        
        if not scans:
            print("❌ ÉCHEC: Pas de scans collectés")
            return False
        
        # Vérifier chaque zone
        zones_with_points = set()
        
        for scan in scans:
            for zone in self.detector.zones:
                points = scan.get_points_in_sector(
                    zone.angle_start, zone.angle_end, 
                    zone.distance_warning
                )
                if points:
                    zones_with_points.add(zone.name)
        
        print(f"   Zones avec points détectés: {len(zones_with_points)}/{len(self.detector.zones)}")
        
        for zone in self.detector.zones:
            status = "✅" if zone.name in zones_with_points else "❌"
            print(f"      {status} {zone.name}")
        
        # Au moins 50% des zones doivent avoir des points
        coverage_ratio = len(zones_with_points) / len(self.detector.zones)
        if coverage_ratio >= 0.5:
            print(f"✅ SUCCÈS: Couverture à {coverage_ratio*100:.0f}%")
            return True
        else:
            print(f"⚠️  ATTENTION: Couverture insuffisante ({coverage_ratio*100:.0f}%)")
            return False
    
    def run_all_tests(self) -> dict:
        """Exécute tous les tests et retourne un rapport."""
        print("\n" + "="*60)
        print("       SUITE DE TESTS LIDAR LD19")
        print("="*60)
        print(f"Mode: {'SIMULATION' if self.use_simulator else 'RÉEL'}")
        print(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        results = {}
        
        tests = [
            ("Connexion", self.test_connection),
            ("Qualité données", self.test_data_quality),
            ("Détection obstacles", self.test_obstacle_detection),
            ("Performance", self.test_performance),
            ("Couverture zones", self.test_zones_coverage),
        ]
        
        for name, test_func in tests:
            try:
                results[name] = test_func()
            except Exception as e:
                print(f"❌ ERREUR dans {name}: {e}")
                results[name] = False
        
        # Résumé
        print("\n" + "="*60)
        print("                    RÉSUMÉ")
        print("="*60)
        
        passed = sum(results.values())
        total = len(results)
        
        for name, success in results.items():
            status = "✅ PASS" if success else "❌ FAIL"
            print(f"   {status}: {name}")
        
        print("-"*60)
        print(f"   Total: {passed}/{total} tests réussis")
        
        if passed == total:
            print("\n🎉 TOUS LES TESTS RÉUSSIS!")
        else:
            print(f"\n⚠️  {total - passed} test(s) échoué(s)")
        
        self.disconnect()
        return results
    
    def visualize_scan(self, scan: LidarScan, result: DetectionResult = None,
                        save_path: str = None, show: bool = True):
        """
        Visualise un scan LiDAR avec les obstacles détectés.

        Args:
            scan: Scan LiDAR à visualiser
            result: Résultat de détection d'obstacles (optionnel)
            save_path: Chemin pour sauvegarder l'image (optionnel)
            show: Afficher l'image à l'écran
        """
        if not MATPLOTLIB_AVAILABLE:
            print("❌ matplotlib non installé. Installez avec: pip install matplotlib")
            return

        # Créer la figure avec 2 sous-graphiques
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('LiDAR LD19 - Visualisation du Scan', fontsize=14, fontweight='bold')

        # Récupérer les données
        x, y, intensities = scan.to_cartesian_arrays()
        angles, distances, _ = scan.to_polar_arrays()

        # Filtrer les points valides
        valid_mask = (distances > 10) & (distances < 12000)  # En mm

        # === GRAPHIQUE 1: Vue de dessus (Cartésien) ===
        ax1.set_title('Vue de dessus', fontsize=12)
        ax1.set_xlabel('X (mètres)')
        ax1.set_ylabel('Y (mètres)')
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)

        # Définir les limites
        max_range = 5.0  # mètres
        ax1.set_xlim(-max_range, max_range)
        ax1.set_ylim(-max_range, max_range)

        # Dessiner les zones de sécurité
        zone_colors = {
            'front': '#90EE90',      # Vert clair
            'front_left': '#98FB98',
            'front_right': '#98FB98',
            'left': '#FFFFE0',       # Jaune clair
            'right': '#FFFFE0',
            'rear': '#FFB6C1',       # Rose clair
        }

        for zone in self.detector.zones:
            if zone.enabled:
                # Dessiner le secteur de la zone
                theta1 = np.radians(zone.angle_start)
                theta2 = np.radians(zone.angle_end)

                # Ajuster pour le wrap-around
                if zone.angle_start > zone.angle_end:
                    angles_zone = np.linspace(theta1, theta2 + 2*np.pi, 30)
                else:
                    angles_zone = np.linspace(theta1, theta2, 30)

                # Zone warning (externe)
                r_warn = zone.distance_warning
                wedge = patches.Wedge(
                    (0, 0), r_warn, zone.angle_start - 90, zone.angle_end - 90,
                    alpha=0.15, color=zone_colors.get(zone.name, '#DDDDDD'),
                    label=f'{zone.name}' if zone.name == 'front' else None
                )
                ax1.add_patch(wedge)

        # Dessiner le robot au centre
        robot = plt.Circle((0, 0), 0.15, color='blue', label='Robot')
        ax1.add_patch(robot)

        # Direction avant
        ax1.arrow(0, 0, 0, 0.4, head_width=0.1, head_length=0.05, fc='blue', ec='blue')

        # Dessiner les points du scan
        valid_x = x[valid_mask]
        valid_y = y[valid_mask]
        valid_int = intensities[valid_mask]

        # Colormap basée sur l'intensité (points agrandis pour meilleure visibilité)
        scatter = ax1.scatter(valid_x, valid_y, c=valid_int, cmap='viridis',
                             s=20, alpha=0.8, label='Points LiDAR',
                             edgecolors='black', linewidths=0.3)

        # Colorbar pour l'intensité
        cbar = plt.colorbar(scatter, ax=ax1, shrink=0.8)
        cbar.set_label('Intensité')

        # Dessiner les obstacles détectés
        if result and result.obstacles:
            for i, obs in enumerate(result.obstacles):
                cx, cy = obs.center_position

                # Couleur selon le niveau d'alerte
                alert_colors = {
                    AlertLevel.WARNING: 'yellow',
                    AlertLevel.DANGER: 'orange',
                    AlertLevel.CRITICAL: 'red'
                }
                color = alert_colors.get(obs.alert_level, 'gray')

                # Cercle autour de l'obstacle
                circle = plt.Circle((cx, cy), obs.width/2 + 0.1,
                                    fill=False, color=color, linewidth=2)
                ax1.add_patch(circle)

                # Étiquette
                ax1.annotate(f'{obs.zone.name}\n{obs.min_distance:.1f}m',
                            (cx, cy), fontsize=8, ha='center',
                            bbox=dict(boxstyle='round', facecolor=color, alpha=0.5))

        ax1.legend(loc='upper right', fontsize=8)

        # === GRAPHIQUE 2: Vue polaire ===
        ax2 = plt.subplot(122, projection='polar')
        ax2.set_title('Vue polaire', fontsize=12)

        # Convertir les angles en radians (0° = devant)
        angles_rad = np.radians(angles[valid_mask])
        distances_m = distances[valid_mask] / 1000.0  # Convertir en mètres

        # Scatter plot polaire (points agrandis pour meilleure visibilité)
        scatter2 = ax2.scatter(angles_rad, distances_m, c=intensities[valid_mask],
                              cmap='viridis', s=20, alpha=0.8,
                              edgecolors='black', linewidths=0.3)

        # Configuration de la vue polaire
        ax2.set_ylim(0, max_range)
        ax2.set_theta_zero_location('N')  # 0° en haut
        ax2.set_theta_direction(-1)  # Sens horaire

        # Ajouter les informations de détection
        if result:
            info_text = f"Alerte: {result.max_alert_level.name}\n"
            info_text += f"Obstacles: {len(result.obstacles)}\n"
            info_text += f"Temps: {result.processing_time_ms:.1f}ms"

            # Couleur selon l'alerte
            alert_bg = {
                AlertLevel.NONE: 'lightgreen',
                AlertLevel.WARNING: 'yellow',
                AlertLevel.DANGER: 'orange',
                AlertLevel.CRITICAL: 'red'
            }

            fig.text(0.02, 0.02, info_text, fontsize=10,
                    bbox=dict(boxstyle='round', facecolor=alert_bg.get(result.max_alert_level, 'white'),
                             alpha=0.8))

        # Info sur le scan
        scan_info = f"Points: {scan.point_count}\n"
        if scan.min_distance:
            scan_info += f"Dist. min: {scan.min_distance:.2f}m"
        fig.text(0.98, 0.02, scan_info, fontsize=10, ha='right',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

        plt.tight_layout()

        # Sauvegarder si demandé
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✅ Image sauvegardée: {save_path}")

        # Afficher
        if show:
            plt.show()
        else:
            plt.close()

    def interactive_mode(self):
        """Mode interactif pour tester manuellement."""
        print("\n" + "="*60)
        print("       MODE INTERACTIF LIDAR LD19")
        print("="*60)
        print("Commandes disponibles:")
        print("  s - Scanner et afficher (texte)")
        print("  v - Visualiser le scan (graphique)")
        print("  d - Détecter les obstacles")
        print("  c - Changer d'environnement (simulation)")
        print("  r - Enregistrer un scan")
        print("  q - Quitter")
        print("-"*60)
        
        if not self.connect():
            return
        
        environments = {
            '1': ("Pièce meublée", create_room_with_furniture()),
            '2': ("Couloir", create_corridor_environment()),
            '3': ("Parking", create_parking_environment()),
            '4': ("Parcours obstacles", create_obstacle_course()),
        }
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'q':
                    break
                    
                elif cmd == 's':
                    print("Scanning...")
                    scan = self.get_scan()
                    print(f"Points: {scan.point_count}")
                    if scan.min_distance:
                        print(f"Distance min: {scan.min_distance:.2f}m")
                    x, y, _ = scan.to_cartesian_arrays()
                    valid = np.sqrt(x**2 + y**2) > 0.01
                    print(f"Points valides: {np.sum(valid)}")

                elif cmd == 'v':
                    print("Scanning et visualisation...")
                    scan = self.get_scan()
                    result = self.detector.detect(scan)
                    print(f"Points: {scan.point_count}, Alerte: {result.max_alert_level.name}")
                    self.visualize_scan(scan, result)

                elif cmd == 'd':
                    print("Détection en cours...")
                    scan = self.get_scan()
                    result = self.detector.detect(scan)
                    print(f"Alerte: {result.max_alert_level.name}")
                    print(f"Obstacles: {len(result.obstacles)}")
                    for zone, level in result.zones_status.items():
                        if level != AlertLevel.NONE:
                            print(f"  {zone}: {level.name}")
                    
                elif cmd == 'c' and self.use_simulator:
                    print("Environnements disponibles:")
                    for key, (name, _) in environments.items():
                        print(f"  {key} - {name}")
                    choice = input("Choix: ").strip()
                    if choice in environments:
                        name, env = environments[choice]
                        self.simulator = LidarSimulator(env)
                        print(f"✅ Environnement changé: {name}")
                    
                elif cmd == 'r':
                    scan = self.get_scan()
                    result = self.detector.detect(scan)

                    # Créer le dossier data s'il n'existe pas
                    data_dir = Path(__file__).parent.parent / 'data'
                    data_dir.mkdir(exist_ok=True)

                    timestamp = int(time.time())

                    # Sauvegarder le JSON
                    json_path = data_dir / f"scan_{timestamp}.json"
                    save_scan_to_file(scan, str(json_path))
                    print(f"✅ Scan JSON: {json_path}")

                    # Sauvegarder l'image
                    img_path = data_dir / f"scan_{timestamp}.png"
                    self.visualize_scan(scan, result, save_path=str(img_path), show=False)
                    print(f"✅ Image PNG: {img_path}")
                    
                else:
                    print("Commande inconnue")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Erreur: {e}")
        
        self.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description="Suite de tests pour LiDAR LD19",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Test avec simulateur
  python lidar_test_main.py --simulate
  
  # Test avec vrai LiDAR
  python lidar_test_main.py --port /dev/ttyUSB0
  
  # Mode interactif
  python lidar_test_main.py --simulate --interactive
        """
    )
    
    parser.add_argument('--port', '-p', type=str, default=None,
                       help='Port série du LiDAR (ex: /dev/ttyUSB0)')
    parser.add_argument('--simulate', '-s', action='store_true',
                       help='Utiliser le simulateur')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Mode interactif')
    parser.add_argument('--test', '-t', type=str, default='all',
                       choices=['all', 'connection', 'quality', 'obstacles', 
                               'performance', 'coverage'],
                       help='Test spécifique à exécuter')
    
    args = parser.parse_args()
    
    # Créer la suite de tests
    suite = LidarTestSuite(port=args.port, use_simulator=args.simulate)
    
    # Gérer Ctrl+C
    def signal_handler(sig, frame):
        print("\n\nInterruption...")
        suite.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if args.interactive:
        suite.interactive_mode()
    elif args.test == 'all':
        suite.run_all_tests()
    else:
        suite.connect()
        test_map = {
            'connection': suite.test_connection,
            'quality': suite.test_data_quality,
            'obstacles': suite.test_obstacle_detection,
            'performance': suite.test_performance,
            'coverage': suite.test_zones_coverage,
        }
        test_map[args.test]()
        suite.disconnect()


if __name__ == "__main__":
    main()
    