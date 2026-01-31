#!/usr/bin/env python3
"""
Tests Unitaires pour le LiDAR LD19
===================================
Tests automatisés pour valider:
- Parser de données
- Détection d'obstacles
- Simulateur
- Visualisation
"""

import unittest
import sys
import os
import tempfile
import json
import numpy as np
from pathlib import Path

# Ajouter le chemin des modules
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from ld19_parser import (
    LD19Parser, LidarPacket, LidarMeasurement, LidarScan,
    CRC8, create_test_packet, HEADER_BYTE, LENGTH_BYTE,
    PACKET_LENGTH, MEASUREMENTS_PER_PACKET
)
from obstacle_detector import (
    ObstacleDetector, SafetyZone, AlertLevel, DetectionResult, Obstacle
)
from lidar_simulator import (
    LidarSimulator, SimulatedEnvironment, Wall, CircleObstacle,
    create_corridor_environment, create_room_with_furniture,
    save_scan_to_file, load_scan_from_file
)


class TestCRC8(unittest.TestCase):
    """Tests pour le calcul CRC8."""
    
    def test_crc_initialization(self):
        """Test initialisation CRC."""
        crc = CRC8()
        self.assertIsNotNone(crc._table)
        self.assertEqual(len(crc._table), 256)
    
    def test_crc_empty_data(self):
        """Test CRC sur données vides."""
        crc = CRC8()
        result = crc.calculate(b'')
        self.assertEqual(result, 0x00)
    
    def test_crc_known_value(self):
        """Test CRC sur valeur connue."""
        crc = CRC8()
        # Le CRC doit être déterministe
        result1 = crc.calculate(b'\x54\x2C\x00\x00')
        result2 = crc.calculate(b'\x54\x2C\x00\x00')
        self.assertEqual(result1, result2)
    
    def test_crc_different_data(self):
        """Test que différentes données donnent différents CRC."""
        crc = CRC8()
        result1 = crc.calculate(b'\x00\x00\x00\x00')
        result2 = crc.calculate(b'\xFF\xFF\xFF\xFF')
        self.assertNotEqual(result1, result2)


class TestLidarMeasurement(unittest.TestCase):
    """Tests pour LidarMeasurement."""
    
    def test_measurement_creation(self):
        """Test création d'une mesure."""
        m = LidarMeasurement(angle_deg=45.0, distance_mm=1000, intensity=200)
        self.assertEqual(m.angle_deg, 45.0)
        self.assertEqual(m.distance_mm, 1000)
        self.assertEqual(m.intensity, 200)
    
    def test_distance_conversion(self):
        """Test conversion mm -> m."""
        m = LidarMeasurement(angle_deg=0, distance_mm=1500, intensity=100)
        self.assertEqual(m.distance_m, 1.5)
    
    def test_angle_conversion(self):
        """Test conversion deg -> rad."""
        m = LidarMeasurement(angle_deg=180, distance_mm=1000, intensity=100)
        self.assertAlmostEqual(m.angle_rad, np.pi, places=5)
    
    def test_cartesian_conversion_0deg(self):
        """Test conversion cartésienne à 0°."""
        m = LidarMeasurement(angle_deg=0, distance_mm=1000, intensity=100)
        x, y = m.to_cartesian()
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 1.0, places=3)
    
    def test_cartesian_conversion_90deg(self):
        """Test conversion cartésienne à 90°."""
        m = LidarMeasurement(angle_deg=90, distance_mm=1000, intensity=100)
        x, y = m.to_cartesian()
        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
    
    def test_is_valid(self):
        """Test validation des mesures."""
        valid = LidarMeasurement(angle_deg=0, distance_mm=1000, intensity=100)
        self.assertTrue(valid.is_valid())
        
        invalid_distance = LidarMeasurement(angle_deg=0, distance_mm=0, intensity=100)
        self.assertFalse(invalid_distance.is_valid())
        
        too_far = LidarMeasurement(angle_deg=0, distance_mm=15000, intensity=100)
        self.assertFalse(too_far.is_valid(max_distance_mm=12000))


class TestLD19Parser(unittest.TestCase):
    """Tests pour le parser LD19."""
    
    def setUp(self):
        self.parser = LD19Parser()
    
    def test_parser_initialization(self):
        """Test initialisation du parser."""
        self.assertIsNotNone(self.parser.crc_calculator)
        self.assertEqual(len(self.parser._buffer), 0)
    
    def test_parse_valid_packet(self):
        """Test parsing d'un paquet valide."""
        test_data = create_test_packet()
        packets = self.parser.add_data(test_data)
        
        self.assertEqual(len(packets), 1)
        packet = packets[0]
        self.assertEqual(len(packet.measurements), MEASUREMENTS_PER_PACKET)
        self.assertTrue(packet.crc_valid)
    
    def test_parse_invalid_header(self):
        """Test avec header invalide."""
        invalid_data = b'\x00' + create_test_packet()[1:]
        packets = self.parser.add_data(invalid_data)
        self.assertEqual(len(packets), 0)
    
    def test_parse_incomplete_data(self):
        """Test avec données incomplètes."""
        incomplete_data = create_test_packet()[:20]
        packets = self.parser.add_data(incomplete_data)
        self.assertEqual(len(packets), 0)
    
    def test_parse_multiple_packets(self):
        """Test parsing de plusieurs paquets."""
        test_data = create_test_packet() * 3
        packets = self.parser.add_data(test_data)
        self.assertEqual(len(packets), 3)
    
    def test_parser_reset(self):
        """Test reset du parser."""
        self.parser.add_data(create_test_packet()[:20])  # Données partielles
        self.parser.reset()
        self.assertEqual(len(self.parser._buffer), 0)
    
    def test_parser_stats(self):
        """Test statistiques du parser."""
        self.parser.add_data(create_test_packet())
        stats = self.parser.stats
        self.assertIn('packets_parsed', stats)
        self.assertIn('packets_invalid', stats)
        self.assertEqual(stats['packets_parsed'], 1)


class TestLidarScan(unittest.TestCase):
    """Tests pour LidarScan."""
    
    def setUp(self):
        self.measurements = [
            LidarMeasurement(angle_deg=i * 10, distance_mm=1000 + i * 10, intensity=150)
            for i in range(36)
        ]
        self.scan = LidarScan(self.measurements)
    
    def test_scan_point_count(self):
        """Test nombre de points."""
        self.assertEqual(self.scan.point_count, 36)
    
    def test_scan_min_distance(self):
        """Test distance minimale."""
        self.assertEqual(self.scan.min_distance, 1.0)  # 1000mm = 1m
    
    def test_scan_to_polar(self):
        """Test conversion polaire."""
        angles, distances, intensities = self.scan.to_polar_arrays()
        self.assertEqual(len(angles), 36)
        self.assertEqual(len(distances), 36)
        self.assertEqual(len(intensities), 36)
    
    def test_scan_to_cartesian(self):
        """Test conversion cartésienne."""
        x, y, intensities = self.scan.to_cartesian_arrays()
        self.assertEqual(len(x), 36)
        self.assertEqual(len(y), 36)
    
    def test_get_points_in_sector(self):
        """Test filtrage par secteur."""
        points = self.scan.get_points_in_sector(0, 90)
        self.assertTrue(all(0 <= p.angle_deg <= 90 for p in points))
    
    def test_get_valid_measurements(self):
        """Test filtrage des mesures valides."""
        # Ajouter une mesure invalide
        self.measurements.append(
            LidarMeasurement(angle_deg=0, distance_mm=0, intensity=0)
        )
        scan = LidarScan(self.measurements)
        valid = scan.get_valid_measurements()
        self.assertEqual(len(valid), 36)


class TestSafetyZone(unittest.TestCase):
    """Tests pour SafetyZone."""
    
    def test_zone_contains_angle(self):
        """Test si un angle est dans la zone."""
        zone = SafetyZone(
            name="front",
            angle_start=315,
            angle_end=45,
            distance_warning=2.0,
            distance_danger=1.0,
            distance_critical=0.3
        )
        
        self.assertTrue(zone.contains_angle(0))
        self.assertTrue(zone.contains_angle(350))
        self.assertTrue(zone.contains_angle(30))
        self.assertFalse(zone.contains_angle(180))
    
    def test_zone_alert_levels(self):
        """Test niveaux d'alerte."""
        zone = SafetyZone(
            name="test",
            angle_start=0,
            angle_end=90,
            distance_warning=2.0,
            distance_danger=1.0,
            distance_critical=0.3
        )
        
        self.assertEqual(zone.get_alert_level(3.0), AlertLevel.NONE)
        self.assertEqual(zone.get_alert_level(1.5), AlertLevel.WARNING)
        self.assertEqual(zone.get_alert_level(0.5), AlertLevel.DANGER)
        self.assertEqual(zone.get_alert_level(0.2), AlertLevel.CRITICAL)


class TestObstacleDetector(unittest.TestCase):
    """Tests pour ObstacleDetector."""
    
    def setUp(self):
        self.detector = ObstacleDetector()
    
    def test_detector_initialization(self):
        """Test initialisation du détecteur."""
        self.assertTrue(len(self.detector.zones) > 0)
    
    def test_detect_no_obstacles(self):
        """Test détection sans obstacles."""
        # Tous les points à grande distance
        measurements = [
            LidarMeasurement(angle_deg=i * 10, distance_mm=10000, intensity=150)
            for i in range(36)
        ]
        scan = LidarScan(measurements)
        result = self.detector.detect(scan)
        
        self.assertEqual(result.max_alert_level, AlertLevel.NONE)
        self.assertEqual(len(result.obstacles), 0)
    
    def test_detect_front_obstacle(self):
        """Test détection obstacle devant."""
        measurements = []
        for i in range(36):
            angle = i * 10
            # Obstacle proche devant (0° ± 20°)
            if angle <= 20 or angle >= 340:
                distance = 250  # 25cm - critique
            else:
                distance = 5000
            measurements.append(
                LidarMeasurement(angle_deg=angle, distance_mm=distance, intensity=200)
            )
        
        scan = LidarScan(measurements)
        result = self.detector.detect(scan)
        
        self.assertEqual(result.max_alert_level, AlertLevel.CRITICAL)
        self.assertTrue(len(result.obstacles) > 0)
    
    def test_path_clear(self):
        """Test vérification chemin libre."""
        # Pas d'obstacles
        measurements = [
            LidarMeasurement(angle_deg=i * 10, distance_mm=5000, intensity=150)
            for i in range(36)
        ]
        scan = LidarScan(measurements)
        result = self.detector.detect(scan)
        
        self.assertTrue(self.detector.get_path_clear(result))
    
    def test_alert_callback(self):
        """Test callback d'alerte."""
        callback_called = [False]
        
        def test_callback(result):
            callback_called[0] = True
        
        self.detector.register_alert_callback(test_callback)
        
        measurements = [
            LidarMeasurement(angle_deg=i * 10, distance_mm=5000, intensity=150)
            for i in range(36)
        ]
        scan = LidarScan(measurements)
        self.detector.detect(scan)
        
        self.assertTrue(callback_called[0])


class TestLidarSimulator(unittest.TestCase):
    """Tests pour le simulateur."""
    
    def setUp(self):
        self.env = create_room_with_furniture()
        self.simulator = LidarSimulator(self.env)
    
    def test_simulator_initialization(self):
        """Test initialisation du simulateur."""
        self.assertIsNotNone(self.simulator.environment)
        self.assertTrue(len(self.simulator.environment.walls) > 0)
    
    def test_generate_scan(self):
        """Test génération de scan."""
        scan = self.simulator.generate_scan(480)
        self.assertEqual(scan.point_count, 480)
    
    def test_scan_has_valid_points(self):
        """Test que le scan a des points valides."""
        scan = self.simulator.generate_scan(480)
        valid = scan.get_valid_measurements()
        self.assertTrue(len(valid) > 100)
    
    def test_generate_packet(self):
        """Test génération de paquet."""
        packet = self.simulator.generate_packet()
        self.assertEqual(len(packet.measurements), MEASUREMENTS_PER_PACKET)
        self.assertTrue(packet.crc_valid)
    
    def test_generate_raw_packet(self):
        """Test génération de données brutes."""
        raw_data = self.simulator.generate_raw_packet()
        self.assertEqual(len(raw_data), PACKET_LENGTH)
        self.assertEqual(raw_data[0], HEADER_BYTE)
        self.assertEqual(raw_data[1], LENGTH_BYTE)
    
    def test_raw_packet_parseable(self):
        """Test que le paquet brut peut être parsé."""
        raw_data = self.simulator.generate_raw_packet()
        parser = LD19Parser()
        packets = parser.add_data(raw_data)
        self.assertEqual(len(packets), 1)


class TestScanPersistence(unittest.TestCase):
    """Tests pour la sauvegarde/chargement des scans."""
    
    def test_save_and_load_scan(self):
        """Test sauvegarde et chargement."""
        measurements = [
            LidarMeasurement(angle_deg=i * 10, distance_mm=1000 + i, intensity=150)
            for i in range(36)
        ]
        original_scan = LidarScan(measurements)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            filepath = f.name
        
        try:
            save_scan_to_file(original_scan, filepath)
            loaded_scan = load_scan_from_file(filepath)
            
            self.assertEqual(original_scan.point_count, loaded_scan.point_count)
            
            # Vérifier quelques mesures
            for i in range(len(original_scan.measurements)):
                orig = original_scan.measurements[i]
                loaded = loaded_scan.measurements[i]
                self.assertEqual(orig.angle_deg, loaded.angle_deg)
                self.assertEqual(orig.distance_mm, loaded.distance_mm)
                self.assertEqual(orig.intensity, loaded.intensity)
        finally:
            os.unlink(filepath)


class TestEnvironments(unittest.TestCase):
    """Tests pour les environnements prédéfinis."""
    
    def test_corridor_environment(self):
        """Test environnement couloir."""
        env = create_corridor_environment()
        self.assertEqual(env.name, "corridor")
        self.assertTrue(len(env.walls) >= 2)
    
    def test_room_environment(self):
        """Test environnement pièce meublée."""
        env = create_room_with_furniture()
        self.assertEqual(env.name, "room_furniture")
        self.assertTrue(len(env.walls) == 4)  # 4 murs
        self.assertTrue(len(env.rectangles) > 0)  # Meubles
        self.assertTrue(len(env.circles) > 0)  # Objets ronds
    
    def test_environment_simulation(self):
        """Test simulation de chaque environnement."""
        from lidar_simulator import (
            create_parking_environment, create_obstacle_course
        )
        
        environments = [
            create_corridor_environment(),
            create_room_with_furniture(),
            create_parking_environment(),
            create_obstacle_course()
        ]
        
        for env in environments:
            sim = LidarSimulator(env)
            scan = sim.generate_scan(480)
            self.assertEqual(scan.point_count, 480)
            self.assertIsNotNone(scan.min_distance)


class TestIntegration(unittest.TestCase):
    """Tests d'intégration end-to-end."""
    
    def test_full_pipeline(self):
        """Test pipeline complet: simulation -> parsing -> détection."""
        # 1. Simuler un environnement avec obstacle
        env = SimulatedEnvironment(name="test_env")
        env.add_wall(-3, -3, 3, -3)
        env.add_wall(3, -3, 3, 3)
        env.add_wall(3, 3, -3, 3)
        env.add_wall(-3, 3, -3, -3)
        env.add_circle(0, 0.5, 0.2)  # Obstacle devant proche
        
        simulator = LidarSimulator(env, noise_stddev=0.0)
        
        # 2. Générer des paquets bruts
        raw_data = b''
        for _ in range(50):  # ~360° de données
            raw_data += simulator.generate_raw_packet()
        
        # 3. Parser les données
        parser = LD19Parser()
        packets = parser.add_data(raw_data)
        self.assertTrue(len(packets) > 0)
        
        # 4. Construire un scan
        scan = LidarScan()
        for packet in packets:
            scan.add_packet(packet)
        
        # 5. Détecter les obstacles
        detector = ObstacleDetector()
        result = detector.detect(scan)
        
        # 6. Vérifier qu'on détecte l'obstacle devant
        self.assertGreater(result.max_alert_level.value, AlertLevel.NONE.value)
        
        # L'obstacle est dans la zone "front"
        front_status = result.zones_status.get("front", AlertLevel.NONE)
        self.assertGreater(front_status.value, AlertLevel.NONE.value)


def run_tests():
    """Exécute tous les tests et affiche un rapport."""
    # Créer un TestSuite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Ajouter tous les tests
    suite.addTests(loader.loadTestsFromTestCase(TestCRC8))
    suite.addTests(loader.loadTestsFromTestCase(TestLidarMeasurement))
    suite.addTests(loader.loadTestsFromTestCase(TestLD19Parser))
    suite.addTests(loader.loadTestsFromTestCase(TestLidarScan))
    suite.addTests(loader.loadTestsFromTestCase(TestSafetyZone))
    suite.addTests(loader.loadTestsFromTestCase(TestObstacleDetector))
    suite.addTests(loader.loadTestsFromTestCase(TestLidarSimulator))
    suite.addTests(loader.loadTestsFromTestCase(TestScanPersistence))
    suite.addTests(loader.loadTestsFromTestCase(TestEnvironments))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    
    # Exécuter avec verbosité
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Résumé
    print("\n" + "="*60)
    print("RÉSUMÉ DES TESTS UNITAIRES")
    print("="*60)
    print(f"Tests exécutés: {result.testsRun}")
    print(f"Réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Échecs: {len(result.failures)}")
    print(f"Erreurs: {len(result.errors)}")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
