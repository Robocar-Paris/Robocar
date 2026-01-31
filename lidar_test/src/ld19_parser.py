#!/usr/bin/env python3
"""
LD19 LiDAR Parser Module
========================
Parser complet pour le LiDAR LD19 (LDROBOT) avec validation CRC.

Protocole de communication:
- Port série: 230400 baud, 8N1
- Communication unidirectionnelle (LiDAR -> Host)
- Paquets de 47 bytes avec 12 mesures par paquet

Format du paquet:
- Header (1 byte): 0x54
- Length (1 byte): 0x2C (fixe, 44 en décimal)
- Speed (2 bytes): Vitesse de rotation en deg/s
- Start Angle (2 bytes): Angle de départ * 100
- Data (36 bytes): 12 mesures de 3 bytes chacune
  - Distance (2 bytes): En millimètres
  - Intensity (1 byte): Intensité de réflexion
- End Angle (2 bytes): Angle de fin * 100  
- Timestamp (2 bytes): En millisecondes
- CRC (1 byte): CRC8 polynomial 0x4D
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np


# Constantes du protocole LD19
HEADER_BYTE = 0x54
LENGTH_BYTE = 0x2C
PACKET_LENGTH = 47
MEASUREMENTS_PER_PACKET = 12
BAUD_RATE = 230400

# Format struct pour le parsing (little endian)
# B: Header, B: Length, H: Speed, H: StartAngle, 
# (HB)*12: 12x(Distance, Intensity), H: EndAngle, H: Timestamp, B: CRC
PACKET_FORMAT = "<BBHH" + "HB" * MEASUREMENTS_PER_PACKET + "HHB"


@dataclass
class LidarMeasurement:
    """Une mesure individuelle du LiDAR."""
    angle_deg: float      # Angle en degrés (0-360)
    distance_mm: int      # Distance en millimètres
    intensity: int        # Intensité de réflexion (0-255)
    
    @property
    def distance_m(self) -> float:
        """Distance en mètres."""
        return self.distance_mm / 1000.0
    
    @property
    def angle_rad(self) -> float:
        """Angle en radians."""
        return np.radians(self.angle_deg)
    
    def to_cartesian(self) -> Tuple[float, float]:
        """Convertit en coordonnées cartésiennes (x, y) en mètres."""
        x = np.sin(self.angle_rad) * self.distance_m
        y = np.cos(self.angle_rad) * self.distance_m
        return (x, y)
    
    def is_valid(self, min_distance_mm: int = 10, max_distance_mm: int = 12000,
                 min_intensity: int = 0) -> bool:
        """Vérifie si la mesure est valide."""
        return (min_distance_mm <= self.distance_mm <= max_distance_mm and
                self.intensity >= min_intensity and
                self.distance_mm > 0)


@dataclass  
class LidarPacket:
    """Un paquet complet du LiDAR LD19."""
    speed_dps: int                    # Vitesse en deg/s
    start_angle_deg: float            # Angle de départ
    end_angle_deg: float              # Angle de fin
    timestamp_ms: int                 # Timestamp
    measurements: List[LidarMeasurement]  # 12 mesures
    crc_valid: bool                   # CRC valide
    
    @property
    def angle_step_deg(self) -> float:
        """Pas angulaire entre chaque mesure."""
        end = self.end_angle_deg
        if end < self.start_angle_deg:
            end += 360.0
        return (end - self.start_angle_deg) / (MEASUREMENTS_PER_PACKET - 1)


class CRC8:
    """Calculateur CRC8 pour le protocole LD19."""
    
    def __init__(self, polynomial: int = 0x4D, initial: int = 0x00):
        self.polynomial = polynomial
        self.initial = initial
        self._table = self._generate_table()
    
    def _generate_table(self) -> List[int]:
        """Génère la table de lookup CRC."""
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ self.polynomial) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
            table.append(crc)
        return table
    
    def calculate(self, data: bytes) -> int:
        """Calcule le CRC8 pour les données."""
        crc = self.initial
        for byte in data:
            crc = self._table[crc ^ byte]
        return crc


class LD19Parser:
    """Parser pour le LiDAR LD19."""
    
    def __init__(self):
        self.crc_calculator = CRC8()
        self._buffer = bytearray()
        self._packets_parsed = 0
        self._packets_invalid = 0
        
    def reset(self):
        """Réinitialise le parser."""
        self._buffer.clear()
        
    def add_data(self, data: bytes) -> List[LidarPacket]:
        """
        Ajoute des données au buffer et retourne les paquets parsés.
        
        Args:
            data: Données brutes du port série
            
        Returns:
            Liste des paquets valides parsés
        """
        self._buffer.extend(data)
        packets = []
        
        while len(self._buffer) >= PACKET_LENGTH:
            # Chercher le header
            try:
                header_idx = self._buffer.index(HEADER_BYTE)
                if header_idx > 0:
                    # Supprimer les données avant le header
                    del self._buffer[:header_idx]
            except ValueError:
                # Pas de header trouvé
                self._buffer.clear()
                break
            
            if len(self._buffer) < PACKET_LENGTH:
                break
                
            # Vérifier le byte de longueur
            if self._buffer[1] != LENGTH_BYTE:
                del self._buffer[0]
                continue
            
            # Extraire et parser le paquet
            packet_data = bytes(self._buffer[:PACKET_LENGTH])
            packet = self._parse_packet(packet_data)
            
            if packet:
                packets.append(packet)
                self._packets_parsed += 1
            else:
                self._packets_invalid += 1
                
            del self._buffer[:PACKET_LENGTH]
            
        return packets
    
    def _parse_packet(self, data: bytes) -> Optional[LidarPacket]:
        """Parse un paquet de données brutes."""
        try:
            # Vérifier le CRC
            calculated_crc = self.crc_calculator.calculate(data[:-1])
            received_crc = data[-1]
            crc_valid = (calculated_crc == received_crc)
            
            # Unpacker les données
            unpacked = struct.unpack(PACKET_FORMAT, data)
            
            header = unpacked[0]
            length = unpacked[1]
            speed = unpacked[2]
            start_angle_raw = unpacked[3]
            
            # Extraire les mesures (12 x (distance, intensity))
            measurement_data = unpacked[4:4 + MEASUREMENTS_PER_PACKET * 2]
            
            end_angle_raw = unpacked[4 + MEASUREMENTS_PER_PACKET * 2]
            timestamp = unpacked[5 + MEASUREMENTS_PER_PACKET * 2]
            crc = unpacked[6 + MEASUREMENTS_PER_PACKET * 2]
            
            # Convertir les angles
            start_angle = start_angle_raw / 100.0
            end_angle = end_angle_raw / 100.0
            
            # Calculer l'angle pour chaque mesure
            if end_angle < start_angle:
                end_angle_calc = end_angle + 360.0
            else:
                end_angle_calc = end_angle
            step = (end_angle_calc - start_angle) / (MEASUREMENTS_PER_PACKET - 1)
            
            # Créer les mesures
            measurements = []
            for i in range(MEASUREMENTS_PER_PACKET):
                distance = measurement_data[i * 2]
                intensity = measurement_data[i * 2 + 1]
                angle = (start_angle + step * i) % 360.0
                
                measurements.append(LidarMeasurement(
                    angle_deg=angle,
                    distance_mm=distance,
                    intensity=intensity
                ))
            
            return LidarPacket(
                speed_dps=speed,
                start_angle_deg=start_angle,
                end_angle_deg=end_angle,
                timestamp_ms=timestamp,
                measurements=measurements,
                crc_valid=crc_valid
            )
            
        except Exception as e:
            return None
    
    @property
    def stats(self) -> dict:
        """Statistiques du parser."""
        total = self._packets_parsed + self._packets_invalid
        return {
            "packets_parsed": self._packets_parsed,
            "packets_invalid": self._packets_invalid,
            "total": total,
            "success_rate": self._packets_parsed / total if total > 0 else 0
        }


class LidarScan:
    """Représente un scan complet à 360° du LiDAR."""
    
    def __init__(self, measurements: List[LidarMeasurement] = None):
        self.measurements = measurements or []
        self.timestamp = None
        
    def add_measurement(self, measurement: LidarMeasurement):
        """Ajoute une mesure au scan."""
        self.measurements.append(measurement)
        
    def add_packet(self, packet: LidarPacket):
        """Ajoute toutes les mesures d'un paquet."""
        self.measurements.extend(packet.measurements)
        self.timestamp = packet.timestamp_ms
        
    def get_valid_measurements(self, min_distance_mm: int = 10,
                               max_distance_mm: int = 12000,
                               min_intensity: int = 0) -> List[LidarMeasurement]:
        """Retourne uniquement les mesures valides."""
        return [m for m in self.measurements 
                if m.is_valid(min_distance_mm, max_distance_mm, min_intensity)]
    
    def to_polar_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Retourne les données en format polaire (angles, distances, intensités)."""
        angles = np.array([m.angle_deg for m in self.measurements])
        distances = np.array([m.distance_mm for m in self.measurements])
        intensities = np.array([m.intensity for m in self.measurements])
        return angles, distances, intensities
    
    def to_cartesian_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Retourne les données en format cartésien (x, y, intensités) en mètres."""
        x_coords = []
        y_coords = []
        intensities = []
        
        for m in self.measurements:
            x, y = m.to_cartesian()
            x_coords.append(x)
            y_coords.append(y)
            intensities.append(m.intensity)
            
        return np.array(x_coords), np.array(y_coords), np.array(intensities)
    
    def get_points_in_sector(self, angle_min: float, angle_max: float,
                            distance_max: float = 12.0) -> List[LidarMeasurement]:
        """
        Retourne les points dans un secteur angulaire.
        
        Args:
            angle_min: Angle minimum en degrés
            angle_max: Angle maximum en degrés
            distance_max: Distance maximum en mètres
        """
        results = []
        for m in self.measurements:
            angle = m.angle_deg
            # Gestion du wrap-around à 360°
            if angle_min <= angle_max:
                in_sector = angle_min <= angle <= angle_max
            else:
                in_sector = angle >= angle_min or angle <= angle_max
                
            if in_sector and m.distance_m <= distance_max:
                results.append(m)
                
        return results
    
    @property
    def min_distance(self) -> Optional[float]:
        """Distance minimale détectée (en mètres)."""
        valid = self.get_valid_measurements()
        if valid:
            return min(m.distance_m for m in valid)
        return None
    
    @property
    def point_count(self) -> int:
        """Nombre de points dans le scan."""
        return len(self.measurements)


def create_test_packet() -> bytes:
    """Crée un paquet de test valide pour le debug."""
    # Header et length
    data = bytearray([HEADER_BYTE, LENGTH_BYTE])
    
    # Speed (3600 deg/s = typique)
    data.extend(struct.pack("<H", 3600))
    
    # Start angle (0 deg * 100)
    data.extend(struct.pack("<H", 0))
    
    # 12 mesures (distance=1000mm, intensity=200)
    for i in range(MEASUREMENTS_PER_PACKET):
        data.extend(struct.pack("<HB", 1000 + i * 100, 200))
    
    # End angle (30 deg * 100)
    data.extend(struct.pack("<H", 3000))
    
    # Timestamp
    data.extend(struct.pack("<H", 12345))
    
    # Calculer et ajouter le CRC
    crc = CRC8().calculate(bytes(data))
    data.append(crc)
    
    return bytes(data)


if __name__ == "__main__":
    # Test du parser
    print("=== Test du Parser LD19 ===\n")
    
    parser = LD19Parser()
    
    # Créer un paquet de test
    test_data = create_test_packet()
    print(f"Paquet de test: {len(test_data)} bytes")
    print(f"Header: 0x{test_data[0]:02X}, Length: 0x{test_data[1]:02X}")
    
    # Parser le paquet
    packets = parser.add_data(test_data)
    
    if packets:
        packet = packets[0]
        print(f"\nPaquet parsé avec succès!")
        print(f"  - Vitesse: {packet.speed_dps} deg/s")
        print(f"  - Angle début: {packet.start_angle_deg}°")
        print(f"  - Angle fin: {packet.end_angle_deg}°")
        print(f"  - Timestamp: {packet.timestamp_ms} ms")
        print(f"  - CRC valide: {packet.crc_valid}")
        print(f"  - Mesures: {len(packet.measurements)}")
        
        print("\n  Premières mesures:")
        for i, m in enumerate(packet.measurements[:3]):
            x, y = m.to_cartesian()
            print(f"    [{i}] Angle: {m.angle_deg:.1f}°, Distance: {m.distance_mm}mm, "
                  f"Intensité: {m.intensity}, Cartésien: ({x:.3f}m, {y:.3f}m)")
    else:
        print("Erreur: Aucun paquet parsé")
    
    print(f"\nStatistiques: {parser.stats}")
  