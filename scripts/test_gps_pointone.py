#!/usr/bin/env python3
"""
Test GPS Point One RTK

Script simple pour tester la connexion avec le GPS Point One
et récupérer les coordonnées GPS.

Usage:
    python scripts/test_gps_pointone.py
    python scripts/test_gps_pointone.py --port /dev/ttyUSB0
"""

import sys
import time
import argparse
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from driver.gps import GPSDriver


def main():
    parser = argparse.ArgumentParser(description='Test GPS Point One RTK')
    parser.add_argument('--port', default='/dev/ttyUSB1',
                        help='Port série du GPS (défaut: /dev/ttyUSB1)')
    parser.add_argument('--duration', type=int, default=30,
                        help='Durée du test en secondes (défaut: 30)')
    args = parser.parse_args()

    print("=" * 60)
    print("TEST GPS POINT ONE RTK")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Durée: {args.duration}s")
    print()
    print("IMPORTANT: L'antenne doit avoir une vue dégagée du ciel!")
    print("=" * 60)
    print()

    # Créer le driver GPS (460800 baud pour Point One RTK)
    gps = GPSDriver(port=args.port, baudrate=460800)

    # Démarrer le GPS
    print("[GPS] Connexion...")
    if not gps.start():
        print("[GPS] ERREUR: Impossible de se connecter au GPS")
        print("      Vérifiez que le GPS est branché et le port correct")
        print("      Essayez: --port /dev/ttyUSB0 ou --port /dev/ttyUSB1")
        return 1

    print("[GPS] Connecté!")
    print("[GPS] Attente de la première position...")
    print()

    try:
        start_time = time.time()
        last_quality = None
        position_count = 0

        while time.time() - start_time < args.duration:
            pos = gps.get_position()

            if pos:
                position_count += 1

                # Afficher si la qualité change ou toutes les 2 secondes
                if pos.quality != last_quality or position_count % 20 == 0:
                    # Couleurs pour la qualité
                    quality_color = {
                        4: '\033[92m',  # Vert pour RTK Fixed
                        5: '\033[93m',  # Jaune pour RTK Float
                    }.get(pos.quality, '\033[0m')

                    print(f"[{time.strftime('%H:%M:%S')}] "
                          f"{quality_color}{pos.quality_string:10}\033[0m | "
                          f"Lat: {pos.latitude:11.6f} | "
                          f"Lon: {pos.longitude:11.6f} | "
                          f"Alt: {pos.altitude:6.1f}m | "
                          f"Acc: {pos.accuracy_h:5.2f}m | "
                          f"Sats: {pos.satellites}")

                    last_quality = pos.quality
            else:
                # Pas encore de position
                elapsed = int(time.time() - start_time)
                print(f"\r[GPS] Recherche de satellites... ({elapsed}s)", end='', flush=True)

            time.sleep(0.1)

        print()
        print("=" * 60)
        print("RÉSUMÉ")
        print("=" * 60)

        final_pos = gps.get_position()
        if final_pos:
            print(f"Dernière position:")
            print(f"  Latitude:  {final_pos.latitude:.6f}°")
            print(f"  Longitude: {final_pos.longitude:.6f}°")
            print(f"  Altitude:  {final_pos.altitude:.1f}m")
            print(f"  Qualité:   {final_pos.quality_string}")
            print(f"  Précision: {final_pos.accuracy_h:.2f}m horizontal")
            print(f"  Satellites: {final_pos.satellites}")

            if final_pos.is_rtk_fixed:
                print()
                print("\033[92m[OK] RTK FIXED - Précision centimétrique!\033[0m")
            elif final_pos.quality == 5:
                print()
                print("\033[93m[OK] RTK FLOAT - Précision ~50cm\033[0m")
                print("     Pour RTK Fixed, vérifiez les corrections Polaris")
            elif final_pos.is_valid:
                print()
                print("\033[93m[OK] GPS standard - Précision ~2-5m\033[0m")
                print("     Pour RTK, configurez les corrections Polaris")
        else:
            print("\033[91m[ERREUR] Aucune position GPS reçue\033[0m")
            print("Vérifiez:")
            print("  - L'antenne a une vue dégagée du ciel")
            print("  - Le GPS est correctement alimenté")
            print("  - Le port série est correct")
            return 1

    except KeyboardInterrupt:
        print("\n\n[GPS] Arrêt par l'utilisateur")

    finally:
        gps.stop()
        print("[GPS] Déconnecté")

    return 0


if __name__ == '__main__':
    sys.exit(main())