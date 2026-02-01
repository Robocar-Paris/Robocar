#!/usr/bin/env python3
"""
Test GPS RTK avec Centipede (GRATUIT)

Centipede est un réseau RTK communautaire français gratuit.
Trouve ta base la plus proche sur: https://centipede.fr/index.php/view/map

Usage:
    python scripts/test_gps_centipede.py --base NOM_DE_LA_BASE

Exemples de bases en Île-de-France:
    - IDFM (Paris)
    - ENSG (Marne-la-Vallée)
    - CT77 (Seine-et-Marne)
"""

import sys
import os
import time
import argparse
import serial

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from driver.ntrip_client import NTRIPClient
from driver.gps import GPSDriver


def main():
    parser = argparse.ArgumentParser(
        description='Test GPS RTK avec Centipede (gratuit)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Trouve ta base Centipede sur: https://centipede.fr/index.php/view/map

Exemples:
    python scripts/test_gps_centipede.py --base IDFM
    python scripts/test_gps_centipede.py --base ENSG --port /dev/ttyUSB0
        """
    )
    parser.add_argument('--base', required=True,
                        help='Nom de la base Centipede (ex: IDFM, ENSG)')
    parser.add_argument('--port', default='/dev/ttyUSB1',
                        help='Port série du GPS (default: /dev/ttyUSB1)')
    parser.add_argument('--duration', type=int, default=120,
                        help='Durée du test en secondes (default: 120)')
    parser.add_argument('--list-bases', action='store_true',
                        help='Afficher les infos pour trouver une base')
    args = parser.parse_args()

    if args.list_bases:
        NTRIPClient.list_centipede_bases()
        return 0

    print("=" * 70)
    print("GPS RTK TEST - Centipede (réseau RTK français GRATUIT)")
    print("=" * 70)
    print(f"Base Centipede: {args.base}")
    print(f"Port GPS:       {args.port}")
    print(f"Durée:          {args.duration}s")
    print()
    print("IMPORTANT: L'antenne doit avoir une vue dégagée du ciel!")
    print("=" * 70)
    print()

    # Open serial port for GPS
    try:
        gps_serial = serial.Serial(
            port=args.port,
            baudrate=460800,
            timeout=1.0
        )
        print(f"[GPS] Connecté sur {args.port}")
    except serial.SerialException as e:
        print(f"[GPS] ERREUR: {e}")
        print("      Vérifiez le port avec: ls /dev/ttyUSB*")
        return 1

    # Create Centipede client
    ntrip = NTRIPClient.centipede(args.base)

    # Create GPS driver for reading
    gps = GPSDriver(port=args.port, baudrate=460800)

    # Set approximate position (Paris area - update for your location)
    # This helps get corrections faster
    ntrip.set_position(48.85, 2.35, 35.0)

    print(f"[NTRIP] Connexion à Centipede base {args.base}...")
    if not ntrip.start():
        print("[NTRIP] ERREUR: Impossible de se connecter")
        return 1

    print("[GPS] Démarrage...")
    if not gps.start():
        print("[GPS] ERREUR: Impossible de démarrer")
        ntrip.stop()
        return 1

    print()
    print("Attente des corrections RTK et de la position GPS...")
    print()

    # Stats
    rtcm_total = 0
    best_accuracy = float('inf')
    first_rtk = None

    try:
        start_time = time.time()

        while time.time() - start_time < args.duration:
            elapsed = time.time() - start_time

            # Get RTCM corrections and send to GPS
            rtcm = ntrip.get_corrections(timeout=0.05)
            if rtcm:
                try:
                    gps_serial.write(rtcm)
                    rtcm_total += len(rtcm)
                except:
                    pass

            # Get GPS position
            pos = gps.get_position()

            # Update NTRIP with current position
            if pos and pos.is_valid:
                ntrip.set_position(pos.latitude, pos.longitude, pos.altitude)

            # Display status
            if pos:
                # Color based on quality
                if pos.quality == 4:
                    color = '\033[92m'  # Green
                    if first_rtk is None:
                        first_rtk = elapsed
                        print(f"\n\033[92m[RTK FIXED] Précision centimétrique atteinte à {elapsed:.1f}s!\033[0m\n")
                elif pos.quality == 5:
                    color = '\033[93m'  # Yellow
                else:
                    color = '\033[0m'

                if pos.accuracy_h < best_accuracy and pos.quality >= 4:
                    best_accuracy = pos.accuracy_h

                # NTRIP status
                ntrip_status = f"RTCM: {rtcm_total/1024:.1f}KB"
                if ntrip.seconds_since_correction < 5:
                    ntrip_status += " \033[92m●\033[0m"
                elif ntrip.is_connected():
                    ntrip_status += " \033[93m●\033[0m"
                else:
                    ntrip_status += " \033[91m●\033[0m"

                print(f"\r[{elapsed:3.0f}s] {color}{pos.quality_string:10}\033[0m | "
                      f"({pos.latitude:10.6f}, {pos.longitude:10.6f}) | "
                      f"Acc: {pos.accuracy_h:5.3f}m | "
                      f"Sats: {pos.satellites:2d} | "
                      f"{ntrip_status}     ", end='', flush=True)
            else:
                print(f"\r[{elapsed:3.0f}s] Recherche satellites... | "
                      f"RTCM: {rtcm_total} bytes     ", end='', flush=True)

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\n[Test] Interrompu")

    finally:
        gps.stop()
        ntrip.stop()
        gps_serial.close()

    # Summary
    print()
    print("=" * 70)
    print("RÉSUMÉ")
    print("=" * 70)
    print(f"RTCM reçu:       {rtcm_total/1024:.1f} KB")
    print(f"Base Centipede:  {args.base}")

    if first_rtk:
        print(f"RTK Fixed après: {first_rtk:.1f}s")
    if best_accuracy < float('inf'):
        print(f"Meilleure précision: {best_accuracy:.3f}m")

    final_pos = gps.get_position()
    if final_pos:
        print()
        print(f"Dernière position:")
        print(f"  Qualité:   {final_pos.quality_string}")
        print(f"  Latitude:  {final_pos.latitude:.7f}°")
        print(f"  Longitude: {final_pos.longitude:.7f}°")
        print(f"  Précision: {final_pos.accuracy_h:.3f}m")

        if final_pos.is_rtk_fixed:
            print()
            print("\033[92m[SUCCESS] RTK FIXED - Précision centimétrique!\033[0m")
            return 0

    return 0


if __name__ == '__main__':
    sys.exit(main())