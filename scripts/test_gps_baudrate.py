#!/usr/bin/env python3
"""
Script pour detecter le bon baudrate du GPS.
Teste plusieurs baudrates courants et affiche les donnees recues.
"""

import serial
import sys
import time

# Baudrates courants pour GPS
BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

def test_baudrate(port: str, baudrate: int, timeout: float = 2.0) -> bool:
    """Teste un baudrate et retourne True si des donnees NMEA sont recues."""
    print(f"\n{'='*50}")
    print(f"Test baudrate: {baudrate}")
    print(f"{'='*50}")

    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
        start = time.time()
        buffer = ""
        nmea_found = False

        while time.time() - start < timeout:
            if ser.in_waiting > 0:
                try:
                    data = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                    buffer += data

                    # Afficher les donnees brutes (premieres lignes)
                    lines = buffer.split('\n')
                    for line in lines[:-1]:
                        line = line.strip()
                        if line:
                            # Verifier si c'est du NMEA valide
                            if line.startswith('$'):
                                print(f"  [NMEA] {line[:80]}")
                                nmea_found = True
                            else:
                                # Afficher les premiers caracteres pour diagnostic
                                preview = line[:50].replace('\r', '\\r')
                                print(f"  [DATA] {preview}...")

                    buffer = lines[-1]  # Garder le reste

                except Exception as e:
                    print(f"  [ERR] {e}")
            else:
                time.sleep(0.05)

        ser.close()

        if nmea_found:
            print(f"\n  >>> BAUDRATE {baudrate} FONCTIONNE! <<<")
            return True
        else:
            print(f"  Pas de donnees NMEA valides")
            return False

    except serial.SerialException as e:
        print(f"  Erreur port: {e}")
        return False

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

    print(f"Detection baudrate GPS sur {port}")
    print(f"Baudrates a tester: {BAUDRATES}")

    working_baudrates = []

    for baudrate in BAUDRATES:
        if test_baudrate(port, baudrate):
            working_baudrates.append(baudrate)

    print("\n" + "="*50)
    print("RESULTATS")
    print("="*50)

    if working_baudrates:
        print(f"Baudrates fonctionnels: {working_baudrates}")
        print(f"\nUtilisez: baudrate={working_baudrates[0]}")
    else:
        print("Aucun baudrate n'a fonctionne!")
        print("\nVerifiez:")
        print("  1. Le cable USB est bien connecte")
        print("  2. Le GPS est alimente")
        print("  3. Le port est correct (ls /dev/ttyUSB*)")

if __name__ == '__main__':
    main()
