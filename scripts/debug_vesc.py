import sys
import os

print("--- DIAGNOSTIC PyVESC ---")
try:
    import pyvesc
    print(f"[OK] pyvesc importe depuis: {pyvesc.__file__}")
    
    # Lister le contenu du dossier
    folder = os.path.dirname(pyvesc.__file__)
    print(f"Contenu du dossier: {os.listdir(folder)}")
    
    # Essayer de trouver le sous-dossier VESC
    vesc_folder = os.path.join(folder, 'VESC')
    if os.path.exists(vesc_folder):
        print(f"Contenu de VESC: {os.listdir(vesc_folder)}")
        msg_folder = os.path.join(vesc_folder, 'messages')
        if os.path.exists(msg_folder):
             print(f"Contenu de VESC/messages: {os.listdir(msg_folder)}")
    
    # Tester les imports
    print("\nTests d'import:")
    try:
        from pyvesc.VESC.messages import SetDutyCycle
        print("  [OK] from pyvesc.VESC.messages import SetDutyCycle")
    except ImportError as e:
        print(f"  [FAIL] pyvesc.VESC.messages: {e}")

    try:
        from pyvesc.messages import SetDutyCycle
        print("  [OK] from pyvesc.messages import SetDutyCycle")
    except ImportError as e:
        print(f"  [FAIL] pyvesc.messages: {e}")

except ImportError as e:
    print(f"[CRITIQUE] Impossible d'importer pyvesc: {e}")
