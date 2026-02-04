import sys
import time
import os

# Ajout du chemin pour trouver les modules
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))
from driver.vesc_motor import VESCController

def test_direction():
    vesc = VESCController(port='/dev/ttyACM0')
    
    print("--- TEST MOTEUR ---")
    if not vesc.start():
        print("Erreur connexion VESC")
        return

    try:
        print("1. Test POSITIF (+0.1) pendant 2 secondes...")
        print("   -> Si les roues tournent vers l'AVANT : C'est cablé Normalement.")
        print("   -> Si les roues tournent vers l'ARRIERE : C'est Inversé.")
        
        # On envoie une commande positive pure
        vesc.set_duty_cycle(0.1) 
        time.sleep(2.0)
        vesc.set_duty_cycle(0.0)
        print("   STOP.\n")
        
        time.sleep(1.0)
        
        print("2. Test NEGATIF (-0.1) pendant 2 secondes...")
        vesc.set_duty_cycle(-0.1)
        time.sleep(2.0)
        vesc.set_duty_cycle(0.0)
        print("   STOP.\n")

    except KeyboardInterrupt:
        vesc.stop()
    finally:
        vesc.stop()

if __name__ == "__main__":
    test_direction()

