from lib.Gamepad.Gamepad import Gamepad, available
from pyvesc import VESC
import time

serial_port = '/dev/ttyACM0'

MAX_DUTY_CYCLE = 0.2   # Limite de securite pour le duty cycle
MAX_STEERING = 0.3     # Limite max pour le steering
diff_steering = 0.05   # Pour lissage de la direction
duty_smoothing = 10
duty_colapse = 0.01

if not available():
    print('Veuillez connecter votre manette...')
    while not available():
        time.sleep(1)
print('Manette connectée')

gamepad = Gamepad()
gamepad.startBackgroundUpdates()

motor = VESC(serial_port=serial_port)

steering = 0
duty = 0

try:
    print("Contrôle en cours. Appuie sur 'BACK' pour quitter.")

    while True:
        # Lecture des axes
        axis_throttle = gamepad.axis(5)  # Gâchette droite (accélérer)
        axis_brake = gamepad.axis(2)     # Gâchette gauche (freiner)
        axis_steering = gamepad.axis(6)  # Joystick gauche horizontal pour direction
        axis_turn_max = gamepad.axis(0)  # buttom direction left

        # Calcul du duty cycle
        if axis_throttle != -1:
            duty += ((axis_throttle + 1) / 2) / duty_smoothing
        if axis_brake != -1:
            duty -= ((axis_brake + 1) / 2) / duty_smoothing

        if duty > MAX_DUTY_CYCLE:
            duty = MAX_DUTY_CYCLE
        if duty < -MAX_DUTY_CYCLE:
            duty = -MAX_DUTY_CYCLE
        motor.set_duty_cycle(duty)
        if duty > 0:
            duty = max(0, duty - duty_colapse)
        if duty < 0:
            duty = min(0, duty + duty_colapse)

        if abs(axis_turn_max) > 0.1:
            # Direction maximale forcée avec un bouton ou axe dédié
            steering = axis_turn_max * MAX_STEERING
        
        elif abs(axis_steering) > 0.1:
            # Mode progressif : on approche la cible progressivement
            target_steering = axis_steering * MAX_STEERING
        
            if abs(steering - target_steering) <= diff_steering:
                steering = target_steering
            elif steering < target_steering:
                steering += diff_steering
            elif steering > target_steering:
                steering -= diff_steering
        
        else:
            # Si aucun joystick n'est actif, retour progressif au neutre
            if abs(steering) < diff_steering:
                steering = 0
            elif steering > 0:
                steering -= diff_steering
            elif steering < 0:
                steering += diff_steering

        # Normalisation steering de [-MAX_STEERING, MAX_STEERING] à [0,1]
        steering_normalized = (steering / MAX_STEERING + 1) / 2
        steering_normalized = max(0, min(1, steering_normalized))

        motor.set_servo(steering_normalized)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nArrêt manuel.")

finally:
    print("Arrêt du moteur...")
    motor.set_duty_cycle(0)
    motor.set_servo(0.5)  # position neutre steering
    motor.stop_heartbeat()
    gamepad.disconnect()
    print("Déconnecté proprement.")


