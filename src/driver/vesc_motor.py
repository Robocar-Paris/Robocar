"""
VESC Motor Controller Driver
"""
import serial
import time
import threading
from typing import Optional, Tuple
from dataclasses import dataclass

# --- IMPORTATION INTELLIGENTE ET AUTONOME ---
PYVESC_AVAILABLE = False
SetDutyCycle = None
SetServoPos = None
GetValues = None
SetCurrentBrake = None

try:
    import pyvesc
    from pyvesc.protocol.base import VESCMessage 
    
    # 1. On essaye d'importer SetDutyCycle
    try:
        from pyvesc.VESC.messages.setters import SetDutyCycle
    except ImportError:
        class SetDutyCycle(VESCMessage):
            id = 5
            fields = [('duty', 'i', 100000)]

    # 2. On essaye d'importer SetServoPos
    try:
        from pyvesc.VESC.messages.setters import SetServoPos
    except ImportError:
        class SetServoPos(VESCMessage):
            id = 12
            fields = [('pos', 'h', 1000)]

    # 3. On essaye d'importer GetValues
    try:
        from pyvesc.VESC.messages.getters import GetValues
    except ImportError:
         class GetValues(VESCMessage):
            id = 4
            fields = []

    # 4. On essaye d'importer SetCurrentBrake (POUR LE FREINAGE ACTIF)
    try:
        from pyvesc.VESC.messages.setters import SetCurrentBrake
    except ImportError:
        class SetCurrentBrake(VESCMessage):
            id = 7
            fields = [('current', 'i', 1000)]

    PYVESC_AVAILABLE = True
    print("[VESC] Driver charge (Mode Autonome)")

except ImportError as e:
    print(f"[VESC] ERREUR CRITIQUE: {e}")
    PYVESC_AVAILABLE = False
# -------------------------------------------

@dataclass
class VESCState:
    timestamp: float
    duty_cycle: float
    rpm: float
    current: float
    voltage: float
    tachometer: int
    temp_motor: float
    temp_mos: float

class VESCController:
    # ====================================================================
    #                     CONFIGURATION DU MOTEUR
    # ====================================================================
    # SI LA VOITURE RECULE AU LIEU D'AVANCER : Changez False en True (ou l'inverse)
    INVERSER_MOTEUR = False  
    
    MAX_DUTY_CYCLE = 0.05     # VITESSE (0.1 = 10% de puissance)
    MAX_STEERING = 0.3
    RAMP_STEP = 0.02
    RAMP_INTERVAL = 0.02
    # ====================================================================

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        self._current_duty = 0.0
        self._target_duty = 0.0
        self._current_servo = 0.5
        self._latest_state: Optional[VESCState] = None
        self._last_tachometer = 0
        self._total_distance = 0.0
        self._wheel_circumference = 0.3
        self._motor_poles = 14
        self._gear_ratio = 1.0

    def start(self) -> bool:
        if not PYVESC_AVAILABLE:
            return False
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()
            
            time.sleep(0.1)
            self._set_duty_raw(0.0)
            self._set_servo_raw(0.5)
            return True
        except Exception as e:
            print(f"[VESC] Erreur connexion: {e}")
            return False

    def stop(self):
        self._target_duty = 0.0
        time.sleep(0.5)
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._set_duty_raw(0.0)
            self._set_servo_raw(0.5)
        except:
            pass
        if self._serial:
            self._serial.close()

    def set_speed(self, speed: float):
        """
        Commande la vitesse.
        NOTE: On force la marche avant (abs) et on applique l'inversion si demandée.
        """
        # 1. On prend la valeur absolue pour "Forcer l'avancée" (Hack GPS)
        target = abs(speed)
        
        # 2. On applique l'inversion configurée en haut du fichier
        if self.INVERSER_MOTEUR:
            target = -target
            
        # 3. On applique la limite de puissance
        duty = max(-1.0, min(1.0, target)) * self.MAX_DUTY_CYCLE
        self.set_duty(duty)

    def brake(self, current: float = 20.0):
        """
        Freinage actif avec un courant spécifique (Ampères).
        Contrairement à set_speed(0) qui met en roue libre, ceci force le moteur à s'arrêter.
        """
        # Note: Le courant de freinage doit être positif dans la commande PyVESC,
        # mais le VESC le gère comme une force opposée au mouvement.
        if self._serial and 'SetCurrentBrake' in globals() and SetCurrentBrake:
            try:
                # On envoie la commande de freinage
                self._serial.write(pyvesc.encode(SetCurrentBrake(abs(current))))
                # IMPORTANT : On reset la consigne de vitesse pour que le loop
                # de ramping ne relance pas le moteur immédiatement
                self._target_duty = 0.0 
                self._current_duty = 0.0
            except Exception as e:
                print(f"[VESC] Erreur freinage: {e}")

    def set_steering(self, steering: float):
        """Interface standard: steering entre -1.0 (gauche) et 1.0 (droite)"""
        servo_pos = 0.5 - (steering * 0.5) 
        self.set_servo(servo_pos)

    def set_duty(self, duty: float):
        self._target_duty = max(-self.MAX_DUTY_CYCLE, min(self.MAX_DUTY_CYCLE, duty))

    def set_duty_cycle(self, duty: float):
        self.set_duty(duty)

    def set_servo(self, position: float):
        self._current_servo = max(0.0, min(1.0, position))
        self._set_servo_raw(self._current_servo)

    def get_state(self) -> Optional[VESCState]:
        with self._lock:
            return self._latest_state

    def _update_loop(self):
        last_ramp = time.time()
        while self._running:
            try:
                now = time.time()
                # Gestion du ramping (accélération progressive)
                if now - last_ramp >= self.RAMP_INTERVAL:
                    self._ramp_duty()
                    last_ramp = now
                self._read_state()
                time.sleep(0.01)
            except Exception:
                pass

    def _ramp_duty(self):
        if abs(self._target_duty - self._current_duty) < 0.001:
            return
        if self._target_duty > self._current_duty:
            self._current_duty = min(self._target_duty, self._current_duty + self.RAMP_STEP)
        else:
            self._current_duty = max(self._target_duty, self._current_duty - self.RAMP_STEP)
        self._set_duty_raw(self._current_duty)

    def _set_duty_raw(self, duty: float):
        if self._serial and SetDutyCycle:
            try:
                self._serial.write(pyvesc.encode(SetDutyCycle(duty)))
            except: pass

    def _set_servo_raw(self, position: float):
        if self._serial and SetServoPos:
            try:
                self._serial.write(pyvesc.encode(SetServoPos(position)))
            except: pass

    def _read_state(self):
        if not self._serial or not GetValues: return
        try:
            self._serial.write(pyvesc.encode_request(GetValues))
            if self._serial.in_waiting > 0:
                buffer = self._serial.read(self._serial.in_waiting)
                response, consumed = pyvesc.decode(buffer)
                if response:
                    state = VESCState(
                        timestamp=time.time(),
                        duty_cycle=getattr(response, 'duty_now', 0.0),
                        rpm=getattr(response, 'rpm', 0.0),
                        current=getattr(response, 'avg_motor_current', 0.0),
                        voltage=getattr(response, 'v_in', 0.0),
                        tachometer=getattr(response, 'tachometer', 0),
                        temp_motor=getattr(response, 'temp_motor', 0.0),
                        temp_mos=getattr(response, 'temp_mos', 0.0)
                    )
                    with self._lock:
                        self._latest_state = state
        except: pass

