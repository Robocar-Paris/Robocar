# from mlagents_envs.logging_util import set_log_level, DEBUG
# set_log_level(DEBUG)
from lib.robocar_simulation import simulation, Observation, Action
from lib.Gamepad.Gamepad import Gamepad, available
from lib.data_recorder import DataRecorder
from simulation_parameters import RAY_COUNT
from math import isclose
import time



INPUT_COUNT = RAY_COUNT + 2
OUTPUT_COUNT = 2
RECORD_EACH_X_FRAME = 3



data_recorder = DataRecorder(columns_name=["user speed", "user steering"] + ["r" + str(i) for i in range(RAY_COUNT)] + ["speed", "steering"])
diff_steering = 0.1
diff_speed = 0.06
steering = 0
speed = 0
action = Action()

# Wait for a connection
if not available():
    print('Please connect your gamepad...')
    while not available():
        time.sleep(1.0)
print('Gamepad connected')

gamepad = Gamepad()
gamepad.startBackgroundUpdates()

def step(frame_index: int, observation: Observation) -> Action:
    global steering, speed
    if isclose(observation.x, 0.0, rel_tol=1e-6) and isclose(observation.y, 0.10000038, rel_tol=1e-6):
        print('reset')
    
    print(gamepad.axis(0))
    if gamepad.axis(0) > 0.1 or gamepad.axis(0) < -0.1:
        steering = gamepad.axis(0) * 0.5
        # steering = max(-1, min(1, steering - diff_steering * -gamepad.axis(0)))
    else:
        if abs(steering) < diff_steering / 2:
            steering = 0
        if steering > 0:
            steering -= diff_steering / 2
        elif steering < 0:
            steering += diff_steering / 2
    action.steering = steering
    
    if gamepad.axis(2) != -1:
        speed = max(-1, min(1, speed - diff_speed * 4 * ((gamepad.axis(2) + 1) / 2)))
    elif gamepad.axis(5) != -1:
        speed = max(-1, min(1, speed + diff_speed * ((gamepad.axis(5) + 1) / 2)))
    else:
        if abs(speed) < diff_speed / 2:
            speed = 0
        if speed > 0:
            speed -= diff_speed / 2
        elif speed < 0:
            speed += diff_speed / 2
    if speed > 0.4:
        speed = 0.4
    action.speed = speed
    
    # if frame_index % RECORD_EACH_X_FRAME == 0:
    # data_recorder.record([action.speed, action.steering] + observation.rays + [observation.speed, observation.steering])
    # if is_key_pressed("esc"):
    #     data_recorder.save("data.csv")
    
    return action

simulation.run(step, step_count=-1)
gamepad.disconnect()
simulation.exit()
