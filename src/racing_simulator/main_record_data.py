# from mlagents_envs.logging_util import set_log_level, DEBUG
# set_log_level(DEBUG)
from lib.robocar_simulation import simulation, Observation, Action
from lib.data_recorder import DataRecorder
from lib.my_keyboard import is_key_pressed
from simulation_parameters import RAY_COUNT
from math import isclose



INPUT_COUNT = RAY_COUNT + 2
OUTPUT_COUNT = 2
RECORD_EACH_X_FRAME = 3



data_recorder = DataRecorder(columns_name=["user speed", "user steering"] + ["r" + str(i) for i in range(RAY_COUNT)] + ["speed", "steering"])
diff_steering = 0.01
diff_speed = 0.06
steering = 0
speed = 0
action = Action()



def step(frame_index: int, observation: Observation) -> Action:
    global steering, speed
    if isclose(observation.x, 0.0, rel_tol=1e-6) and isclose(observation.y, 0.10000038, rel_tol=1e-6):
        print('reset')
    
    print(observation.rays)
    
    if is_key_pressed("q"):
        if steering > 0.1:
            steering = 0.1
        steering = max(-1, min(1, steering - diff_steering))
    elif is_key_pressed("d"):
        if steering < -0.1:
            steering = -0.1
        steering = max(-1, min(1, steering + diff_steering))
    else:
        if abs(steering) < diff_steering / 2:
            steering = 0
        if steering > 0:
            steering -= diff_steering / 2
        elif steering < 0:
            steering += diff_steering / 2
    action.steering = steering
    
    if is_key_pressed("z"):
        speed = max(-1, min(1, speed + diff_speed))
    elif is_key_pressed("s"):
        speed = max(-1, min(1, speed - diff_speed * 4))
    else:
        if abs(speed) < diff_speed / 2:
            speed = 0
        if speed > 0:
            speed -= diff_speed / 2
        elif speed < 0:
            speed += diff_speed / 2
    action.speed = speed
    
    # if frame_index % RECORD_EACH_X_FRAME == 0:
    data_recorder.record([action.speed, action.steering] + observation.rays + [observation.speed, observation.steering])
    if is_key_pressed("esc"):
        data_recorder.save("data.csv")
    
    return action

simulation.run(step, step_count=-1)
simulation.exit()
