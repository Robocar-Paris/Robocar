# from mlagents_envs.logging_util import set_log_level, DEBUG
# set_log_level(DEBUG)
from lib.robocar_simulation import simulation, Observation, Action
from lib.my_keyboard import is_key_pressed
from lib.RobocarModel import RobocarModel
from math import isclose
import torch



RECORD_EACH_X_FRAME = 3



model = RobocarModel()
model.load_state_dict(torch.load("models/current_model", weights_only=True))
model.eval()

diff_steering = 0.005
diff_speed = 0.02
steering = 0
speed = 0
action = Action()



def step(frame_index: int, observation: Observation) -> Action:
    global steering, speed
    if isclose(observation.x, 0.0, rel_tol=1e-6) and isclose(observation.y, 0.10000038, rel_tol=1e-6):
        print('reset')
    
    pred = model.forward(torch.tensor(observation.rays + [observation.speed, observation.steering], dtype=torch.float))
    
    if is_key_pressed("q"):
        steering = max(-1, min(1, steering - diff_steering))
        action.steering = steering
    elif is_key_pressed("d"):
        steering = max(-1, min(1, steering + diff_steering))
        action.steering = steering
    else:
        if abs(steering) < diff_steering / 4:
            steering = 0
        if steering > 0:
            steering -= diff_steering / 4
        elif steering < 0:
            steering += diff_steering / 4
        action.steering = pred[1]
    
    if is_key_pressed("z"):
        speed = max(-1, min(1, speed + diff_speed))
        action.speed = speed
    elif is_key_pressed("s"):
        speed = max(-1, min(1, speed - diff_speed))
        action.speed = speed
    else:
        if abs(speed) < diff_speed / 4:
            speed = 0
        if speed > 0:
            speed -= diff_speed / 4
        elif speed < 0:
            speed += diff_speed / 4
        action.speed = pred[0]
    
    # not needded for the track 1
    # if action.speed > 0.4:
    #     action.speed = 0.4
    
    return action

simulation.run(step, step_count=-1)
simulation.exit()
