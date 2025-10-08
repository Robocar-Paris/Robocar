# from mlagents_envs.logging_util import set_log_level, DEBUG
# set_log_level(DEBUG)
from lib.robocar_simulation import simulation, Observation, Action

action = Action(1, 0.1)
def step(frame_index: int, observation: Observation) -> Action:
    print(f"frame {frame_index}:")
    print(observation)
    return action


simulation.run(step, step_count=-1)
simulation.exit()
