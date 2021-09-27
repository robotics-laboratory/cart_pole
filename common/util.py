import math

from common.interface import State

STEP_COUNT = 'step_count'

def reward(state: State) -> float:
    return math.exp(-math.cos(state.pole_angle))
