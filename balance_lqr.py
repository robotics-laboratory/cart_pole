from control import LinearBalanceControl
from interface import Config, Error, State
from simulator import CartPoleSimulator, PhysicalParams

import time
import math

control = LinearBalanceControl()

env = CartPoleSimulator(debug_mode=True)
env.reset_physical_params(PhysicalParams())
env.reset(Config.default())


def wait(eps):
    while True:
        state = env.get_state()
        if (state.pole_angle - math.pi) < eps:
            return

        time.sleep(0.1)


def run(eps):
    while True:
        state = env.get_state()
        if (state.pole_angle - math.pi) > eps:
            env.reset()
            return

        target = control(state)
        env.set_target(target)
        env.make_step()

eps = 0.2
wait(eps)
run(eps)