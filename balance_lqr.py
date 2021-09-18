from control import LinearBalanceControl
from interface import Config, Error, State
from simulator import CartPoleSimulator, PhysicalParams

import time
import math
import winsound


def main(device, max_iters=500, timedelta=0.1):
    control = LinearBalanceControl()

    def wait(eps):
        while True:
            state = device.get_state()
            if abs(state.pole_angle - math.pi) < eps:
                return
            time.sleep(timedelta)

    def run(eps):
        for _ in range(max_iters):
            state = device.get_state()
            if abs(state.pole_angle - math.pi) > eps:
                # device.reset()
                return

            target = control(state)
            device.set_target(target)
            # device.make_step()
            time.sleep(timedelta)

    wait(0.2)
    print("ACTIVATING")
    for i in range(3):
        time.sleep(1)
        winsound.Beep(2000, 100)
    run(0.3)


if __name__ == '__main__':
    device = CartPoleSimulator(debug_mode=True)
    device.reset_physical_params(PhysicalParams())
    device.reset(Config.default())

    main(device)
