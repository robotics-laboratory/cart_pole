from common import Config, State, generate_pyplot_animation
from control import BalanceLQRControl, Trajectory, TrajectoryLQRControl
from simulator import CartPoleSimulator
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import HTML
from control import calibration
from math import sin, cos


def main():
    config = Config()
    real_config = Config()
    real_config.pole_length = 0.4
    
    config.pole_length = calibration.calibrate(real_config)
    print("Calibrated pole length = " + str(config.pole_length))
    
    init = State.home()
    cart_pole = CartPoleSimulator()
    cart_pole.reset(config)
    
    trajectory = Trajectory(config, init)
    balance_control = BalanceLQRControl(config)
    trajectory_control = TrajectoryLQRControl(config, trajectory)

    timestamps, states, targets = [], [], []
    states_expected, targets_expected = [], []
    start = cart_pole.timestamp()

    for _ in range(150):
        state = cart_pole.get_state()
        stamp = cart_pole.timestamp() - start

        if stamp < trajectory.duration:
            target = trajectory_control(stamp, state)
        else:
            target = balance_control(state)

        cart_pole.set_target(target)
        cart_pole.advance(0.01)

        timestamps.append(stamp)
        states.append(state)
        targets.append(target)

        state_expected, target_expected = trajectory(stamp)

        states_expected.append(state_expected)
        targets_expected.append(target_expected)
        
    anim = generate_pyplot_animation(config, states, states_expected)
    HTML(anim.to_jshtml())
    
    
if __name__ == "__main__":
    main()

