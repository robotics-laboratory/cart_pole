from cartpole.common.interface import State
from cartpole.sessions.actor import Actor
from cartpole.control import BalanceLQRControl, TrajectoryLQRControl, Trajectory
from logging import getLogger

logger = getLogger(__file__)

class DemoActor(Actor):
    def __init__(self, config, **kwargs):
        super().__init__(**kwargs)
        logger.info("Calculating trajectory...")
        init = State.home()
        self.trajectory = Trajectory(config, init)

        self.balance_control = BalanceLQRControl(config)
        self.trajectory_control = TrajectoryLQRControl(config, self.trajectory)
        self.begin = None
        self.proxy = None
        logger.info("Trajectory ready")

    def __call__(self, state: State, stamp=None) -> float:
        self.save_expected_state(stamp)
        if stamp < self.trajectory.duration:
            target = self.trajectory_control(stamp, state)
        else:
            target = self.balance_control(state)
        return float(target)

    def save_expected_state(self, stamp):
        state_expected, target_expected = self.trajectory(stamp)
        ts = self.proxy._timestamp()
        self.proxy._add_value('expected.cart_position', ts, float(state_expected.cart_position))
        self.proxy._add_value('expected.cart_velocity', ts, float(state_expected.cart_velocity))
        self.proxy._add_value('expected.pole_angle', ts, float(state_expected.pole_angle))
        self.proxy._add_value('expected.pole_angular_velocity', ts, float(state_expected.pole_angular_velocity))
        self.proxy._add_value('expected.target', ts, float(target_expected))
