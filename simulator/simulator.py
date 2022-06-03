from pydrake.systems.analysis import Simulator

from common import CartPoleBase, Error, State
from simulator.system import CartPoleSystem

import logging
import numpy

log = logging.getLogger('simulator')


def clamp(value, limit):
    if value > limit:
        return limit, True

    if value < -limit:
        return -limit, True

    return value, False


class CartPoleSimulator(CartPoleBase):
    '''
    Description:
        Сlass implements a physical simulation of the cart-pole device.
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing the cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    Motor simulation:
        Current implementation is specified for stepper motor.
        It is controlled by discrete velocity changes every time step.
        The target is desired acceleration of cart.
    Technical details:
        Each environment runs its own isolated pybullet physics engine.
    '''

    def __init__(self):
        self.system = CartPoleSystem()
        self.context = self.system.CreateDefaultContext()
        self.simulator = Simulator(self.system, self.context)
        self.config = None
        self.error = Error.NEED_RESET  # Formally, we need reset env to reset error.
        self.target_acceleration = 0

    def reset_to(self, config, state):
        self.config = config
        self.context = self.system.CreateContext(config, state.as_array())
        self.simulator = Simulator(self.system, self.context)
        self.error = Error.NO_ERROR
        self.target_acceleration = 0
        self.system.get_input_port().FixValue(self.context, numpy.array([0]))

    def reset(self, config):
        self.reset_to(config, State.home())

    def get_state(self):
        return State.from_array(
            self.context.get_continuous_state_vector()
        )

    def get_target(self):
        return self.target_acceleration

    def get_config(self):
        assert self.config
        return self.config

    def set_target(self, target):
        if self.error:
            log.warning('set target, error=%s', self.error)
            return

        config = self.get_config()

        self.target_acceleration, clamped = clamp(target, config.hard_max_acceleration)
        self.system.get_input_port().FixValue(self.context, numpy.array([target]))

        if clamped:
            self.error = Error.A_OVERFLOW
            return

        log.debug('set acc=%.2f', target)

    def validate(self):
        state = self.get_state()
        config = self.get_config()

        if abs(state.cart_position) > config.hard_max_position:
            self.error = Error.X_OVERFLOW
            return False

        if abs(state.cart_velocity) > config.hard_max_velocity:
            self.error = Error.V_OVERFLOW
            return False

        return True

    def advance(self, delta):
        if self.error:
            log.warning('advance, error=%i', self.error)
            return

        self.simulator.AdvanceTo(self.context.get_time() + delta)

        if not self.validate():
            return

    def timestamp(self):
        return self.context.get_time()

    def get_info(self):
        return {
            'time': self.context.GetTime(),
        }

    def close(self):
        pass
