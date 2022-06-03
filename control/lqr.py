from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.controllers import FiniteHorizonLinearQuadraticRegulator
from pydrake.systems.controllers import FiniteHorizonLinearQuadraticRegulatorOptions
from pydrake.systems.primitives import Linearize

from common import Config, Error, State
from simulator.system import CartPoleSystem

import math
import numpy


class BalanceLQRControl:
    def __init__(self, config):
        state = State(
            cart_position=0,
            cart_velocity=0,
            pole_angle=math.pi,
            pole_angular_velocity=0
        )

        self.q0 = state.as_array()
        self.u0 = numpy.array([0])

        system = CartPoleSystem()
        context = system.CreateContext(config, self.q0)
        system.get_input_port().FixValue(context, self.u0)

        # Q = numpy.diag([1, 13, 1, 4])
        # R = numpy.diag([0.18])
        Q = numpy.diag([10.0, 1, 1, 1])
        R = numpy.diag([0.5])

        linearized = Linearize(system, context)
        self.K, _ = LinearQuadraticRegulator(linearized.A(), linearized.B(), Q, R)

    def __call__(self, state):
        q = state.as_array()
        error = q - self.q0
        u = -self.K @ error
        return u[0]


class TrajectoryLQRControl:
    def __init__(self, config, trajectory):
        Q = numpy.diag([1, 1, 1, 1])
        R = numpy.diag([1])

        options = FiniteHorizonLinearQuadraticRegulatorOptions()
        options.x0 = trajectory.states
        options.u0 = trajectory.targets
        options.Qf = Q

        q0 = State()

        system = CartPoleSystem()
        context = system.CreateContext(config, q0.as_array())

        self.trajectory = trajectory
        self.regulator = FiniteHorizonLinearQuadraticRegulator(
            system,
            context,
            t0=options.u0.start_time(),
            tf=options.u0.end_time(),
            Q=Q,
            R=R,
            options=options
        )

    def __call__(self, stamp, state):
        q = state.as_array_4x1()
        q0 = self.trajectory.states.value(stamp)
        u0 = self.trajectory.targets.value(stamp)

        error = q - q0
        K = self.regulator.K.value(stamp)
        u = u0 - K @ error

        return u[0]
