import pydrake

from pydrake.solvers.mathematicalprogram import Solve
from pydrake.systems.primitives import Linearize
from pydrake.systems.trajectory_optimization import DirectCollocation
from pydrake.trajectories import PiecewisePolynomial

from common import Config, Error, State
from simulator.system import CartPoleSystem

import math
import numpy


def build_trajectory(config, initial_state, sample_n=100, max_duration=15):
    system = CartPoleSystem()
    context = system.CreateContext(config, initial_state.as_array())

    program = DirectCollocation(
        system,
        context,
        num_time_samples=sample_n,
        minimum_timestep=0.001,
        maximum_timestep=0.1,
        input_port_index=system.get_input_port().get_index())

    program.AddEqualTimeIntervalsConstraints()
    program.AddDurationBounds(0, max_duration)

    program.prog().AddBoundingBoxConstraint(
        initial_state.as_array(),
        initial_state.as_array(),
        program.initial_state())

    target_x_max = config.max_position * 0.75

    program.prog().AddBoundingBoxConstraint(
        [-target_x_max, math.pi, 0.0, 0.0],
        [+target_x_max, math.pi, 0.0, 0.0],
        program.final_state())

    q = program.state()
    x, a, v, w = q[0], q[1], q[2], q[3]

    program.AddConstraintToAllKnotPoints(-config.max_position <= x)
    program.AddConstraintToAllKnotPoints(x <= +config.max_position)

    program.AddConstraintToAllKnotPoints(-config.max_velocity <= v)
    program.AddConstraintToAllKnotPoints(v <= +config.max_velocity)

    u = program.input()[0]
    program.AddConstraintToAllKnotPoints(-config.max_acceleration <= u)
    program.AddConstraintToAllKnotPoints(u <= config.max_acceleration)

    program.AddRunningCost(u ** 2)
    program.AddRunningCost(program.time())
    # program.AddFinalCost(u**2)
    program.AddFinalCost(x ** 2)

    result = Solve(program.prog())
    assert result.is_success(), 'Impossible find trajectory'

    targets = program.ReconstructInputTrajectory(result)
    states = program.ReconstructStateTrajectory(result)

    return states, targets


class Trajectory:
    def __init__(self, config, initial_state, sample_n=100, max_duration=5):
        states, targets = build_trajectory(config, initial_state, sample_n, max_duration)
        self.states = states
        self.targets = targets
        self.duration = targets.end_time() - targets.start_time()

    def sample(self, sample_n):
        time_steps = numpy.linspace(self.states.start_time(), self.states.end_time(), sample_n)
        state_values = self.states.vector_values(time_steps).transpose()
        target_values = self.targets.vector_values(time_steps).transpose()

        return time_steps, state_values, target_values

    def __call__(self, timestamp):
        state = self.states.value(timestamp)
        target = self.targets.value(timestamp)

        return State.from_array(state), target[0]
