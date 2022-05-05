import pydrake

from pydrake.solvers.mathematicalprogram import Solve
from pydrake.systems.trajectory_optimization import DirectCollocation

from cartpole.simulator.system import CartPoleSystem

import math
import numpy

def build_trajectory(config, initial_state, sample_n=100, duration=5):
    system = CartPoleSystem()
    context = system.CreateContext(config, initial_state)
        
    program = DirectCollocation(
        system,
        context,
        num_time_samples=sample_n,
        minimum_timestep=0.001,
        maximum_timestep=0.1,
        input_port_index=system.get_input_port().get_index())

    program.AddEqualTimeIntervalsConstraints()
    program.AddDurationBounds(0, duration)

    program.AddBoundingBoxConstraint(
        initial_state.as_array(),
        initial_state.as_array(),
        program.initial_state())

    target_x_max = config.config.max_position * 0.75

    program.AddBoundingBoxConstraint(
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

    program.AddRunningCost(u**2)
    program.AddRunningCost(program.time())
    # program.AddFinalCost(u**2)
    program.AddFinalCost(x**2)

    result = Solve(program)
    assert result.is_success()

    state_trajectory = program.ReconstructStateTrajectory(result)
    target_trajectory = program.ReconstructInputTrajectory(result)

    return state_trajectory, target_trajectory



class Trajectory:
    def __init__(self, config, initial_state, sample_n=100, max_duration=5):
        states, targets = build_trajectory(config, initial_state, sample_n=100, max_duration=5)
        self.states = states
        self.targets = targets

    def sammple(self, sample_n):
        time_steps = numpy.linspace(self.states.start_time(), self.states.end_time(), sample_n)
        state_values = self.states.vector_values(time_steps).transpose()
        target_values = self.targets.vector_values(time_steps).transpose()

        return time_steps, state_values, target_values
