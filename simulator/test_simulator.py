import unittest

from common.interface import Config, Error, State
from simulator import CartPoleSimulator, PhysicalParams


def eval_path_length(velocity, acceleration, step_n, delta_time):
    '''
    Evaluate path with uniform acceleration and discrete speed control.
    Args:
        * velocity - start velocity
        * acceleration - constant acceleration
        * step_n - number of discrete velocity changes
        * delta_time - time delta between velocities changes
    '''
    duration = step_n * delta_time
    return velocity * duration + (duration + delta_time) * duration / 2 * acceleration

class TestCaseBase(unittest.TestCase):
    def setUp(self):
        phisical_params = PhysicalParams()
        config = Config.default()
        simulator = CartPoleSimulator()

        simulator.reset_physical_params(phisical_params)
        simulator.reset(config)

        self.physical_params = phisical_params
        self.config = config
        self.simulator = simulator

    def tearDown(self):
        self.simulator.close()

    def assert_equal_float(self, first, second, delta=1e-6):
        self.assertAlmostEqual(first, second, delta=delta)

    def assert_equal_state(self, first, second, delta=1e-6):
        self.assert_equal_float(first.position, second.position, delta=delta)
        self.assert_equal_float(first.velocity, second.velocity, delta=delta)
        self.assert_equal_float(first.acceleration, second.acceleration, delta=delta)

        self.assert_equal_float(first.pole_angle, second.pole_angle, delta=delta)
        self.assert_equal_float(
            first.pole_angular_velocity,
            second.pole_angular_velocity,
            delta=delta
        )

        self.assertEqual(first.error_code, second.error_code)


class ResetTest(TestCaseBase):
    def test_state(self):
        '''
        After reset pole is in thet rest state, cart at the x=0 and has no velocity.
        '''

        expected = State(
            position=0,
            velocity=0,
            acceleration=0,
            pole_angle=0,
            pole_angular_velocity=0,
            error_code=Error.NO_ERROR
        )

        self.assert_equal_state(self.simulator.get_state(), expected)

    def test_target(self):
        "Also cart has no acceleration target"

        self.assert_equal_float(self.simulator.get_target(), 0.0)


class AccelerationTest(TestCaseBase):
    def test_single_step_acc(self):
        '''
        The simulation is run by small discrete steps.
        When we set the target acceleration, it changes instantly.
        The cart velocity changes discretely step by step.
        '''

        simulator = self.simulator

        before = simulator.get_state()

        target = 0.5
        simulator.set_target(target)

        self.assert_equal_float(simulator.get_target(), target)

        state = simulator.get_state()
        self.assert_equal_float(state.position, before.position)
        self.assert_equal_float(state.velocity, before.velocity)
        self.assert_equal_float(state.acceleration, target)

        simulator.make_step()

        delta_t = self.physical_params.time_step
        delta_v = target * delta_t
        delta_x = delta_v * delta_t

        self.assert_equal_float(simulator.get_target(), target)

        state = simulator.get_state()
        self.assert_equal_float(state.acceleration, target)
        self.assert_equal_float(state.velocity, before.velocity + delta_v)
        self.assert_equal_float(state.position, before.position + delta_x)


    def test_multiple_step_acc(self):
        '''
        The continuous simulation is run as numerical integration of base steps.
        The cart moves with the required acceleration (if restrictions allow).
        '''

        time_step = self.physical_params.time_step
        simulator = self.simulator

        before = simulator.get_state()

        target = 0.1
        simulator.set_target(target)
        self.assert_equal_float(simulator.get_target(), target)

        state = simulator.get_state()
        self.assert_equal_float(state.position, before.position)
        self.assert_equal_float(state.velocity, before.velocity)
        self.assert_equal_float(state.acceleration, target)

        delta_t, step_n = simulator.run(1.0)

        delta_v = target * delta_t
        delta_x = eval_path_length(state.velocity, target, step_n, time_step)

        self.assert_equal_float(simulator.get_target(), target)

        state = simulator.get_state()
        self.assert_equal_float(state.acceleration, target)
        self.assert_equal_float(state.velocity, before.velocity + delta_v)
        self.assert_equal_float(state.position, before.position + delta_x)

    def test_complex_acc(self):
        '''
        A sequential series of accelerations.
        1. acceleration
        2. uniform velocity
        3. full braking
        '''

        time_step = self.physical_params.time_step
        simulator = self.simulator

        before = simulator.get_state()

        # acceleration
        target = 0.2
        simulator.set_target(target)

        delta_t, step_n = simulator.run(0.3)
        delta_v = target * delta_t
        delta_x = eval_path_length(before.velocity, target, step_n, time_step)

        state = simulator.get_state()
        self.assert_equal_float(state.acceleration, target)
        self.assert_equal_float(state.velocity, before.velocity + delta_v)
        self.assert_equal_float(state.position, before.position + delta_x)

        before = state

        # uniform
        target = 0.0
        simulator.set_target(target)
        delta_t, step_n = simulator.run(0.3)
        delta_v = target * delta_t
        delta_x = eval_path_length(before.velocity, target, step_n, time_step)

        state = simulator.get_state()
        self.assert_equal_float(state.acceleration, target)
        self.assert_equal_float(state.velocity, before.velocity + delta_v)
        self.assert_equal_float(state.position, before.position + delta_x)

        before = state

        # breaking
        target = 0.6
        simulator.set_target(target)
        delta_t, step_n = simulator.run(0.1)
        delta_v = target * delta_t
        delta_x = eval_path_length(before.velocity, target, step_n, time_step)

        state = simulator.get_state()
        self.assert_equal_float(state.acceleration, target)
        self.assert_equal_float(state.velocity, before.velocity + delta_v)
        self.assert_equal_float(state.position, before.position + delta_x)

class AccelerationClampTest(unittest.TestCase):
    def test_clamp(self):
        '''
        Simulator limits acceleration with clipping.
        '''

        simulator = CartPoleSimulator()
        simulator.reset_physical_params(PhysicalParams())

        config = Config.default()
        config.max_acceleration = 1.0
        config.clamp_acceleration = True
        simulator.reset(config)

        simulator.set_target(config.max_acceleration + 1)
        self.assertAlmostEqual(simulator.get_target(), config.max_acceleration)

        state = simulator.get_state()
        self.assertEqual(state.error_code, Error.NO_ERROR)

    def test_error(self):
        '''
        Simulator limits acceleration throwing out an error.
        '''

        simulator = CartPoleSimulator()
        simulator.reset_physical_params(PhysicalParams())

        config = Config.default()
        config.max_acceleration = 1.0
        config.clamp_acceleration = False
        simulator.reset(config)

        simulator.set_target(config.max_acceleration + 1)
        state = simulator.get_state()
        self.assertEqual(state.error_code, Error.A_OVERFLOW)

class VelocityClampTest(unittest.TestCase):
    def test_error(self):
        '''
        Simulator limits velocity throwing out an error.
        '''

        simulator = CartPoleSimulator()
        simulator.reset_physical_params(PhysicalParams())

        config = Config.default()
        config.max_acceleration = 10.0
        config.max_velocity = 1.0
        config.clamp_velocity = False
        simulator.reset(config)

        simulator.set_target(config.max_acceleration)
        simulator.run(10)
        state = simulator.get_state()
        self.assertEqual(state.error_code, Error.V_OVERFLOW)

class PositionClampTest(unittest.TestCase):
    def test_error(self):
        '''
        Simulator limits position throwing out an error.
        '''

        simulator = CartPoleSimulator()
        simulator.reset_physical_params(PhysicalParams())

        config = Config.default()
        config.max_acceleration = 10.0
        config.max_velocity = 10.0
        config.clamp_position = False
        simulator.reset(config)

        simulator.set_target(config.max_acceleration)
        simulator.run(10)
        state = simulator.get_state()
        self.assertEqual(state.error_code, Error.X_OVERFLOW)
