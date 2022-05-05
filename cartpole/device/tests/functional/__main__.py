#!/usr/bin/env python

'''
Functional tests for cart pole device serial interface and logic.

At the beginning of most tests it is presumed that caret is centered
as if right after homing procedure.
'''

import logging
import time

from device.wire_interface import (
    WireInterface, 
    DeviceTarget, 
    DeviceState,
    DeviceConfig,
)
from common.interface import Error


LOGGER = logging.getLogger(__name__)
EPS = 1e-3


def test_homing(interface: WireInterface) -> None:
    interface.reset()
    time.sleep(0.5)
    s: DeviceState = interface.get(DeviceState.full())
    assert abs(s.position) < EPS
    assert abs(s.velocity) < EPS
    assert abs(s.acceleration) < EPS
    # assert abs(s.pole_angle) < EPS
    # assert abs(s.pole_angular_velocity) < EPS
    assert s.error_code == Error.NO_ERROR
    
    t: DeviceTarget = interface.get(DeviceTarget.full())
    assert abs(t.position) < EPS
    assert abs(t.velocity) < EPS
    assert abs(t.acceleration) < EPS
    

def test_move_by_acceleration(interface: WireInterface) -> None:
    a = 0.4
    t = DeviceTarget(acceleration=a)    
    for _ in range(20):
        interface.set(t)
        
        for i in range(5):
            time.sleep(0.1)
            _ = interface.get(DeviceState.full())

        t.acceleration = -t.acceleration
        interface.set(t)

        for i in range(5):
            time.sleep(0.1)
            _ = interface.get(DeviceState.full())

    interface.set(DeviceTarget(position=0))


def test_command_logging(interface: WireInterface) -> None:
    pass


def test_run_left_right(interface: WireInterface) -> None:
    c: DeviceConfig = interface.get(DeviceConfig(max_position=True))
    x = c.max_position * 0.9
    acceptable_target = c.max_position * 0.7
    interface.set(DeviceTarget(position=x))
    time.sleep(1)

    s: DeviceState = interface.get(DeviceState(position=True))
    assert s.position > acceptable_target, f'Run to right has not achieved DeviceTarget position of {acceptable_target}'

    interface.set(DeviceTarget(position=-x))
    time.sleep(2)

    s: DeviceState = interface.get(DeviceState(position=True))
    assert s.position < -acceptable_target, f'Run to left has not achieved DeviceTarget position of {-acceptable_target}'

    interface.set(DeviceTarget(position=0))
    time.sleep(1)

    s: DeviceState = interface.get(DeviceState(position=True))
    assert abs(s.position) < EPS, f'Run to center has not achieved DeviceTarget position of 0.0'


def test_change_max_pos_and_run(interface: WireInterface) -> None:
    try:
        interface.set(DeviceTarget(position=100500))
        assert False, 'Expected RuntimeError raised'
    except RuntimeError:
        pass
    interface.reset()  # reset to drop error code


def test_change_speed_and_run(interface: WireInterface) -> None:
    c: DeviceConfig = interface.get(DeviceConfig(max_position=True, max_velocity=True, max_acceleration=True))
    target = c.max_position * 0.9

    velocity = c.max_velocity * 0.25
    increased_velocity = velocity * 2
    decreased_velocity = velocity * 0.5

    run_delay = 0.75
    run_back_delay = 2

    # set baseline velocity
    interface.set(DeviceConfig(max_velocity=velocity))

    # make a run with basic velocity constraint
    interface.set(DeviceTarget(position=target))
    time.sleep(run_delay)
    s: DeviceState = interface.get(DeviceState(position=True, velocity=True, acceleration=True))
    interface.set(DeviceTarget(position=0.0))
    baseline_position = s.position
    time.sleep(run_back_delay)

    LOGGER.info(f'Baseline position = {baseline_position}')

    # increase velocity
    interface.set(DeviceConfig(max_velocity=increased_velocity))

    # make a run with increased velocity constraint
    interface.set(DeviceTarget(position=target))
    time.sleep(run_delay)
    s: DeviceState = interface.get(DeviceState(position=True, velocity=True, acceleration=True))
    interface.set(DeviceTarget(position=0.0))
    iv_position = s.position
    time.sleep(run_back_delay)
    
    LOGGER.info(f'Increased velocity position = {iv_position}')

    # decrease velocity
    interface.set(DeviceConfig(max_velocity=decreased_velocity))

    # make a run with decreased velocity constraint
    interface.set(DeviceTarget(position=target))
    time.sleep(run_delay)
    s: DeviceState = interface.get(DeviceState(position=True, velocity=True, acceleration=True))
    interface.set(DeviceTarget(position=0.0))
    dv_position = s.position
    time.sleep(run_back_delay)

    LOGGER.info(f'Decreased velocity position = {dv_position}')

    interface.set(DeviceConfig(max_velocity=c.max_velocity, max_acceleration=c.max_acceleration))

    assert 1.75 * baseline_position < iv_position < 2.25 * baseline_position, \
        'Increased velocity position is out of expected range'

    assert 0.25 * baseline_position < dv_position < 0.75 * baseline_position, \
        'Decreased velocity position is out of expected range'


def test_x_overflow(interface: WireInterface) -> None:
    pass


def test_halt_on_error(interface: WireInterface) -> None:
    pass


def test_rapid_movement(interface: WireInterface) -> None:
    interface.set(DeviceConfig(max_velocity=1, max_acceleration=2))
    target = 0.1
    for i in range(50):
        interface.set(DeviceTarget(position=target))
        time.sleep(0.5)
        target = -target


def main():
    logging.basicConfig(level=logging.DEBUG, encoding='utf-8')
    logging.getLogger('cart_pole.operator.wire_interface').setLevel(logging.ERROR)

    interface = WireInterface()

    test_homing(interface)
    # test_move_by_acceleration(interface)
    # test_command_logging(interface)
    # test_run_left_right(interface)
    # test_change_max_pos_and_run(interface)
    # test_change_speed_and_run(interface)
    # test_x_overflow(interface)
    # test_halt_on_error(interface)
    test_rapid_movement(interface)

    interface.close()

    LOGGER.info('PASSED')


assert __name__ == '__main__'
main()
