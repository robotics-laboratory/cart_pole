import enum
from typing import Tuple

import numpy as np


class Error(enum.IntEnum):
    NO_CONTROL = -1
    NO_ERROR = 0
    NOT_INITIALIZED = 2
    INVALID_CART_POSITION = 3
    INVALID_CART_VELOCITY = 4

    def __bool__(self):
        '''
        Returns:
        True if there is any error.
        '''
        return self.value != Error.NO_ERROR

class State:
    """
    Full state of cart-pole device.

    - cart_positon (m):
        Position of the cart along the guide axis.
        The middle is a reference point.

    - cart_velocity (m/s):
        Instantaneous linear speed of the cart.

    - pole_angle (rad):
        Angle of pole (ccw). The he lowest position is the reference point. 

    - pole_velocity (rad/s):
        Instantaneous angular velocity of the pole.

    - error (enum):
        Error codes describing the state of the cart-pole.
    """

    def __init__(self,
            cart_position,
            cart_velocity,
            pole_angle,
            pole_velocity,
            error=Error.NO_ERROR):
        self.cart_position = cart_position
        self.cart_velocity = cart_velocity

        self.pole_angle = pole_angle
        self.pole_velocity = pole_velocity

        self.error = error

    def __bool__(self):
        '''
        Returns:
        True if there is no any error
        '''
        return not self.error

    def __repr__(self):
        return f'(cart_position={self.cart_position}, ' + \
            f'cart_velocity={self.cart_velocity}, ' + \
            f'pole_angle={self.pole_angle}, ' + \
            f'pole_velocity={self.pole_velocity}, ' + \
            f'error={self.error})'

    def as_np_vector(self):
        return np.array([self.cart_position, self.cart_velocity,
                         self.pole_angle, self.pole_velocity])


class ActionType(enum.IntEnum):
    POSITION_CONTROL = 1
    VELOCITY_CONTROL = 2
    ACCELERATION_CONTROL = 3

    def target_name(self):
        if self == ActionType.POSITION_CONTROL:
            return 'pos'
        elif self == ActionType.VELOCITY_CONTROL:
            return 'vel'
        elif self == ActionType.ACCELERATION_CONTROL:
            return "acc"
        else:
            raise NotImplementedError

class CartPoleConfig:
    '''
    Common parameters for any cart pole implementation.
    '''

    def __init__(self,
        cart_position_limit: float = 0.15, # m
        cart_velocity_limit: float = 1, # m/s
        cart_acceleration_limit: float = 1, # m/s^2
        action_type: ActionType = ActionType.POSITION_CONTROL,
        clamp_position: bool = True):
        '''
        * cart_position_limit – max allowed distance from cart center to homing position.
        * cart_velocity_limit – max allowed absolute cart velocity.
        * cart_velocity_limit – max allowed absolute cart acceleration.
        * clamp_position – allow to clamp current position or raise exception if limits are exceeded.
        '''
        self.cart_position_limit = cart_position_limit
        self.cart_velocity_limit = cart_velocity_limit
        self.cart_acceleration_limit = cart_acceleration_limit
        self.action_type = action_type
        self.clamp_position = clamp_position

        assert action_type == ActionType.POSITION_CONTROL


class CartPoleBase:
    def reset(self, config: CartPoleConfig) -> Tuple[State, float]:
        '''
        Resets the device to the initial state.
        The pole is at rest position and cart is centered.
        It must be called at the beginning of any session.

        Returns (initial_state, initial_target).
        '''
        raise NotImplementedError

    def step(self, delta: float) -> None:
        '''
        Make delta time step (delta > 1/240 seconds).
        In case of real control it's a fake call.
        '''
        raise NotImplementedError

    def get_state(self) -> State:
        '''
        Returns current device state.
        '''
        raise NotImplementedError

    def get_info(self) -> dict:
        '''
        Returns usefull debug information.
        '''
        raise NotImplementedError

    def get_target(self) -> float:
        '''
        Returns current target value.
        '''
        raise NotImplementedError

    def set_target(self, target: float) -> None:
        '''
        Set desired target value.
        '''
        raise NotImplementedError

    def close(self) -> None:
        '''
        Free all allocated resources.
        '''
        raise NotImplementedError
