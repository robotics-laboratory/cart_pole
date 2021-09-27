import dataclasses as dc
import enum


class Error(enum.Enum):
    NO_ERROR = 0
    NEED_RESET = 1
    X_OVERFLOW = 2
    V_OVERFLOW = 3
    A_OVERFLOW = 4
    MOTOR_STALLED = 5
    ENDSTOP_HIT = 6

    def __bool__(self) -> bool:
        '''
        Returns:
            True if there is any error.
        '''

        return self != Error.NO_ERROR


@dc.dataclass
class Config:
    max_position: float = dc.field(default=None)
    max_velocity: float = dc.field(default=None)
    max_acceleration: float = dc.field(default=None)
    hardware_max_position: float = dc.field(default=None)
    hardware_max_velocity: float = dc.field(default=None)
    hardware_max_acceleration: float = dc.field(default=None)
    clamp_position: bool = dc.field(default=None)
    clamp_velocity: bool = dc.field(default=None)
    clamp_acceleration: bool = dc.field(default=None)

    @staticmethod
    def default():
        return Config(
            max_position=0.19,
            max_velocity=10.0,
            max_acceleration=10.0,
            clamp_position=False,
            clamp_velocity=False,
            clamp_acceleration=False,
        )


@dc.dataclass
class State:
    position: float = dc.field(default=None)
    velocity: float = dc.field(default=None)
    acceleration: float = dc.field(default=None)
    pole_angle: float = dc.field(default=None)
    pole_angular_velocity: float = dc.field(default=None)
    error_code: Error = dc.field(default=None)



class CartPoleBase:
    def reset(self, config: Config) -> None:
        '''
        Resets the device to the initial state.
        The pole is at rest position and cart is centered.
        It must be called at the beginning of any session.
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
