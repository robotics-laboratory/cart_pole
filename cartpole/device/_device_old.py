import logging
import math
import time

from cartpole.common import util
from cartpole.common.interface import (
    Config,
    State,
    CartPoleBase,
)
from cartpole.device.wire_interface import (
    WireInterface,
    ProtobufWireInterface,
    DeviceTarget,
    DeviceConfig,
    DeviceState,
)


LOGGER = logging.getLogger(__name__)


class CartPoleDevice(CartPoleBase):
    def __init__(self, interface: WireInterface = None, target_key='acceleration') -> None:
        self.interface = interface or ProtobufWireInterface()
        self.step_count = 0
        self.target_key = target_key
        self.prev_angle = 0
        self.rotations = 0

    def reset(self, config: Config = None) -> None:
        self.interface.reset()
        config = config or Config.default()
        config.__class__ = DeviceConfig
        self.interface.set(config)
        self.step_count = 0

    def get_state(self) -> State:
        new_state: State = self.interface.get(DeviceState.full())
        curr = new_state.pole_angle
        prev = self.prev_angle
        max_delta = math.pi

        delta = curr - prev
        if delta > max_delta:
            self.rotations -= 1
        elif delta < -max_delta:
            self.rotations += 1
        abs_angle = 2 * math.pi * self.rotations + curr
        # abs_pole_angles.append(abs_angle)
        # rotations_arr.append(2 * math.pi * rotations)
        self.prev_angle = curr

        new_state.pole_angle = abs_angle
        return new_state

    cart_acceleration: float = 0
    def get_info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def get_target(self) -> float:
        request = DeviceTarget(**{self.target_key: True})
        target = self.interface.get(request)
        return getattr(target, self.target_key)

    def set_target(self, target: float) -> None:
        request = DeviceTarget(**{self.target_key: target})
        self.interface.set(request)

    def timestamp(self):
        return time.time()

    def advance(self, delta) -> None:
        pass  # TODO: ???

    def close(self) -> None:
        self.interface.close()
