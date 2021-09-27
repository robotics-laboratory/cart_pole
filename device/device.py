import logging

from common import util
from common.interface import (
    Config,
    State,
    CartPoleBase,
)
from device.wire_interface import (
    WireInterface,
    DeviceTarget,
    DeviceConfig,
    DeviceState,
)


LOGGER = logging.getLogger(__name__)


class CartPoleDevice(CartPoleBase):
    def __init__(self, interface: WireInterface = None) -> None:
        self.interface = interface or WireInterface()
        self.step_count = 0

    def reset(self, config: Config) -> None:
        config.__class__ = DeviceConfig
        self.interface.reset()
        self.interface.set(config)
        self.step_count = 0

    def get_state(self) -> State:
        return self.interface.get(DeviceState.full())

    def get_info(self) -> dict:
        return {util.STEP_COUNT: self.step_count}

    def get_target(self) -> float:
        return self.interface.get(DeviceTarget(position=True)).position

    def set_target(self, target: float) -> None:
        _ = self.interface.set(DeviceTarget(position=target))

    def close(self) -> None:
        self.interface.close()
