from unittest import mock

from common.interface import Config, State
from device.device import CartPoleDevice, DeviceTarget

EPS = 1e-6


class TestCartPoleDevice:
    @staticmethod
    def get_device():
        return CartPoleDevice(interface=mock.MagicMock())

    def test_reset(self):
        op = self.get_device()
        op.reset(Config())
        assert op.interface.reset.called_once

    def test_get_state(self):
        op = self.get_device()
        op.get_state()

        assert op.interface.get.called_once
        assert len(op.interface.get.call_args.args) == 1
        assert isinstance(op.interface.get.call_args.args[0], State)

    def test_get_target(self):
        op = self.get_device()
        op.get_target()

        assert op.interface.get.called_once
        assert len(op.interface.get.call_args.args) == 1
        assert isinstance(op.interface.get.call_args.args[0], DeviceTarget)

    def test_set_target(self):
        op = self.get_device()
        target = 1
        op.set_target(target)

        assert op.interface.set.called_once
        assert len(op.interface.set.call_args.args) == 1
        arg = op.interface.set.call_args.args[0]
        assert isinstance(arg, DeviceTarget)
        assert arg.position == target
