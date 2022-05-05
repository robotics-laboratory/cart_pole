from unittest import mock

from cartpole.common.interface import State, Config
from cartpole.device import CartPoleDevice


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
        assert isinstance(op.interface.get.call_args.args[0], Target)

    def test_set_target(self):
        op = self.get_device()
        target = 1
        op.set_target(target)

        assert op.interface.set.called_once
        assert len(op.interface.set.call_args.args) == 1
        arg = op.interface.set.call_args.args[0]
        assert isinstance(arg, Target)
        assert arg.position == target
