from cartpole.common.interface import State
from cartpole.sessions.actor import Actor


class ConstantActor(Actor):
    def __init__(self, target_const: float = 0, **kwargs):
        super().__init__(**kwargs)
        self.target_const = target_const

    def __call__(self, state: State, stamp=None) -> float:
        return self.target_const
