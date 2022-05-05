from cartpole.common.interface import State
from cartpole.sessions.actor import Actor


class OscillatingActor(Actor):
    '''Sample actor that's trying to oscillate in given X range'''

    def __init__(self, device_config, acceleration, max_position, **_):
        super().__init__()
        self.direction = True  # Right
        self.a = acceleration
        self.max_x = max_position
        self.max_v = device_config.max_velocity

    def __call__(self, state: State, stamp=None):
        if self.direction:
            if state.position < self.max_x:
                return self.a if state.velocity < self.max_v else 0
            else:
                self.direction = False
                return self(state)
        else:
            if state.position > -self.max_x:
                return -self.a if -state.velocity < self.max_v else 0
            else:
                self.direction = True
                return self(state)


# if __name__ == '__main__':
#     from pprint import pprint
#
#     from interface import Config
#     from misc.session_helpers import get_real_device, init_logging, run_session
#
#     init_logging()
#     device = get_real_device()
#     device_config = Config(max_velocity=0.5, max_acceleration=5)
#     actor_class = OscillatingActor
#     actor_params = dict(acceleration=1.0, max_position=0.05)
#     data = run_session(device, device_config, actor_class, actor_params, max_iters=1000)
#     pprint(data)
