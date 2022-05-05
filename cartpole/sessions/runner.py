from cartpole.common.interface import CartPoleBase
import logging
from typing import Type
from cartpole.sessions.actor import Actor
from cartpole.sessions.collector import CollectorProxy
from cartpole.web_view import server


LOGGER = logging.getLogger(__name__)


class Runner:
    def __init__(
        self,
        cart_pole: CartPoleBase,
        cart_pole_config: dict,
        actor_class: Type[Actor],
        actor_config: dict,
    ) -> None:
        self.proxy = CollectorProxy(cart_pole, actor_class, actor_config)
        self.cart_pole_config = cart_pole_config
        self.actor_class = actor_class
        self.actor_config = actor_config

    def run(self, max_iterations: int = -1) -> None:
        try:
            self.proxy.reset(self.cart_pole_config)
            self._loop(max_iterations)
        except Exception:
            LOGGER.exception('Aborting run due to error')
        finally:
            self.proxy.close()
            LOGGER.info('Run finished')

    def start_server(self) -> None:
        server.run_server(self.proxy)

    def _loop(self, max_iterations: int) -> None:
        actor = self.actor_class(**self.actor_config)
        current_iteration = 0
        while current_iteration != max_iterations:
            with self.proxy.time_trace('iteration'):
                state = self.proxy.get_state()
                target = actor(state)
                self.proxy.set_target(target)
                self.proxy.make_step()
            current_iteration += 1
