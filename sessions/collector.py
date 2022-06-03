import copy
import dataclasses as dc
import json
import logging
from pathlib import Path

import numpy as np
import inspect
import string
import threading
import time
from contextlib import contextmanager
from collections import defaultdict
from io import StringIO
from typing import Callable, List, Dict, Type, Tuple, Union
import dacite

from common.interface import CartPoleBase, Config, State
from common.util import init_logging
from sessions.actor import Actor


LOGGER = logging.getLogger(__name__)
HEX_DIGITS = list(string.digits + string.ascii_lowercase[:6])


def generate_id(size: int = 6):
    return ''.join(np.random.choice(HEX_DIGITS, size=size, replace=True))


class Units:
    METERS = 'm'
    METERS_PER_SECOND = 'm/s'
    METERS_PER_SECOND_SQUARED = 'm/s^2'
    RADIANS = 'rad'
    RADIANS_PER_SECOND = 'rad/s'
    UNKNOWN = '?'


@dc.dataclass
class SessionData:
    @dc.dataclass
    class SessionMeta:
        session_id: str = dc.field(default_factory=generate_id)
        name: str = None
        start_timestamp: int = None
        duration: int = None
        device_class: str = None
        device_config: Config = None
        actor_class: str = None
        actor_config: dict = None

    @dc.dataclass
    class Log:
        timestamp: int
        message: str

    @dc.dataclass
    class Value:
        id: str
        name: str
        unit: str
        x: List[float] = dc.field(default_factory=list, repr=False)
        y: List[float] = dc.field(default_factory=list, repr=False)

    @dc.dataclass
    class Group:
        name: str
        values: List[str] = dc.field(default_factory=list)

    @dc.dataclass
    class Style:
        name: str
        # todo

    @dc.dataclass
    class TimeTrace:
        action: str
        start_timestamp: int
        end_timestamp: int = None

    meta: SessionMeta = dc.field(default_factory=SessionMeta)
    values: Dict[str, Value] = dc.field(default_factory=dict)
    groups: List[Group] = dc.field(default_factory=list)
    logs: List[Log] = dc.field(default_factory=list)
    time_traces: List[TimeTrace] = dc.field(default_factory=list)

    # def __post_init__(self) -> None:
    #     V = SessionData.Value
    #     values = [
    #         V(id='state.position', name='position', unit=Units.METERS),
    #         V(id='state.velocity', name='velocity', unit=Units.METERS_PER_SECOND),
    #         V(id='state.acceleration', name='acceleration', unit=Units.METERS_PER_SECOND_SQUARED),
    #         V(id='state.pole_angle', name='angle', unit=Units.RADIANS),
    #         V(id='state.pole_velocity', name='angle', unit=Units.RADIANS_PER_SECOND),
    #         V(id='target.acceleration', name='acceleration', unit=Units.RADIANS_PER_SECOND),
    #     ]
    #     self.values = {v.id: v for v in values}
    #     self.groups = [
    #         # todo
    #     ]

    @classmethod
    def load(cls, path: Union[str, Path]) -> 'SessionData':
        with open(path) as file:
            raw_data = json.load(file)
            parse_config = dacite.Config(check_types=False)
            return dacite.from_dict(SessionData, raw_data, parse_config)


class CollectorProxy(CartPoleBase):
    LOGGING_FORMAT = (
        '%(created)f [%(levelname)s] %(name)s (%(filename)s:%(lineno)d) :: %(message)s'
    )
    HUMAN_LOGGING_FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'
    DEFAULT_SAVE_PATH = 'data/sessions'

    def __init__(
        self,
        cart_pole: CartPoleBase,
        actor_class: Type[Actor],
        actor_config: dict,
        reset_callbacks: List[Callable] = None,
        close_callbacks: List[Callable] = None,
    ) -> None:
        self.cart_pole = cart_pole
        self.actor_class = actor_class
        self.actor_config = actor_config
        self.data: SessionData = None
        self.reset_callbacks = reset_callbacks or []
        self.close_callbacks = close_callbacks or []

        self._locks = defaultdict(threading.Lock)
        self._consumed_offset = defaultdict(int)
        self._started_flag = threading.Event()
        self._available_values = threading.Semaphore(0)
        self._start_perf_timestamp = None

    def _timestamp(self):
        return time.perf_counter_ns() // 1000 - self._start_perf_timestamp

    def save(self, path=None) -> Path:
        if path is None:
            path = Path(self.DEFAULT_SAVE_PATH) / f'{self.data.meta.session_id}.json'
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            json.dump(dc.asdict(self.data), f)
        LOGGER.info('Saved session %s to %s', self.data.meta.session_id, path)
        return path

    @contextmanager
    def time_trace(self, action=None):
        if action is None:
            stack =  inspect.stack()
            action = stack[1][3] if stack[1][3] != '__enter__' else stack[2][3]
        trace = SessionData.TimeTrace(action=action, start_timestamp=self._timestamp())
        yield trace
        trace.finish_timestamp = self._timestamp()
        self.data.time_traces.append(trace)

    def _add_value(self, key, x, y):
        with self._locks[key]:
            if key not in self.data.values:
                self.data.values[key] = SessionData.Value(
                    id=key, name=key, unit=Units.UNKNOWN
                )
            value = self.data.values[key]
            value.x.append(x)
            value.y.append(y)

            if self._consumed_offset[key] == len(value.x) - 1:
                self._available_values.release()

    def _init_logging(self):
        init_logging()
        self._logging_stream = StringIO()
        self._logging_handler = logging.StreamHandler()
        self._logging_handler.setStream(self._logging_stream)
        self._logging_handler.setLevel(logging.DEBUG)
        self._logging_handler.setFormatter(logging.Formatter(self.LOGGING_FORMAT))
        logging.getLogger().addHandler(self._logging_handler)

    def _cleanup_logging(self):
        logging.getLogger().removeHandler(self._logging_handler)
        self._logging_handler.flush()

        log_lines = self._logging_stream.getvalue().splitlines(keepends=False)
        timestamp = 0
        for line in log_lines:
            try:
                time, message = line.split(' ', 1)
                timestamp = int(float(time) * 1000000)
                self.data.logs.append(SessionData.Log(timestamp=timestamp, message=message))
            except ValueError:  # on float conversion
                if len(self.data.logs) > 0:
                    self.data.logs[-1].message += '\n' + message

        self._logging_stream.close()
        LOGGER.debug("Collected %s log messages", len(log_lines))

    def reset(self, config: Config) -> None:
        self.data = SessionData(meta=SessionData.SessionMeta(
            device_class=self.cart_pole.__class__.__name__,
            device_config=config,
            actor_class=self.actor_class.__name__,
            actor_config=self.actor_config,
        ))
        self._available_values = threading.Semaphore(0)
        self._init_logging()

        self.cart_pole.reset(config)
        for callback in self.reset_callbacks:
            callback()
        self.data.meta.start_timestamp = time.time()
        self._start_perf_timestamp = time.perf_counter_ns() // 1000
        self._started_flag.set()

    def get_state(self) -> State:
        with self.time_trace() as trace:
            state = self.cart_pole.get_state()

        for field in dc.fields(state):
            key = f'state.{field.name}'
            value = getattr(state, field.name)
            if value is None:
                continue

            self._add_value(key, trace.start_timestamp, value)

        LOGGER.info(f"Get state: {state}")
        return state

    def get_info(self) -> dict:
        with self.time_trace():
            return self.cart_pole.get_info()

    def get_target(self) -> float:
        with self.time_trace():
            return self.cart_pole.get_target()

    def set_target(self, target: float) -> None:
        with self.time_trace() as trace:
            key = 'target.acceleration'
            self._add_value(key, trace.start_timestamp, target)
            LOGGER.info(f"Set target: {target}")
            return self.cart_pole.set_target(target)

    def timestamp(self):
        return self.cart_pole.timestamp()

    def close(self) -> None:
        with self.time_trace():
            self.cart_pole.close()
        self.data.meta.duration = self._timestamp()
        for callback in self.close_callbacks:
            callback()
        self._cleanup_logging()
        # self.save()
        self._started_flag.clear()

    def meta(self) -> dict:
        if not self._started_flag.is_set():
            raise RuntimeError('Session has not started yet')
        return dc.asdict(self.data)

    def consume_value(self) -> Tuple[dict, bool]:
        while True:
            if not self._started_flag.is_set():
                raise ValueError('Session has finished')
            if self._available_values.acquire(timeout=1.0):
                break
        for key, value in self.data.values.items():
            offset = self._consumed_offset[key]
            if offset == len(value.x):
                continue

            with self._locks[key]:
                vc = copy.deepcopy(value)
                vc.x = vc.x[offset:]
                vc.y = vc.y[offset:]
                self._consumed_offset[key] = len(value.x)
                return dc.asdict(vc)


if __name__ == '__main__':
    class FakeCartPole(CartPoleBase):
        def reset(self, config: Config) -> None:
            pass

        def get_state(self) -> State:
            return State(position=1)

        def set_target(self, target: float) -> None:
            pass

        def close(self) -> None:
            pass

    class FakeActor(Actor):
        pass

    logging.getLogger().setLevel(logging.DEBUG)
    c = CollectorProxy(FakeCartPole(), FakeActor, {})
    print()
    c.reset(Config())
    time.sleep(2)
    c.set_target(1)
    time.sleep(2)
    c.set_target(2)
    time.sleep(1)
    c.set_target(3)

    print(c._available_values._value)
    print(c.consume_value())
    print(c._available_values._value)

    c.close()
    c.reset(Config())
    time.sleep(5)
    c.close()
