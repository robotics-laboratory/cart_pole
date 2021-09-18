import dataclasses as dc
from device.device import CartPoleDevice
import json
import logging
import numpy as np
import os
import string
import threading
import time
from collections import defaultdict
from typing import Type, Union, List

from interface import CartPoleBase, Config, State
from device.wire_interface import DeviceTarget


LOGGER = logging.getLogger(__name__)
HEX_DIGITS = list(string.digits + string.ascii_lowercase[:6])


class CollectorProxy(CartPoleBase):
    STORAGE_DIR = 'sessions'
    TARGET_KEY = 'target'

    def __init__(self, cart_pole: CartPoleBase, interval: float = 0.5) -> None:
        self.cart_pole = cart_pole
        self.interval = interval
        self.session_id: str = None        
        self.session = defaultdict(lambda: defaultdict(list))
        
        self._start_timestamp: float = None
        self._start_timestamp_lock = threading.Lock()
        self._run_thread: threading.Thread = None
        self._is_running_signal = threading.Event()
        self._stop_signal = threading.Event()        
        self._consumer_flags = defaultdict(threading.Event)
        self._locks = defaultdict(threading.Lock)

    def _save_session(self):
        if not os.path.exists(self.STORAGE_DIR):
            os.mkdir(self.STORAGE_DIR)

        with open(f'{self.STORAGE_DIR}/{self.session_id}.json', 'w') as f:
            json.dump({
                'meta': {
                    'session_id': self.session_id,
                    'interval': self.interval,
                    'start_timestamp': self._start_timestamp,
                },
                'session': self.session,
            }, f, indent=4)

    @staticmethod
    def _generate_id(size: int):
        return ''.join(np.random.choice(HEX_DIGITS, size=size, replace=True))

    def _get_timestamp(self) -> int:
        timestamp = time.monotonic_ns() // 1000
        if self._start_timestamp is None:
            self._start_timestamp_lock.acquire()
            if self._start_timestamp is None:
                self._start_timestamp = timestamp
            self._start_timestamp_lock.release()
        return timestamp - self._start_timestamp

    def _run(self) -> None:
        self._stop_signal.clear()
        self.session_id = self._generate_id(4)
        self._start_timestamp = None
        self._is_running_signal.set()

        self._stop_signal.wait()  # FIXME

        # while not self._stop_signal.is_set():
        #     timestamp = self._get_timestamp()
        #     state = self.cart_pole.get_state()
        #     self._push_state(timestamp, state)
        #     time.sleep(self.interval)

        self._save_session()
        self._is_running_signal.clear()

    def _push_state(self, timestamp, state):
        for field in dc.fields(state):
            if field.name == 'error_code': continue  # FIXME
            key = f'state.{field.name}'

            lock = self._locks[key]
            lock.acquire()

            session_values = self.session[key]
            session_values['x'].append(timestamp)
            session_values['y'].append(getattr(state, field.name))

            self._consumer_flags[key].set()
            lock.release()

    def start(self) -> None:
        if self._is_running_signal.is_set():
            raise RuntimeError('Collector is already running')

        self._run_thread = threading.Thread(target=self._run)
        self._run_thread.start()
        assert self._is_running_signal.wait(timeout=5.0)
        LOGGER.info(f'Started collector thread with id {self._run_thread.ident}')

    def stop(self) -> None:
        self._stop_signal.set()
        self._run_thread.join(timeout=5.0)
        assert not self._run_thread.is_alive()

    @property
    def is_running(self) -> bool:
        return self._is_running_signal.is_set()

    def consume(self, key: str, index: int) -> List[dict]:
        if key not in self._consumer_flags:
            return []

        consumer_flag = self._consumer_flags[key]
        lock = self._locks[key]

        consumer_flag.wait()
        lock.acquire()

        session_item = self.session[key]
        size = len(session_item['x'])
        batch = {
            'x': session_item['x'][index:size],
            'y': session_item['y'][index:size],
        }

        consumer_flag.clear()
        lock.release()

        return batch

    def reset(self, config: Config) -> None:
        return self.cart_pole.reset(config)

    def get_state(self) -> State:  # FIXME
        state = self.cart_pole.get_state()
        timestamp = self._get_timestamp()
        self._push_state(timestamp, state)
        return state

    def get_info(self) -> dict:
        return self.cart_pole.get_info()

    def get_target(self) -> float:
        return self.cart_pole.get_target()

    def set_target(self, target: float) -> None:
        timestamp = self._get_timestamp()
        key = f'target.acceleration'
        
        lock = self._locks[key]
        lock.acquire()

        session_values = self.session[key]
        session_values['x'].append(timestamp)
        session_values['y'].append(target)

        self._consumer_flags[key].set()
        lock.release()

        return self.cart_pole.set_target(target)

    def close(self) -> None:
        return self.cart_pole.close()


if __name__ == '__main__':
    class FakeCartPole(CartPoleBase):
        def get_state(self) -> State:
            return State(position=1)
        
        def set_target(self, target: float) -> None:
            pass

    logging.getLogger().setLevel(logging.DEBUG)
    c = CollectorProxy(FakeCartPole())
    c.start()
    time.sleep(2)
    c.set_target(1)
    time.sleep(2)
    c.set_target(2)
    time.sleep(1)
    c.set_target(3)

    print(len(c.consume('state.position', 0)['x']))
    c.stop()

    c.start()
    time.sleep(5)
    c.stop()