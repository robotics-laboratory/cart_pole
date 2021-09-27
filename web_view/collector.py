import dataclasses as dc
import json
import logging
import numpy as np
import os
import string
import threading
import time
from typing import Type, Union, List

from device.wire_interface import WireInterface, DeviceState, DeviceTarget


LOGGER = logging.getLogger(__name__)
HEX_DIGITS = list(string.digits + string.ascii_lowercase[:6])


class Collector:
    GROUP_CLASSES = (DeviceState, DeviceTarget)
    STORAGE_DIR = 'sessions'

    def __init__(self, interface: WireInterface, interval: float = 0.5) -> None:
        self.interface = interface
        self.interval = interval
        self.session_id: str = None        
        self.session = {}
        
        self._start_timestamp: float = None
        self._run_thread: threading.Thread = None
        self._is_running_signal = threading.Event()
        self._stop_signal = threading.Event()        
        self._consumer_flags = {}
        self._locks = {}

        self._reset_dicts()

    def _reset_dicts(self) -> None:
        self.session = {cls.GROUP_NAME: {field.name: [] for field in dc.fields(cls)} for cls in self.GROUP_CLASSES}
        self._consumer_flags = {cls.GROUP_NAME: {field.name: threading.Event() for field in dc.fields(cls)} for cls in self.GROUP_CLASSES}
        self._locks = {cls.GROUP_NAME: {field.name: threading.Lock() for field in dc.fields(cls)} for cls in self.GROUP_CLASSES}

    def _save_session(self):
        if not os.path.exists(self.STORAGE_DIR):
            os.mkdir(self.STORAGE_DIR)

        with open(f'{self.STORAGE_DIR}/{self.session_id}.json', 'w') as f:
            json.dump({
                'session_id': self.session_id,
                'interval': self.interval,
                'start_timestamp': self._start_timestamp,
                'data': self.session,
            }, f, indent=4)

    def _poll_and_store(self, cls: Type[Union[DeviceState, DeviceTarget]], timestamp: str):
        # value = self.interface.get(cls.full())
        value = cls(position=1.0)
        for field in dc.fields(value):
            self.session[cls.GROUP_NAME][field.name].append({
                'x': timestamp,
                'y': getattr(value, field.name),
            })

            lock = self._locks[cls.GROUP_NAME][field.name]
            lock.acquire()
            self._consumer_flags[cls.GROUP_NAME][field.name].set()
            lock.release()

    @staticmethod
    def _generate_id(size: int):
        return ''.join(np.random.choice(HEX_DIGITS, size=size, replace=True))

    def _run(self) -> None:
        self._stop_signal.clear()
        self.session_id = self._generate_id(4)
        self._start_timestamp = None
        self._reset_dicts()
        self._is_running_signal.set()

        while not self._stop_signal.is_set():
            timestamp = time.time()
            if self._start_timestamp is None:
                self._start_timestamp = timestamp

            time_string = str(timestamp - self._start_timestamp)
            for cls in self.GROUP_CLASSES:
                self._poll_and_store(cls, time_string)

            time.sleep(self.interval)

        self._save_session()
        self._is_running_signal.clear()

    def start(self) -> None:
        if self._is_running_signal.is_set():
            raise RuntimeError('Collector is already running')

        self._run_thread = threading.Thread(target=self._run)
        self._run_thread.start()
        assert self._is_running_signal.wait(timeout=5.0)
        LOGGER.info(f'Started collector thread with id {self._run_thread.ident}')

    def stop(self) -> None:
        self._stop_signal.set()
        self._run_thread.join(timeout=60.0)
        assert not self._run_thread.is_alive()

    @property
    def is_running(self) -> bool:
        return self._is_running_signal.is_set()

    def consume(self, group: str, field: str, index: int) -> List[dict]:
        consumer_flag = self._consumer_flags[group][field]
        lock = self._locks[group][field]

        consumer_flag.wait()
        lock.acquire()

        size = len(self.session[group][field])
        batch = self.session[group][field][index:size]

        consumer_flag.clear()
        lock.release()

        return batch


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    c = Collector(None)
    c.start()
    time.sleep(5)
    print(len(c.consume('state', 'position', 0)))
    c.stop()

    c.start()
    time.sleep(5)
    c.stop()