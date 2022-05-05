# import dataclasses as dc
# import datetime as dt
# import logging
# import time
# from collections import defaultdict
# from contextlib import contextmanager
# from io import StringIO
# from typing import Callable, Dict, List, NamedTuple, Set, Tuple, Type, Union
#
# from common.interface import CartPoleBase, Config, State
#
# # class SessionProxy(CartPoleBase):
# #     '''Proxy class that records various method calls of a wrapped cartpole device'''
# #
# #     def __init__(self, device: CartPoleBase):
# #         self._device = device
# #         self._started = False
# #         self._ended = False
# #         self._start_wall_time = None
# #         self._start_clock_time = None
# #         self._end_wall_time = None
# #         self._end_clock_time = None
# #         self._data = []
# #
# #     def start_session(self):
# #         '''Starts session recording'''
# #         assert not self._started, "Session already started"
# #         self._started = True
# #         self._start_wall_time = time.time()
# #         self._start_clock_time = time.perf_counter()
# #
# #     def stop_session(self):
# #         '''Stops session recording'''
# #         assert not self._ended, "Session already ended"
# #         self._ended = True
# #         self._end_wall_time = time.time()
# #         self._end_clock_time = time.perf_counter()
# #
# #     def reset(self, config):
# #         return self._device.reset(config)
# #
# #     def get_info(self):
# #         return self._device.get_info()
# #
# #     def get_target(self):
# #         return self._device.get_target()
# #
# #     def close(self):
# #         if self._started and not self._ended:
# #             self.stop_session()
# #         return self._device.close()
# #
# #     def get_state(self):
# #         start = time.perf_counter()
# #         ret = self._device.get_state()
# #         self._push_state(ret, start, time.perf_counter() - start)
# #         return ret
# #
# #     def set_target(self, target):
# #         start = time.perf_counter()
# #         ret = self._device.set_target(target)
# #         self._push_target(target, start, time.perf_counter() - start)
# #         return ret
# #
# #     def _push_state(self, state: State, start: float, duration: float):
# #         self._data.append({'type': 'get_state_result', 'time': start + duration, 'value': state})
# #         self._data.append({'type': 'get_state_duration', 'time': start, 'value': duration})
# #
# #     def _push_target(self, target: float, start: float, duration: float):
# #         self._data.append({'type': 'set_target_result', 'time': start + duration, 'value': target})
# #         self._data.append({'type': 'set_target_duration', 'time': start, 'value': duration})
#
#
# class Actor:  # TODO: Move to common?
#     '''Base class for cartpole control algorithms'''
#
#     def __init__(self, device_config: Config = None, **_):
#         '''Implement this to use extra params (kwargs) for your algorithm'''
#         self.device_config = device_config
#
#     def __call__(self, state: State) -> float:
#         raise NotImplementedError
#
#
# @dc.dataclass()
# class SessionData:
#     '''TODO'''
#
#     @dc.dataclass()
#     class Value:
#         id: str = None
#         name: str = None
#         unit: str = None
#         x: List[float] = dc.field(default_factory=list, repr=False)
#         y: List[float] = dc.field(default_factory=list, repr=False)
#
#     @dc.dataclass()
#     class Group:
#         name: str = None
#         values: List[str] = dc.field(default_factory=list)
#
#     @dc.dataclass()
#     class Log:
#         time: float = None
#         message: str = None
#
#     device_class: str = None
#     device_config: Config = None
#     actor_class: str = None
#     actor_config: dict = None
#
#     start: dt.datetime = None
#     duration: float = None
#     values: Dict[str, Value] = dc.field(default_factory=dict)
#     groups: List[Group] = dc.field(default_factory=list)
#     logs: List[Log] = dc.field(default_factory=list)
#
#
# class SessionManager:
#     '''Main class for session recording'''
#
#     # Typedefs to avoid confusion in signatures
#     Target = float  # aka cart acceleration
#     Time = float  # timestamp in seconds
#     Duration = float  # duration in seconds
#     TimeTrace = NamedTuple('TimeTrace', start=Time, duration=Duration)
#
#     LOGGING_FORMAT = (
#         '%(created)f [%(levelname)s] %(name)s (%(filename)s:%(lineno)d) :: %(message)s'
#     )
#     HUMAN_LOGGING_FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'
#
#     def __init__(
#         self,
#         device: CartPoleBase,
#         device_config: Config,
#         actor_class: Type[Actor],
#         actor_config: dict = None,
#         on_start_callbacks: Set[Callable[['SessionManager'], None]] = None,
#         on_end_callbacks: Set[Callable[['SessionManager'], None]] = None,
#     ):
#         '''TODO'''
#         self.device = device
#         self.device_config = device_config
#         self.actor_class = actor_class
#         self.actor_config = actor_config or {}
#         self.on_start_callbacks = on_start_callbacks or set()
#         self.on_end_callbacks = on_end_callbacks or set()
#         self.data = SessionData()
#
#         self._logger = logging.getLogger('session-manager')
#         self._logging_stream: StringIO = None
#         self._logging_handler: logging.StreamHandler = None
#         SM = SessionManager
#         self._start_wall_time: SM.Time = None
#         self._start_perf_time: SM.Time = None
#         self._end_perf_time: SM.Time = None
#         self._state_probes: List[Tuple[SM.Time, State]] = []
#         self._target_probes: List[Tuple[SM.Time, SM.Target]] = []
#         self._timing_probes: Dict[str, List[SM.TimeTrace]] = defaultdict(list)
#         self._started: bool = False
#
#     def run(self, max_iterations=-1):
#         '''Prepares and runs the session'''
#         assert not self._started, "Session already started"
#         self._started = True
#         try:
#             self._init_logging()
#             self._init_device()
#             self._start_wall_time = time.time()
#             self._start_perf_time = time.perf_counter()
#             self._notify(self.on_start_callbacks)
#             self._logger.info("Session started")
#             self._control_loop(max_iterations)
#         except Exception:
#             self._logger.exception("Aborting due to error")
#         finally:
#             self._end_perf_time = time.time()
#             self._try_close_device()
#             self._cleanup_logging()
#             self._update_metadata()
#             self._logger.info("Session finished")
#
#     def _init_logging(self):
#         self._logging_stream = StringIO()
#         self._logging_handler = logging.StreamHandler()
#         self._logging_handler.setStream(self._logging_stream)
#         self._logging_handler.setLevel(logging.DEBUG)
#         self._logging_handler.setFormatter(logging.Formatter(self.LOGGING_FORMAT))
#         if len(logging.getLogger().handlers) == 0:
#             # Logging hasn't been initialized yet
#             logging.basicConfig(format=self.HUMAN_LOGGING_FORMAT, level=logging.DEBUG)
#         logging.getLogger().addHandler(self._logging_handler)
#
#     def _init_device(self):
#         self._logger.info("Resetting device...")
#         self.device.reset(self.device_config)
#         self._logger.info("Reset completed")
#
#     def _get_qualname(self, class_or_object: Union[Type, object]):
#         if not isinstance(class_or_object, type):
#             class_or_object = class_or_object.__class__
#         module = class_or_object.__module__
#         qualname = class_or_object.__qualname__
#         if module == '__main__':
#             self._logger.warning('Failed to detect package of class %s', qualname)
#         return f'{module}.{qualname}'
#
#     def _notify(self, callbacks: Set[Callable[["SessionManager"], None]]):
#         for callback in callbacks:
#             callback(self)
#
#     @contextmanager
#     def _timeit(self):
#         # TODO: Maybe move to separate class? Using lambda is a bit hacky
#         start = time.perf_counter()
#         yield lambda: self.TimeTrace(
#             start=start - self._start_perf_time,
#             duration=time.perf_counter() - start,
#         )
#
#     def _control_loop(self, max_iterations: int):
#         actor = self.actor_class(device_config=self.device_config, **self.actor_config)
#         # FIXME: This should be in the common interface
#         step_func = getattr(self.device, 'make_step') or (lambda: None)
#         current_iteration = 0
#         while current_iteration != max_iterations:
#             with self._timeit() as cm_outer:
#                 with self._timeit() as cm:
#                     state = self.device.get_state()
#                 self._push_state(state, cm())
#                 with self._timeit() as cm:
#                     target = actor(state)
#                 self._push_timing('actor_call', cm())
#                 with self._timeit() as cm:
#                     self.device.set_target(target)
#                 self._push_target(target, cm())
#                 step_func()
#             self._push_timing('iteration', cm_outer())
#             current_iteration += 1
#
#     def _push_state(self, state: State, timetrace: TimeTrace):
#         self._push_timing('get_state', timetrace)
#         self._state_probes.append((timetrace.start, state))
#
#     def _push_target(self, target: Target, timetrace: TimeTrace):
#         self._push_timing('set_target', timetrace)
#         self._target_probes.append((timetrace.start, target))
#
#     def _push_timing(self, key: str, timetrace: TimeTrace):
#         self._timing_probes[key].append(timetrace)
#
#     def _try_close_device(self):
#         try:
#             self.device.close()
#             self._logger.debug("Device closed successfully")
#         except Exception:
#             self._logger.exception("Failed to close the device")
#
#     def _cleanup_logging(self):
#         logging.getLogger().removeHandler(self._logging_handler)
#         self._logging_handler.flush()
#         self._logging_stream.close()
#
#         log_lines = self._logging_stream.getvalue().splitlines(keepends=False)
#         for line in log_lines:
#             time, message = line.split(' ', 1)
#             self.data.logs.append(SessionData.Log(time=float(time), message=message))
#         self._logger.debug("Collected %s log messages", len(log_lines))
#
#     def _update_metadata(self):
#         self.data.device_class = self._get_qualname(self.device)
#         self.data.device_config = self.device_config
#         self.data.actor_class = self._get_qualname(self.actor_class)
#         self.data.actor_config = self.actor_config
#         self.data.start = self._start_wall_time
#         self.data.duration = self._end_perf_time - self._start_perf_time
#         # TODO: Fill session data
#
#
# # def run_session(
# #         device: CartPoleBase,
# #         device_config: Config,
# #         actor_class: Type[Actor],
# #         actor_params: dict = None,
# #         max_iters=10000
# # ):
# #     '''Plays and records a session.
# #
# #     Args:
# #         device: cartpole device to control
# #         device_config: device config, will be passed to .reset() call
# #         actor_class: subclass of Actor base class
# #         actor_params: extra params for actor class
# #         max_iters: max iterations to run, -1 for unlimited
# #
# #     Returns:
# #         ???
# #     '''
# #     logger = logging.getLogger('run_session')
# #     proxy = SessionProxy(device)
# #     actor_params = actor_params.copy() if actor_params is not None else {}
# #     actor = actor_class(device_config=device_config, **actor_params)
# #     session_meta = {
# #         'device_class': None,
# #         'device_config': dc.asdict(device_config),
# #         'actor_class': actor_class.__qualname__,
# #         'actor_params': actor_params,
# #     }
# #
# #     logger.info('Resetting...')
# #     proxy.reset(device_config)
# #     try:
# #         logger.info('Session started')
# #         proxy.start_session()
# #         curr_iter = 0
# #         while curr_iter != max_iters:
# #             state = proxy.get_state()
# #             target = actor(state)
# #             proxy.set_target(target)
# #             curr_iter += 1
# #         logger.exception('Iteration limit reached - stopping')
# #     except:
# #         logger.exception('Aborting session due to error')
# #     finally:
# #         proxy.close()
# #
# #     return proxy._data  # TODO: Session storage format
#
#
# # def init_logging():  # TODO: Move to common?
# #     DEFAULT_FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'
# #     FORMAT = os.environ.get('LOGGING_FORMAT', DEFAULT_FORMAT)
# #     LEVEL = getattr(logging, os.environ.get('LOGGING_LEVEL', 'INFO').upper())
# #     logging.basicConfig(format=FORMAT, level=LEVEL)
#
#
# # def get_real_device(port: str = None):  # TODO: Move to common?
# #     from device.device import CartPoleDevice
# #     from device.wire_interface import WireInterface
# #
# #     PORT = port or os.environ.get('CARTPOLE_PORT', None)
# #     assert PORT is not None, "No port specified"
# #     interface = WireInterface(PORT)
# #     return CartPoleDevice(interface)
