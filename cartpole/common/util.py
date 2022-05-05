import logging
import socket
import math
import os
from contextlib import closing
import sys

from cartpole.common.interface import State

STEP_COUNT = 'step_count'


def reward(state: State) -> float:
    return math.exp(-math.cos(state.pole_angle))


def init_logging():
    '''
    Helper function to initialize logging. In top-level scripts (__name__ == __main__)
    you should call this function as soon as possible, to ensure no logs will be printed
    before logging is initialized. Only first call of this function is effective - all
    later calls are ignored.

    Logging options are customizable via environment variables:
    * LOGGING_FORMAT - logging format string (default: see below)
    * LOGGING_LEVEL - logging level (default: INFO)
    '''
    DEFAULT_FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'

    if getattr(init_logging, '__initialized', False):
        return  # Already initialized

    format = os.environ.get('LOGGING_FORMAT', DEFAULT_FORMAT)
    level = getattr(logging, os.environ.get('LOGGING_LEVEL', 'INFO').upper())
    kwargs = dict(format=format, level=level)

    # if sys.version_info < (3, 8):
    #     # https://stackoverflow.com/questions/20240464/python-logging-file-is-not-working-when-using-logging-basicconfig
    #     from importlib import reload
    #     reload(logging)
    #
    # else:
    #     kwargs['force'] = True

    logging.basicConfig(**kwargs)
    setattr(init_logging, '__initialized', True)


def find_free_port():
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(('', 0))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return s.getsockname()[1]
