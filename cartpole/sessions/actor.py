import logging
import time
from math import pi

import numpy as np
import scipy.linalg as linalg

from cartpole.common.interface import State


class Actor:
    '''Base class for cartpole control algorithms'''

    def __init__(self, **_):
        '''Implement this to use extra params (kwargs) for your algorithm'''
        pass

    def __call__(self, state: State, stamp: float = None) -> float:
        raise NotImplementedError
