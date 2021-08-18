from collections import deque
import random


class ReplayBuffer:
    def __init__(self, buffer_size, batch_size):
        self._max_size = buffer_size
        self._batch_size = batch_size
        self._buffer = deque(maxlen=buffer_size)

    def append(self, item):
        self._buffer.append(item)
    
    def __len__(self):
        return len(self._buffer)
    
    def sample(self):
        num_samples = min(len(self._buffer), self._batch_size)
        return zip(*random.sample(self._buffer, num_samples))
