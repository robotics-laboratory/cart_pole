

class LinearEpsilonGenerator:
    def __init__(
        self,
        max_epsilon,
        min_epsilon,
        decay_steps
    ):
        self._max_epsilon = max_epsilon
        self._min_epsilon = min_epsilon
        self._decay_steps = decay_steps

    def get_epsilon(self, step):
        if step >= self._decay_steps:
            return 0.
        elif step >= 0:
            return step * (self._min_epsilon - self._max_epsilon) / self._decay_steps + self._max_epsilon
