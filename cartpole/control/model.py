import numpy as np
import math


class Model:
    def __init__(self, params):
        self.params = params

        # caching
        self.coef = 1.5 * self.params.pole_length

        self.A = np.array(
            [[0, 0, 1, 0],
             [0, 0, 0, 1],
             [0, 0, 0, 0],
             [0, 0, 0, 0]],
            dtype=np.float32
        )

        self.B = np.array([[0], [0], [1], [0]], dtype=np.float32)

    def linearize(self, q, u):
        theta = q[1][0]
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        self.A[3][1] = self.coef * (u[0][0]*sin_theta - self.params.gravity*cos_theta)
        self.B[3][0] = -self.coef * cos_theta

        return self.A, self.B