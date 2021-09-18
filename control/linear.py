from math import cos, sin, pi

import scipy.linalg as linalg
import numpy as np
import numpy as np

from interface import State
from common import Params


class LQR:
    '''
    Infinite-horizon, continuous-time linear–quadratic regulator.

    Evolution of system: x' = A*x + B*u
    Quadratic cost function: J = I[0, inf](x^T*Q*x + u^T*R*u dt)

    More info:
    - https://en.wikipedia.org/wiki/Linear–quadratic_regulator 
    - https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
    '''

    def __init__(self, A, B, Q, R):
        '''
        Create regulator. K matrix is cached.
        '''

        P = linalg.solve_continuous_are(A, B, Q, R)
        self.K = -linalg.inv(R) @ (B.T @ P)

    def __call__(self, x):
        '''
        Returns control u for current state x.
        '''
        return self.K @ x

# class Model:
#     '''
#     Cart-pole physical model.
    
#     Standart actuator form:
#     M(q)@q'' + C(q, q')@q' = t(q) + B*u, where q = [x, theta]
#     '''

#     def __init__(self, params):
#         self.params = params

#     def dynamics(self, state, target):
#         d_x = state.velocity
#         theta = state.pole_angle
#         d_theta = state.pole_angular_velocity
#         u = target
#         print(d_x, theta, d_theta, u)

#         M = self.get_M(state)

#         print("M=", M)
#         C = self.get_C(state)
#         print("C=", C)
#         t = self.get_t(state)

#         print("t=", t)
#         B = self.get_B()

#         print("B=", B)

#         print("u=", u)

#         d_q = np.array([[d_x], [d_theta]])

#         M_inv = linalg.inv(M)
#         u = np.array([[target], [0]])
#         result = M_inv @ (t + B@u - C@d_q)
#         acceleration = result[0][0]
#         pole_angular_acceleration = result[1][0]
#         return acceleration, pole_angular_acceleration

#     def get_M(self, state):
#         '''
#         Generate M(q) for current state.
#         '''

#         theta = state.pole_angle
#         m_c = self.params.cart_mass
#         m_p = self.params.pole_mass
#         l = self.params.pole_length

#         return np.array(
#             [[m_c + m_p, m_p*l/2*cos(theta)],
#              [m_p*l/2*cos(theta), m_p*l*l/3]]
#         )

#     def get_C(self, state):
#         '''
#         Generate C(q, q') for current state.
#         '''

#         theta = state.pole_angle
#         d_theta = state.pole_angular_velocity

#         m_p = self.params.pole_mass
#         l = self.params.pole_length

#         return np.array(
#             [[0, -m_p*l/2*d_theta*sin(theta)],
#              [0, 0]]
#         )

#     def get_t(self, state):
#         '''
#         Generate t(q) for current state.
#         '''

#         theta = state.pole_angle
#         m_p = self.params.pole_mass
#         l = self.params.pole_length
#         g = self.params.gravity

#         return np.array(
#             [[0],
#              [-m_p*l/2*g*sin(theta)]]
#         )

#     def get_dt_dq(self, state):
#         theta = state.pole_angle
#         m_p = self.params.pole_mass
#         l = self.params.pole_length
#         g = self.params.gravity

#         return np.array(
#             [[0, 0],
#              [0, -m_p*l/2*g*cos(theta)]]
#         )

#     def get_B(self):
#         '''
#         Returns B.
#         '''
#         return np.array(
#             [[1, 0],
#              [0, 0]]
#         )


class LinearBalanceControl:
    def __init__(self, params=Params()):
        g = params.gravity
        l = params.pole_length

        A = np.array(
            [[0,         0, 1, 0],
             [0,         0, 0, 1],
             [0,         0, 0, 0],
             [0, 3*g/(2*l), 0, 0]
            ]
        )

        B = np.array(
            [[0],
             [0],
             [1],
             [3/(2*l)]]
        )

        Q = np.zeros((4, 4))
        Q[0, 0] = 1
        Q[1, 1] = 100
        Q[2, 2] = 0
        Q[3, 3] = 0

        R = np.zeros((1, 1))
        R[0,0] = 1

        self.lqr = LQR(A, B, Q, R)

    def __call__(self, state: State) -> float:
        '''
        Return desired acceleration for current state.
        '''

        error = np.array([
            state.position,
            state.pole_angle - pi,
            state.velocity,
            state.pole_angular_velocity
        ])

        print('error=', error)
        u = self.lqr(error)[0]
        print("u=", u)
        return u

