import numpy as np


class LinearLongAccel:
    """
    Linear longitudinal dynamics, using the model described in Zhang et al.,
    'Stability and scalability of homogeneous vehicular platoon: Study on the
    influence of information flow topologies.'

    Parameters:
        dt: discrete timestep
        x_min/max: state bounds
        u_min/max: control bounds
        tau: inertial lag of longitudinal dynamics
    """

    def __init__(self, dt: float, tau: float):
        self.dt = dt

        # continuous-time dynamics
        self.A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, -1 / tau]])
        self.B = np.array([[0], [0], [1 / tau]])

        self.n = self.A.shape[0]  # x dimension
        self.m = self.B.shape[1]  # u dimension

        # discrete-time dynamics
        self.A = np.eye(self.n) + self.dt * self.A
        self.B = self.dt * self.B

        # sensing
        self.C = np.array([[1, 0, 0], [0, 1, 0]])

        self.p = self.C.shape[0]  # y dimension

    def forward(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return self.A @ x + self.B @ u

    def sense(self, x: np.ndarray) -> np.ndarray:
        return self.C @ x
