import numpy as np


class LinearLongVel:
    """
    Linear longitudinal dynamics, using the model described in Zhang et al.,
    'Stability and scalability of homogeneous vehicular platoon: Study on the
    influence of information flow topologies,' but without acceleration (so
    desired velocity is the input - this should help hardware performance).

    Parameters:
        dt: discrete timestep
        tau: inertial lag of longitudinal dynamics
    """

    def __init__(self, dt: float, tau: float):
        self.dt = dt

        # continuous-time dynamics
        self.A = np.array([[0, 1], [0, -1 / tau]])
        self.B = np.array([[0], [1 / tau]])

        self.n = self.A.shape[0]  # x dimension
        self.m = self.B.shape[1]  # u dimension

        # discrete-time dynamics
        self.A = np.eye(self.n) + self.dt * self.A
        self.B = self.dt * self.B

        # sensing
        self.C = np.eye(self.n)

        self.p = self.C.shape[0]  # y dimension

    def forward(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return self.A @ x + self.B @ u

    def sense(self, x: np.ndarray) -> np.ndarray:
        return self.C @ x
