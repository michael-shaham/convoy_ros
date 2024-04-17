#!/usr/bin/env python3
""" @file bicycle_dynamics.py
    authors: Michael Shaham
    This file contains classes describing different vehicle dynamics models.
"""

import numpy as np


"""
The linearized and discretized single track kinematic model. See google for more 
info. Model information:

state:
    x1: x, x position           (meters)
    x2: y, y position           (meters)
    x3: theta, vehicle heading  (radians)
    x4: v, longitudinal speed   (m/s)
    x5: delta, steering angle   (rad)
control:
    u1: a_long, longitudinal acceleration   (m/s^2)
    u2: v_delta, steering angle velocity    (rad/s)

args:
    dt (float): timestep
    wheelbase (float): length between front and rear wheel axles
    v_min (float): min longitudinal velocity 
    v_max (float): max longitudinal velocity
    a_min (float): min longitudinal acceleration (max braking force)
    a_max (float): max longitudinal acceleration
    steer_min (float): min steering angle
    steer_max (float): max steering angle
    steer_v_min (float): min steering angle velocity
    steer_v_max (float): max steering angle velocity
"""


class LDSTKinematics:

    def __init__(
        self,
        dt,
        wheelbase,
        v_min=-np.inf,
        v_max=np.inf,
        a_min=-np.inf,
        a_max=np.inf,
        steer_min=-np.inf,
        steer_max=np.inf,
        steer_v_min=-np.inf,
        steer_v_max=np.inf,
    ):
        self.n, self.m = 5, 2  # state and control dimensions
        self.dt = dt
        self.w = wheelbase
        self.x_min = np.array([-np.inf, -np.inf, -np.inf, v_min, steer_min])
        self.x_max = np.array([np.inf, np.inf, np.inf, v_max, steer_max])
        self.u_min = np.array([a_min, steer_v_min])
        self.u_max = np.array([a_max, steer_v_max])

    def _f(self, x, u):
        """
        Evaluate the single track kinematic model f(x, u) at the given x, u.

        args:
            x (np.ndarray): Shape (n,), state
            u (np.ndarray): Shape (m,), state

        returns:
            np.ndarray: Shape (n,), f(x, u)
        """
        assert x.shape == (self.n,), f"x must be shape ({self.n},)"
        assert u.shape == (self.m,), f"u must be shape ({self.m},)"
        return np.array(
            [
                x[3] * np.cos(x[2]),
                x[3] * np.sin(x[2]),
                x[3] / self.w * np.tan(x[4]),
                u[0],
                u[1],
            ]
        )

    def _A(self, x, u):
        """
        Evaluate the partial derivative of f with respect to x at the point
        (x, u).

        args:
            x (np.ndarray): Shape (n,), state
            u (np.ndarray): Shape (m,), control

        returns:
            np.ndarray: Shape (n, n), df(x, u)/dx
        """
        assert x.shape == (self.n,), f"x must be shape ({self.n},)"
        assert u.shape == (self.m,), f"u must be shape ({self.m},)"
        return np.array(
            [
                [0, 0, -x[3] * np.sin(x[2]), np.cos(x[2]), 0],
                [0, 0, x[3] * np.cos(x[2]), np.sin(x[2]), 0],
                [
                    0,
                    0,
                    0,
                    1.0 / self.w * np.tan(x[4]),
                    x[3] / self.w * (1 / np.cos(x[4])) ** 2,
                ],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ]
        )

    def _B(self, x=None, u=None):
        """
        Evaluate the partial derivative of f with respect to u at the point
        (x, u).

        args:
            x (np.ndarray): Shape (n,), state
            u (np.ndarray): Shape (m,), control

        returns:
            np.ndarray: Shape (n, n), df(x, u)/du
        """
        return np.r_[np.zeros((self.n - self.m, self.m)), np.eye(self.m)]

    def forward(self, x, u, x_bar, u_bar):
        """
        Get the estimated next state given current state and action (x, u) and
        state and action we linearize about (x_bar, u_bar).

        args:
            x (np.ndarray): Shape (n,), state
            u (np.ndarray): Shape (m,), control
            x_bar (np.ndarray): Shape (n,), state we linearize around
            u_bar (np.ndarray): Shape (m,), control we linearize around

        returns:
            np.ndarray: Shape (n,), next state
        """
        assert x.shape == (self.n,), f"x must be shape ({self.n},)"
        assert u.shape == (self.m,), f"u must be shape ({self.m},)"
        assert x_bar.shape == (self.n,), f"x_bar must be shape ({self.n},)"
        assert u_bar.shape == (self.m,), f"u_bar must be shape ({self.m},)"
        return (
            x_bar
            + self.dt * self._f(x_bar, u_bar)
            + (np.eye(self.n) + self.dt * self._A(x_bar, u_bar)) @ (x - x_bar)
            + self.dt * self._B() @ (u - u_bar)
        )


"""
The discretized single track kinematic model. See google for more info. Model 
information:

state:
    x1: x, x position           (meters)
    x2: y, y position           (meters)
    x3: theta, vehicle heading  (radians)
    x4: v, longitudinal speed   (m/s)
    x5: delta, steering angle   (rad)
control:
    u1: a_long, longitudinal acceleration   (m/s^2)
    u2: v_delta, steering angle velocity    (rad/s)

args:
    dt (float): timestep
    wheelbase (float): length between front and rear wheel axles
    v_min (float): min longitudinal velocity 
    v_max (float): max longitudinal velocity
    a_min (float): min longitudinal acceleration (max braking force)
    a_max (float): max longitudinal acceleration
    steer_min (float): min steering angle
    steer_max (float): max steering angle
    steer_v_min (float): min steering angle velocity
    steer_v_max (float): max steering angle velocity
"""


class DSTKinematics:

    def __init__(
        self,
        dt,
        wheelbase,
        v_min=-np.inf,
        v_max=np.inf,
        a_min=-np.inf,
        a_max=np.inf,
        steer_min=-np.inf,
        steer_max=np.inf,
        steer_v_min=-np.inf,
        steer_v_max=np.inf,
    ):
        self.n, self.m = 5, 2  # state and control dimensions
        self.dt = dt
        self.w = wheelbase
        self.x_min = np.array([-np.inf, -np.inf, -np.inf, v_min, steer_min])
        self.x_max = np.array([np.inf, np.inf, np.inf, v_max, steer_max])
        self.u_min = np.array([a_min, steer_v_min])
        self.u_max = np.array([a_max, steer_v_max])

    def _f(self, x):
        """
        Dynamics function used in forward below.

        args:
            x (np.ndarray): Shape (5,), state

        returns:
            np.ndarray: Shape (5,), f(x)
        """
        return np.array(
            [
                x[0] + self.dt * x[3] * np.cos(x[2]),
                x[1] + self.dt * x[3] * np.sin(x[2]),
                x[2] + self.dt * x[3] / self.w * np.tan(x[4]),
                x[3],
                x[4],
            ]
        )

    def _g(self, x):
        """
        Input equation for the dynamics model.

        args:
            x (np.ndarray): Shape (5,), state (actually not used for this model)

        returns:
            np.ndarray: Shape (5,), g(x)
        """
        return self.dt * np.r_[np.zeros((3, 2)), np.eye(2)]

    def forward(self, x, u):
        """
        Returns new state based on current state and action.
            x_new = f(x) + g(x) u

        args:
            x (np.ndarray): Shape (5,), state
            u (np.ndarray): Shape (2,), control

        returns:
            np.ndarray: Shape (5,), new state according to single track bicycle
                kinematic model
        """
        return self._f(x) + self._g(x) @ u
