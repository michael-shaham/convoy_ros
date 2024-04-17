#!/usr/bin/env python3
""" @file bicycle_dynamics.py
    authors: Michael Shaham
    This file contains a class for the discrete-time double integrator in R, 
    i.e., \ddot{x} = u, u is a scalar input.
"""
import numpy as np


"""
The discretized double integrator model.

state:
    x1: position           [m]
    x2: velocity           [m/s]
control:
    u: input acceleration  [m/s^2]

args:
    dt (float): timestep
"""


class DTDoubleIntegrator:

    def __init__(self, dt):
        self.dt = dt

        # continuous-time dynamics
        self.A = np.array([[0, 1], [0, 0]])
        self.B = np.array([[0], [1]])

        self.n = self.A.shape[0]  # x dimension
        self.m = self.B.shape[1]  # u dimension

        # discrete-time dynamics
        self.A = np.eye(self.n) + self.dt * self.A
        self.B = self.dt * self.B

        # sensing
        self.C = np.array([[1, 0, 0], [0, 1, 0]])
        self.p = self.C.shape[0]  # y dimension

    def forward(self, x, u):
        """
        Returns new state based on current state and action.
            x_new = f(x) + g(x) u

        args:
            x (np.ndarray): Shape (2,), state
            u float: Control (acceleration) input

        returns:
            np.ndarray: Shape (5,), new state according to single track bicycle
                kinematic model
        """
        return self.A @ x + self.B @ u

    def sense(self, x):
        return self.C @ x
