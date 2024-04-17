#!/usr/bin/env python3
""" @file double_int_mpc.py
    authors: Michael Shaham
    Model predictive control class for tracking a given trajectory. Uses 
    discrete time double integrator dynamics model.
"""

import cvxpy as cp
import numpy as np


"""
MPC class. Takes in dynamics (which include timestep), horizon, waypoints to 
track, and a few other params, and provides functions to solve MPC problem to 
find find optimal control to take.

args:
    A, B: dynamics and input matrices
    u_min, u_max: max acceleration input
    horizon (int): planning horizon
    waypoints (np.ndarray): Shape (horizon, n), waypoints to track
    x_0 (np.ndarray): Shape (n,), vehicle initial state
    Q (np.ndarray): Shape (n, n), cost matrix on state
    R (np.ndarray): Shape (m, m), cost matrix on control
"""


class DoubleIntMPC:

    def __init__(
        self,
        A,
        B,
        x_min,
        x_max,
        u_min,
        u_max,
        horizon,
        waypoints,
        x_0,
        Q,
        R,
    ):
        self.A, self.B = A, B
        self.x_min, self.x_max = x_min, x_max
        self.u_min, self.u_max = u_min, u_max
        self.H = horizon
        self.z = waypoints
        self.x_0 = x_0
        self.Q = Q
        self.R = R

        # create opt variables, constraints, objective, and problem
        self.x = cp.Variable((self.H, self.A.shape[0]))
        self.u = cp.Variable((self.H - 1, self.B.shape[1]))
        self.cost, self.constraints = self.gen_cost_constraints()
        self.problem = self.generate_cvx_problem()

    def gen_cost_constraints(self):
        constraints = [self.x[0, :] == self.x_0]
        cost = 0.0
        for k in range(self.H - 1):
            cost += self.Q * (self.x[k, 0] - self.z[k]) ** 2 + self.R * self.u[k] ** 2
            constraints += [
                self.x[k, :] <= self.x_max,
                self.x[k, :] >= self.x_min,
                self.u[k, :] <= self.u_max,
                self.u[k, :] >= self.u_min,
                self.x[k + 1, :] == self.A @ self.x[k, :] + self.B @ self.u[k, :],
            ]
        constraints += [
            self.x[self.H - 1, :] <= self.x_max,
            self.x[self.H - 1, :] >= self.x_min,
        ]
        cost += self.Q * (self.x[self.H - 1, 0] - self.z[self.H - 1]) ** 2
        return cost, constraints

    def generate_cvx_problem(self):
        """
        Generates a cvxpy.Problem class that will be solved.

        returns:
            cp.Problem: convex MPC problem
        """
        return cp.Problem(cp.Minimize(self.cost), self.constraints)

    def act(self):
        """
        Solves self.problem.

        returns:
            np.ndarray: Shape (2,), control action to take
            np.ndarray: Shape (H, 5), planned trajectory opt solved for
        """
        self.problem.solve()
        return self.u[:, 0].value, self.x.value, self.problem
