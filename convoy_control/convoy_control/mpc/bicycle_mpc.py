#!/usr/bin/env python3
""" @file bicyle_mpc.py
    authors: Michael Shaham
    Model predictive control class for tracking a given trajectory. Uses 
    linearized and discrete time bicycle models.
"""

import cvxpy as cp
import numpy as np


"""
MPC class. Takes in dynamics (which include timestep), horizon, waypoints to 
track, and a few other params, and provides functions to solve MPC problem to 
find find optimal control to take.

args:
    dynamics: dynamics class, must have forward method for constraint
    horizon (int): planning horizon
    waypoints (np.ndarray): Shape (horizon, n), waypoints to track
    x_0 (np.ndarray): Shape (n,), vehicle initial state
    Q (np.ndarray): Shape (n, n), cost matrix on state
    R (np.ndarray): Shape (m, m), cost matrix on control
"""


class BicycleMPC:

    def __init__(
        self,
        dynamics,
        horizon,
        waypoints,
        x_0,
        Q,
        R,
    ):
        self.dyn = dynamics
        self.H = horizon
        self.z = waypoints
        self.x_0 = x_0
        self.Q = Q
        self.R = R

        # create opt variables, constraints, objective, and problem
        self.x = cp.Variable((self.H, self.dyn.n))
        self.u = cp.Variable((self.H - 1, self.dyn.m))
        self.cost, self.constraints = self.gen_cost_constraints()
        self.problem = self.generate_cvx_problem()

    def gen_cost_constraints(self):
        constraints = [self.x[0, :] == self.x_0]
        cost = 0.0
        for k in range(self.H - 1):
            cost += cp.quad_form(self.x[k, :] - self.z[k, :], self.Q) + cp.quad_form(
                self.u[k, :], self.R
            )
            constraints += [
                self.x[k, :] <= self.dyn.x_max,
                self.x[k, :] >= self.dyn.x_min,
                self.u[k, :] <= self.dyn.u_max,
                self.u[k, :] >= self.dyn.u_min,
                self.x[k + 1, :]
                == self.dyn.forward(
                    self.x[k, :], self.u[k, :], self.x_0, np.zeros(self.dyn.m)
                ),
            ]
        cost += cp.quad_form(self.x[self.H - 1, :] - self.z[self.H - 1, :], self.Q)
        constraints += [
            self.x[self.H - 1, :] <= self.dyn.x_max,
            self.x[self.H - 1, :] >= self.dyn.x_min,
        ]
        return cost, constraints

    def generate_cvx_problem(self):
        """
        Generates a cvxpy.Problem class that will be solved.

        returns:
            cp.Problem: convex MPC problem
        """
        # TODO: instead of adding to the cost for each timestep, make large Q, R
        #       matrices and stack x, u. might improve performance
        return cp.Problem(cp.Minimize(self.cost), self.constraints)

    def act(self):
        """
        Solves self.problem.

        returns:
            np.ndarray: Shape (2,), control action to take
            np.ndarray: Shape (H, 5), planned trajectory opt solved for
        """
        self.problem.solve()
        action = self.u[0, :].value
        next_state = self.x[1, :].value
        return action, next_state
