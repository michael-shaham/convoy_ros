#!/usr/bin/env python3
""" @file stanley.py
    authors: Michael Shaham
    This file contains a class that implements the Stanley lateral controller.
"""

import numpy as np


"""
Stanley controller class.

args:
    crosstrack_error_gain (float): Crosstrack error gain to trade off between 
        heading error and crosstrack error
    softness_gain(float): Softness gain for better low-speed control
"""


class Stanley:

    def __init__(self, crosstrack_error_gain, softness_gain, steer_min, steer_max):
        self._k_e = crosstrack_error_gain
        self._k_s = softness_gain
        self._steer_min = steer_min
        self._steer_max = steer_max

    def control(self, crosstrack_error, heading_error, speed):
        steer = heading_error + np.arctan2(
            self._k_e * crosstrack_error, self._k_s + speed
        )
        steer = self._steer_min if steer < self._steer_min else steer
        steer = self._steer_max if steer > self._steer_max else steer
        return steer
