#!/usr/bin/env python3
""" @file pure_pursuit.py
    authors: Michael Shaham
    This file contains modules for performing the pure pursuit algorithm for 
    path tracking.
"""

import numpy as np


"""
Pure pursuit class.

args:
    waypoints (np.ndarray): Shape (N, 2), waypoints to track
    veh_pose (np.ndarray): Shape (3,), x, y, theta position of vehicle. If 
        zeros, trajectory must be in local coordinate frame
    const_l_d (float): If given, always use this lookahead distance
    l_d_speed_gain (float): speed lookahead gain, tuning parameter
    min_l_d (float): If we calculate lookahead distance, clip values below this
    max_l_d (float): If we calculate lookahead distance, clip values above this
    const_speed (float): If given, always use this speed
    veh_speed (float): Current vehicle speed
    min_speed (float): If we calculate speed, clip values below this
    max_speed (float): If we calculate speed, clip values above this
    wheelbase (float): distance between the front and rear wheel axles
"""


class PurePursuit:

    def __init__(
        self,
        waypoints,
        steer_min,
        steer_max,
        veh_pose=np.zeros(3),
        const_l_d=None,
        l_d_speed_gain=None,
        min_l_d=None,
        max_l_d=None,
        veh_speed=None,
        wheelbase=None,
    ):
        self.veh_pos = veh_pose[:-1]
        self.veh_heading = veh_pose[-1]

        self.waypoints = waypoints
        self.dists = np.linalg.norm(self.waypoints - self.veh_pos, axis=1)

        self.wheelbase = wheelbase

        # closest index, closest point, closest distance
        self.c_i, self.c_p, self.c_d = self.get_closest_point()

        # calculate speed based on curvature at closest point
        self.curr_speed = veh_speed

        self.steer_min = steer_min
        self.steer_max = steer_max

        # calculate lookahead distance
        if const_l_d:
            self.l_d = const_l_d
        elif l_d_speed_gain and min_l_d and max_l_d:
            self.l_d = self.compute_lookahead_distance(l_d_speed_gain, min_l_d, max_l_d)
        else:
            raise Exception("Must pass required args for lookahead distance")

    def get_closest_point(self):
        """
        Get the waypoint closest to the vehicle position.

        args:
            self.dists (np.ndarray): Shape (N,), distances between waypoints
                and vehicle position

        returns:
            int: index of waypoint closest to vehicle
            np.ndarray: Shape (2,), waypoint closest to vehicle
            float: distance of waypoint to vehicle
        """
        c_i = np.argmin(self.dists)
        return c_i, self.waypoints[c_i], self.dists[c_i]

    def compute_lookahead_distance(self, Kd=None, min_l_d=None, max_l_d=None):
        """
        Computes the lookahead distance using the formula
            l_d = clip(Kd*speed, min=min_l_d, max=max_l_d)
        where Kd, min_l_d, max_l_d are parameters to be tuned.

        args:
            Kd (float): speed to lookahead distance gain
            min_l_d (float): minimum lookahead distance, assumed > 0
            max_l_d (float): maximum lookahead distance, assumed > 0
        """
        if Kd and min_l_d and max_l_d:
            return np.clip(Kd * self.curr_speed, min_l_d, max_l_d)
        else:
            raise Exception("Incorrect args to compute_lookahead_distance")

    def get_intersection_point(self):
        """
        Get the point on the trajectory that intersects the lookahead circle.

        args:
            self.waypoints (np.ndarray): Shape (N, 2), waypoints to track
            self.dists (np.ndarray): Shape (N,), distances between waypoints
                and vehicle position
            self.veh_pos (np.ndarray): Shape (2,), vehicle position

        returns:
            bool: success in finding target point
            int: index of point prior to target point in waypoints
            np.ndarray: Shape (2,), target point x, y
            float: distance from vehicle to target point
        """
        dist_current = self.c_d
        for i in range(self.c_i, self.waypoints.shape[0] - 1):
            dist_next = self.dists[i + 1]
            if dist_current < self.l_d and dist_next > self.l_d:
                ratio = (self.l_d - dist_current) / (dist_next - dist_current)
                target_point = (
                    ratio * self.waypoints[i] + (1 - ratio) * self.waypoints[i + 1]
                )
                target_dist = np.linalg.norm(target_point - self.veh_pos)
                return True, i, target_point, target_dist
            dist_current = dist_next
        ind = self.waypoints.shape[0] - 1
        target_point = self.waypoints[ind]
        target_dist = self.dists[ind]
        return True, self.waypoints.shape[0] - 1, target_point, target_dist

    def get_actuation(self, target_point):
        """
        Determines the steering angle and speed based on the current vehicle
        pose, the calculated lookahead point, the lookahead distance, and
        wheelbase distance.

        args:
            target_point (np.ndarray): Shape (2,), point that intersects
                lookahead circle that we move towards
            self.veh_pos (np.ndarray): Shape (2,), vehicle position
            self.wheelbase (float): Distance between front and rear wheel axles

        """
        alpha = np.arctan2(
            target_point[1] - self.veh_pos[1], target_point[0] - self.veh_pos[0]
        )
        steering_angle = np.arctan2(2 * self.wheelbase * np.sin(alpha), self.l_d)
        return steering_angle, self.curr_speed

    def act(self):
        """
        Controller method --- should be only method visible to user.

        returns:
            float: steering angle
            float: speed
        """
        found_target, target_ind, target_point, target_dist = (
            self.get_intersection_point()
        )
        if not found_target:
            return 0.0, 0.0
        steer, speed = self.get_actuation(target_point)
        steer = self.steer_min if steer < self.steer_min else steer
        steer = self.steer_max if steer > self.steer_max else steer
        return steer, speed
