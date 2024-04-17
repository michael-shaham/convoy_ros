#!/usr/bin/env python3
""" @file plan_utils.py
    authors: Michael Shaham
    Provides modules or classes for converting data received from perception 
    into polynomial trajectories.
"""

import numpy as np

from numpy.polynomial.polynomial import Polynomial

from convoy_interfaces.msg import Trajectory
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray


def line_of_best_fit(points, deg):
    """
    Finds a polynomial (of degree d) line of best fit to the data given by
    points.

    args:
        points (np.ndarray): Shape (L, 2), L points of form (x, y)
        deg (int): Degree of the polynomial we will find

    returns:
        numpy.polynomial.polynomial.Polynomial: degree d
    """
    assert len(points.shape) == 2, "points should have two dimensions"
    assert points.shape[1] == 2, "points should be shape (L, 2)"
    assert type(deg) == int, "deg should be int"
    poly = Polynomial.fit(points[:, 0], points[:, 1], deg)
    return poly


def get_centerline(poly_1, poly_2):
    """
    Finds the polynomial curve that averages two other polynomials. This
    function is used to find the centerline between the two walls or lanes
    given by two clusters of points.

    args:
        poly_1 (Polynomial): numpy Polynomial object that approximates the
            point cloud for one of the walls or lanes
        poly_2 (Polynomial): numpy Polynomial object that approximates the
            point cloud for the other wall or lane

    returns:
        numpy Polynomial: numpy Polynomial object that is the average of the
            two polynomials
    """
    assert (
        type(poly_1) == Polynomial and type(poly_2) == Polynomial
    ), "poly_1 and poly_2 should be numpy Polynomial objects"
    assert poly_1.has_samedomain(poly_2) and poly_1.has_samewindow(
        poly_2
    ), "poly_1 and poly_2 should have same domain and window"
    return (poly_1 + poly_2) / 2


"""
Helper functions to create objects for publishing.
"""


def create_marker_array(points, color_msg, frame_id):
    marker_array = MarkerArray()
    for i in range(points.shape[0]):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = points[i, 0]
        marker.pose.position.y = points[i, 1]
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = color_msg
        marker_array.markers.append(marker)
    return marker_array


def create_trajectory_msg(points):
    traj = Trajectory()
    for p in points:
        pose = Pose()
        pose.position.x, pose.position.y = p[0], p[1]
        traj.poses.append(pose)
    return traj
