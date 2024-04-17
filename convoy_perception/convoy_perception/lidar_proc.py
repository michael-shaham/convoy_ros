#!/usr/bin/env python3
""" @file lidar_proc.py
    author: Michael Shaham
    This file contains functions for processing 2D lidar data. Specifically, we 
    use 2D lidar data to find a center line in a race course for a 1/10th scale 
    vehicle. These functions will get used in a node that processes lidar data.
"""

import numpy as np
from sklearn.cluster import DBSCAN


def threshold_laser_scan(ranges, angles, r):
    """
    Removes points in laser scan that are further than r meters away.

    args:
        ranges (np.ndarray): Shape (L,), laser scan range data
        angles (np.ndarray): Shape (L,), corresponding angles for laser data
        r (float): threshold distance

    returns:
        np.ndarray: Shape (L_new,), new set of ranges (all <= r)
        np.ndarray: Shape (L_new,), angles corresponding to new ranges
    """
    ranges, angles = np.array(ranges), np.array(angles)
    assert (
        len(ranges.shape) == 1 and len(angles.shape) == 1
    ), "ranges and angles must be of shape (L, 1)"
    assert len(ranges) == len(angles), "ranges and angles must have same size"

    keep_inds = ranges <= r
    return ranges[keep_inds], angles[keep_inds]


def segment_neighbor_vehicle(neighbor_pos, pc, r):
    """
    Removes points in point cloud that are within r meters of the neighbor.

    args:
        neighbor_pos (np.ndarray): Shape (2,), position of the neighbor vehicle
            with respect to the ego vehicle
        pc (np.ndarray): Shape (L, 2), point cloud data, first column is x data,
            second column is y data
        r (float): threshold distance

    returns:
        np.ndarray: Shape (L_new, 2), point cloud data with the points near the
            neighbor removed
    """
    neighbor_pos, pc = np.array(neighbor_pos).flatten(), np.array(pc)
    assert len(neighbor_pos) == 2, "neighbor_pos should be np.array([x, y])"
    assert len(pc.shape) == 2, "pc should have two dimensions"
    assert pc.shape[1] == 2, "pc should be shape (L, 2)"

    dists = np.linalg.norm(pc - neighbor_pos, axis=1)
    keep_inds = dists > r
    return pc[keep_inds]


def cluster_pc_dbscan(pc, eps, min_samples):
    """
    Cluster nearby lidar points using DBSCAN

    args:
        pc (np.ndarray): Shape (L, 2), point cloud data we cluster
    returns:
        List of np.ndarray: [..., Shape (L_i, 2), ...], list containing arrays
            of clusters discovered by DBSCAN algorithm
        np.ndarray: Shape (N_outliers, 2), points in point cloud that did not
            get clustered during DBSCAN algorithm
    """
    pc = np.array(pc)
    assert pc.shape[0] > 3, "pc should have at least 4 points"
    assert len(pc.shape) == 2, "pc should have two dimensions"
    assert pc.shape[1] == 2, "pc should be shape (L, 2)"

    dbscan = DBSCAN(eps=eps, min_samples=min_samples, algorithm="ball_tree").fit(pc)
    num_clusters = np.max(dbscan.labels_) + 1
    outliers = pc[dbscan.labels_ == -1]
    clusters = [pc[dbscan.labels_ == i] for i in range(num_clusters)]

    return clusters, outliers


def closest_point_to_origin(pc: np.ndarray):
    """
    Finds point in point cloud closest to origin.

    args:
        pc (np.ndarray): Shape (L, 2), point cloud data we cluster
    returns:
        np.ndarray: shape (2,), closest point
        int: index of point in pc
    """
    dists = np.linalg.norm(pc, axis=1)
    closest_ind = np.argmin(dists)
    return pc[closest_ind], closest_ind


def merge_clusters(cluster_list, track_width):
    """
    Merges clusters in list if we believe they belong to the same wall.

    args:
        list[np.ndarray]: list of clusters that belong to a particular wall
    returns:
        np.ndarray: merged points when it makes sense to merge
    """
    if len(cluster_list) == 0:
        return np.empty((1, 2))
    elif len(cluster_list) == 1:
        return cluster_list[0]

    min_dists = [np.linalg.norm(closest_point_to_origin(c)[0]) for c in cluster_list]
    closest_cluster_ind = np.argmin(np.array(min_dists))
    # cluster_list = sorted(cluster_list, lambda pc:
    pc = cluster_list.pop(closest_cluster_ind)
    tol = track_width / 2
    for c in cluster_list:
        d1 = np.linalg.norm(c[0, :] - pc[0, :])
        d2 = np.linalg.norm(c[0, :] - pc[-1, :])
        d3 = np.linalg.norm(c[-1, :] - pc[0, :])
        d4 = np.linalg.norm(c[-1, :] - pc[-1, :])
        if d1 < tol or d2 < tol or d3 < tol or d4 < tol:
            pc_angs = np.arctan2(pc[:, 1], pc[:, 0])
            c_angs = np.arctan2(c[:, 1], c[:, 0])
            if pc_angs[-1] < c_angs[0] and c_angs[0] - pc_angs[-1] < np.pi / 6:
                pc = np.vstack((pc, c))
            elif pc_angs[0] > c_angs[-1] and pc_angs[0] - c_angs[-1] < np.pi / 6:
                pc = np.vstack((c, pc))
    return pc
