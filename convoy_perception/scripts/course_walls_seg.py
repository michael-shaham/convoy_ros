#! /usr/bin/env python3
""" @file course_walls_seg.py
    authors: Michael Shaham
    This file implements a node that uses the lidar data to find the walls of a 
    course, and publish them for use by a planner.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs_py import point_cloud2

from convoy_interfaces.msg import CourseData, NeighborInfo
from sensor_msgs.msg import LaserScan, PointField, PointCloud2

from convoy_perception import lidar_proc


"""
Subscribe to laser scan topic and publish points associated with the course 
walls. Note that since the laser scans are given in the frame of the lidar, we 
will need to convert the processed point cloud to the base_link frame for 
tracking (maybe we should convert to world or map frame?).
"""


class WallSegNode(Node):

    def __init__(self):
        super().__init__("course_walls_seg_node")

        # ROS parameters
        self.declare_parameter("veh_ns")
        self.declare_parameter("veh_frame")
        self.declare_parameter("processed_scan_topic")
        self.declare_parameter("course_data_topic")
        self.declare_parameter("left_wall_pc_topic")
        self.declare_parameter("right_wall_pc_topic")
        self.declare_parameter("neighbor_info_topic")
        self.declare_parameter("base_lidar_distance")
        self.declare_parameter("lidar_range_threshold")
        self.declare_parameter("dbscan_eps")
        self.declare_parameter("dbscan_min_samples")
        self.declare_parameter("min_cluster_size")
        self.declare_parameter("neighbor_ball_radius")

        veh_ns = self.get_parameter("veh_ns").value
        self.veh_frame = veh_ns + "/" + self.get_parameter("veh_frame").value
        scan_topic = self.get_parameter("processed_scan_topic").value
        data_topic = self.get_parameter("course_data_topic").value
        left_pc_topic = self.get_parameter("left_wall_pc_topic").value
        right_pc_topic = self.get_parameter("right_wall_pc_topic").value
        info_topic = self.get_parameter("neighbor_info_topic").value
        self.base_lidar_dist = self.get_parameter("base_lidar_distance").value
        self.lidar_threshold = self.get_parameter("lidar_range_threshold").value
        self.dbscan_eps = self.get_parameter("dbscan_eps").value
        self.dbscan_min_samples = self.get_parameter("dbscan_min_samples").value
        self.min_cluster_size = self.get_parameter("min_cluster_size").value
        self.r_neighbor = self.get_parameter("neighbor_ball_radius").value

        # ROS publishers
        self.course_data_pub = self.create_publisher(CourseData, data_topic, 1)
        self.left_pc_pub = self.create_publisher(PointCloud2, left_pc_topic, 1)
        self.right_pc_pub = self.create_publisher(PointCloud2, right_pc_topic, 1)

        # ROS subscribers
        self.create_subscription(NeighborInfo, info_topic, self.info_cb, 1)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 1)

        # helper variables
        self.neighbor_info = None
        self.angles = None
        self.empty_pc2 = PointCloud2()
        self.empty_pc2.header.frame_id = self.veh_frame
        self.prev_track_width = 0.0

    def info_cb(self, msg):
        self.neighbor_info = msg

    def scan_cb(self, msg):
        course_data = CourseData()
        course_data.header.frame_id = self.veh_frame
        course_data.header.stamp = msg.header.stamp

        if type(self.angles) != np.ndarray:
            self.angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        right_inds = np.logical_and(
            self.angles >= -np.pi / 2 - np.pi / 6, self.angles <= -np.pi / 2 + np.pi / 6
        )
        left_inds = np.logical_and(
            self.angles >= np.pi / 2 - np.pi / 6, self.angles <= np.pi / 2 + np.pi / 6
        )
        ranges = np.array(msg.ranges)
        right_closest_dist = min(ranges[right_inds])
        left_closest_dist = min(ranges[left_inds])
        track_width = float(left_closest_dist + right_closest_dist)

        proc_ranges, proc_angles = lidar_proc.threshold_laser_scan(
            msg.ranges, self.angles, self.lidar_threshold
        )

        pc = np.c_[proc_ranges * np.cos(proc_angles), proc_ranges * np.sin(proc_angles)]

        if self.neighbor_info:
            neighbor_pos = np.array(
                [
                    self.neighbor_info.relative_pose.position.x,
                    self.neighbor_info.relative_pose.position.y,
                ]
            )
            # since the pozyx currently only provides distances, the relative
            # position will always be zeros, so we skip this step
            if not np.isclose(neighbor_pos, np.zeros(2)).all():
                pc = lidar_proc.segment_neighbor_vehicle(
                    neighbor_pos, pc, self.r_neighbor
                )

        # convert points to base_link frame
        pc[:, 0] += self.base_lidar_dist

        # will need header and fields to create PointCloud2 objects
        header = msg.header
        header.frame_id = self.veh_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        clusters, _ = lidar_proc.cluster_pc_dbscan(
            pc, self.dbscan_eps, self.dbscan_min_samples
        )

        if len(clusters) == 0:
            self.get_logger().info("DBSCAN didn't find any clusters")
            course_data.left_wall = self.empty_pc2
            course_data.right_wall = self.empty_pc2
            course_data.track_width = track_width
            self.left_pc_pub.publish(self.empty_pc2)
            self.right_pc_pub.publish(self.empty_pc2)
            self.course_data_pub.publish(course_data)
            return
        elif len(clusters) == 1:
            self.get_logger().info("DBSCAN found only one cluster")
            cluster = np.c_[clusters[0], np.zeros(clusters[0].shape[0])]
            # convert cluster to point cloud
            cluster_pc2 = point_cloud2.create_cloud(header, fields, cluster)

            # determine if pc corresponds to left or right wall:
            # if closest point is on right (y > 0), then it's right wall, else left
            closest_point, _ = lidar_proc.closest_point_to_origin(cluster)
            if closest_point[1] > 0:
                course_data.left_wall = self.empty_pc2
                course_data.right_wall = cluster_pc2
            else:
                course_data.left_wall = cluster_pc2
                course_data.right_wall = self.empty_pc2
            course_data.track_width = track_width
            self.left_pc_pub.publish(course_data.left_wall)
            self.right_pc_pub.publish(course_data.right_wall)
            self.course_data_pub.publish(course_data)
            return

        pc_clusters = [c for c in clusters if c.shape[0] > self.min_cluster_size]
        left_clusters = []
        right_clusters = []
        for i, c in enumerate(pc_clusters):
            closest_point, _ = lidar_proc.closest_point_to_origin(c)
            if closest_point[1] < 0:
                right_clusters.append(c)
            else:
                left_clusters.append(c)

        left_pc = lidar_proc.merge_clusters(left_clusters, track_width)
        right_pc = lidar_proc.merge_clusters(right_clusters, track_width)

        left_pc = np.c_[left_pc, np.zeros(left_pc.shape[0])]
        right_pc = np.c_[right_pc, np.zeros(right_pc.shape[0])][::-1]

        left_pc2 = point_cloud2.create_cloud(header, fields, left_pc)
        right_pc2 = point_cloud2.create_cloud(header, fields, right_pc)

        left_min_dist = np.min(np.linalg.norm(left_pc, axis=1))
        right_min_dist = np.min(np.linalg.norm(right_pc, axis=1))
        # track_width = left_min_dist + right_min_dist

        if left_pc.shape[0] < self.min_cluster_size:
            left_pc2 = self.empty_pc2
            course_data.track_width = self.prev_track_width
        elif left_min_dist > track_width:
            left_pc2 = self.empty_pc2
            course_data.track_width = self.prev_track_width
        if right_pc.shape[0] < self.min_cluster_size:
            right_pc2 = self.empty_pc2
            course_data.track_width = self.prev_track_width
        elif right_min_dist > track_width:
            right_pc2 = self.empty_pc2
            course_data.track_width = self.prev_track_width

        course_data.left_wall = left_pc2
        course_data.right_wall = right_pc2
        course_data.track_width = track_width
        course_data.left_wall_dist = float(left_closest_dist)
        course_data.right_wall_dist = float(right_closest_dist)
        self.left_pc_pub.publish(course_data.left_wall)
        self.right_pc_pub.publish(course_data.right_wall)
        self.course_data_pub.publish(course_data)


def main(args=None):
    rclpy.init(args=args)
    wall_seg_node = WallSegNode()
    rclpy.spin(wall_seg_node)
    wall_seg_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
