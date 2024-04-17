#! /usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class PreprocessLidar(Node):

    def __init__(self):
        super().__init__("preprocess_lidar_node")

        # ROS parameters
        self.declare_parameter("scan_topic", "")
        self.declare_parameter("processed_scan_topic", "")
        self.declare_parameter("fov_deg", 0.0)

        scan_topic = self.get_parameter("scan_topic").value
        proc_scan_topic = self.get_parameter("processed_scan_topic").value

        self.fov_rad = self.get_parameter("fov_deg").value * np.pi / 180.0
        self.min_angle = -self.fov_rad / 2.0
        self.max_angle = self.fov_rad / 2.0

        self.found_new_range = False
        self.min_ind = None
        self.max_ind = None

        # ROS publishers
        self.proc_scan_pub = self.create_publisher(LaserScan, proc_scan_topic, 1)

        # ROS subscribers
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 1)

    def scan_cb(self, msg: LaserScan):
        fov_og = msg.angle_max - msg.angle_min

        if np.isclose(fov_og, self.fov_rad, rtol=0.0, atol=1e-2):
            self.proc_scan_pub.publish(msg)
            return

        if not self.found_new_range:
            self.found_new = True
            angles = np.arange(
                msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment
            )
            self.min_ind = np.abs(angles - self.min_angle).argmin()
            self.max_ind = np.abs(angles - self.max_angle).argmin()
            self.min_angle = angles[self.min_ind]
            self.max_angle = angles[self.max_ind]
            self.min_range = msg.ranges[self.min_ind]
            self.max_range = msg.ranges[self.max_ind]

        scan_msg = LaserScan()
        scan_msg.header = msg.header
        scan_msg.angle_min = self.min_angle
        scan_msg.angle_max = self.max_angle
        scan_msg.angle_increment = msg.angle_increment
        scan_msg.time_increment = msg.time_increment
        scan_msg.scan_time = msg.scan_time
        scan_msg.range_min = msg.range_min
        scan_msg.range_max = msg.range_max
        scan_msg.ranges = msg.ranges[self.min_ind : (self.max_ind + 1)]
        self.proc_scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    preprocess_lidar = PreprocessLidar()
    rclpy.spin(preprocess_lidar)
    preprocess_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
