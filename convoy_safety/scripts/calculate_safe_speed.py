#!/usr/bin/env python3
""" @file calculate_safe_speed.py
    authors: Michael Shaham
    Calculates a safe speed based on the lidar scan.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs_py import point_cloud2

from convoy_interfaces.msg import CourseData, SafeSpeed


class CalculateSafeSpeed(Node):

    def __init__(self):
        super().__init__("safe_speed_node")

        # params
        safe_speed_topic = (
            self.declare_parameter("safe_speed_topic")
            .get_parameter_value()
            .string_value
        )
        course_topic = (
            self.declare_parameter("course_data_topic")
            .get_parameter_value()
            .string_value
        )
        self.min_traj_len = (
            self.declare_parameter("min_trajectory_length")
            .get_parameter_value()
            .double_value
        )
        self.max_traj_len = (
            self.declare_parameter("max_trajectory_length")
            .get_parameter_value()
            .double_value
        )
        self.min_safe_speed = (
            self.declare_parameter("min_safe_speed").get_parameter_value().double_value
        )
        self.max_safe_speed = (
            self.declare_parameter("max_safe_speed").get_parameter_value().double_value
        )

        # publisher
        self.speed_pub = self.create_publisher(SafeSpeed, safe_speed_topic, 1)

        # subscriber
        self.create_subscription(CourseData, course_topic, self.course_cb, 1)

    def course_cb(self, msg: CourseData):
        left_points_list = point_cloud2.read_points_list(msg.left_wall)
        left_pc = np.array(left_points_list)
        right_points_list = point_cloud2.read_points_list(msg.right_wall)
        right_pc = np.array(right_points_list)
        left_wall_len, right_wall_len = 0.0, 0.0
        if len(left_pc) > 0:
            left_pc = left_pc[:, :2]
            left_wall_len = np.sum(np.linalg.norm(left_pc[1:] - left_pc[:-1], axis=1))
        if len(right_pc) > 0:
            right_pc = right_pc[:, :2]
            right_wall_len = np.sum(
                np.linalg.norm(right_pc[1:] - right_pc[:-1], axis=1)
            )
        traj_len = min(left_wall_len, right_wall_len)
        traj_len = np.clip(traj_len, self.min_traj_len, self.max_traj_len)
        ratio = (traj_len - self.min_traj_len) / (self.max_traj_len - self.min_traj_len)
        vdes = ratio * (self.max_safe_speed - self.min_safe_speed) + self.min_safe_speed

        safe_speed_msg = SafeSpeed()
        safe_speed_msg.safe_speed = vdes
        self.speed_pub.publish(safe_speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalculateSafeSpeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
