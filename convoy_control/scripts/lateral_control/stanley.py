#!/usr/bin/env python3
""" @file pure_pursuit.py
    authors: Michael Shaham
    Implements a node that uses the pure_pursuit.py module to find desired 
    steering angles.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import LateralControl, Trajectory
from nav_msgs.msg import Odometry

from convoy_control.stanley.stanley import Stanley


class StanleyNode(Node):

    def __init__(self):
        super().__init__("stanley_node")

        # ROS parameters

        lat_control_topic = (
            self.declare_parameter("lateral_control_topic")
            .get_parameter_value()
            .string_value
        )
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        pt_topic = (
            self.declare_parameter("planner_trajectory_topic")
            .get_parameter_value()
            .string_value
        )
        k_e = (
            self.declare_parameter("crosstrack_error_gain")
            .get_parameter_value()
            .double_value
        )
        k_s = self.declare_parameter("softness_gain").get_parameter_value().double_value
        steer_min = (
            self.declare_parameter("steer_min").get_parameter_value().double_value
        )
        steer_max = (
            self.declare_parameter("steer_max").get_parameter_value().double_value
        )

        # ROS publishers
        self.lat_pub = self.create_publisher(LateralControl, lat_control_topic, 1)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(Trajectory, pt_topic, self.traj_cb, 1)

        # member variables
        self.lat_controller = Stanley(k_e, k_s, steer_min, steer_max)
        self.speed = 0.0

    def odom_cb(self, msg: Odometry):
        self.speed = np.linalg.norm(
            np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]
            )
        )

    def traj_cb(self, msg: Trajectory):
        steer = self.lat_controller.control(
            msg.crosstrack_error, msg.heading_error, self.speed
        )
        lat_control_msg = LateralControl()
        lat_control_msg.steering_angle = steer
        self.lat_pub.publish(lat_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
