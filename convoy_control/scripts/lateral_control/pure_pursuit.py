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

from convoy_control.pure_pursuit.pure_pursuit import PurePursuit


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__("pure_pursuit_node")

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
        self.const_l_d = (
            self.declare_parameter("constant_lookahead_distance")
            .get_parameter_value()
            .double_value
        )
        self.l_d_speed_gain = (
            self.declare_parameter("lookahead_speed_gain")
            .get_parameter_value()
            .double_value
        )
        self.min_l_d = (
            self.declare_parameter("min_lookahead").get_parameter_value().double_value
        )
        self.max_l_d = (
            self.declare_parameter("max_lookahead").get_parameter_value().double_value
        )
        self.steer_min = (
            self.declare_parameter("steer_min").get_parameter_value().double_value
        )
        self.steer_max = (
            self.declare_parameter("steer_max").get_parameter_value().double_value
        )
        self.wheelbase = (
            self.declare_parameter("wheelbase").get_parameter_value().double_value
        )

        # ROS publishers
        self.lat_pub = self.create_publisher(LateralControl, lat_control_topic, 1)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(Trajectory, pt_topic, self.traj_cb, 1)

        # member variables
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
        waypoints = np.array([[p.position.x, p.position.y] for p in msg.poses])

        controller = PurePursuit(
            waypoints=waypoints,
            steer_min=self.steer_min,
            steer_max=self.steer_max,
            l_d_speed_gain=self.l_d_speed_gain,
            min_l_d=self.min_l_d,
            max_l_d=self.max_l_d,
            veh_speed=self.speed,
            wheelbase=self.wheelbase,
        )

        steer, _ = controller.act()

        lat_control_msg = LateralControl()
        lat_control_msg.steering_angle = steer
        self.lat_pub.publish(lat_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
