#!/usr/bin/env python3
""" @file vl_track.py
    authors: Risha Ranjan
    Tracks the trajectory published by the virtual leader 
"""

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from convoy_interfaces.msg import (
    LongitudinalControl,
    LongitudinalStateTrajectory,
    ConvoyControlData,
)


class TrackVL(Node):

    def __init__(self):
        super().__init__("track_vl")

        # ROS parameters
        vl_traj_topic = (
            self.declare_parameter("virtual_leader_traj_topic")
            .get_parameter_value()
            .string_value
        )
        long_ctrl_topic = (
            self.declare_parameter("longitudinal_control_topic")
            .get_parameter_value()
            .string_value
        )
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        control_data_topic = (
            self.declare_parameter("convoy_control_data_topic")
            .get_parameter_value()
            .string_value
        )

        # ROS subscribers
        self.create_subscription(
            LongitudinalStateTrajectory, vl_traj_topic, self.vl_cb, 1
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)
        self.data_pub = self.create_publisher(ConvoyControlData, control_data_topic, 1)

        self.curr_speed = 0.0

    def vl_cb(self, msg: LongitudinalStateTrajectory):

        des_state = msg.longitudinal_trajectory[0]
        long_control_msg = LongitudinalControl()
        long_control_msg.speed = des_state.velocity
        long_control_msg.acceleration = des_state.acceleration
        self.long_pub.publish(long_control_msg)

        control_data_msg = ConvoyControlData()
        control_data_msg.header = msg.header
        control_data_msg.ego_speed = self.curr_speed
        control_data_msg.opt_input = des_state.acceleration
        self.data_pub.publish(control_data_msg)

    def odom_cb(self, msg: Odometry):
        self.curr_speed = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
    node = TrackVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
