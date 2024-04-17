#!/usr/bin/env python3
""" @file constant_input.py
    authors: Michael Shaham
    Implements a node that makes the vehicle drive with constant input.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import ConvoyControlData, LongitudinalControl
from nav_msgs.msg import Odometry


class ConstantInputNode(Node):

    def __init__(self):
        super().__init__("constant_input_node")

        # ROS parameters
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        long_ctrl_topic = (
            self.declare_parameter("longitudinal_control_topic")
            .get_parameter_value()
            .string_value
        )
        data_topic = (
            self.declare_parameter("convoy_control_data_topic")
            .get_parameter_value()
            .string_value
        )
        self.speed_des = (
            self.declare_parameter("constant_speed").get_parameter_value().double_value
        )
        self.accel_des = (
            self.declare_parameter("constant_accel").get_parameter_value().double_value
        )
        dt = self.declare_parameter("dt").get_parameter_value().double_value
        self.speed_min = (
            self.declare_parameter("speed_min").get_parameter_value().double_value
        )
        self.speed_max = (
            self.declare_parameter("speed_max").get_parameter_value().double_value
        )

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)
        self.long_pub_timer = self.create_timer(0.02, self.long_pub_cb)
        self.data_pub = self.create_publisher(ConvoyControlData, data_topic, 1)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)

        self.curr_speed = 0.0
        self.counter = 0.0

        # member variables
        self.long_control_msg = LongitudinalControl()
        self.long_control_msg.speed = 0.0 if self.accel_des else self.speed_des
        self.long_control_msg.acceleration = self.accel_des

        self.vel_time = 2.0
        secs, nanosecs = self.get_clock().now().seconds_nanoseconds()
        self.start_time = secs + nanosecs * 1e-9

    def long_pub_cb(self):
        stamp = self.get_clock().now().to_msg()
        secs, nanosecs = self.get_clock().now().seconds_nanoseconds()
        curr_time = secs + nanosecs * 1e-9

        if curr_time - self.start_time >= self.vel_time:
            new_speed = np.random.uniform(self.speed_min, self.speed_max)
            self.get_logger().info(f"new_speed: {new_speed}")
            self.long_control_msg.speed = new_speed
            self.start_time = curr_time
        self.long_pub.publish(self.long_control_msg)

        data_msg = ConvoyControlData()
        data_msg.header.stamp = stamp
        data_msg.ego_speed = self.curr_speed
        data_msg.predecessor_speed = self.long_control_msg.speed
        data_msg.opt_input = self.accel_des
        self.data_pub.publish(data_msg)

    def odom_cb(self, msg: Odometry):
        self.curr_speed = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
    node = ConstantInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
