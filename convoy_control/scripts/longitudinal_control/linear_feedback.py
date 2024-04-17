#!/usr/bin/env python3
""" @file linear_feedback.py
    authors: Michael Shaham
    Uses linear feedback on positions/speeds of neighbor vehicle(s) to select 
    accelerations for platoon.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import (
    DesiredDistance,
    LongitudinalControl,
    ConvoyControlData,
    NeighborInfo,
)
from nav_msgs.msg import Odometry

from convoy_control.linear_feedback.linear_feedback import LinearFeedback


class LinearFeedbackNode(Node):

    def __init__(self):
        super().__init__("linear_feedback_node")

        # ROS parameters
        self.use_sim = (
            self.declare_parameter("use_sim").get_parameter_value().bool_value
        )
        long_ctrl_topic = (
            self.declare_parameter("longitudinal_control_topic")
            .get_parameter_value()
            .string_value
        )
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        neighbor_info_topic = (
            self.declare_parameter("neighbor_info_topic")
            .get_parameter_value()
            .string_value
        )
        d_des_topic = (
            self.declare_parameter("desired_distance_topic")
            .get_parameter_value()
            .string_value
        )
        control_data_topic = (
            self.declare_parameter("convoy_control_data_topic")
            .get_parameter_value()
            .string_value
        )
        # dynamics params
        self.dt = self.declare_parameter("dt").get_parameter_value().double_value
        self.speed_min = (
            self.declare_parameter("speed_min").get_parameter_value().double_value
        )
        self.speed_max = (
            self.declare_parameter("speed_max").get_parameter_value().double_value
        )
        self.accel_min = (
            self.declare_parameter("accel_min").get_parameter_value().double_value
        )
        self.accel_max = (
            self.declare_parameter("accel_max").get_parameter_value().double_value
        )
        # linear feedback params
        self.d_des = (
            self.declare_parameter("desired_distance")
            .get_parameter_value()
            .double_value
        )
        k_p_a = (
            self.declare_parameter("relative_position_gain_accel")
            .get_parameter_value()
            .double_value
        )
        k_s_a = (
            self.declare_parameter("relative_speed_gain_accel")
            .get_parameter_value()
            .double_value
        )
        k_p_v = (
            self.declare_parameter("relative_position_gain_vel")
            .get_parameter_value()
            .double_value
        )
        k_s_v = (
            self.declare_parameter("relative_speed_gain_vel")
            .get_parameter_value()
            .double_value
        )

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)
        self.data_pub = self.create_publisher(ConvoyControlData, control_data_topic, 1)

        # ROS subscribers
        self.create_subscription(DesiredDistance, d_des_topic, self.d_des_cb, 1)
        self.create_subscription(
            NeighborInfo, neighbor_info_topic, self.neighbor_info_cb, 1
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)

        # variables
        if self.use_sim:
            self.long_controller = LinearFeedback(k_p_a, k_s_a)
        else:
            self.long_controller = LinearFeedback(k_p_v, k_s_v)
        self.speed = 0.0
        self.distance_error = 0.0

    def d_des_cb(self, msg: DesiredDistance):
        self.d_des = msg.desired_distance

    def neighbor_info_cb(self, msg: NeighborInfo):
        self.distance_error = self.d_des - msg.distance
        self.speed_error = self.speed - msg.speed

        input = self.long_controller.control(self.distance_error, self.speed_error)
        if self.use_sim:
            des_speed = 0.0
            des_accel = input
        else:
            if self.speed < 0.8:
                des_speed = self.speed + 5.0 * input
            else:
                des_speed = self.speed + input
            des_speed = self.speed_min if des_speed < self.speed_min else des_speed
            des_speed = self.speed_max if des_speed > self.speed_max else des_speed
            des_accel = 0.0

        # self.get_logger().info(f"\nlinear feedback info:" +
        #                        f"\ndesired speed:  {des_speed}" +
        #                        f"\ndesired accel:  {des_accel}")

        long_control_msg = LongitudinalControl()
        long_control_msg.speed = des_speed
        long_control_msg.acceleration = des_accel
        self.long_pub.publish(long_control_msg)

        control_data_msg = ConvoyControlData()
        control_data_msg.header = msg.header
        control_data_msg.desired_distance = self.d_des
        control_data_msg.predecessor_distance = msg.distance
        control_data_msg.distance_error = self.distance_error
        control_data_msg.ego_speed = self.speed
        control_data_msg.predecessor_speed = msg.speed
        control_data_msg.speed_error = self.speed_error
        if self.use_sim:
            control_data_msg.opt_input = des_accel
        else:
            control_data_msg.opt_input = des_speed
        self.data_pub.publish(control_data_msg)

    def odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
    node = LinearFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
