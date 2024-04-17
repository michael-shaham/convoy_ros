#!/usr/bin/env python3
""" @file pure_pursuit_head.py
    authors: Michael Shaham
    Implements a node that uses the pure_pursuit_head.py module to perform path 
    tracking of a trajectory.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from convoy_interfaces.msg import LateralControl, LongitudinalControl, SafeSpeed


class DriveControllerNode(Node):

    def __init__(self):
        super().__init__("drive_controller_node")

        # ROS parameters
        veh_ns = self.declare_parameter("veh_ns").get_parameter_value().string_value
        veh_frame = (
            self.declare_parameter("veh_frame").get_parameter_value().string_value
        )
        drive_topic = (
            self.declare_parameter("safe_drive_topic")
            .get_parameter_value()
            .string_value
        )
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        lat_control_topic = (
            self.declare_parameter("lateral_control_topic")
            .get_parameter_value()
            .string_value
        )
        long_control_topic = (
            self.declare_parameter("longitudinal_control_topic")
            .get_parameter_value()
            .string_value
        )
        # dynamics parameters
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
        # be close to speed designated by safe speed node
        self.max_safe_speed_deviation = (
            self.declare_parameter("max_safe_speed_deviation")
            .get_parameter_value()
            .double_value
        )
        safe_speed_topic = (
            self.declare_parameter("safe_speed_topic")
            .get_parameter_value()
            .string_value
        )

        # ROS publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        # ROS subscribers
        self.create_subscription(LateralControl, lat_control_topic, self.lat_cb, 1)
        self.create_subscription(
            LongitudinalControl, long_control_topic, self.long_cb, 1
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(SafeSpeed, safe_speed_topic, self.speed_cb, 1)

        # member variables
        self.curr_speed = 0.0
        self.safe_speed = 0.0
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = veh_ns + "/" + veh_frame

    def lat_cb(self, msg: LateralControl):
        self.drive_msg.drive.steering_angle = msg.steering_angle
        self.drive_msg.drive.steering_angle_velocity = msg.steering_angle_velocity
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_pub.publish(self.drive_msg)

    def long_cb(self, msg: LongitudinalControl):
        speed = np.clip(msg.speed, 0.0, self.speed_max)
        speed = np.clip(
            speed, a_min=None, a_max=self.safe_speed + self.max_safe_speed_deviation
        )
        accel = 0.0 if speed < 0.0 else msg.acceleration
        accel = np.clip(accel, self.accel_min, self.accel_max)

        self.drive_msg.drive.speed = speed
        self.drive_msg.drive.acceleration = accel
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_pub.publish(self.drive_msg)

    def odom_cb(self, msg: Odometry):
        self.curr_speed = np.linalg.norm(
            np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]
            )
        )

    def speed_cb(self, msg: SafeSpeed):
        self.safe_speed = msg.safe_speed


def main(args=None):
    rclpy.init(args=args)
    node = DriveControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
