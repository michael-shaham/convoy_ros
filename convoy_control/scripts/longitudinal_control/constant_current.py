#!/usr/bin/env python3
""" @file constant_current.py
    authors: Michael Shaham, Risha Ranjan
    Implements a node that makes the vehicle drive with constant current.
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from convoy_interfaces.msg import ConvoyControlData


class ConstantCurrent(Node):

    def __init__(self):
        super().__init__("constant_current_node")

        # ROS publishers
        self.curr_pub = self.create_publisher(Float64, "commands/motor/current", 1)
        self.brake_pub = self.create_publisher(Float64, "commands/motor/brake", 1)
        self.curr_timer = self.create_timer(0.1, self.pub_cb)

        self.data_pub = self.create_publisher(
            ConvoyControlData, "convoy_control_data", 1
        )

        # ROS subscribers
        self.create_subscription(Odometry, "odom", self.odom_cb, 1)
        self.speed = 0.0

    def pub_cb(self):
        stamp = self.get_clock().now().to_msg()

        current = 10.0
        msg = Float64()
        msg.data = current
        self.curr_pub.publish(msg)

        msg = ConvoyControlData()
        msg.header.stamp = stamp
        msg.ego_speed = self.speed
        msg.opt_input = current
        self.data_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
    node = ConstantCurrent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
