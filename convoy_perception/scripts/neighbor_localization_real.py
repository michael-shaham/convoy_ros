#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import NeighborInfo
from pozyx_interfaces.msg import PozyxRangeArray
from nav_msgs.msg import Odometry


class NeighborLocalizationReal(Node):

    def __init__(self):
        super().__init__("neighbor_localization_node")

        # ROS parameters
        self.declare_parameter("veh_frame")
        self.declare_parameter("odom_topic")
        self.declare_parameter("neighbor_info_topic")
        self.declare_parameter("preceding_vehicle_ind")
        self.declare_parameter("desired_distance")
        self.declare_parameter("pozyx_namespace")
        self.declare_parameter("pozyx_range_topic")
        self.declare_parameter("remote_id")
        self.declare_parameter("vehicle_length")

        self.veh_frame = self.get_parameter("veh_frame").value
        odom_topic = self.get_parameter("odom_topic").value
        info_topic = self.get_parameter("neighbor_info_topic").value
        self.prec_ind = self.get_parameter("preceding_vehicle_ind").value
        self.desired_distance = self.get_parameter("desired_distance").value
        pozyx_ns = self.get_parameter("pozyx_namespace").value
        range_topic = pozyx_ns + "/" + self.get_parameter("pozyx_range_topic").value
        self.remote_id = self.get_parameter("remote_id").value
        self.remote_id = hex(self.remote_id)
        self.veh_length = self.get_parameter("vehicle_length").value

        # ROS publishers
        self.info_pub = self.create_publisher(NeighborInfo, info_topic, 10)

        # ROS subscribers
        prec_odom_topic = "/" + "veh_" + str(self.prec_ind) + "/" + odom_topic
        self.create_subscription(Odometry, prec_odom_topic, self.prec_odom_cb, 1)
        self.prec_speed = 0.0

        self.create_subscription(PozyxRangeArray, range_topic, self.pozyx_cb, 1)
        self.prec_dist = 0.0

    def prec_odom_cb(self, msg: Odometry):
        # neighbor speed
        self.prec_speed = np.sqrt(
            msg.twist.twist.linear.x**2
            + msg.twist.twist.linear.y**2
            + msg.twist.twist.linear.z**2
        )

    def pozyx_cb(self, msg: PozyxRangeArray):
        for pozyx_range in msg.pozyx_ranges:
            if pozyx_range.remote_id == self.remote_id:
                self.prec_dist = pozyx_range.distance - self.veh_length
            else:
                self.prec_dist = self.desired_distance
        neighbor_info = NeighborInfo()
        neighbor_info.header.frame_id = self.veh_frame
        neighbor_info.header.stamp = self.get_clock().now().to_msg()
        neighbor_info.vehicle_ind = self.prec_ind
        neighbor_info.distance = self.prec_dist
        neighbor_info.speed = self.prec_speed
        self.info_pub.publish(neighbor_info)


def main(args=None):
    rclpy.init(args=args)
    neighbor_localization_node = NeighborLocalizationReal()
    rclpy.spin(neighbor_localization_node)
    neighbor_localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
