#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import NeighborInfo
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from transforms3d.euler import quat2euler, euler2quat


class NeighborLocalizationSim(Node):

    def __init__(self):
        super().__init__("neighbor_localization_node")

        # ROS parameters
        self.declare_parameter("odom_topic")
        self.declare_parameter("neighbor_info_topic")
        self.declare_parameter("preceding_vehicle_ind")

        odom_topic = self.get_parameter("odom_topic").value
        info_topic = self.get_parameter("neighbor_info_topic").value
        self.prec_ind = self.get_parameter("preceding_vehicle_ind").value
        prec_odom_topic = "/" + "veh_" + str(self.prec_ind) + "/" + odom_topic

        # ROS publishers
        neighbor_timer_period = 1 / 250
        self.info_timer = self.create_timer(neighbor_timer_period, self.neighbor_pub_cb)
        self.info_pub = self.create_publisher(NeighborInfo, info_topic, 10)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.ego_odom_cb, 1)
        self.create_subscription(Odometry, prec_odom_topic, self.prec_odom_cb, 1)

        # needed variables
        self.prec_speed = None
        self.neighbor_pose = None
        self.ego_pose = Pose()
        self.ego_frame = ""
        self.ego_R = np.eye(2)
        self.neighbor_pose = Pose()

        self.ego_updated = False
        self.neighbor_updated = False

    def neighbor_pub_cb(self):
        if self.prec_speed == None:
            return
        neighbor_info = NeighborInfo()
        neighbor_info.header.stamp = self.get_clock().now().to_msg()
        neighbor_info.header.frame_id = self.ego_frame
        neighbor_info.vehicle_ind = self.prec_ind

        # neighbor speed
        neighbor_info.speed = self.prec_speed

        # neighbor relative position
        relative_position = self.ego_R @ np.array(
            [
                self.neighbor_pose.position.x - self.ego_pose.position.x,
                self.neighbor_pose.position.y - self.ego_pose.position.y,
            ]
        )
        neighbor_info.relative_pose.position.x = relative_position[0]
        neighbor_info.relative_pose.position.y = relative_position[1]
        neighbor_info.relative_pose.position.z = 0.0

        # neighbor distance
        neighbor_info.distance = np.linalg.norm(relative_position)

        # neighbor relative orientation
        ego_yaw = quat2euler(
            [
                self.ego_pose.orientation.w,
                self.ego_pose.orientation.x,
                self.ego_pose.orientation.y,
                self.ego_pose.orientation.z,
            ]
        )[2]
        prec_yaw = quat2euler(
            [
                self.neighbor_pose.orientation.w,
                self.neighbor_pose.orientation.x,
                self.neighbor_pose.orientation.y,
                self.neighbor_pose.orientation.z,
            ]
        )[2]
        relative_yaw = prec_yaw - ego_yaw
        relative_quat = euler2quat(0.0, 0.0, relative_yaw)
        neighbor_info.relative_pose.orientation.w = relative_quat[0]
        neighbor_info.relative_pose.orientation.x = relative_quat[1]
        neighbor_info.relative_pose.orientation.y = relative_quat[2]
        neighbor_info.relative_pose.orientation.z = relative_quat[3]

        if self.ego_updated and self.neighbor_updated:
            self.info_pub.publish(neighbor_info)
            self.ego_updated = False
            self.neighbor_updated = False

    def ego_odom_cb(self, msg):
        if not self.ego_updated:
            self.ego_frame = msg.child_frame_id
            self.ego_pose = msg.pose.pose
            ego_heading = quat2euler(
                [
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ]
            )[2]
            self.ego_R = np.array(
                [
                    [np.cos(ego_heading), np.sin(ego_heading)],
                    [-np.sin(ego_heading), np.cos(ego_heading)],
                ]
            )
            self.ego_updated = True

    def prec_odom_cb(self, msg):
        if not self.neighbor_updated:
            self.neighbor_pose = msg.pose.pose
            # neighbor speed
            self.prec_speed = np.sqrt(
                msg.twist.twist.linear.x**2
                + msg.twist.twist.linear.y**2
                + msg.twist.twist.linear.z**2
            )
            self.neighbor_updated = True


def main(args=None):
    rclpy.init(args=args)
    neighbor_localization_node = NeighborLocalizationSim()
    rclpy.spin(neighbor_localization_node)
    neighbor_localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
