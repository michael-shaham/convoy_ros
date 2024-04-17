#! /usr/bin/env python3

import os
import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from convoy_interfaces.msg import CourseData, NeighborInfo
from nav_msgs.msg import Odometry


class BagProcess(Node):

    def __init__(self):
        super().__init__("bag_process")

        # ROS parameters
        self.declare_parameter("veh_ns", "")
        self.declare_parameter("test_name", "")
        self.veh_ns = self.get_parameter("veh_ns").value
        self.test_name = self.get_parameter("test_name").value

        # ROS subscribers
        self.create_subscription(AckermannDriveStamped, "drive", self.drive_cb, 100)
        self.create_subscription(
            NeighborInfo, "neighbor_info", self.neighbor_info_cb, 100
        )
        self.create_subscription(Odometry, "odom", self.odom_cb, 100)

        self.drive_times = []
        self.drive_speeds = []
        self.drive_accels = []
        self.drive_steers = []

        self.neighbor_times = []
        self.neighbor_dists = []
        self.neighbor_speeds = []

        self.speed_times = []
        self.speeds = []

    def drive_cb(self, msg):
        self.drive_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        self.drive_speeds.append(msg.drive.speed)
        self.drive_accels.append(msg.drive.acceleration)
        self.drive_steers.append(msg.drive.steering_angle)

    def neighbor_info_cb(self, msg):
        self.neighbor_times.append(
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        )
        self.neighbor_dists.append(msg.distance)
        self.neighbor_speeds.append(msg.speed)

    def odom_cb(self, msg):
        self.speed_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        self.speeds.append(
            np.linalg.norm(
                np.array(
                    [
                        msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z,
                    ]
                )
            )
        )

    def on_shutdown(self):
        save_location = os.path.join(
            os.path.expanduser("~"),
            "cvy_ws",
            "src",
            "convoy_ros",
            "data_analysis",
            "data",
            "sim",
            self.test_name,
        )
        print("Saving bag data.")

        drive_data_file = os.path.join(save_location, f"{self.veh_ns}_drive_data.csv")
        drive_data = np.array(
            [self.drive_times, self.drive_speeds, self.drive_accels, self.drive_steers]
        )
        np.savetxt(drive_data_file, drive_data)

        neighbor_data_file = os.path.join(
            save_location, f"{self.veh_ns}_neighbor_data.csv"
        )
        neighbor_data = np.array(
            [self.neighbor_times, self.neighbor_dists, self.neighbor_speeds]
        )
        np.savetxt(neighbor_data_file, neighbor_data)

        speed_data_file = os.path.join(save_location, f"{self.veh_ns}_speed_data.csv")
        speed_data = np.array([self.speed_times, self.speeds])
        np.savetxt(speed_data_file, speed_data)


def main(args=None):
    rclpy.init(args=args)
    bag_process = BagProcess()
    try:
        rclpy.spin(bag_process)
    except KeyboardInterrupt:
        pass
    bag_process.on_shutdown()
    bag_process.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
