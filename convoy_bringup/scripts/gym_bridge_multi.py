#!/usr/bin/env python3

# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

""" @file f1tenth_gym_multi.py
    authors: Michael Shaham
    Extends the f1tenth_gym_ros gym_bridge node to allow an arbitrary number of 
    vehicles.
"""

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import TransformBroadcaster

import copy
import gym
import numpy as np
from transforms3d.euler import euler2quat


class F1TenthVehicle:

    def __init__(
        self,
        gym_bridge_node,
        tf_broadcaster,
        veh_ns,
        veh_frame,
        lidar_frame,
        scan_topic,
        odom_topic,
        drive_topic,
    ):

        self.node = gym_bridge_node
        self.br = tf_broadcaster
        self.veh_ns = veh_ns
        self.veh_frame = veh_ns + "/" + veh_frame
        self.lidar_frame = veh_ns + "/" + lidar_frame

        self.scan_pub = self.node.create_publisher(
            LaserScan, veh_ns + "/" + scan_topic, 10
        )
        self.odom_pub = self.node.create_publisher(
            Odometry, veh_ns + "/" + odom_topic, 10
        )
        self.node.create_subscription(
            AckermannDriveStamped, veh_ns + "/" + drive_topic, self.drive_cb, 10
        )

        self.requested_speed = 0.0
        self.requested_accel = 0.0
        self.requested_steer = 0.0
        self.requested_steer_v = 0.0

    def drive_cb(self, drive_msg: AckermannDriveStamped):
        self.requested_speed = drive_msg.drive.speed
        self.requested_accel = drive_msg.drive.acceleration
        self.requested_steer = drive_msg.drive.steering_angle
        self.requested_steer_v = drive_msg.drive.steering_angle_velocity

    def publish_scan(self, scan_msg: LaserScan):
        scan_msg.header.frame_id = self.lidar_frame
        self.scan_pub.publish(scan_msg)

    def publish_odom(self, odom_msg: Odometry):
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.veh_frame
        self.odom_pub.publish(odom_msg)

    def publish_tfs(self, map_veh_tf, laser_tf, wheel_tf):

        map_veh_tf.child_frame_id = self.veh_frame

        laser_tf.header.frame_id = self.veh_frame
        laser_tf.child_frame_id = self.lidar_frame

        wheel_tf_left = copy.deepcopy(wheel_tf)
        wheel_tf_left.header.frame_id = self.veh_ns + "/front_left_hinge"
        wheel_tf_left.child_frame_id = self.veh_ns + "/front_left_wheel"
        wheel_tf_right = copy.deepcopy(wheel_tf)
        wheel_tf_right.header.frame_id = self.veh_ns + "/front_right_hinge"
        wheel_tf_right.child_frame_id = self.veh_ns + "/front_right_wheel"

        self.br.sendTransform(map_veh_tf)
        self.br.sendTransform(laser_tf)
        self.br.sendTransform(wheel_tf_left)
        self.br.sendTransform(wheel_tf_right)


class GymBridgeMulti(Node):

    def __init__(self):
        super().__init__("gym_bridge_multi")

        # ROS parameters
        self.declare_parameter("namespace").get_parameter_value().string_value
        self.declare_parameter("n_vehicles").get_parameter_value().integer_value
        self.declare_parameter("vehicle_frame").get_parameter_value().string_value
        self.declare_parameter("lidar_frame").get_parameter_value().string_value
        self.declare_parameter("scan_topic").get_parameter_value().string_value
        self.declare_parameter("odom_topic").get_parameter_value().string_value
        self.declare_parameter("drive_topic").get_parameter_value().string_value
        self.declare_parameter(
            "scan_distance_to_base_link"
        ).get_parameter_value().double_value
        self.declare_parameter("scan_fov").get_parameter_value().double_value
        self.declare_parameter("scan_beams").get_parameter_value().integer_value
        self.declare_parameter("range_min").get_parameter_value().double_value
        self.declare_parameter("range_max").get_parameter_value().double_value
        self.declare_parameter("map_path").get_parameter_value().string_value
        self.declare_parameter("map_img_ext").get_parameter_value().string_value
        self.declare_parameter("x_arr").get_parameter_value().double_array_value
        self.declare_parameter("y_arr").get_parameter_value().double_array_value
        self.declare_parameter("theta_arr").get_parameter_value().double_array_value

        veh_ns = self.get_parameter("namespace").value
        self.N = self.get_parameter("n_vehicles").value
        veh_frame = self.get_parameter("vehicle_frame").value
        lidar_frame = self.get_parameter("lidar_frame").value
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        drive_topic = self.get_parameter("drive_topic").value
        lidar_base_dist = self.get_parameter("scan_distance_to_base_link").value
        scan_fov = self.get_parameter("scan_fov").value
        scan_beams = self.get_parameter("scan_beams").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value
        x_arr = self.get_parameter("x_arr").value
        y_arr = self.get_parameter("y_arr").value
        theta_arr = self.get_parameter("theta_arr").value

        # scan parameters
        self.angle_min = -scan_fov / 2
        self.angle_max = scan_fov / 2
        self.angle_inc = scan_fov / scan_beams

        # create gym environment
        self.env = gym.make(
            "f110_gym:f110-v0",
            map=self.get_parameter("map_path").value,
            map_ext=self.get_parameter("map_img_ext").value,
            num_agents=self.N,
        )

        # gym env simulation step timer
        self.gym_timer = self.create_timer(0.01, self.gym_cb)
        # topic publishing timer
        self.pub_timer = self.create_timer(0.1, self.pub_cb)

        # set up variables for each vehicle that this class can access
        self.scans = [[]] * self.N
        self.twists = [np.zeros(3)] * self.N
        self.poses = [[x_arr[i], y_arr[i], theta_arr[i]] for i in range(self.N)]
        self.init_poses = [[x_arr[i], y_arr[i], theta_arr[i]] for i in range(self.N)]
        self.steering_angles = [0.0] * self.N
        self.drive_speeds = [0.0] * self.N

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # objects we publish
        self.scan_msgs = [LaserScan()] * self.N
        self.odom_msgs = [Odometry()] * self.N
        self.map_veh_tfs = [TransformStamped()] * self.N
        self.wheel_tfs = [TransformStamped()] * self.N

        self.laser_ts = TransformStamped()
        self.laser_ts.transform.translation.x = lidar_base_dist
        self.laser_ts.transform.rotation.w = 1.0
        self.laser_tfs = [self.laser_ts] * self.N

        # initialize F1TenthVehicle classes: used to publish odom and scan and
        # receive drive commands
        self.vehicles = []
        for i in range(self.N):
            self.vehicles.append(
                F1TenthVehicle(
                    gym_bridge_node=self,
                    tf_broadcaster=self.br,
                    veh_ns=veh_ns + str(i),
                    veh_frame=veh_frame,
                    lidar_frame=lidar_frame,
                    scan_topic=scan_topic,
                    odom_topic=odom_topic,
                    drive_topic=drive_topic,
                )
            )

        # initialize gym environments
        self.obs, _, self.done, _ = self.env.reset(np.array(self.poses))

    def gym_cb(self):
        self.obs, _, self.done, _ = self.env.step(
            np.array(
                [
                    [
                        v.requested_steer,
                        v.requested_speed,
                        v.requested_steer_v,
                        v.requested_accel,
                    ]
                    for v in self.vehicles
                ]
            )
        )
        self._update_sim_state()

    def pub_cb(self):
        # publish scan and odometry msgs for each vehicle and transforms
        for i, v in enumerate(self.vehicles):
            v.publish_scan(self.scan_msgs[i])
            v.publish_odom(self.odom_msgs[i])
            v.publish_tfs(self.map_veh_tfs[i], self.laser_tfs[i], self.wheel_tfs[i])

    def _update_sim_state(self):
        ts = self.get_clock().now().to_msg()

        # get actions from vehicles that subscribe to drive topic
        self.steering_angles = [v.requested_steer for v in self.vehicles]
        self.drive_speeds = [v.requested_speed for v in self.vehicles]

        # create scan and odom msgs to publish for each vehicle
        # also create transforms b/w map and vehicle, laser and base_link,
        # and wheel transforms (wheel rotations)
        for i in range(self.N):
            # create scan msg
            self.scan_msgs[i] = self.create_scan_msg(list(self.obs["scans"][i]), ts)

            # create odom msg
            self.poses[i] = [
                self.obs["poses_x"][i],
                self.obs["poses_y"][i],
                self.obs["poses_theta"][i],
            ]
            self.twists[i] = [
                self.obs["linear_vels_x"][i],
                self.obs["linear_vels_y"][i],
                self.obs["ang_vels_z"][i],
            ]
            self.odom_msgs[i] = self.create_odom_msg(self.poses[i], self.twists[i], ts)

            # create transforms
            self.map_veh_tfs[i] = self.create_veh_tf(self.poses[i], ts)
            self.laser_tfs[i] = self.create_laser_tf(ts)
            self.wheel_tfs[i] = self.create_wheel_tf(self.steering_angles[i], ts)

    def create_scan_msg(self, ranges, timestamp):
        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        return scan

    def create_odom_msg(self, pose, twist, timestamp):
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]
        quat = euler2quat(0.0, 0.0, pose[2], axes="sxyz")
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]
        odom.twist.twist.linear.x = twist[0]
        odom.twist.twist.linear.y = twist[1]
        odom.twist.twist.angular.z = twist[2]
        return odom

    def create_veh_tf(self, pose, ts):
        tf = TransformStamped()
        tf.header.stamp = ts
        tf.header.frame_id = "map"
        tf.transform.translation.x = pose[0]
        tf.transform.translation.y = pose[1]
        quat = euler2quat(0.0, 0.0, pose[2], axes="sxyz")
        tf.transform.rotation.x = quat[1]
        tf.transform.rotation.y = quat[2]
        tf.transform.rotation.z = quat[3]
        tf.transform.rotation.w = quat[0]
        return tf

    def create_laser_tf(self, timestamp):
        tf = copy.deepcopy(self.laser_ts)
        tf.header.stamp = timestamp
        return tf

    def create_wheel_tf(self, steer, timestamp):
        tf = TransformStamped()
        tf.header.stamp = timestamp
        quat = euler2quat(0.0, 0.0, steer, axes="sxyz")
        tf.transform.rotation.x = quat[1]
        tf.transform.rotation.y = quat[2]
        tf.transform.rotation.z = quat[3]
        tf.transform.rotation.w = quat[0]
        return tf


def main(args=None):
    rclpy.init(args=args)
    gbm = GymBridgeMulti()
    rclpy.spin(gbm)
    gbm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
