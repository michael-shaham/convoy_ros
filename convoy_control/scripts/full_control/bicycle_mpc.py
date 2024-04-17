#!/usr/bin/env python3
""" @file bicycle_mpc.py
    authors: Michael Shaham
    Implements a node that uses the bicycle_mpc.py module to perform path 
    tracking of a trajectory.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from convoy_interfaces.msg import Trajectory
from nav_msgs.msg import Odometry

from convoy_control.mpc import bicycle_mpc
from convoy_control.dynamics import bicycle_dynamics


"""
MPC node class. Subscribes to trajectory topic to obtain a trajectory, then 
performs MPC. Note that we assume the trajectory is in the local frame, so we 
don't need the vehicle position.
"""


class BicycleMPCNode(Node):

    def __init__(self):
        super().__init__("bicycle_mpc_node")

        # ROS parameters
        self.declare_parameter("safe_drive_topic")
        self.declare_parameter("odom_topic")
        self.declare_parameter("planner_trajectory_topic")
        # dynamics parameters
        self.declare_parameter("wheelbase")
        self.declare_parameter("speed_min")
        self.declare_parameter("speed_max")
        self.declare_parameter("accel_min")
        self.declare_parameter("accel_max")
        self.declare_parameter("steer_min")
        self.declare_parameter("steer_max")
        self.declare_parameter("steer_v_min")
        self.declare_parameter("steer_v_max")
        # MPC parameters
        self.declare_parameter("dt")
        self.declare_parameter("Q_bicycle_weight")
        self.declare_parameter("R_bicycle_weight")

        drive_topic = self.get_parameter("safe_drive_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        pt_topic = self.get_parameter("planner_trajectory_topic").value
        # dynamics parameters
        wheelbase = self.get_parameter("wheelbase").value
        v_min = self.get_parameter("speed_min").value
        v_max = self.get_parameter("speed_max").value
        a_min = self.get_parameter("accel_min").value
        a_max = self.get_parameter("accel_max").value
        steer_min = self.get_parameter("steer_min").value
        steer_max = self.get_parameter("steer_max").value
        steer_v_min = self.get_parameter("steer_v_min").value
        steer_v_max = self.get_parameter("steer_v_max").value
        # MPC parameters
        dt = self.get_parameter("dt").value
        Q_weight = self.get_parameter("Q_bicycle_weight").value
        R_weight = self.get_parameter("R_bicycle_weight").value

        # ROS publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(Trajectory, pt_topic, self.traj_cb, 1)

        # other member variables
        self.dyn = bicycle_dynamics.LDSTKinematics(
            dt=dt,
            wheelbase=wheelbase,
            v_min=v_min,
            v_max=v_max,
            a_min=a_min,
            a_max=a_max,
            steer_min=steer_min,
            steer_max=steer_max,
            steer_v_min=steer_v_min,
            steer_v_max=steer_v_max,
        )
        self.curr_speed = 0.0
        self.curr_state = np.zeros(self.dyn.n)
        self.Q = np.zeros((self.dyn.n, self.dyn.n))
        self.Q[:2, :2] = Q_weight * np.eye(2)
        self.R = R_weight * np.eye(self.dyn.m)

    def odom_cb(self, msg):
        self.curr_speed = np.linalg.norm(
            np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]
            )
        )

    def traj_cb(self, msg):
        frame = msg.header.frame_id
        waypoints = np.array([[p.position.x, p.position.y] for p in msg.poses])
        horizon = waypoints.shape[0]
        waypoints = np.c_[waypoints, np.zeros((horizon, 3))]

        controller = bicycle_mpc.BicycleMPC(
            dynamics=self.dyn,
            horizon=horizon,
            waypoints=waypoints,
            x_0=self.curr_state,
            Q=self.Q,
            R=self.R,
        )

        action, next_state = controller.act()
        accel, steer_vel = action
        _, _, _, v_next, steer_next = next_state

        # self.get_logger().info(f"accel: {accel}")
        # self.get_logger().info(f"steer_vel: {steer_vel}")
        # self.get_logger().info(f"v_next: {v_next}")
        # self.get_logger().info(f"steer_next: {steer_next}")

        self.curr_state = np.array([0, 0, 0, v_next, steer_next])

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = frame
        drive_msg.drive.steering_angle = steer_next
        drive_msg.drive.steering_angle_velocity = steer_vel
        drive_msg.drive.speed = v_next
        drive_msg.drive.acceleration = accel
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    bicycle_mpc_node = BicycleMPCNode()
    rclpy.spin(bicycle_mpc_node)
    bicycle_mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
