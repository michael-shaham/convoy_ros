#!/usr/bin/env python3
""" @file double_int_mpc_head.py
    authors: Michael Shaham
    Implements a node that uses the double integrator longitudinal model to 
    plan accelerations/speeds, and a lateral controller to track the centerline.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import LongitudinalControl, Trajectory
from nav_msgs.msg import Odometry

from convoy_control.dynamics import double_int_dynamics
from convoy_control.mpc import double_int_mpc


class DoubleIntMPCNode(Node):

    def __init__(self):
        super().__init__("double_int_mpc_node")

        # ROS parameters
        long_ctrl_topic = (
            self.declare_parameter("longitudinal_control_topic")
            .get_parameter_value()
            .string_value
        )
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        pt_topic = (
            self.declare_parameter("planner_trajectory_topic")
            .get_parameter_value()
            .string_value
        )
        # dynamics parameters
        v_min = self.declare_parameter("speed_min").get_parameter_value().double_value
        v_max = self.declare_parameter("speed_max").get_parameter_value().double_value
        a_min = self.declare_parameter("accel_min").get_parameter_value().double_value
        a_max = self.declare_parameter("accel_max").get_parameter_value().double_value
        # MPC parameters
        dt = self.declare_parameter("dt").get_parameter_value().double_value
        self.Q = (
            self.declare_parameter("Q_double_int_weight")
            .get_parameter_value()
            .double_value
        )
        self.R = (
            self.declare_parameter("R_double_int_weight")
            .get_parameter_value()
            .double_value
        )
        self.x_min = np.array([-np.inf, v_min])
        self.x_max = np.array([np.inf, v_max])
        self.u_min, self.u_max = a_min, a_max

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(Trajectory, pt_topic, self.traj_cb, 1)

        # other member variables
        self.dyn = double_int_dynamics.DTDoubleIntegrator(dt=dt)
        self.curr_speed = 0.0
        self.curr_accel = 0.0

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
        xy_traj = np.array([[p.position.x, p.position.y] for p in msg.poses])
        dists = np.linalg.norm(xy_traj[1:, :] - xy_traj[:-1, :], axis=1)
        for i in range(1, len(dists)):
            dists[i] += dists[i - 1]
        waypoints = np.concatenate((np.array([0.0]), dists))
        horizon = len(waypoints)

        long_controller = double_int_mpc.DoubleIntMPC(
            self.dyn.A,
            self.dyn.B,
            self.x_min,
            self.x_max,
            self.u_min,
            self.u_max,
            horizon=horizon,
            waypoints=waypoints,
            x_0=np.array([0.0, np.minimum(self.curr_speed, self.x_max[1])]),
            Q=self.Q,
            R=self.R,
        )

        u_opt, x_opt, prob = long_controller.act()
        if prob.status != "optimal":
            self.get_logger().info(
                f"double integrator mpc returned " + f"{prob.status}"
            )
            zero_long_msg = LongitudinalControl()
            self.long_pub.publish(zero_long_msg)
            return

        long_control_msg = LongitudinalControl()
        long_control_msg.speed = x_opt[1, 1]
        long_control_msg.acceleration = u_opt[0]
        self.long_pub.publish(long_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DoubleIntMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
