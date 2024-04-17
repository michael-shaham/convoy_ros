#!/usr/bin/env python3
""" @file linear_mpc.py
    authors: Michael Shaham, Risha Ranjan
    Implements a node that uses linear MPC to track a longitudinal trajectory.
"""
import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import (
    LongitudinalControl,
    LongitudinalStateTrajectory,
    ConvoyControlData,
)
from nav_msgs.msg import Odometry

from convoy_control.utils.longitudinal_state import (
    create_long_state_trajectory,
    create_long_state_np,
)
from convoy_control.mpc.sparse_linear_mpc import SparseLinearMPC
from convoy_control.dynamics.linear_long_accel import LinearLongAccel
from convoy_control.dynamics.linear_long_vel import LinearLongVel


class LinearMPCNode(Node):

    def __init__(self):
        super().__init__("linear_mpc_node")

        # get parameters
        use_sim = self.declare_parameter("use_sim").get_parameter_value().bool_value
        self.veh_ns = (
            self.declare_parameter("veh_ns").get_parameter_value().string_value
        )
        veh_frame = (
            self.declare_parameter("veh_frame").get_parameter_value().string_value
        )
        long_ctrl_topic = (
            self.declare_parameter("longitudinal_control_topic")
            .get_parameter_value()
            .string_value
        )
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        control_data_topic = (
            self.declare_parameter("convoy_control_data_topic")
            .get_parameter_value()
            .string_value
        )
        shared_long_traj_topic = (
            self.declare_parameter("shared_long_traj_topic")
            .get_parameter_value()
            .string_value
        )
        vl_traj_topic = (
            self.declare_parameter("virtual_leader_traj_topic")
            .get_parameter_value()
            .string_value
        )
        # MPC parameters
        self.neighbor_ind = (
            self.declare_parameter("neighbor_ind").get_parameter_value().integer_value
        )
        self.H = self.declare_parameter("horizon").get_parameter_value().integer_value
        self.dt = self.declare_parameter("dt").get_parameter_value().double_value
        self.tau = self.declare_parameter("tau").get_parameter_value().double_value
        v_min = self.declare_parameter("speed_min").get_parameter_value().double_value
        v_max = self.declare_parameter("speed_max").get_parameter_value().double_value
        a_min = self.declare_parameter("accel_min").get_parameter_value().double_value
        a_max = self.declare_parameter("accel_max").get_parameter_value().double_value
        q = self.declare_parameter("q_mpc").get_parameter_value().double_value
        q_f = self.declare_parameter("q_f_mpc").get_parameter_value().double_value
        r = self.declare_parameter("r_mpc").get_parameter_value().double_value

        # some variables
        self.veh_frame = self.veh_ns + "/" + veh_frame
        self.curr_speed = 0.0
        self.curr_accel = 0.0
        self.neighbor_dist = None

        neighbor_long_traj_topic = (
            "/veh_" + str(self.neighbor_ind) + "/" + shared_long_traj_topic
        )
        if self.veh_ns == "veh_0":
            neighbor_long_traj_topic = vl_traj_topic
            self.neighbor_dist = 0.0

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)
        self.shared_traj_pub = self.create_publisher(
            LongitudinalStateTrajectory, shared_long_traj_topic, 1
        )
        self.data_pub = self.create_publisher(ConvoyControlData, control_data_topic, 1)

        # ROS subscribers
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(
            LongitudinalStateTrajectory,
            neighbor_long_traj_topic,
            self.neighbor_traj_cb,
            1,
        )

        # shared trajectory msg
        self.traj = LongitudinalStateTrajectory()
        self.traj.horizon = self.H

        # longitudinal dynamics and MPC
        use_sim = False
        if use_sim:
            self.dyn = LinearLongAccel(self.dt, self.tau)
        else:
            self.dyn = LinearLongVel(self.dt, self.tau)
        self.n, self.m = self.dyn.n, self.dyn.m
        Q = q * np.eye(self.n)
        R = r * np.eye(self.m)
        Q_f = q_f * np.eye(self.n)
        x_min = -np.inf * np.ones(self.n)
        x_max = np.inf * np.ones(self.n)
        x_min[1] = v_min
        x_max[1] = v_max

        if self.n == 3:
            x_min[2] = a_min
            x_max[2] = a_max
            u_min = a_min
            u_max = a_max
        elif self.n == 2:
            u_min = v_min
            u_max = v_max
        self.mpc = SparseLinearMPC(
            Q, Q_f, R, self.dyn.A, self.dyn.B, u_min, u_max, x_min, x_max, self.H
        )

        self.zero_long_msg = LongitudinalControl()

    def neighbor_traj_cb(self, msg: LongitudinalStateTrajectory):
        if self.neighbor_dist is None:
            return

        x_0 = np.array([0.0, self.curr_speed, self.curr_accel])
        n, m, H = self.n, self.m, self.H

        x_ref = create_long_state_np(msg.longitudinal_trajectory)
        x_ref = x_ref.T
        x_ref[0, :] -= x_ref[0, 0]
        if self.n == 2:
            x_0 = x_0[:2]
            x_ref = x_ref[:2, :]

        # mpc optimal control problem
        y_opt, prob = self.mpc.control(x_0, x_ref)
        if prob.status != "optimal":
            self.get_logger().info(
                f"problem returned {prob.status} "
                + f"for vehicle {self.neighbor_ind + 1}"
            )
            # TODO: handle somehow, for now just stop the vehicle
            self.long_pub.publish(self.zero_long_msg)
            return

        x_opt = np.zeros((self.n, H + 1))
        u_opt = np.zeros((self.m, self.H))
        x_opt[:, 0] = x_0
        for t, i in enumerate(range(0, H * (n + m), n + m)):
            u_opt[:, t] = y_opt[i : i + m]
            x_opt[:, t + 1] = y_opt[i + m : i + m + n] + x_ref[:, t + 1]

        if self.n == 2:
            x_opt = np.r_[x_opt, np.zeros((1, x_opt.shape[1]))]

        self.traj.longitudinal_trajectory = create_long_state_trajectory(x_opt)
        self.traj.header.frame_id = self.veh_frame
        self.traj.header.stamp = self.get_clock().now().to_msg()
        self.shared_traj_pub.publish(self.traj)

        if self.n == 2:
            des_speed = u_opt[0, 0]
            des_accel = 0.0
        elif self.n == 3:
            des_accel = u_opt[0, 0]
            self.curr_accel = x_opt[2, 1]
            des_speed = x_opt[1, 1]

        self.get_logger().info(f"des speed: {des_speed}")
        self.get_logger().info(f"des accel: {des_accel}")

        long_control_msg = LongitudinalControl()
        long_control_msg.speed = des_speed
        long_control_msg.acceleration = des_accel
        self.long_pub.publish(long_control_msg)

        if self.veh_ns == "veh_0":
            self.neighbor_dist = 0.0
            self.neighbor_speed = msg.longitudinal_trajectory[0].velocity

        control_data_msg = ConvoyControlData()
        control_data_msg.header = msg.header
        control_data_msg.predecessor_distance = self.neighbor_dist
        control_data_msg.ego_speed = self.curr_speed
        control_data_msg.predecessor_speed = self.neighbor_speed
        control_data_msg.speed_error = self.curr_speed - self.neighbor_speed
        control_data_msg.opt_input = des_accel
        self.data_pub.publish(control_data_msg)

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


def main(args=None):
    rclpy.init(args=args)
    node = LinearMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
