#!/usr/bin/env python3
""" @file quad_pfmpc.py
    authors: Michael Shaham, Risha Ranjan
    Implements a node that uses the distributed MPC, specifically the 
    predecessor following variant, to follow preceding vehicles.
"""
import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import (
    DesiredDistance,
    LongitudinalControl,
    LongitudinalStateTrajectory,
    NeighborInfo,
    ConvoyControlData,
)
from nav_msgs.msg import Odometry

from convoy_control.utils.longitudinal_state import (
    create_long_state_trajectory,
    create_long_state_np,
)
from convoy_control.dynamics.linear_long_accel import LinearLongAccel
from convoy_control.mpc.sparse_quad_pfmpc import SparseQuadPFMPC
from convoy_control.dynamics.linear_long_vel import LinearLongVel
from convoy_control.mpc.sparse_quad_pfmpc_vel import SparseQuadPFMPCVel


class QuadPFMPCNode(Node):

    def __init__(self):
        super().__init__("quad_pfmpc_node")

        # ROS parameters
        self.use_sim = (
            self.declare_parameter("use_sim").get_parameter_value().bool_value
        )
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
        # DMPC params
        self.neighbor_ind = (
            self.declare_parameter("neighbor_ind").get_parameter_value().integer_value
        )
        self.d_des = (
            self.declare_parameter("desired_distance")
            .get_parameter_value()
            .double_value
        )
        self.H = self.declare_parameter("horizon").get_parameter_value().integer_value
        self.dt = self.declare_parameter("dt").get_parameter_value().double_value
        self.tau = self.declare_parameter("tau").get_parameter_value().double_value
        v_min = self.declare_parameter("speed_min").get_parameter_value().double_value
        v_max = self.declare_parameter("speed_max").get_parameter_value().double_value
        a_min = self.declare_parameter("accel_min").get_parameter_value().double_value
        a_max = self.declare_parameter("accel_max").get_parameter_value().double_value
        q_pos = (
            self.declare_parameter("q_self_quad_position")
            .get_parameter_value()
            .double_value
        )
        q_speed = (
            self.declare_parameter("q_self_quad_speed")
            .get_parameter_value()
            .double_value
        )
        q_pred_pos = (
            self.declare_parameter("q_pred_quad_position")
            .get_parameter_value()
            .double_value
        )
        q_pred_speed = (
            self.declare_parameter("q_pred_quad_speed")
            .get_parameter_value()
            .double_value
        )
        r = self.declare_parameter("r_quad").get_parameter_value().double_value

        # self.use_sim = False  # when testing hardware versions on sim

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
            self.d_des = 0.0
            self.neighbor_dist = 0.0
            self.neighbor_ind = -1

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)
        self.shared_traj_pub = self.create_publisher(
            LongitudinalStateTrajectory, shared_long_traj_topic, 1
        )
        self.data_pub = self.create_publisher(ConvoyControlData, control_data_topic, 1)

        # ROS subscribers
        self.create_subscription(DesiredDistance, d_des_topic, self.d_des_cb, 1)
        self.create_subscription(
            NeighborInfo, neighbor_info_topic, self.neighbor_info_cb, 1
        )
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

        # longitudinal dynamics and PFMPC
        if self.neighbor_ind == -1:
            q_pos = 0.0
            q_speed = 0.0
        if self.use_sim:
            self.dyn = LinearLongAccel(self.dt, self.tau)
        else:
            self.dyn = LinearLongVel(self.dt, self.tau)

        self.n, self.m, self.p = self.dyn.n, self.dyn.m, self.dyn.p
        Q = np.eye(self.p)
        Q[0, 0] = q_pos
        Q[1, 1] = q_speed
        Q_p = np.eye(self.p)
        Q_p[0, 0] = q_pred_pos
        Q_p[1, 1] = q_pred_speed
        R = r * np.eye(self.m)

        if self.use_sim:
            self.pfmpc = SparseQuadPFMPC(
                Q, Q_p, R, self.dyn.A, self.dyn.B, self.dyn.C, a_min, a_max, self.H
            )
        else:
            self.pfmpc = SparseQuadPFMPCVel(
                Q, Q_p, R, self.dyn.A, self.dyn.B, v_min, v_max, a_max, self.dt, self.H
            )

        self.y_a = np.zeros((self.dyn.p, self.H + 1))

        self.zero_long_msg = LongitudinalControl()

    def neighbor_traj_cb(self, msg: LongitudinalStateTrajectory):
        if self.neighbor_dist is None:
            return

        x_0 = np.array([0.0, self.curr_speed, self.curr_accel])
        x_pred = create_long_state_np(msg.longitudinal_trajectory)
        x_pred = x_pred.T
        if len(x_pred.shape) < 2:
            return
        x_pred[0, :] -= x_pred[0, 0]
        x_pred[0, :] += self.neighbor_dist
        y_pred = x_pred[:2, :]
        if self.n < len(x_0):
            x_0 = x_0[:self.n]

        # pfmpc optimal control problem
        u_opt, x_opt, prob = self.pfmpc.control(x_0, self.y_a, y_pred, self.d_des)
        if prob.status != "optimal":
            if self.veh_ns == "veh_0":
                self.get_logger().info(
                    f"problem returned {prob.status} " + f"for vehicle 0"
                )
            else:
                self.get_logger().info(
                    f"problem returned {prob.status} "
                    + f"for vehicle {self.neighbor_ind + 1}"
                )
            # TODO: handle somehow, for now just stop the vehicle
            self.long_pub.publish(self.zero_long_msg)
            return
        y_opt = x_opt[:2, :]

        u_a = np.zeros_like(u_opt)
        x_a = np.zeros_like(x_opt)
        self.y_a = np.zeros_like(y_opt)
        u_a[:, : self.H - 1] = u_opt[:, 1 : self.H]
        if not self.use_sim:
            u_a[:, -1] = y_opt[1, -1]
        x_a[:, 0] = x_opt[:, 1]
        self.y_a[:, 0] = self.dyn.sense(x_a[:, 0])
        for t in range(self.H):
            x_a[:, t + 1] = self.dyn.forward(x_a[:, t], u_a[:, t])
            self.y_a[:, t + 1] = self.dyn.sense(x_a[:, t + 1])
        self.y_a[0, :] -= self.y_a[0, 0]

        if self.n == 2:
            x_a = np.r_[x_a, np.zeros((1, x_a.shape[1]))]

        self.traj.longitudinal_trajectory = create_long_state_trajectory(x_a)
        self.traj.header.frame_id = self.veh_frame
        self.traj.header.stamp = self.get_clock().now().to_msg()
        self.shared_traj_pub.publish(self.traj)

        if self.use_sim:  # acceleration based controller
            des_speed = x_opt[1, 1]
            des_accel = u_opt[0, 0]
            self.curr_accel = x_opt[2, 1]
        else:  # velocity based controller
            des_speed = u_opt[0, 0]
            des_accel = 0.0

        long_control_msg = LongitudinalControl()
        long_control_msg.speed = des_speed
        long_control_msg.acceleration = des_accel
        self.long_pub.publish(long_control_msg)

        if self.veh_ns == "veh_0":
            self.neighbor_dist = self.d_des
            self.neighbor_speed = msg.longitudinal_trajectory[0].velocity

        control_data_msg = ConvoyControlData()
        control_data_msg.header = msg.header
        control_data_msg.desired_distance = self.d_des
        control_data_msg.predecessor_distance = self.neighbor_dist
        control_data_msg.distance_error = self.d_des - self.neighbor_dist
        control_data_msg.ego_speed = self.curr_speed
        control_data_msg.predecessor_speed = self.neighbor_speed
        control_data_msg.speed_error = self.curr_speed - self.neighbor_speed
        if self.use_sim:
            control_data_msg.opt_input = des_accel
        else:
            control_data_msg.opt_input = des_speed
        self.data_pub.publish(control_data_msg)

    def d_des_cb(self, msg: DesiredDistance):
        self.d_des = msg.desired_distance

    def neighbor_info_cb(self, msg: NeighborInfo):
        # this callback should never get used if veh_ns == veh_0
        self.neighbor_dist = msg.distance
        self.neighbor_speed = msg.speed

    def odom_cb(self, msg: Odometry):
        self.curr_speed = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
    node = QuadPFMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
