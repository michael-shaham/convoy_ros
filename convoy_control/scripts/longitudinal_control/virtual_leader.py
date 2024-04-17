#!/usr/bin/env python3
""" @file virtual_leader.py
    authors: Michael Shaham, Risha Ranjan
    Implements a "virtual leader" for the platoon.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import LongitudinalStateTrajectory, SafeSpeed
from nav_msgs.msg import Odometry

from convoy_control.utils.longitudinal_state import create_long_state_trajectory
from convoy_control.utils.sinusoidal_reference import sinusoidal_ref
from convoy_control.utils.step_reference import accel_decel_step, accel_step, brake_step


class VirtualLeader(Node):

    def __init__(self):
        super().__init__("virtual_leader_node")

        # params for trajectories
        # ROS parameters
        odom_topic = (
            self.declare_parameter("odom_topic").get_parameter_value().string_value
        )
        vl_traj_topic = (
            self.declare_parameter("virtual_leader_traj_topic")
            .get_parameter_value()
            .string_value
        )
        self.vl_frame = (
            self.declare_parameter("virtual_leader_frame")
            .get_parameter_value()
            .string_value
        )
        self.dt = self.declare_parameter("dt").get_parameter_value().double_value
        self.H = self.declare_parameter("horizon").get_parameter_value().integer_value
        # randomly selected trajectory params
        self.stop_time = (
            self.declare_parameter("stop_time").get_parameter_value().double_value
        )
        self.wait_time = (
            self.declare_parameter("wait_time").get_parameter_value().double_value
        )
        self.brake_decel = (
            self.declare_parameter("brake_decel").get_parameter_value().double_value
        )
        self.traj_type = (
            self.declare_parameter("leader_trajectory_type")
            .get_parameter_value()
            .string_value
        )
        self.zero_pct = (
            self.declare_parameter("zero_speed_percentage")
            .get_parameter_value()
            .double_value
        )
        self.stag_T_min = (
            self.declare_parameter("stagger_time_min")
            .get_parameter_value()
            .double_value
        )
        self.stag_T_max = (
            self.declare_parameter("stagger_time_max")
            .get_parameter_value()
            .double_value
        )
        self.low_v_min = (
            self.declare_parameter("low_speed_min").get_parameter_value().double_value
        )
        self.low_v_max = (
            self.declare_parameter("low_speed_max").get_parameter_value().double_value
        )
        self.high_v_min = (
            self.declare_parameter("high_speed_min").get_parameter_value().double_value
        )
        self.high_v_max = (
            self.declare_parameter("high_speed_max").get_parameter_value().double_value
        )
        self.ref_accel_min = (
            self.declare_parameter("reference_accel_min")
            .get_parameter_value()
            .double_value
        )
        self.ref_accel_max = (
            self.declare_parameter("reference_accel_max")
            .get_parameter_value()
            .double_value
        )
        self.sin_T_min = (
            self.declare_parameter("sinusoid_period_min")
            .get_parameter_value()
            .double_value
        )
        self.sin_T_max = (
            self.declare_parameter("sinusoid_period_max")
            .get_parameter_value()
            .double_value
        )
        self.accel_max = (
            self.declare_parameter("accel_max").get_parameter_value().double_value
        )
        course_topic = (
            self.declare_parameter("course_data_topic")
            .get_parameter_value()
            .string_value
        )
        safe_speed_topic = (
            self.declare_parameter("safe_speed_topic")
            .get_parameter_value()
            .string_value
        )
        self.min_safe_speed = (
            self.declare_parameter("min_safe_speed").get_parameter_value().double_value
        )
        veh_ns = self.declare_parameter("veh_ns").get_parameter_value().string_value
        course_topic = "/" + veh_ns + "/" + course_topic

        # ROS publishers
        self.vl_pub = self.create_publisher(
            LongitudinalStateTrajectory, vl_traj_topic, 1
        )
        self.vl_timer = self.create_timer(self.dt, self.vl_cb)

        # ROS subscribers
        safe_speed_topic = "/" + veh_ns + "/" + safe_speed_topic
        odom_topic = "/" + veh_ns + "/" + odom_topic
        self.create_subscription(SafeSpeed, safe_speed_topic, self.speed_cb, 1)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)

        # some variables
        self.curr_speed = 0.0
        self.counter = 0
        self.ref_states = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.start_moving = False
        self.safe_speed = 0.0

    def odom_cb(self, msg: Odometry):
        if self.traj_type == "curvature":
            self.curr_speed = msg.twist.twist.linear.x

    def speed_cb(self, msg: SafeSpeed):
        self.safe_speed = msg.safe_speed

    def vl_cb(self):
        states = self.ref_states[self.counter : self.counter + self.H + 1]
        curr_time = self.get_clock().now().nanoseconds / 1e9

        vl_traj = LongitudinalStateTrajectory()
        vl_traj.header.frame_id = self.vl_frame
        vl_traj.header.stamp = self.get_clock().now().to_msg()
        vl_traj.horizon = self.H

        if not self.start_moving:
            x_ref = np.zeros((3, self.H + 1))
            self.ref_states = create_long_state_trajectory(x_ref)
            self.start_moving = curr_time - self.start_time > self.wait_time
            if self.start_moving:
                self.get_logger().info("Starting to move!")
            vl_traj.longitudinal_trajectory = states
        elif self.traj_type == "curvature":
            if curr_time - self.start_time > self.stop_time:
                self.get_logger().info("Braking!")
                x_ref, _ = brake_step(
                    self.brake_decel,
                    self.dt,
                    self.stag_T_min,
                    max(self.curr_speed, 0.0),
                )
                x_ref = x_ref[:, : self.H + 1]
            else:
                x_ref = self.generate_trajectory()
            states = create_long_state_trajectory(x_ref)
            vl_traj.longitudinal_trajectory = states
        else:
            if len(states) < self.H + 1:
                if curr_time - self.start_time > self.stop_time:
                    self.get_logger().info("Braking!")
                    x_ref, _ = brake_step(
                        self.brake_decel,
                        self.dt,
                        self.stag_T_min,
                        self.ref_states[-1].velocity,
                    )
                else:
                    x_ref = self.generate_trajectory()
                if self.ref_states:
                    x_ref[0, :] += self.ref_states[-1].position
                self.ref_states = self.ref_states[self.counter :]
                self.ref_states += create_long_state_trajectory(x_ref)
                states = self.ref_states[: self.H + 1]
                self.counter = 0

            vl_traj.longitudinal_trajectory = states

            self.counter += 1
            self.curr_speed = states[0].velocity

        self.vl_pub.publish(vl_traj)

    def generate_trajectory(self):

        if self.traj_type == "curvature":
            x_ref = np.zeros((3, self.H + 1))
            t_range = np.linspace(0, self.H * self.dt, self.H + 1)
            vdes = np.clip(
                self.safe_speed,
                0.0,
                max(self.min_safe_speed, self.curr_speed + self.dt * self.accel_max),
            )
            x_ref[1, :] = vdes
            x_ref[0, :] = t_range * vdes
            return x_ref

        v_low = np.random.uniform(self.low_v_min, self.low_v_max)
        if np.random.uniform() < self.zero_pct:
            v_low = 0.0
        v_high = np.random.uniform(self.high_v_min, self.high_v_max)
        ref_accel = np.random.uniform(self.ref_accel_min, self.ref_accel_max)
        x_ref_1 = np.array([])
        if self.traj_type == "random":
            # randomly select staggered (0) or sinusoidal velocity (1)
            traj_type = np.random.randint(2)
            if traj_type == 0:
                stag_T = np.random.uniform(self.stag_T_min, self.stag_T_max)
                self.get_logger().info(
                    f"\nusing staggered velocity"
                    + f"\n\tv_low: {v_low}"
                    + f"\n\tv_high: {v_high}"
                    + f"\n\tstag_T: {stag_T}"
                    + f"\n\tref_accel: {ref_accel}"
                )

                init_accel_time = abs(self.curr_speed - v_low) / ref_accel
                x_ref_1, _ = accel_step(
                    v_low,
                    init_accel_time,
                    self.dt,
                    init_accel_time + 1.0,
                    self.curr_speed,
                )

            elif traj_type == 1:
                sin_T = np.random.uniform(self.sin_T_min, self.sin_T_max)
                self.get_logger().info(
                    f"\nusing sinusoidal velocity"
                    + f"\n\tv_low: {v_low}"
                    + f"\n\tv_high: {v_high}"
                    + f"\n\tsin_T: {sin_T}"
                    + f"\n\tref_accel: {ref_accel}"
                )
                init_accel_time = abs(self.curr_speed - v_low) / ref_accel
                x_ref_1, _ = accel_step(
                    v_low,
                    init_accel_time,
                    self.dt,
                    init_accel_time + 1.0,
                    self.curr_speed,
                )
        elif self.traj_type == "stagger":
            traj_type = 0
            v_low = self.low_v_min
            v_high = self.high_v_max
            ref_accel = self.ref_accel_max
            stag_T = self.stag_T_min
            if self.curr_speed == 0.0:
                init_accel_time = v_low / ref_accel
                x_ref_1, _ = accel_step(
                    v_low,
                    init_accel_time,
                    self.dt,
                    init_accel_time + 1.0,
                    self.curr_speed,
                )
        elif self.traj_type == "sinusoid":
            traj_type = 1
            v_low = self.low_v_min
            v_high = self.high_v_max
            ref_accel = self.ref_accel_max
            sin_T = self.sin_T_min
            if self.curr_speed == 0.0:
                init_accel_time = v_low / ref_accel
                x_ref_1, _ = accel_step(
                    v_low,
                    init_accel_time,
                    self.dt,
                    init_accel_time + 1.0,
                    self.curr_speed,
                )

        if traj_type == 0:  # staggered velocity
            accel_time = (v_high - v_low) / ref_accel
            repeat_accel_start = 1.0
            repeat_accel_end = repeat_accel_start + accel_time
            repeat_decel_start = stag_T / 2.0 + 1
            repeat_decel_end = repeat_decel_start + accel_time
            x_ref_2, _ = accel_decel_step(
                stag_T,
                self.dt,
                repeat_accel_start,
                repeat_accel_end,
                repeat_decel_start,
                repeat_decel_end,
                v_low,
                v_high,
            )

            if x_ref_1.size > 0:
                x_ref_2[0, :] += x_ref_1[0, -1]
                x_ref = np.c_[x_ref_1[:, :-1], x_ref_2]
                return x_ref
            return x_ref_2

        elif traj_type == 1:  # sinsuoidal velocity
            x_ref_2, _ = sinusoidal_ref(sin_T, v_low, v_high, self.dt)

            if x_ref_1.size > 0:
                x_ref_2[0, :] += x_ref_1[0, -1]
                x_ref = np.c_[x_ref_1[:, :-1], x_ref_2]
                return x_ref
            return x_ref_2


def main(args=None):
    rclpy.init(args=args)
    node = VirtualLeader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
