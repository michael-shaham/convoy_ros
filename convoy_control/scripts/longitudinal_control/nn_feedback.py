#!/usr/bin/env python3
""" @file nn_feedback.py
    authors: Jonah Jaffe, Michael Shaham, Risha Ranjan
    Implements a node that uses a neural network model's output to specify the 
    longitudinal control action for the vehicle.
"""
import numpy as np
import os
import torch

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

from convoy_control.nn_model.feedforward import FullyConnected


class NeuralNetworkControllerNode(Node):

    def __init__(self):
        super().__init__("nn_feedback_node")

        # ROS parameters
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
        # platooning params
        self.d_des = (
            self.declare_parameter("desired_distance")
            .get_parameter_value()
            .double_value
        )
        self.dt = self.declare_parameter("dt").get_parameter_value().double_value
        a_min = self.declare_parameter("accel_min").get_parameter_value().double_value
        a_max = self.declare_parameter("accel_max").get_parameter_value().double_value
        self.tau = self.declare_parameter("tau").get_parameter_value().double_value
        # learning params
        self.epp = (
            self.declare_parameter("encode_platoon_position")
            .get_parameter_value()
            .bool_value
        )
        self.max_vehs = (
            self.declare_parameter("max_vehicles").get_parameter_value().integer_value
        )
        model_file = (
            self.declare_parameter("model_file").get_parameter_value().string_value
        )
        model_dir = (
            self.declare_parameter("model_dir").get_parameter_value().string_value
        )
        hidden_dims = (
            self.declare_parameter("hidden_layer_sizes")
            .get_parameter_value()
            .integer_array_value
        )
        lrelu_slope = (
            self.declare_parameter("leaky_relu_slope")
            .get_parameter_value()
            .double_value
        )
        # parameters if we track virtual leader instead of predecessor
        track_vl = self.declare_parameter("track_vl").get_parameter_value().bool_value
        vl_traj_topic = (
            self.declare_parameter("virtual_leader_traj_topic")
            .get_parameter_value()
            .string_value
        )

        # some variables
        self.curr_speed = 0.0
        self.curr_accel = 0.0

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, long_ctrl_topic, 1)
        self.data_pub = self.create_publisher(ConvoyControlData, control_data_topic, 1)

        # ROS subscribers
        self.create_subscription(DesiredDistance, d_des_topic, self.d_des_cb, 1)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        if track_vl:
            self.create_subscription(
                LongitudinalStateTrajectory,
                vl_traj_topic,
                self.vl_traj_cb,
                1,
            )
        else:
            self.create_subscription(
                NeighborInfo, neighbor_info_topic, self.neighbor_info_cb, 1
            )

        # model
        if self.epp:
            assert self.max_vehs > 0
        else:
            self.max_vehs = 0
        model_file = os.path.join(model_dir, model_file)
        if "leaky" in model_file:
            act_fn = torch.nn.LeakyReLU(lrelu_slope)
        else:
            act_fn = torch.nn.ReLU()
        if self.epp:
            model_file += f"_{self.max_vehs}_vehs_encoded"
        model_file += ".pt"
        in_dim = 2 + self.max_vehs
        bias = self.epp
        self.model = FullyConnected(
            in_dim,
            1,
            hidden_dims,
            out_min=torch.tensor([a_min]).float(),
            out_max=torch.tensor([a_max]).float(),
            bias=bias,
            act_fn=act_fn,
        )
        self.model.load_state_dict(
            torch.load(model_file, map_location=torch.device("cpu"))
        )
        self.model.set_clamp_layers(
            torch.tensor([a_min]).float(), torch.tensor([a_max]).float()
        )
        self.model.eval()

    def d_des_cb(self, msg: DesiredDistance):
        self.d_des = msg.desired_distance

    def neighbor_info_cb(self, msg: NeighborInfo):
        neighbor_dist = msg.distance
        neighbor_speed = msg.speed
        dist_err = msg.distance - self.d_des
        speed_err = msg.speed - self.curr_speed
        err = torch.tensor([dist_err, speed_err]).float()
        if self.epp:
            enc = torch.zeros(self.max_vehs).float()
            enc[msg.vehicle_ind + 1] = 1.0
            err = torch.cat((err, enc))
        out = self.model(err.unsqueeze(0)).squeeze().item()
        desired_accel = out
        desired_speed = out * self.tau + self.curr_speed

        long_control_msg = LongitudinalControl()
        long_control_msg.speed = desired_speed
        long_control_msg.acceleration = desired_accel

        control_data_msg = ConvoyControlData()
        control_data_msg.desired_distance = self.d_des
        control_data_msg.predecessor_distance = neighbor_dist
        control_data_msg.distance_error = self.d_des - neighbor_dist
        control_data_msg.ego_speed = self.curr_speed
        control_data_msg.predecessor_speed = neighbor_speed
        control_data_msg.speed_error = self.curr_speed - neighbor_speed
        control_data_msg.opt_input = desired_accel

        self.long_pub.publish(long_control_msg)
        self.data_pub.publish(control_data_msg)

    def vl_traj_cb(self, msg: LongitudinalStateTrajectory):
        dist_err = 0.0
        vl_speed = msg.longitudinal_trajectory[-1].velocity
        speed_err = vl_speed - self.curr_speed
        err = torch.tensor([dist_err, speed_err]).float()
        if self.epp:
            enc = torch.zeros(self.max_vehs).float()
            enc[0] = 1.0
            err = torch.cat((err, enc))
        out = self.model(err.unsqueeze(0)).squeeze().item()
        desired_accel = out
        desired_speed = out * self.tau + self.curr_speed

        long_control_msg = LongitudinalControl()
        long_control_msg.speed = desired_speed
        long_control_msg.acceleration = desired_accel

        control_data_msg = ConvoyControlData()
        control_data_msg.desired_distance = self.d_des
        control_data_msg.predecessor_distance = self.d_des
        control_data_msg.distance_error = 0.0
        control_data_msg.ego_speed = self.curr_speed
        control_data_msg.predecessor_speed = vl_speed
        control_data_msg.speed_error = self.curr_speed - vl_speed
        control_data_msg.opt_input = desired_accel

        self.long_pub.publish(long_control_msg)
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
    node = NeuralNetworkControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
