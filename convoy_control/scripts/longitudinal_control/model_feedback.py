#!/usr/bin/env python3
""" @file model_feedback.py
    authors: Jonah Jaffe, Michael Shaham, Risha Ranjan
    Implements a node that uses a NN model's output to specify the acceleration
    of the vehicle.
"""
import numpy as np
import os
import torch

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import (DesiredDistance, LongitudinalControl, 
                                   NeighborInfo, ConvoyControlData, 
                                   TrainResetPositions)
from nav_msgs.msg import Odometry

from convoy_learning.model import FullyConnected
from convoy_control.linear_feedback.linear_feedback import LinearFeedback

class ModelFeedbackNode(Node):

    def __init__(self):
        super().__init__('model_feedback_node')

        # ROS parameters
        self.declare_parameter('longitudinal_control_topic')
        self.declare_parameter('odom_topic')
        self.declare_parameter('neighbor_info_topic')
        self.declare_parameter('desired_distance_topic')
        self.declare_parameter('convoy_control_data_topic')
        self.declare_parameter('reset_topic')
        # DMPC params
        self.declare_parameter('desired_distance')
        self.declare_parameter('dt')
        self.declare_parameter('speed_min')
        self.declare_parameter('speed_max')
        self.declare_parameter('accel_min')
        self.declare_parameter('accel_max')
        # learning params
        self.declare_parameter('prev_distance_length')
        self.declare_parameter('prev_speed_length')
        self.declare_parameter('model_path')
        self.declare_parameter('layer_sizes')
        # linear feedback params
        self.declare_parameter('relative_position_gain')
        self.declare_parameter('relative_speed_gain')

        # get parameters
        long_ctrl_topic = self.get_parameter('longitudinal_control_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        neighbor_info_topic = self.get_parameter('neighbor_info_topic').value
        d_des_topic = self.get_parameter('desired_distance_topic').value
        control_data_topic = self.get_parameter('convoy_control_data_topic').value
        reset_topic = self.get_parameter('reset_topic').value
        # vehicle parameters
        self.d_des = self.get_parameter('desired_distance').value
        self.dt = self.get_parameter('dt').value
        self.v_min = self.get_parameter('speed_min').value
        self.v_max = self.get_parameter('speed_max').value
        self.a_min = self.get_parameter('accel_min').value
        self.a_max = self.get_parameter('accel_max').value
        # learning params
        self.prev_dist_len = self.get_parameter('prev_distance_length').value
        self.prev_speed_len = self.get_parameter('prev_speed_length').value
        self.model_path = self.get_parameter('model_path').value + '.txt'
        layer_sizes = self.get_parameter('layer_sizes').value
        # linear feedback params
        k_p = self.get_parameter('relative_position_gain').value
        k_s = self.get_parameter('relative_speed_gain').value

        # some variables
        self.curr_speed = 0.
        self.curr_accel = 0.
        self.neighbor_dist = None
        self.dist_errors = np.zeros(self.prev_dist_len)
        self.speed_errors = np.zeros(self.prev_speed_len)

        # ROS publishers
        self.long_pub = self.create_publisher(LongitudinalControl, 
                                              long_ctrl_topic, 1)
        self.data_pub = self.create_publisher(ConvoyControlData, 
                                              control_data_topic, 1)

        # ROS subscribers
        self.create_subscription(DesiredDistance, d_des_topic, self.d_des_cb, 1)
        self.create_subscription(NeighborInfo, neighbor_info_topic, 
                                 self.neighbor_info_cb, 1)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 1)
        self.create_subscription(TrainResetPositions, reset_topic, 
                                 self.reset_cb, 10)

        # model
        self.model = FullyConnected(self.prev_dist_len + 2 + 
                                    self.prev_speed_len, 1, 
                                    layer_sizes) 
        if os.path.exists(self.model_path):
            self.model.load_state_dict(torch.load(self.model_path))
        self.model.eval()

        # linear feedback controller
        self.lfbk = LinearFeedback(k_p, k_s)

    def d_des_cb(self, msg: DesiredDistance):
        self.d_des = msg.desired_distance

    def neighbor_info_cb(self, msg: NeighborInfo):
        self.neighbor_dist = msg.distance
        self.neighbor_speed = msg.speed
        dist_error = self.d_des - msg.distance
        speed_error = self.curr_speed - msg.speed

        self.dist_errors = np.append(self.dist_errors, dist_error)
        self.speed_errors = np.append(self.speed_errors, speed_error)

        if self.curr_speed > 0.0:
            inputs = self.dist_errors[-self.prev_dist_len:]
            inputs = np.append(inputs, self.speed_errors[-self.prev_speed_len:])
            inputs = np.append(inputs, self.curr_speed)
            inputs = np.append(inputs, self.neighbor_speed)
            des_accel = self.model(torch.tensor(inputs).float())
            self.get_logger().info(f"des accel: {des_accel}")
        else:
            des_accel = self.lfbk.control(dist_error, speed_error)

        long_control_msg = LongitudinalControl()
        long_control_msg.acceleration = float(des_accel)
        long_control_msg.speed = self.curr_speed + float(des_accel * self.dt)

        control_data_msg = ConvoyControlData()
        control_data_msg.desired_distance = self.d_des
        control_data_msg.predecessor_distance = self.neighbor_dist 
        control_data_msg.distance_error = self.d_des - self.neighbor_dist
        control_data_msg.ego_speed = self.curr_speed
        control_data_msg.predecessor_speed = self.neighbor_speed 
        control_data_msg.speed_error = self.curr_speed - self.neighbor_speed 
        control_data_msg.opt_input = float(des_accel)

        self.long_pub.publish(long_control_msg)
        self.data_pub.publish(control_data_msg)

    def odom_cb(self, msg: Odometry):
        self.curr_speed = np.linalg.norm(np.array([msg.twist.twist.linear.x,
                                                   msg.twist.twist.linear.y,
                                                   msg.twist.twist.linear.z]))

    def reset_cb(self, msg: TrainResetPositions):
        self.curr_speed = 0.
        self.curr_accel = 0.
        self.dist_errors = np.zeros(self.prev_dist_len)
        self.speed_errors = np.zeros(self.prev_speed_len)
        self.neighbor_dist = None
        self.y_a = np.zeros((self.dyn.p, self.H + 1))


def main(args=None):
    rclpy.init(args=args)
    node = ModelFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()