#!/usr/bin/env python3
""" @file convoy_learn.py
    authors: Jonah Jaffe
    A node that gets data from our convoy and uses it to clasically train 
    a neural network to predict an acceleration.
"""

import os
import numpy as np
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import (DesiredDistance, TrainResetPositions, 
                                   ConvoyControlData)
from convoy_learning.model import FullyConnected


class convoy_learning(Node):

    def __init__(self):
        super().__init__('convoy_learning')
        # learning parameters
        self.declare_parameter('reset_distance')
        self.declare_parameter('n_vehicles')
        self.declare_parameter('prev_distance_length')
        self.declare_parameter('prev_speed_length')
        self.declare_parameter('model_path')
        self.declare_parameter('learning_rate')
        self.declare_parameter('layer_sizes')
        # sim parameters
        self.declare_parameter('convoy_control_data_topic')
        self.declare_parameter('reset_topic')
        self.declare_parameter('odom_topic')
        self.declare_parameter('longitudinal_control_topic')
        self.declare_parameter('desired_distance')
        self.declare_parameter('desired_distance_topic')
        self.declare_parameter('min_desired_distance')
        self.declare_parameter('max_desired_distance')
        self.declare_parameter('noise_range')
        self.declare_parameter('y_arr')

        self.rd = self.get_parameter('reset_distance').value
        self.d_dist = self.get_parameter('desired_distance').value
        self.n_vehicles = self.get_parameter('n_vehicles').value 
        self.prev_dist_len = self.get_parameter('prev_distance_length').value
        self.prev_speed_len = self.get_parameter('prev_speed_length').value
        self.model_path = self.get_parameter('model_path').value + '.txt'
        self.min_d_dist= self.get_parameter('min_desired_distance').value
        self.max_d_dist = self.get_parameter('max_desired_distance').value
        self.noise_range = self.get_parameter('noise_range').value
        self.lr = self.get_parameter('learning_rate').value 
        self.init_posns = self.get_parameter('y_arr').value
        layer_sizes = self.get_parameter('layer_sizes').value
        # topics
        data_topic = self.get_parameter('convoy_control_data_topic').value
        reset_topic = self.get_parameter('reset_topic').value
        d_dist_topic = self.get_parameter('desired_distance_topic').value
        
        self.timesteps = 0
        self.dist_errors = {}
        self.speed_errors = {}
        self.speeds = {}
        self.opt_inputs = {}
        self.speeds[0] = 0.0
        self.opt_inputs[0] = 0.0
        self.all_losses = []
        for i in range(1, self.n_vehicles):
            self.dist_errors[i] = np.zeros(self.prev_dist_len)
            self.speed_errors[i] = np.zeros(self.prev_speed_len)
            self.speeds[i] = 0.0
            self.opt_inputs[i] = 0.0
            cb = self.custom_data_callback(i)
            self.create_subscription(ConvoyControlData, '/veh_' + str(i) + 
                                     '/' + data_topic, cb, 1)

            self.create_subscription(DesiredDistance, '/veh_' + str(i) + '/' 
                                    + d_dist_topic, self.d_dist_callback, 10)

        self.create_subscription(TrainResetPositions, 
                                 reset_topic, self.reset_cb, 10)     
        
        self.model = FullyConnected(self.prev_dist_len + 2 + 
                                    self.prev_speed_len, 1, 
                                    layer_sizes) 
        if os.path.exists(self.model_path):
            self.model.load_state_dict(torch.load(self.model_path))
        self.model.eval()
        self.loss_fn = nn.MSELoss(reduction='sum')

    def d_dist_callback(self, msg: DesiredDistance):
        self.d_dist = msg.desired_distance

    def reset_cb(self, msg: TrainResetPositions):
            self.get_logger().info(f"Reset: {self.timesteps}, {self.d_dist}")
            self.timesteps = 0
            for i in range(self.n_vehicles):
                self.speeds[i] = 0.0
                self.opt_inputs[i] = 0.0
                if i > 0:
                    self.dist_errors[i] = np.repeat(
                        msg.noise[i-1] - msg.noise[i], self.prev_dist_len)
                self.speed_errors[i] = np.zeros(self.prev_speed_len)
    
    def custom_data_callback(self, veh_num):
        def cb(msg: ConvoyControlData):
            self.opt_inputs[veh_num] = msg.opt_input
            self.speeds[veh_num - 1] = msg.predecessor_speed
            self.speeds[veh_num] = msg.ego_speed
            self.dist_errors[veh_num] = \
                np.append(self.dist_errors[veh_num], msg.distance_error)
            self.speed_errors[veh_num] = \
                np.append(self.speed_errors[veh_num], msg.speed_error)
            self.timesteps += 1
            losses = []
            data = self.all_veh_learn_data()
            if self.timesteps % self.n_vehicles - 1 == self.n_vehicles - 2:
                for i in range(1, self.n_vehicles):
                    losses.append(self.classical_learn_step(
                        torch.tensor(data[i][0]).float(), 
                        torch.tensor([data[i][1]]).float()))
                self.all_losses.append(losses)
                if self.timesteps % 100 == 99:
                    means = [sum(loss)/len(loss) for loss in self.all_losses]
                    self.get_logger().info(f"\r{self.timesteps}, " 
                                    f"min: {min(losses)}, " 
                                    f"mean: {sum(losses)/len(losses)}, " 
                                    f"max: {max(losses)}, "
                                    f"total: {sum(means[-90:])/90}")
        return cb

    # gives an output of all of the most recent values to learn from in the form
    # ([dist_errors, speed_errors, ego_speed, ego prev, pred_speed, pred_ prev] 
    # , opt_input)
    def all_veh_learn_data(self):
        veh_dict = {}
        for i in range(1, self.n_vehicles):
            inputs = self.dist_errors[i][-self.prev_dist_len:]
            inputs = np.append(inputs, 
                               self.speed_errors[i][-self.prev_speed_len:])
            inputs = np.append(inputs, self.speeds[i])
            inputs = np.append(inputs, self.speeds[i - 1]) 
            veh_dict[i] = (inputs, self.opt_inputs[i])
        return veh_dict

    def classical_learn_step(self, x, y):
        y_pred = self.model(x)

        loss = self.loss_fn(y_pred, y)
        self.model.model.zero_grad()
        loss.backward()
        with torch.no_grad():
            for param in self.model.parameters():
                param -= self.lr * param.grad

        return loss.item()

def main(args=None):
    rclpy.init(args=args)
    convoy_l = convoy_learning()
    try:
        rclpy.spin(convoy_l)
    except KeyboardInterrupt:
        torch.save(convoy_l.model.state_dict(), convoy_l.model_path)            
    convoy_l.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
