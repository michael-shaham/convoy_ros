#!/usr/bin/env python3
""" @file convoy_learn.py
    authors: Jonah Jaffe
    A node that gets data from our convoy and uses reinforcement learning to
    train our convoy model.
"""
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from convoy_interfaces.msg import (DesiredDistance, TrainResetPositions)


class crash_detect(Node): 
    def __init__(self):
        super().__init__('convoy_learning')
        # topics
        self.declare_parameter('scan_topic')
        self.declare_parameter('reset_topic')
        self.declare_parameter('desired_distance_topic')

        # sim params
        self.declare_parameter('n_vehicles')
        self.declare_parameter('desired_distance')

        # reset params
        self.declare_parameter('min_desired_distance')
        self.declare_parameter('max_desired_distance')
        self.declare_parameter('noise_range')

        # topic
        reset_topic = self.get_parameter('reset_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        d_dist_topic = self.get_parameter('desired_distance_topic').value

        # sim params
        self.d_dist = self.get_parameter('desired_distance').value
        self.n_vehicles = self.get_parameter('n_vehicles').value 

        # reset params
        self.min_d_dist= self.get_parameter('min_desired_distance').value
        self.max_d_dist = self.get_parameter('max_desired_distance').value
        self.noise_range = self.get_parameter('noise_range').value
        
        self.timesteps = 0
        self.d_dist_pubs = {}
        for i in range(self.n_vehicles):
            self.create_subscription(LaserScan, '/veh_' + str(i) + '/' 
                                     + scan_topic, self.scan_cb, 10)
            if i > 0:
                self.d_dist_pubs[i] = self.create_publisher(
                    DesiredDistance, '/veh_' + str(i) + '/' + d_dist_topic, 10)
        
        self.reset_pub = self.create_publisher(TrainResetPositions, 
                                               reset_topic, 10)     
        
    def scan_cb(self, msg: LaserScan):
        self.timesteps += 1
        for dist in msg.ranges:
            if dist < 0.03 and self.timesteps > 20:
                self.reset()

    def reset(self):
        reset_msg = TrainResetPositions()
        self.d_dist = np.random.uniform(self.min_d_dist, self.max_d_dist)
        self.get_logger().info(f"Reset: {self.timesteps}, {self.d_dist}")
        offset = [0.0] + [np.random.uniform(-1*self.noise_range, self.noise_range)
              for _ in range(self.n_vehicles - 1)]
        reset_msg.noise = offset
        reset_msg.positions = [-i * self.d_dist for i in range(self.n_vehicles)]
        d_dist_msg = DesiredDistance()
        d_dist_msg.desired_distance = self.d_dist
        self.timesteps = 0
        for i in range(1, self.n_vehicles):        
            self.d_dist_pubs[i].publish(d_dist_msg)
        self.reset_pub.publish(reset_msg)
              
            

def main(args=None):
    rclpy.init(args=args)
    node = crash_detect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()