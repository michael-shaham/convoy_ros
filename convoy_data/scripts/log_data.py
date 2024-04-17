#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from convoy_interfaces.msg import ConvoyControlData
from sensor_msgs.msg import Joy

import pandas as pd


class LogData(Node):
    def __init__(self):
        super().__init__("log_data")

        # ROS parameters
        joy_topic = (
            self.declare_parameter("joy_topic").get_parameter_value().string_value
        )
        self.safety_button_ind = (
            self.declare_parameter("safety_button_ind")
            .get_parameter_value()
            .integer_value
        )
        self.use_sim = (
            self.declare_parameter("use_sim").get_parameter_value().bool_value
        )
        control_data_topic = (
            self.declare_parameter("convoy_control_data_topic")
            .get_parameter_value()
            .string_value
        )
        n_veh = self.declare_parameter("n_vehicles").get_parameter_value().integer_value
        self.trial_name = (
            self.declare_parameter("trial_name").get_parameter_value().string_value
        )
        self.samples = (
            self.declare_parameter("learn_samples_cut")
            .get_parameter_value()
            .integer_value
        )

        self.column_names = [
            "veh_ind",
            "time",
            "des_dist",
            "pred_dist",
            "dist_err",
            "ego_speed",
            "pred_speed",
            "speed_err",
            "opt_input",
        ]

        self.log_data_flag = False
        self.df = pd.DataFrame(columns=self.column_names)
        self.data_subs = []
        self.start_time = None
        self.counter = 0
        self.cut_trial_name = ""
        self.cut_trail_counter = 1

        if not self.use_sim:
            self.create_subscription(Joy, "/veh_0/" + joy_topic, self.joy_cb, 10)
        for i in range(n_veh):
            cb = self.custom_data_cb(i)
            self.data_subs.append(
                self.create_subscription(
                    ConvoyControlData,
                    "veh_" + str(i) + "/" + control_data_topic,
                    cb,
                    10,
                )
            )

    def joy_cb(self, msg: Joy):
        if msg.buttons[self.safety_button_ind]:
            self.log_data_flag = True
        else:
            self.log_data_flag = False

    def custom_data_cb(self, veh_ind):
        def cb(msg: ConvoyControlData):
            if (not self.use_sim and self.log_data_flag) or (self.use_sim):
                if not self.start_time:
                    self.start_time = self.get_clock().now().nanoseconds / 1e9
                time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                new_row_dict = {
                    "veh_ind": veh_ind,
                    "time": time,
                    "des_dist": msg.desired_distance,
                    "pred_dist": msg.predecessor_distance,
                    "dist_err": msg.distance_error,
                    "ego_speed": msg.ego_speed,
                    "pred_speed": msg.predecessor_speed,
                    "speed_err": msg.speed_error,
                    "opt_input": msg.opt_input,
                }
                new_row = pd.DataFrame(new_row_dict, index=[self.counter])
                self.df = pd.concat([self.df, new_row])
                self.counter += 1

                if self.counter / self.samples >= 1:
                    self.counter = 0

                    self.save_cut()
                    self.cut_trail_counter += 1
                    self.df = pd.DataFrame(columns=self.column_names)

        return cb

    def save_cut(self):
        cut_trial_name = "{}_{}-{}k".format(
            self.trial_name,
            round((self.cut_trail_counter - 1) * self.samples / 1000),
            round(self.cut_trail_counter * self.samples / 1000),
        )
        path_to_save = (
            "~/cvy_ws/src/convoy_ros/convoy_data/data/" + cut_trial_name + ".csv"
        )
        self.df.to_csv(path_or_buf=path_to_save, index=False)


def main(args=None):
    rclpy.init(args=args)
    node = LogData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_cut()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
