#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class AutonomousControl(Node):

    def __init__(self):
        super().__init__("autonomous_control")

        # ROS parameters
        self.declare_parameter("use_sim", False)
        self.use_sim = self.get_parameter("use_sim").value

        self.declare_parameter("is_trail", False)
        is_trail = self.get_parameter("is_trail").value

        self.declare_parameter("head_ind", 0)
        head_ind = self.get_parameter("head_ind").value

        self.declare_parameter("safe_drive_topic", "")
        safe_drive_topic = self.get_parameter("safe_drive_topic").value

        self.declare_parameter("real_drive_topic", "")
        real_drive_topic = self.get_parameter("real_drive_topic").value

        self.declare_parameter("veh_frame", "")
        self.veh_frame_ = self.get_parameter("veh_frame").value

        self.declare_parameter("joy_topic", "")
        joy_topic = self.get_parameter("joy_topic").value

        self.declare_parameter("safety_button_ind")
        self.safety_button_ind_ = self.get_parameter("safety_button_ind").value

        self.declare_parameter("brake_button_ind")
        self.brake_button_ind_ = self.get_parameter("brake_button_ind").value

        # publishers
        self.drive_pub_ = self.create_publisher(
            AckermannDriveStamped, real_drive_topic, 1
        )

        # subscribers
        if not self.use_sim:
            if is_trail:
                joy_topic = "/veh_" + str(head_ind) + "/" + joy_topic
                # pass
            self.create_subscription(Joy, joy_topic, self.joy_cb, 1)
        self.create_subscription(
            AckermannDriveStamped, safe_drive_topic, self.safe_drive_cb, 1
        )

        # variables
        self.curr_joy_ = Joy()

    def joy_cb(self, msg: Joy):
        if msg.buttons[self.brake_button_ind_]:
            self.braking_ = True
            brake_msg = AckermannDriveStamped()
            brake_msg.header.frame_id = self.veh_frame_
            brake_msg.header.stamp = self.get_clock().now().to_msg()
            self.drive_pub_.publish(brake_msg)
        else:
            self.curr_joy_ = msg

    def safe_drive_cb(self, msg):
        if self.use_sim:
            self.drive_pub_.publish(msg)

        elif self.curr_joy_.buttons[self.safety_button_ind_]:
            self.drive_pub_.publish(msg)

        else:
            brake_msg = AckermannDriveStamped()
            brake_msg.header.frame_id = self.veh_frame_
            brake_msg.header.stamp = self.get_clock().now().to_msg()
            self.drive_pub_.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
