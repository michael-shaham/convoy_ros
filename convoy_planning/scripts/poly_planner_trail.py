#!/usr/bin/env python3
""" @file poly_planner_trail.py
    authors: Michael Shaham, Risha Ranjan
    Node to perform planning (trajectory generation that we will follow using 
    separate control nodes) for various controllers. This node is specifically 
    for trail vehicles, where we ensure that the planned path intersects with 
    the preceding vehicle, including going through their position and matching 
    their orientation.
"""

import cvxpy as cp
import numpy as np
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray

from convoy_interfaces.msg import CourseData, NeighborInfo, Trajectory
from convoy_planning.plan_utils import (
    line_of_best_fit,
    get_centerline,
    create_marker_array,
    create_trajectory_msg,
)


"""
Subscribe to point cloud 2 topics for the boundaries (the course walls).
Using these boundaries, plan a trajectory inside the course.
"""


class PolyPlanner(Node):

    def __init__(self):
        super().__init__("head_planning_node")

        # ROS parameters
        veh_ns = self.declare_parameter("veh_ns").get_parameter_value().string_value
        self.veh_frame = (
            self.declare_parameter("veh_frame").get_parameter_value().string_value
        )

        if veh_ns:
            self.veh_frame = veh_ns + "/" + self.veh_frame

        self.use_sim = (
            self.declare_parameter("use_sim").get_parameter_value().bool_value
        )
        course_data_topic = (
            self.declare_parameter("course_data_topic")
            .get_parameter_value()
            .string_value
        )
        self.controller = (
            self.declare_parameter("controller").get_parameter_value().string_value
        )
        self.deg = (
            self.declare_parameter("poly_deg").get_parameter_value().integer_value
        )
        self.traj_length = (
            self.declare_parameter("trajectory_length")
            .get_parameter_value()
            .integer_value
        )
        self.H = (
            self.declare_parameter("planning_horizon", 0)
            .get_parameter_value()
            .integer_value
        )
        traj_topic = (
            self.declare_parameter("planner_trajectory_topic")
            .get_parameter_value()
            .string_value
        )
        traj_vis_topic = (
            self.declare_parameter("planner_trajectory_vis_topic")
            .get_parameter_value()
            .string_value
        )
        self.wheelbase = (
            self.declare_parameter("wheelbase").get_parameter_value().double_value
        )
        neighbor_info_topic = (
            self.declare_parameter("neighbor_info_topic")
            .get_parameter_value()
            .string_value
        )

        # ROS publishers
        self.traj_pub = self.create_publisher(Trajectory, traj_topic, 1)
        self.traj_vis_pub = self.create_publisher(MarkerArray, traj_vis_topic, 1)

        # ROS subscribers
        self.create_subscription(CourseData, course_data_topic, self.course_data_cb, 1)

        self.create_subscription(
            NeighborInfo, neighbor_info_topic, self.neighbor_info_cb, 1
        )

        # variables
        self.traj_color = ColorRGBA()
        self.traj_color.a = 1.0
        self.traj_color.g = 1.0
        self.A = [self.deg]
        self.x_l_rel = 0.0
        self.y_l_rel = 0.0
        self.theta_rel = 0.0

    def neighbor_info_cb(self, msg):
        self.x_l_rel = msg.relative_pose.position.x
        self.y_l_rel = msg.relative_pose.position.y
        self.theta_rel = quat2euler(
            [
                msg.relative_pose.orientation.w,
                msg.relative_pose.orientation.x,
                msg.relative_pose.orientation.y,
                msg.relative_pose.orientation.z,
            ]
        )[2]

    def course_data_cb(self, msg):
        # get left and right point cloud x, y
        left_points_list = point_cloud2.read_points_list(msg.left_wall)
        left_pc = np.array(left_points_list)
        right_points_list = point_cloud2.read_points_list(msg.right_wall)
        right_pc = np.array(right_points_list)
        if len(left_pc) > 0:
            left_pc = left_pc[:, :2]
        if len(right_pc) > 0:
            right_pc = right_pc[:, :2]

        if right_pc.shape[0] > 0 and left_pc.shape[0] > 0:
            left_poly = line_of_best_fit(left_pc, self.deg)
            right_poly = line_of_best_fit(right_pc, self.deg)

            if right_poly.domain[1] > left_poly.domain[1]:
                far_poly = right_poly
                near_poly = left_poly
            else:
                far_poly = left_poly
                near_poly = right_poly

            domain_near_min = min(left_poly.domain[0], right_poly.domain[0])
            domain_near_max = min(left_poly.domain[1], right_poly.domain[1])
            near_size = domain_near_max - domain_near_min
            domain_near = np.array([domain_near_min, domain_near_max])

            near_poly = near_poly.convert(domain=domain_near)

            domain_far_min = domain_near[1]
            domain_far_max = max(left_poly.domain[1], right_poly.domain[1])
            far_size = domain_near_max - domain_near_min
            domain_far = np.array([domain_far_min, domain_far_max])

            far_poly = far_poly.convert(domain=domain_far)

            near_length = round(near_size / (near_size + far_size) * self.traj_length)
            far_length = self.traj_length - near_length

            # get nearby trajectory points by averaging polys
            left_poly_near = left_poly.convert(domain=domain_near)
            right_poly_near = right_poly.convert(domain=domain_near)
            center_poly_near = get_centerline(left_poly_near, right_poly_near)
            center_x_near, center_y_near = center_poly_near.linspace(near_length)
            center_traj_near = np.c_[center_x_near, center_y_near]

            # slightly more sophisticated method to get far trajectory points
            x_far, y_far = far_poly.linspace(far_length)
            traj_far = np.c_[x_far, y_far]

            furthest_near_x = domain_near_max
            furthest_near_y = near_poly(furthest_near_x)
            furthest_near_point = np.array([furthest_near_x, furthest_near_y])

            center_traj_far = (traj_far + furthest_near_point) / 2.0
            if self.use_sim:
                traj_points = center_traj_near
            else:
                traj_points = np.r_[center_traj_near, center_traj_far]

            # for future use
            poly = center_poly_near

        elif right_pc.shape[0] > 0:
            poly = line_of_best_fit(right_pc, self.deg)
            poly += msg.track_width / 2.0
            traj_x, traj_y = poly.linspace(self.traj_length)
            traj_points = np.c_[traj_x, traj_y]

        elif left_pc.shape[0] > 0:
            poly = line_of_best_fit(left_pc, self.deg)
            poly -= msg.track_width / 2.0
            traj_x, traj_y = poly.linspace(self.traj_length)
            traj_points = np.c_[traj_x, traj_y]

        else:
            self.get_logger().info("planner did not receive course boundaries")
            return

        if self.controller == "mpc":
            # within traj points, find self.H (horizon length) points to track
            dists = np.linalg.norm(traj_points, axis=1)
            closest_ind = np.argmin(dists)
            traj_points = traj_points[closest_ind:]
            lengths = np.linalg.norm(traj_points[1:] - traj_points[:-1], axis=1)
            traj_length = np.sum(lengths)

            step_length = traj_length / (self.H - 1)

            mpc_traj = []
            length_sum = 0.0
            for i in range(len(lengths)):
                length_sum += lengths[i]
                if length_sum >= step_length * len(mpc_traj):
                    mpc_traj.append(traj_points[i])
            traj_points = np.array(mpc_traj)

        # get the crosstrack error and heading error
        poly_deriv = poly.deriv()
        crosstrack_error = poly(self.wheelbase)
        heading_error = np.arctan2(poly_deriv(self.wheelbase), 1)

        # constrained planning

        # x_traj = traj_points[:, 0]
        # y_traj = traj_points[:, 1]

        # least squares data matrix
        # X = np.array([[x**j for j in range(self.deg + 1)] for x in x_traj])
        # self.get_logger().info(f"X shape: {X.shape}")

        # x_tilde_l = np.array([self.x_l_rel**i for i in range(self.deg+1)])

        # opt variable is polynomial coefficients
        # a = cp.Variable(self.deg + 1)
        # a_prime = np.array([i * a[i] for i in range(1, self.deg+1)] + [0])

        # obj = cp.Minimize(cp.sum_squares(X @ a - y_traj))
        # constraints = [
        # a @ x_tilde_l == self.y_l_rel,
        # a_prime @ x_tilde_l == np.tan(self.theta_rel),
        # a[0] == poly(0.0)
        # ]
        # prob = cp.Problem(obj, constraints)
        # prob.solve()

        # if prob.status == 'optimal':
        #     a_opt = a.value
        #     # self.get_logger().info(f"optimal a: {a_opt}")

        #     domain_min = x_traj.min()
        #     domain_max = x_traj.max()
        #     traj_x = np.linspace(domain_min, domain_max, self.traj_length)
        #     X_traj = np.array([[x**j for j in range(self.deg+1)] for x in traj_x])
        #     traj_y = X_traj @ a_opt
        #     constr_traj_points = np.c_[traj_x, traj_y]
        # else:
        #     constr_traj_points = traj_points

        constr_traj_points = traj_points

        # publish contrained trajectory
        position_traj = create_trajectory_msg(constr_traj_points)
        position_traj.header.frame_id = self.veh_frame
        position_traj.header.stamp = self.get_clock().now().to_msg()
        position_traj.crosstrack_error = crosstrack_error
        position_traj.heading_error = heading_error
        self.traj_pub.publish(position_traj)

        # visualize trajectory
        markers = create_marker_array(
            constr_traj_points, self.traj_color, self.veh_frame
        )
        self.traj_vis_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    poly_planner = PolyPlanner()
    rclpy.spin(poly_planner)
    poly_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
