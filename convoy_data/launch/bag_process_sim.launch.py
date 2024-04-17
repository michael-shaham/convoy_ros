from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    test_name = LaunchConfiguration("test_name")
    test_name_la = DeclareLaunchArgument(
        "test_name", description="Name of the folder where we save the data"
    )
    ld.add_action(test_name_la)

    config = os.path.join(
        get_package_share_directory("convoy_bringup"), "config", "gym_bridge_multi.yaml"
    )
    config_dict = yaml.safe_load(open(config, "r"))
    config_dict = config_dict["/**"]["ros__parameters"]

    n_vehicles = config_dict["n_vehicles"]

    for i in range(n_vehicles):
        ld.add_action(
            Node(
                package="convoy_data",
                executable="bag_process_sim.py",
                name="bag_process",
                namespace="veh_" + str(i),
                parameters=[{"veh_ns": "veh_" + str(i)}, {"test_name": test_name}],
                output="screen",
            )
        )

    return ld
