from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

import os


def generate_launch_description():
    ld = LaunchDescription()

    # veh_ns = LaunchConfiguration('veh_ns')
    # veh_ns_la = DeclareLaunchArgument(
    #     'veh_ns',
    #     description='The assigned vehicle namespace in the convoy',
    #     choices=['veh_0', 'veh_1', 'veh_2', 'veh_3', 'veh_4', 'veh_5', 'veh_6'],
    # )
    # ld.add_action(veh_ns_la)

    test_name = LaunchConfiguration("test_name")
    test_name_la = DeclareLaunchArgument(
        "test_name", description="Name of the folder where we save the data"
    )
    ld.add_action(test_name_la)

    bag_save_location = PathJoinSubstitution(
        [
            os.path.expanduser("~"),
            "cvy_ws",
            "src",
            "convoy_ros",
            "convoy_data",
            "bags",
            "sim",
            test_name,
        ]
    )

    bag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_save_location,
            "subset",
            "veh_0/drive",
            "veh_0/odom",
            "veh_1/drive",
            "veh_1/odom",
            "veh_1/neighbor_info",
            "veh_2/drive",
            "veh_2/odom",
            "veh_2/neighbor_info",
            "veh_3/drive",
            "veh_3/odom",
            "veh_3/neighbor_info",
            "veh_4/drive",
            "veh_4/odom",
            "veh_4/neighbor_info",
            "veh_5/drive",
            "veh_5/odom",
            "veh_5/neighbor_info",
            "veh_6/drive",
            "veh_6/odom",
            "veh_6/neighbor_info",
        ],
        output="screen",
    )
    ld.add_action(bag_process)

    return ld
