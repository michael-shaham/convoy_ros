from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    pozyx_ns = LaunchConfiguration("pozyx_ns")
    pozyx_ns_la = DeclareLaunchArgument(
        "pozyx_ns",
        default_value="pozyx",
        description="The namespace for the nodes launched.",
    )
    ld.add_action(pozyx_ns_la)

    pozyx_config = LaunchConfiguration("pozyx_config")
    pozyx_config_la = DeclareLaunchArgument(
        "pozyx_config",
        default_value="",
        description="The params file used for pozyx ranging",
    )
    ld.add_action(pozyx_config_la)

    neighbor_ind = LaunchConfiguration("neighbor_ind")
    neighbor_ind_la = DeclareLaunchArgument(
        "neighbor_ind", default_value="", description="index of ranging device"
    )
    ld.add_action(neighbor_ind_la)

    default_remote_id = PythonExpression(
        [
            "str(0x000f) if '",
            neighbor_ind,
            "' == '0'",
            " else str('0x000') + str(",
            neighbor_ind,
            ")",
        ]
    )
    remote_id = LaunchConfiguration("remote_id")
    remote_id_la = DeclareLaunchArgument(
        "remote_id",
        default_value=default_remote_id,
        description="index of ranging device",
    )
    ld.add_action(remote_id_la)

    pozyx_node = Node(
        package="pozyx_driver",
        executable="pozyx_ranging",
        name="pozyx_ranging",
        namespace=pozyx_ns,
        parameters=[pozyx_config, {"remote_id": remote_id}],
        output="screen",
    )
    ld.add_action(pozyx_node)

    return ld
