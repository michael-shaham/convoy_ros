from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    head_long_controller = LaunchConfiguration("head_long_controller")
    head_long_controller_la = DeclareLaunchArgument(
        "head_long_controller",
        default_value="dmpc",
        description="The controller that will be used to select speeds/accels",
        choices=[
            "constant_input",
            "track_vl",
            "dmpc",
            "l1_pfmpc",
            "quad_pfmpc",
            "nn_feedback",
            "linear_mpc",
            "double_int_mpc",
            "bicycle_mpc",
        ],
    )
    ld.add_action(head_long_controller_la)

    head_lat_controller = LaunchConfiguration("head_lat_controller")
    head_lat_controller_la = DeclareLaunchArgument(
        "head_lat_controller",
        default_value="pure_pursuit",
        description="The controller that will be used to select steering angles.",
        choices=["bicycle_mpc", "pure_pursuit", "stanley"],
    )
    ld.add_action(head_lat_controller_la)

    trail_long_controller = LaunchConfiguration("trail_long_controller")
    trail_long_controller_la = DeclareLaunchArgument(
        "trail_long_controller",
        default_value="dmpc",
        description="Controller used to determine acceleration of trail cars.",
        choices=["linear_feedback", "dmpc", "quad_pfmpc", "l1_pfmpc", "nn_feedback"],
    )
    ld.add_action(trail_long_controller_la)

    trail_lat_controller = LaunchConfiguration("trail_lat_controller")
    trail_lat_controller_la = DeclareLaunchArgument(
        "trail_lat_controller",
        default_value="pure_pursuit",
        description="Controller used to determine acceleration of trail cars.",
        choices=["pure_pursuit", "stanley"],
    )
    ld.add_action(trail_lat_controller_la)

    head_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("convoy_control"),
                    "launch",
                    "head_algorithms.launch.py",
                ]
            )
        ),
        launch_arguments={
            "veh_ns": "veh_0",
            "use_sim": "true",
            "long_controller": head_long_controller,
            "lat_controller": head_lat_controller,
        }.items(),
    )
    ld.add_action(head_include)

    data_vis = LaunchConfiguration("data_vis")
    data_vis_la = DeclareLaunchArgument(
        "data_vis", default_value="false", choices=["true", "false"]
    )
    ld.add_action(data_vis_la)

    plotjuggler_layout = os.path.join(
        get_package_share_directory("convoy_data"),
        "plotjuggler",
        "predecessor_error_layout.xml",
    )
    plotjuggler = ExecuteProcess(
        cmd=[["ros2 run plotjuggler plotjuggler -l ", plotjuggler_layout]],
        shell=True,
        condition=IfCondition(data_vis),
    )
    ld.add_action(plotjuggler)

    log_data = LaunchConfiguration("log_data")
    log_data_la = DeclareLaunchArgument(
        "log_data", default_value="false", choices=["true", "false"]
    )
    ld.add_action(log_data_la)

    trial_name = LaunchConfiguration("trial_name")
    trial_name_la = DeclareLaunchArgument("trial_name", default_value="test")
    ld.add_action(trial_name_la)

    log_data_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("convoy_data"), "launch", "log_data.launch.py"]
            )
        ),
        launch_arguments={"use_sim": "true", "trial_name": trial_name}.items(),
        condition=IfCondition(log_data),
    )
    ld.add_action(log_data_include)

    bridge_config = os.path.join(
        get_package_share_directory("convoy_bringup"), "config", "gym_bridge_multi.yaml"
    )
    bridge_config_dict = yaml.safe_load(open(bridge_config, "r"))
    bridge_config_dict = bridge_config_dict["/**"]["ros__parameters"]
    num_trail = bridge_config_dict["n_vehicles"] - 1

    for i in range(1, num_trail + 1):
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("convoy_control"),
                            "launch",
                            "trail_algorithms.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "veh_ns": "veh_" + str(i),
                    "use_sim": "true",
                    "neighbor_ind": str(i - 1),
                    "long_controller": trail_long_controller,
                    "lat_controller": trail_lat_controller,
                }.items(),
            )
        )

    return ld
