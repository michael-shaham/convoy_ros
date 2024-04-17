from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    ld = LaunchDescription()

    veh_ns = LaunchConfiguration("veh_ns")
    veh_ns_la = DeclareLaunchArgument(
        "veh_ns",
        default_value="",
        description="The assigned vehicle namespace in the convoy",
        choices=["", "veh_0", "veh_1", "veh_2", "veh_3", "veh_4", "veh_5", "veh_6"],
    )
    ld.add_action(veh_ns_la)

    rviz = LaunchConfiguration("rviz")
    rviz_la = DeclareLaunchArgument(
        "rviz", default_value="false", choices=["true", "false"]
    )
    ld.add_action(rviz_la)

    plotjuggler = LaunchConfiguration("plotjuggler")
    plotjuggler_la = DeclareLaunchArgument(
        "plotjuggler", default_value="false", choices=["true", "false"]
    )
    ld.add_action(plotjuggler_la)

    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "rviz", "veh_vis.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=veh_ns,
        arguments=["-d", rviz_config_dir],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/initialpose", "initialpose"),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
        ],
        condition=IfCondition(rviz),
    )
    ld.add_action(rviz_node)

    plotjuggler_layout = os.path.join(
        get_package_share_directory("convoy_data"),
        "plotjuggler",
        "predecessor_error_layout.xml",
    )
    plotjuggler_process = ExecuteProcess(
        cmd=[["ros2 run plotjuggler plotjuggler -l ", plotjuggler_layout]],
        shell=True,
        condition=IfCondition(plotjuggler),
    )
    ld.add_action(plotjuggler_process)

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
        launch_arguments={"use_sim": "false", "trial_name": trial_name}.items(),
        condition=IfCondition(log_data),
    )
    ld.add_action(log_data_include)

    return ld
