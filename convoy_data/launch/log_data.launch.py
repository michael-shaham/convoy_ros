from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    use_sim = LaunchConfiguration("use_sim")
    use_sim_la = DeclareLaunchArgument(
        "use_sim", default_value="false", choices=["true", "false"]
    )
    ld.add_action(use_sim_la)

    trial_name = LaunchConfiguration("trial_name")
    trial_name_la = DeclareLaunchArgument("trial_name", default_value="test")
    ld.add_action(trial_name_la)

    convoy_sim_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "sim.yaml"]
    )

    convoy_real_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "real.yaml"]
    )

    bridge_config = os.path.join(
        get_package_share_directory("convoy_bringup"), "config", "gym_bridge_multi.yaml"
    )
    bridge_config_dict = yaml.safe_load(open(bridge_config, "r"))
    bridge_config_dict = bridge_config_dict["/**"]["ros__parameters"]
    num_veh = bridge_config_dict["n_vehicles"]

    log_data_sim_node = Node(
        package="convoy_data",
        executable="log_data.py",
        name="log_data",
        namespace="",
        parameters=[
            convoy_sim_config,
            {"trial_name": trial_name, "n_vehicles": num_veh, "use_sim": use_sim},
        ],
        condition=IfCondition(use_sim),
        output="screen",
    )
    ld.add_action(log_data_sim_node)

    log_data_real_node = Node(
        package="convoy_data",
        executable="log_data.py",
        name="log_data",
        namespace="",
        parameters=[
            convoy_real_config,
            {"trial_name": trial_name, "n_vehicles": num_veh, "use_sim": use_sim},
        ],
        condition=UnlessCondition(use_sim),
        output="screen",
    )
    ld.add_action(log_data_real_node)

    return ld
