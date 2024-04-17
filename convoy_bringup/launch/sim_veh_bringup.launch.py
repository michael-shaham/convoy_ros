# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os
import xacro
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    use_sim = LaunchConfiguration("use_sim")
    use_sim_la = DeclareLaunchArgument("use_sim", default_value="true")
    ld.add_action(use_sim_la)

    config = os.path.join(
        get_package_share_directory("convoy_bringup"), "config", "gym_bridge_multi.yaml"
    )
    config_dict = yaml.safe_load(open(config, "r"))
    config_dict = config_dict["/**"]["ros__parameters"]

    maps_path = PathJoinSubstitution([FindPackageShare("convoy_bringup"), "maps"])
    map_name = config_dict["map_name"]
    map_path = PathJoinSubstitution([maps_path, map_name])
    map_yaml = PathJoinSubstitution([maps_path, map_name + ".yaml"])

    bridge_node = Node(
        package="convoy_bringup",
        executable="gym_bridge_multi.py",
        name="gym_bridge_multi",
        parameters=[config, {"map_path": map_path}],
        output="screen",
    )
    ld.add_action(bridge_node)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("convoy_bringup"), "rviz", "sim_gym_bridge.rviz"]
            ),
        ],
    )
    ld.add_action(rviz_node)

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {"yaml_filename": map_yaml},
            {"topic": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
    )
    ld.add_action(map_server_node)

    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )
    ld.add_action(nav_lifecycle_node)

    # include safety launch file

    safety = LaunchConfiguration("safety")
    safety_la = DeclareLaunchArgument("safety", default_value="true")
    ld.add_action(safety_la)

    ns = config_dict["namespace"]
    n_vehicles = config_dict["n_vehicles"]
    for i in range(n_vehicles):
        urdf_path = os.path.join(
            get_package_share_directory("convoy_bringup"), "urdf", "racecar.xacro"
        )
        veh_ns = ns + str(i)
        urdf = xacro.process_file(urdf_path, mappings={"veh_ns": veh_ns}).toxml()
        ld.add_action(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=ns + str(i),
                parameters=[{"use_sim_time": True}, {"robot_description": urdf}],
            )
        )

        preprocess_lidar_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("convoy_perception"),
                        "launch",
                        "preprocess_lidar.launch.py",
                    ]
                ),
            ),
            launch_arguments={"veh_ns": veh_ns, "use_sim": "true"}.items(),
        )
        ld.add_action(preprocess_lidar_include)

        is_trail = "false"
        if i > 0:
            is_trail = "true"
        safety_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("convoy_safety"), "launch/safety.launch.py"]
                )
            ),
            condition=IfCondition(safety),
            launch_arguments={
                "veh_ns": veh_ns,
                "use_sim": use_sim,
                "is_trail": is_trail,
                "head_ind": "0",
            }.items(),
        )
        ld.add_action(safety_include)

        perception_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("convoy_perception"),
                        "launch",
                        "wall_seg_node.launch.py",
                    ]
                )
            ),
            launch_arguments={"veh_ns": veh_ns, "use_sim": "true"}.items(),
        )
        ld.add_action(perception_include)

    return ld
