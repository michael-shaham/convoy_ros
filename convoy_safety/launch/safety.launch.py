from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    veh_ns = LaunchConfiguration("veh_ns")
    veh_ns_la = DeclareLaunchArgument(
        "veh_ns",
        choices=["veh_0", "veh_1", "veh_2", "veh_3", "veh_4", "veh_5", "veh_6"],
    )
    ld.add_action(veh_ns_la)

    use_sim = LaunchConfiguration("use_sim")
    use_sim_la = DeclareLaunchArgument("use_sim", default_value="false")
    ld.add_action(use_sim_la)

    is_trail = LaunchConfiguration("is_trail")
    is_trail_la = DeclareLaunchArgument(
        "is_trail",
        default_value="false",
        description="Indicates if the vehicle is a trail vehicle",
        choices=["true", "false"],
    )
    ld.add_action(is_trail_la)

    head_ind = LaunchConfiguration("head_ind")
    head_ind_la = DeclareLaunchArgument(
        "head_ind",
        default_value="0",
        description="The index of the vehicle that is head of the convoy.",
        choices=["0", "1", "2", "3", "4", "5", "6"],
    )
    ld.add_action(head_ind_la)

    convoy_sim_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "sim.yaml"]
    )

    convoy_real_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "real.yaml"]
    )

    # nodes

    autonomous_control_sim_node = Node(
        package="convoy_safety",
        executable="autonomous_control_node.py",
        name="autonomous_control",
        namespace=veh_ns,
        parameters=[
            convoy_sim_config,
            {"use_sim": use_sim},
            {"is_trail": is_trail},
            {"head_ind": head_ind},
        ],
        condition=IfCondition(use_sim),
        output="screen",
    )
    ld.add_action(autonomous_control_sim_node)

    autonomous_control_real_node = Node(
        package="convoy_safety",
        executable="autonomous_control_node.py",
        name="autonomous_control",
        namespace=veh_ns,
        parameters=[
            convoy_real_config,
            {"use_sim": use_sim},
            {"is_trail": is_trail},
            {"head_ind": head_ind},
        ],
        condition=UnlessCondition(use_sim),
        output="screen",
    )
    ld.add_action(autonomous_control_real_node)

    return ld
