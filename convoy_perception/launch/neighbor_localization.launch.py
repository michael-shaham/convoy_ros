from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    veh_ns = LaunchConfiguration("veh_ns")
    veh_ns_la = DeclareLaunchArgument(
        "veh_ns",
        description="The assigned vehicle namespace in the convoy",
        choices=["veh_0", "veh_1", "veh_2", "veh_3", "veh_4", "veh_5", "veh_6"],
    )
    ld.add_action(veh_ns_la)

    use_sim = LaunchConfiguration("use_sim")
    use_sim_la = DeclareLaunchArgument("use_sim", default_value="false")
    ld.add_action(use_sim_la)

    neighbor_ind = LaunchConfiguration("neighbor_ind")
    neighbor_ind_la = DeclareLaunchArgument(
        "neighbor_ind",
        description="The index of the preceding vehicle",
        choices=["0", "1", "2", "3", "4", "5", "6"],
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

    sim_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "sim.yaml"]
    )

    real_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "real.yaml"]
    )

    pozyx_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", veh_ns, "pozyx.yaml"]
    )

    neighbor_localization_sim_node = Node(
        package="convoy_perception",
        executable="neighbor_localization_sim.py",
        name="neighbor_localization_node",
        namespace=veh_ns,
        parameters=[sim_config, {"preceding_vehicle_ind": neighbor_ind}],
        condition=IfCondition(use_sim),
        output="screen",
    )
    ld.add_action(neighbor_localization_sim_node)

    neighbor_localization_real_node = Node(
        package="convoy_perception",
        executable="neighbor_localization_real.py",
        name="neighbor_localization_node",
        namespace=veh_ns,
        parameters=[
            real_config,
            pozyx_config,
            {"preceding_vehicle_ind": neighbor_ind, "remote_id": remote_id},
        ],
        condition=UnlessCondition(use_sim),
        output="screen",
    )
    ld.add_action(neighbor_localization_real_node)

    return ld
