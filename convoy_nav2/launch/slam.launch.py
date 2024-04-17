from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

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
    use_sim_la = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use sim (will only be true if we develop different sim)",
    )
    ld.add_action(use_sim_la)

    slam_config = PathJoinSubstitution(
        [FindPackageShare("convoy_nav2"), "config", "slam.yaml"]
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=veh_ns,
        parameters=[
            slam_config,
            {"use_sim_time": use_sim},
            {"map_frame": "map"},
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("/scan", "scan")],
        output="screen",
    )
    ld.add_action(slam_node)

    return ld
