from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    ld.add_action(
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
    )

    veh_ns = LaunchConfiguration("veh_ns")
    veh_ns_la = DeclareLaunchArgument(
        "veh_ns",
        description="The assigned vehicle namespace in the convoy",
        choices=["veh_0", "veh_1", "veh_2", "veh_3", "veh_4", "veh_5", "veh_6"],
    )
    ld.add_action(veh_ns_la)

    map_name = LaunchConfiguration("map_name")
    map_name_la = DeclareLaunchArgument(
        "map_name",
        default_value="isec.yaml",
        description="The map name with yaml extension (in convoy_bringup/maps)",
        choices=["isec.yaml"],
    )
    ld.add_action(map_name_la)

    use_sim = LaunchConfiguration("use_sim")
    use_sim_la = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use sim (will only be true if we develop different sim)",
    )
    ld.add_action(use_sim_la)

    autostart = LaunchConfiguration("autostart")
    autostart_la = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )
    ld.add_action(autostart_la)

    amcl_config = PathJoinSubstitution(
        [FindPackageShare("convoy_nav2"), "config", "amcl.yaml"]
    )

    map_yaml = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "maps", map_name]
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=veh_ns,
        output="screen",
        parameters=[{"use_sim_time": use_sim}, {"yaml_filename": map_yaml}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    ld.add_action(map_server_node)

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=veh_ns,
        output="screen",
        parameters=[amcl_config],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("/scan", "scan")],
    )
    ld.add_action(amcl_node)

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace=veh_ns,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim},
            {"autostart": autostart},
            {"node_names": ["map_server", "amcl"]},
        ],
    )
    ld.add_action(lifecycle_manager_node)

    return ld
