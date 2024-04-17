from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    veh_ns = LaunchConfiguration("veh_ns")
    veh_ns_la = DeclareLaunchArgument(
        "veh_ns",
        description="The assigned vehicle namespace in the convoy",
        choices=["veh_0", "veh_1", "veh_2", "veh_3", "veh_4", "veh_5", "veh_6"],
    )
    ld.add_action(veh_ns_la)

    test_name = LaunchConfiguration("test_name")
    test_name_la = DeclareLaunchArgument(
        "test_name", description="Name of the folder where we save the data"
    )
    ld.add_action(test_name_la)

    bag_process_node = Node(
        package="convoy_data",
        executable="bag_process.py",
        name="bag_process",
        namespace=veh_ns,
        parameters=[{"veh_ns": veh_ns}, {"test_name": test_name}],
        output="screen",
    )
    ld.add_action(bag_process_node)

    return ld
