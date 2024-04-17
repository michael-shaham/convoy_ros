from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

import os


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

    bag_save_location = PathJoinSubstitution(
        [
            os.path.expanduser("~"),
            "cvy_ws",
            "src",
            "convoy_ros",
            "convoy_data",
            "bags",
            veh_ns,
            test_name,
        ]
    )

    bag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_save_location,
            "subset",
            PathJoinSubstitution([veh_ns, "sensors/imu"]),
            PathJoinSubstitution([veh_ns, "scan_processed"]),
            PathJoinSubstitution([veh_ns, "left_wall_pc"]),
            PathJoinSubstitution([veh_ns, "right_wall_pc"]),
            PathJoinSubstitution([veh_ns, "course_data"]),
            PathJoinSubstitution([veh_ns, "drive"]),
            PathJoinSubstitution([veh_ns, "odom"]),
            PathJoinSubstitution([veh_ns, "pozyx/ranges"]),
            PathJoinSubstitution([veh_ns, "neighbor_info"]),
        ],
        output="screen",
    )
    ld.add_action(bag_process)

    return ld
