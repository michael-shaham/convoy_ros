from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
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
        choices=["0", "1", "2"],
    )
    ld.add_action(head_ind_la)

    lidar = LaunchConfiguration("lidar")
    lidar_la = DeclareLaunchArgument(
        "lidar",
        default_value="",
        description="The lidar to use",
        choices=["", "rplidar", "hokuyo"],
    )
    ld.add_action(lidar_la)

    use_throttle_interpolator = LaunchConfiguration("use_throttle_interpolator")
    use_throttle_interpolator_la = DeclareLaunchArgument(
        "use_throttle_interpolator", default_value="false"
    )
    ld.add_action(use_throttle_interpolator_la)

    sensors_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "sensors.yaml"]
    )

    joy_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "teleop.yaml"]
    )

    vesc_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", veh_ns, "vesc.yaml"]
    )

    mux_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "mux.yaml"]
    )

    # include lidar launch file
    rplidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sllidar_ros2"), "launch/sllidar_s2_launch.py"]
            ),
        ),
        launch_arguments={"frame_id": "laser"}.items(),
        condition=LaunchConfigurationEquals("lidar", "rplidar"),
    )
    rplidar_include_with_ns = GroupAction(
        actions=[PushRosNamespace(veh_ns), rplidar_include]
    )
    ld.add_action(rplidar_include_with_ns)

    lidar_urg_node = Node(
        package="urg_node",
        executable="urg_node_driver",
        name="urg_node",
        namespace=veh_ns,
        parameters=[sensors_config],
        condition=LaunchConfigurationEquals("lidar", "hokuyo"),
    )
    ld.add_action(lidar_urg_node)

    # joystick/teleop nodes

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        namespace=veh_ns,
        parameters=[joy_config],
    )
    # ld.add_action(joy_node)  # use joy_linux instead

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux",
        namespace=veh_ns,
        parameters=[joy_config],
    )
    ld.add_action(joy_linux_node)

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        namespace=veh_ns,
        parameters=[joy_config],
    )
    ld.add_action(joy_teleop_node)

    # VESC nodes

    ackermann_to_vesc_node = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        namespace=veh_ns,
        parameters=[vesc_config],
    )
    ld.add_action(ackermann_to_vesc_node)

    vesc_to_odom_node = Node(
        package="vesc_ackermann",
        executable="vesc_to_odom_node",
        name="vesc_to_odom_node",
        namespace=veh_ns,
        parameters=[vesc_config],
        remappings=[("/tf", "tf")],
    )
    ld.add_action(vesc_to_odom_node)

    vesc_driver_node = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver_node",
        namespace=veh_ns,
        parameters=[vesc_config],
    )
    ld.add_action(vesc_driver_node)

    throttle_interpolator_node = Node(
        package="f1tenth_stack",
        executable="throttle_interpolator",
        name="throttle_interpolator",
        namespace=veh_ns,
        parameters=[vesc_config],
        condition=IfCondition(use_throttle_interpolator),
    )
    ld.add_action(throttle_interpolator_node)

    # Ackermann mux node

    ackermann_mux_node = Node(
        package="ackermann_mux",
        executable="ackermann_mux",
        name="ackermann_mux",
        namespace=veh_ns,
        parameters=[mux_config],
        remappings=[("ackermann_drive_out", "ackermann_cmd")],
    )
    ld.add_action(ackermann_mux_node)

    # TF publishers
    static_base_lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_baselink_to_laser",
        namespace=veh_ns,
        arguments=["0.27", "0.0", "0.11", "0.0", "0.0", "0.0", "base_link", "laser"],
        remappings=[("/tf_static", "tf_static")],
    )
    ld.add_action(static_base_lidar_tf_node)

    # include safety launch file
    safety = LaunchConfiguration("safety")
    safety_la = DeclareLaunchArgument("safety", default_value="true")
    ld.add_action(safety_la)

    safety_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("convoy_safety"), "launch/safety.launch.py"]
            )
        ),
        launch_arguments={
            "veh_ns": veh_ns,
            "is_trail": is_trail,
            "head_ind": head_ind,
        }.items(),
        condition=IfCondition(safety),
    )
    ld.add_action(safety_include)

    # preprocess lidar
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
        launch_arguments={"veh_ns": veh_ns, "use_sim": "false"}.items(),
    )
    ld.add_action(preprocess_lidar_include)

    # wall segmentation
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
        launch_arguments={"veh_ns": veh_ns}.items(),
    )
    ld.add_action(perception_include)

    return ld
