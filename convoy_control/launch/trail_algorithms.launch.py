import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression

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

    use_sim = LaunchConfiguration("use_sim")
    use_sim_la = DeclareLaunchArgument(
        "use_sim", default_value="false", choices=["true", "false"]
    )
    ld.add_action(use_sim_la)

    neighbor_ind = LaunchConfiguration("neighbor_ind")
    neighbor_ind_la = DeclareLaunchArgument(
        "neighbor_ind", choices=["0", "1", "2", "3", "4", "5", "6"]
    )
    ld.add_action(neighbor_ind_la)

    long_controller = LaunchConfiguration("long_controller")
    long_controller_la = DeclareLaunchArgument(
        "long_controller",
        default_value="dmpc",
        choices=["linear_feedback", "dmpc", "l1_pfmpc", "nn_feedback"],
    )
    ld.add_action(long_controller_la)

    lat_controller = LaunchConfiguration("lat_controller")
    lat_controller_la = DeclareLaunchArgument(
        "lat_controller",
        default_value="pure_pursuit",
        choices=["pure_pursuit", "stanley"],
    )
    ld.add_action(lat_controller_la)

    #### config files ####

    convoy_sim_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "sim.yaml"]
    )

    convoy_real_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", "real.yaml"]
    )

    #### safety ####

    safe_speed_sim_node = Node(
        package="convoy_safety",
        executable="calculate_safe_speed.py",
        name="safe_speed",
        namespace=veh_ns,
        parameters=[convoy_sim_config],
        condition=IfCondition(use_sim),
        output="screen",
    )
    ld.add_action(safe_speed_sim_node)

    safe_speed_real_node = Node(
        package="convoy_safety",
        executable="calculate_safe_speed.py",
        name="safe_speed",
        namespace=veh_ns,
        parameters=[convoy_real_config],
        condition=UnlessCondition(use_sim),
        output="screen",
    )
    ld.add_action(safe_speed_real_node)

    #### drive controller ####

    drive_controller_sim_node = Node(
        package="convoy_control",
        executable="drive_controller.py",
        name="drive_controller",
        namespace=veh_ns,
        parameters=[convoy_sim_config, {"veh_ns": veh_ns}],
        condition=IfCondition(use_sim),
        output="screen",
    )
    ld.add_action(drive_controller_sim_node)

    drive_controller_real_node = Node(
        package="convoy_control",
        executable="drive_controller.py",
        name="drive_controller",
        namespace=veh_ns,
        parameters=[convoy_real_config, {"veh_ns": veh_ns}],
        condition=UnlessCondition(use_sim),
        output="screen",
    )
    ld.add_action(drive_controller_real_node)

    #### neighbor localization ####

    remote_id = PythonExpression(
        [
            "str(0x000f) if '",
            neighbor_ind,
            "' == '0'",
            " else str('0x000') + str(",
            neighbor_ind,
            ")",
        ]
    )

    # include pozyx things
    pozyx_config = PathJoinSubstitution(
        [FindPackageShare("convoy_bringup"), "config", veh_ns, "pozyx.yaml"]
    )
    pozyx_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("convoy_perception"),
                    "launch",
                    "pozyx_ranging.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "pozyx_config": pozyx_config,
            "neighbor_ind": neighbor_ind,
            "remote_id": remote_id,
        }.items(),
        condition=UnlessCondition(use_sim),
    )
    pozyx_include_with_ns = GroupAction(
        actions=[PushRosNamespace(veh_ns), pozyx_include]
    )
    ld.add_action(pozyx_include_with_ns)

    neighbor_localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("convoy_perception"),
                    "launch",
                    "neighbor_localization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "veh_ns": veh_ns,
            "use_sim": use_sim,
            "neighbor_ind": neighbor_ind,
            "remote_id": remote_id,
        }.items(),
    )
    ld.add_action(neighbor_localization_include)

    #### planner ####

    poly_planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("convoy_planning"),
                    "launch",
                    "poly_planner_trail.launch.py",
                ]
            )
        ),
        launch_arguments={
            "veh_ns": veh_ns,
            "use_sim": use_sim,
            "lat_controller": lat_controller,
            "long_controller": long_controller,
        }.items(),
    )
    ld.add_action(poly_planner_include)

    #### lateral control nodes ####

    # pure pursuit

    pure_pursuit_sim_node = Node(
        package="convoy_control",
        executable="pure_pursuit.py",
        name="pure_pursuit",
        namespace=veh_ns,
        parameters=[convoy_sim_config, {"veh_ns": veh_ns}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    lat_controller,
                    "' == 'pure_pursuit' and '",
                    use_sim,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(pure_pursuit_sim_node)

    pure_pursuit_real_node = Node(
        package="convoy_control",
        executable="pure_pursuit.py",
        name="pure_pursuit",
        namespace=veh_ns,
        parameters=[convoy_real_config, {"veh_ns": veh_ns}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    lat_controller,
                    "' == 'pure_pursuit' and '",
                    use_sim,
                    "' == 'false'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(pure_pursuit_real_node)

    # stanley

    stanley_sim_node = Node(
        package="convoy_control",
        executable="stanley.py",
        name="stanley",
        namespace=veh_ns,
        parameters=[convoy_sim_config, {"veh_ns": veh_ns}],
        condition=IfCondition(
            PythonExpression(
                ["'", lat_controller, "' == 'stanley' and '", use_sim, "' == 'true'"]
            )
        ),
        output="screen",
    )
    ld.add_action(stanley_sim_node)

    stanley_real_node = Node(
        package="convoy_control",
        executable="stanley.py",
        name="stanley",
        namespace=veh_ns,
        parameters=[convoy_real_config, {"veh_ns": veh_ns}],
        condition=IfCondition(
            PythonExpression(
                ["'", lat_controller, "' == 'stanley' and '", use_sim, "' == 'false'"]
            )
        ),
        output="screen",
    )
    ld.add_action(stanley_real_node)

    #### longitudinal control nodes ####

    # linear feedback

    linear_feedback_sim_node = Node(
        package="convoy_control",
        executable="linear_feedback.py",
        name="linear_feedback",
        namespace=veh_ns,
        parameters=[convoy_sim_config, {"veh_ns": veh_ns, "use_sim": use_sim}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'linear_feedback' and '",
                    use_sim,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(linear_feedback_sim_node)

    linear_feedback_real_node = Node(
        package="convoy_control",
        executable="linear_feedback.py",
        name="linear_feedback",
        namespace=veh_ns,
        parameters=[convoy_real_config, {"veh_ns": veh_ns, "use_sim": use_sim}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'linear_feedback' and '",
                    use_sim,
                    "' == 'false'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(linear_feedback_real_node)

    # dmpc

    dmpc_sim_node = Node(
        package="convoy_control",
        executable="dmpc.py",
        name="dmpc",
        namespace=veh_ns,
        parameters=[
            convoy_sim_config,
            {"veh_ns": veh_ns, "use_sim": use_sim, "neighbor_ind": neighbor_ind},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'dmpc' and '",
                    use_sim,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(dmpc_sim_node)

    dmpc_real_node = Node(
        package="convoy_control",
        executable="dmpc.py",
        name="dmpc",
        namespace=veh_ns,
        parameters=[
            convoy_real_config,
            {"veh_ns": veh_ns, "use_sim": use_sim, "neighbor_ind": neighbor_ind},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'dmpc' and '",
                    use_sim,
                    "' == 'false'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(dmpc_real_node)

    # quad_pfmpc

    quad_pfmpc_sim_node = Node(
        package="convoy_control",
        executable="quad_pfmpc.py",
        name="quad_pfmpc",
        namespace=veh_ns,
        parameters=[
            convoy_sim_config,
            {"veh_ns": veh_ns, "use_sim": use_sim, "neighbor_ind": neighbor_ind},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'quad_pfmpc' and '",
                    use_sim,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(quad_pfmpc_sim_node)

    quad_pfmpc_real_node = Node(
        package="convoy_control",
        executable="quad_pfmpc.py",
        name="quad_pfmpc",
        namespace=veh_ns,
        parameters=[
            convoy_real_config,
            {"veh_ns": veh_ns, "use_sim": use_sim, "neighbor_ind": neighbor_ind},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'quad_pfmpc' and '",
                    use_sim,
                    "' == 'false'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(quad_pfmpc_real_node)

    # l1_pfmpc

    l1_pfmpc_sim_node = Node(
        package="convoy_control",
        executable="l1_pfmpc.py",
        name="l1_pfmpc",
        namespace=veh_ns,
        parameters=[
            convoy_sim_config,
            {"veh_ns": veh_ns, "use_sim": use_sim, "neighbor_ind": neighbor_ind},
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", long_controller, "' == 'l1_pfmpc' and '", use_sim, "' == 'true'"]
            )
        ),
        output="screen",
    )
    ld.add_action(l1_pfmpc_sim_node)

    l1_pfmpc_real_node = Node(
        package="convoy_control",
        executable="l1_pfmpc.py",
        name="l1_pfmpc",
        namespace=veh_ns,
        parameters=[
            convoy_real_config,
            {"veh_ns": veh_ns, "use_sim": use_sim, "neighbor_ind": neighbor_ind},
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", long_controller, "' == 'l1_pfmpc' and '", use_sim, "' == 'false'"]
            )
        ),
        output="screen",
    )
    ld.add_action(l1_pfmpc_real_node)

    # nn_feedback

    model_dir = PathJoinSubstitution([FindPackageShare("convoy_control"), "models"])

    nn_feedback_sim_node = Node(
        package="convoy_control",
        executable="nn_feedback.py",
        name="nn_feedback",
        namespace=veh_ns,
        parameters=[
            convoy_sim_config,
            {"veh_ns": veh_ns, "model_dir": model_dir, "track_vl": False},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'nn_feedback' and '",
                    use_sim,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(nn_feedback_sim_node)

    nn_feedback_real_node = Node(
        package="convoy_control",
        executable="nn_feedback.py",
        name="nn_feedback",
        namespace=veh_ns,
        parameters=[
            convoy_real_config,
            {"veh_ns": veh_ns, "model_dir": model_dir, "track_vl": False},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    long_controller,
                    "' == 'nn_feedback' and '",
                    use_sim,
                    "' == 'false'",
                ]
            )
        ),
        output="screen",
    )
    ld.add_action(nn_feedback_real_node)

    return ld
