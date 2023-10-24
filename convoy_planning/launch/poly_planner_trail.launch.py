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

    veh_ns = LaunchConfiguration('veh_ns')
    veh_ns_la = DeclareLaunchArgument(
        'veh_ns', 
        description='The assigned vehicle namespace in the convoy',
        choices=['veh_0', 'veh_1', 'veh_2', 'veh_3', 'veh_4', 'veh_5', 'veh_6']
    )
    ld.add_action(veh_ns_la)

    use_sim = LaunchConfiguration('use_sim')
    use_sim_la = DeclareLaunchArgument('use_sim', default_value='false')
    ld.add_action(use_sim_la)

    convoy_sim_config = PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'sim.yaml'
    ])

    convoy_real_config = PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'real.yaml'
    ])

    plan_general_config = PathJoinSubstitution([
        FindPackageShare('convoy_planning'), 'config', 
        'plan_general.yaml'
    ])

    plan_sim_node = Node(
        package='convoy_planning',
        executable='poly_planner_trail.py',
        name='poly_planner_node',
        namespace=veh_ns,
        parameters=[convoy_sim_config, plan_general_config,
                    {'veh_ns': veh_ns, 'use_sim': use_sim}],
        condition=IfCondition(use_sim),
        output='screen'
    )
    ld.add_action(plan_sim_node)

    plan_real_node = Node(
        package='convoy_planning',
        executable='poly_planner_trail.py',
        name='poly_planner_node',
        namespace=veh_ns,
        parameters=[convoy_real_config, plan_general_config,
                    {'veh_ns': '', 'use_sim': use_sim}],
        condition=UnlessCondition(use_sim),
        output='screen'
    )
    ld.add_action(plan_real_node)

    return ld
