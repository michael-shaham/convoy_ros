from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    use_sim = LaunchConfiguration('use_sim')
    use_sim_la = DeclareLaunchArgument('use_sim', default_value='true')
    ld.add_action(use_sim_la)

    bridge_config =  PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'gym_bridge_multi.yaml'
    ])
    
    convoy_sim_config = PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'sim.yaml'
    ])

    convoy_real_config = PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'real.yaml'
    ])

    learning_config = PathJoinSubstitution([
        FindPackageShare('convoy_learning'), 'config', 'learning_params.yaml'
    ])

    learning_sim_node = Node(
        package='convoy_learning',
        executable='crash_detect.py',
        name='crash_detect',
        parameters=[convoy_sim_config, learning_config, bridge_config],
        condition=IfCondition(use_sim)
    )
    ld.add_action(learning_sim_node)

    learning_real_node = Node(
        package='convoy_learning',
        executable='crash_detect_real.py',
        name='crash_detect',
        parameters=[convoy_real_config, learning_config, bridge_config],
        condition=UnlessCondition(use_sim)
    )
    # ld.add_action(learning_real_node)

    return ld