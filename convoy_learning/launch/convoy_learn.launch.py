from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()

    use_sim = LaunchConfiguration('use_sim')
    use_sim_la = DeclareLaunchArgument('use_sim', default_value='true')
    ld.add_action(use_sim_la)

    bridge_config = os.path.join(
        get_package_share_directory('convoy_bringup'), 'config',
        'gym_bridge_multi.yaml'
    )
    bridge_config_dict = yaml.safe_load(open(bridge_config, 'r'))
    bridge_config_dict = bridge_config_dict['/**']['ros__parameters']
    n_vehicles = bridge_config_dict['n_vehicles']
    y_arr = bridge_config_dict['y_arr']

    convoy_sim_config = PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'sim.yaml'
    ])

    convoy_real_config = PathJoinSubstitution([
        FindPackageShare('convoy_bringup'), 'config', 'real.yaml'
    ])

    learning_config = os.path.join(
        get_package_share_directory('convoy_learning'), 
        'config', 'learning_params.yaml'
    )
    learning_config_dict = yaml.safe_load(open(learning_config, 'r'))
    learning_config_dict = learning_config_dict['/**']['ros__parameters']

    model_name = LaunchConfiguration('model_name')
    model_name_la = DeclareLaunchArgument('model_name',
        description='Neural network model name for saving the neural network.')
    ld.add_action(model_name_la)

    model_path = PathJoinSubstitution([
        os.path.expanduser('~'), 'cvy_ws', 'src', 'convoy_ros', 
        'convoy_learning', 'models', model_name
    ])

    learning_sim_node = Node(
        package='convoy_learning',
        executable='convoy_learn.py',
        name='convoy_learn',
        parameters=[convoy_sim_config, learning_config,
                    {'model_path': model_path, 'n_vehicles': n_vehicles,
                     'y_arr': y_arr}],
        condition=IfCondition(use_sim)
    )
    ld.add_action(learning_sim_node)

    return ld