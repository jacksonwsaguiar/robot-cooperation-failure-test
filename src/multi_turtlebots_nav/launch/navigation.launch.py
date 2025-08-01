import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Paths ---
    pkg_nav = get_package_share_directory('multi_turtlebots_nav')
    pkg_map_launcher = get_package_share_directory('map_launcher')
    
    # --- File Paths ---
    map_yaml_file = os.path.join(pkg_map_launcher, 'map', 'map.yaml')
    nav2_params_file = os.path.join(pkg_nav, 'param', 'nav2_params.yaml')
    nav2_launch_file = os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
    rviz_config_file = os.path.join(pkg_map_launcher, 'rviz', 'turtlebot3_navigation.rviz')
    
    # --- Robot Configurations ---
    robots = [
        {'name': 'robot1'},
        {'name': 'robot2'},
        {'name': 'robot3'}
    ]

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # --- Add Shared Nodes ---
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'use_sim_time': True}, {'yaml_filename': map_yaml_file}]
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]
    ))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    ))

    # --- Add Nav2 Stack for Each Robot ---
    for robot in robots:
        nav2_instance = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'namespace': robot['name'],
                'use_sim_time': 'true',
                'params_file': nav2_params_file
            }.items()
        )
        ld.add_action(nav2_instance)

    return ld