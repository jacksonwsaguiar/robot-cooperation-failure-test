import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Declare arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    # Create a namespaced version of the Nav2 parameters
    namespaced_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    return LaunchDescription([
        # --- Declare All Launch Arguments ---
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', description='Full path to the ROS2 parameters file to use'),
        
        # --- Start the Nav2 Stack for a single robot ---
        GroupAction(
            actions=[
                PushRosNamespace(namespace=namespace),
                
                # --- AMCL Node ---
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[namespaced_params]),

                # --- Controller, Planner, and BT Navigator Nodes ---
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    parameters=[namespaced_params]),
                
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[namespaced_params]),
                    
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[namespaced_params]),

                # --- Lifecycle Manager ---
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'autostart': autostart,
                                 'node_names': [
                                     'amcl',
                                     'controller_server',
                                     'planner_server',
                                     'bt_navigator'
                                 ]}])
            ]
        )
    ])