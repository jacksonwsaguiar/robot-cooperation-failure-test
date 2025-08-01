import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Path to your package
    pkg_multi_turtlebots_sim = get_package_share_directory('multi_turtlebots_sim')
    pkg_description = get_package_share_directory('turtlebot3_description')
    # Path to the single robot launch file
    one_robot_launch_file = os.path.join(pkg_multi_turtlebots_sim, 'launch', 'one_robot.launch.py')

    # --- Robot Description (from XACRO) ---
    # This processes the XACRO file to get the robot's URDF description.
    xacro_file = os.path.join(pkg_description, 'urdf', 'turtlebot3_burger.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Robot Configurations ---
    # A list of dictionaries, one for each robot
    robots = [
        {'name': 'robot1', 'x_pose': '3.0', 'y_pose': '1.0', 'z_pose': '0.0'},
        {'name': 'robot2', 'x_pose': '-4.0', 'y_pose': '1.0', 'z_pose': '0.0'},
        {'name': 'robot3', 'x_pose': '1.0', 'y_pose': '-6.0', 'z_pose': '0.0'}
    ]

    # --- Create Launch Actions ---
    launch_description_nodes = []
    
    # Shared robot_state_publisher
    # In ROS 2, you typically run one robot_state_publisher and use namespaces
    # to separate the TF frames if necessary.
    launch_description_nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    ))

    # Loop through the robot configurations
    for robot in robots:
        # Include the launch file for one robot
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(one_robot_launch_file),
            launch_arguments={
                'robot_name': robot['name'],
                'x_pose': robot['x_pose'],
                'y_pose': robot['y_pose'],
                'z_pose': robot['z_pose']
            }.items()
        )
        launch_description_nodes.append(robot_launch)

    return LaunchDescription(launch_description_nodes)