import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Paths ---
    pkg_gazebo_ros = get_package_share_directory('ros_gz')
    pkg_sim = get_package_share_directory('multi_turtlebots_sim')
    pkg_description = get_package_share_directory('turtlebot3_description')

    
    # --- Launch Arguments ---
    x_pose_arg = DeclareLaunchArgument('x_pos', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pos', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pos', default_value='0.0')

    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')

    # --- Gazebo ---
    # In ROS 2, we include gazebo.launch.py and pass the world file as an argument
    world_file = os.path.join(pkg_description, 'world', 'square.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # --- Robot Description (from XACRO) ---
   xacro_file = os.path.join(pkg_description, 'urdf', 'turtlebot3_burger.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Nodes ---
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn Entity (replaces spawn_model)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_burger',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos
        ],
        output='screen'
    )
    
    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node
    ])