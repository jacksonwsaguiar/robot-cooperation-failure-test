import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Caminho para os pacotes
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('turtlebot3_description')

    # Caminho para o arquivo de mundo
    world_file = os.path.join(pkg_description, 'world', 'turtlebot3_house.world')

    # --- Iniciar o Gazebo (novo) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Passe o arquivo de mundo como um argumento para o Gazebo
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # --- Processar o XACRO ---
    # Garanta que seu XACRO foi atualizado com os novos plugins
    xacro_file = os.path.join(pkg_description, 'urdf', 'turtlebot3_burger.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Adicionar o Robô ao Gazebo ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description['robot_description'],
            '-name', 'turtlebot3',
            '-allow_renaming', 'true',
            '-x', '3.0',
            '-y', '1.0',
            '-z', '0.1'
        ]
    )
    
    # --- Ponte ROS-Gazebo ---
    # Esta ponte retransmite tópicos entre o ROS 2 e o Gazebo
    # Ex: /cmd_vel do ROS 2 para o Gazebo, e /scan, /odom do Gazebo para o ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        spawn_entity,
        bridge
    ])