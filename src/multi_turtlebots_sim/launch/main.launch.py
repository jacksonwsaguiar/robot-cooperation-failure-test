import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to your package's share directory
    pkg_multi_turtlebots_sim = get_package_share_directory('multi_turtlebots_sim')

    # Path to the Gazebo world file
    world_file = os.path.join(pkg_multi_turtlebots_sim, 'turtlebot3_description', 'world', 'turtlebot3_house.world')

    # Command to start Gazebo with the specified world
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Include the robots.launch.py file
    include_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_turtlebots_sim, 'launch', 'robots.launch.py')
        )
    )

    return LaunchDescription([
        # Set the use_sim_time parameter to true for all nodes
        SetEnvironmentVariable(name='USE_SIM_TIME', value='true'),
        
        # Start Gazebo
        start_gazebo_cmd,

        # Include the other launch file to spawn the robots
        include_robots_launch
    ])