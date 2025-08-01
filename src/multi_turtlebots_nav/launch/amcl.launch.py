from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Declare the launch arguments
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create a dictionary of substitutions for the YAML file
    param_substitutions = {
        'initial_pose.x': LaunchConfiguration('initial_pose_x'),
        'initial_pose.y': LaunchConfiguration('initial_pose_y'),
        'initial_pose.a': LaunchConfiguration('initial_pose_a'),
        'odom_frame_id': [namespace, '/odom'],
        'base_frame_id': [namespace, '/base_footprint']
    }

    # Create a configured YAML file with the substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=[('scan', 'scan')] # Remaps will be relative to the namespace
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']
        }]
    )

    return LaunchDescription([
        # We don't need to declare the arguments here again,
        # they will be passed from the main launch file.
        amcl_node,
        lifecycle_manager
    ])