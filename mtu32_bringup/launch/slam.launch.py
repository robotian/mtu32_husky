import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to your custom params file
    # Replace 'my_robot_bringup' with your actual package name if using standard ROS layout
    default_params_file = os.path.join(
        get_package_share_directory('mtu32_bringup'), 
        'config', 
        'j100',
        'slam_toolbox_gps.yaml'
    )

    # 2. Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to pass to slam_toolbox'
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/map','map'), ('/map_update','map_update'), ('/map_metadata','map_metadata')]

    # 3. Define the SLAM Toolbox Async Node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='j100_0921',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=remappings,
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='slam_lifecycle_manager',
        namespace='j100_0921',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},  # Automatically triggers configure -> activate
            {'node_names': ['slam_toolbox']},  # Target node to manage
            {'bond_timeout': 0.0}
        ]
    )

    # 4. Create and build Launch Description
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(lifecycle_manager_node)

    return ld