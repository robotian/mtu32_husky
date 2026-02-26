import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Auto-start docking lifecycle manager.'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
]

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    bringup_dir = get_package_share_directory('mtu32_bringup')
    setup_path = LaunchConfiguration('setup_path')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace

    remappings_tf=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ]
    
    docking_params_file = os.path.join(
        bringup_dir, 'config', 'a300', 'husky_docking_config.yaml')
    
    load_nodes = GroupAction(
        actions=[
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time},
                            docking_params_file],
                remappings=remappings_tf
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_docking',
                output='screen',
                namespace=namespace,
                parameters=[{'autostart': autostart}, {'node_names': ['docking_server']}],
            )
        ]
    )
    return [load_nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld