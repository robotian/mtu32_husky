import os


from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch.conditions import IfCondition, UnlessCondition

from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
def generate_launch_description():


    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()


    remappings_tf=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
            ]

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        namespace=f'/{namespace}/sensors/lidar2d_0',
        parameters=[
            PathJoinSubstitution(
                [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "hokuyo_lidar_filter.yaml"]
            )
        ],
        remappings= remappings_tf,        
    )


    mocap_fake_ekf_node = Node(
        package='mocap_fake_localizer',
        executable='mocap_fake_ekf_node',
        name='mocap_fake_ekf_node',
        output='screen',
        namespace=f'/{namespace}',
        parameters=[
            {'mocap_odom_topic': 'ground_truth/odom'},
        ],
        remappings=[
            ('/tf',f'{namespace}/tf'),
            ('/tf_static',f'{namespace}/tf_static'),         
            ('odom_filtered', f'{namespace}/platform/odom_filtered')
        ],
    )

    static_tf_mocap2map_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mocap2map_publisher',
        namespace=f'/{namespace}',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_mocap', 'map'],
        remappings = remappings_tf,
    )

    static_tf_map2odom_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map2odom_publisher',
        namespace=f'/{namespace}',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        remappings = remappings_tf,
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(la_setup_path)
    ld.add_action(laser_filter_node)
    ld.add_action(mocap_fake_ekf_node)
    ld.add_action(static_tf_mocap2map_publisher_node)
    ld.add_action(static_tf_map2odom_publisher_node)
    

    return ld
