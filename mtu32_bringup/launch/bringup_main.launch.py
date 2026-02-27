import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                          default_value='',
                          description='Override the default 2D laserscan topic'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),  # noqa: E501
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),    
    DeclareLaunchArgument('use_mocap_fake_localizer', default_value='false',
                          choices=['true', 'false'],
                          description=''),

]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory('mtu32_bringup')

    setup_path = LaunchConfiguration('setup_path')
    use_mocap_fake_localizer = LaunchConfiguration('use_mocap_fake_localizer')


    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    remappings_tf=[
                ('/tf',f'/{namespace}/tf'),
                ('/tf_static',f'/{namespace}/tf_static'),
            ]

    load_nodes = GroupAction(        
        actions=[
            # laser filter node to filter hokuyo lidar scan data, since the hokuyo lidar is quite noisy and can cause issues for localization and navigation
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                output='screen',
                namespace=f'/{namespace}/sensors/lidar2d_0',
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "hokuyo_lidar_filter.yaml"]
                    )
                ],
                remappings= remappings_tf,  
            ),

            # mocap fake ekf node to provide filtered odometry for localization and navigation, using mocap ground truth as input
            Node(
                package='mocap_fake_localizer',  # tf: odom -> base_link
                executable='mocap_fake_ekf_node',
                name='mocap_fake_ekf_node',
                output='screen',
                namespace=f'/{namespace}',
                parameters=[
                    {'mocap_odom_topic': 'ground_truth/odom'},
                ],
                remappings= remappings_tf  + [('odom_filtered', 'platform/odom_filtered')],        
                condition=UnlessCondition(use_mocap_fake_localizer),
            ),            

            # # static transform from mocap frame to map frame, since mocap provides ground truth in the map frame
            Node(
                package='tf2_ros', # tf: base_mocap -> map
                executable='static_transform_publisher',
                name='static_tf_mocap2map_publisher',
                namespace=f'/{namespace}',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'base_mocap', 'map'],
                remappings = remappings_tf,
                condition=UnlessCondition(use_mocap_fake_localizer),
            ),
            # static transform from map frame to odom frame, since mocap provides ground truth in the map frame and we want to use that as our odometry source for localization and navigation
            Node(
                package='tf2_ros', # tf: map -> odom
                executable='static_transform_publisher',
                name='static_tf_map2odom_publisher',
                namespace=f'/{namespace}',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                remappings = remappings_tf,
                condition=UnlessCondition(use_mocap_fake_localizer),
            ),

            Node(
                package='mocap_fake_localizer', # tf: map -> odom
                executable='mocap_fake_localizer_node',
                name='mocap_fake_localizer_node',
                output='screen',
                namespace=f'/{namespace}',
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "fake_localizer_config.yaml"]
                    )
                ],
                remappings= remappings_tf,        
                condition=IfCondition(use_mocap_fake_localizer),
            ),            
        ],
    )
    return [load_nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld