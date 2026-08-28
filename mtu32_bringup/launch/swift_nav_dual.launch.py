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
from launch.substitutions import PythonExpression

ARGUMENTS = [
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory('mtu32_bringup')

    setup_path = LaunchConfiguration('setup_path')


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
            Node(
                package='swiftnav_ros2_driver',
                executable='sbp-to-ros',
                name='ref_duro_node',
                namespace=f'/{namespace}/sensors/gps_1',
                remappings=[
                    ('navsatfix','fix'),
                ],
                output='screen',
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "dual_duro_heading.yaml"]
                    )
                ]
            ),

            Node(
                package='swiftnav_ros2_driver',
                executable='sbp-to-ros',
                name='att_duro_node',
                namespace=f'/{namespace}/sensors/gps_0',
                remappings=[
                    ('navsatfix','fix'),
                ],
                output='screen',
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "dual_duro_heading.yaml"]
                    )
                ]
            ),

            Node(
                package='dual_duro_heading',
                executable='heading_filter',
                name='dual_duro_heading_node',
                namespace=f'/{namespace}/sensors/gps_0',
                # remappings=[
                #     ('baseline','/baseline'),
                #     ('heading_imu','/heading_imu')
                # ],
                output='screen'
            ),

            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                namespace=f'/{namespace}',
                remappings=remappings_tf + [
                    ('gps/fix', f'/{namespace}/sensors/gps_1/fix'),          # SwiftNav global position topic
                    ('odometry/filtered', f'/{namespace}/odometry/global'),  # from EKF
                    ('imu', f'/{namespace}/sensors/gps_0/heading_imu'),     # Your computed RTK heading topic
                    ('datum', f'/{namespace}/navsat_transform/datum'),
                    ('fromLL', f'/{namespace}/navsat_transform/fromLL'),
                    ('fromLLArray', f'/{namespace}/navsat_transform/fromLLArray'),
                    # ('gps/filtered', f'/{namespace}/gps/filtered'),
                ],
                parameters=[    
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "dual_duro_heading.yaml"]
                    )
                ],
            ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global_node',
                namespace=f'/{namespace}',
                remappings= remappings_tf + [
                    ('odometry/filtered', 'odometry/global'),
                    ('set_pose', 'ekf_global_node/set_pose'),
                    ('enable', 'ekf_global_node/enable'),
                    ('reset', 'ekf_global_node/reset'),
                    ('toggle', 'ekf_global_node/toggle'),
                    ('/diagnostics', 'diagnostics'),
                #     ('imu/data', f'/{namespace}/sensors/gps_0/heading_imu'),
                #     ('odometry/gps', f'/{namespace}/odometry/gps'),
                ],
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("mtu32_bringup"), "config", f'{platform_model}', "dual_duro_heading.yaml"]
                    )   
                ],
            ),
        ],
    )
    return [load_nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
    