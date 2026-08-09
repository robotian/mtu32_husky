# '''
# Copyright (C) 2015-2023 Swift Navigation Inc.
# Contact: https://support.swiftnav.com

# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.

# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
# '''

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
# from launch_ros.substitutions import FindPackageShare

# from clearpath_config.clearpath_config import ClearpathConfig
# from clearpath_config.common.utils.yaml import read_yaml


# def generate_launch_description():
#     launch_desc = LaunchDescription()

#     setup_path = LaunchConfiguration('setup_path')

#     # Read robot YAML
#     config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
#     # Parse robot YAML into config
#     clearpath_config = ClearpathConfig(config)

#     namespace = clearpath_config.system.namespace
#     platform_model = clearpath_config.platform.get_platform_model()


#     robot_namespace = LaunchConfiguration('robot_namespace')


#     attitude_duro_namespace = LaunchConfiguration('attitude_duro_namespace')
#     reference_duro_namespace = LaunchConfiguration('reference_duro_namespace')

#     attitude_duro_parameter =  LaunchConfiguration('attitude_duro_parameter')
#     reference_duro_parameter =  LaunchConfiguration('reference_duro_parameter')

#     arg_setup_path = DeclareLaunchArgument(
#         'setup_path',
#         default_value='/etc/clearpath/',
#         description='Clearpath setup path'
#     )

#     arg_attitude_duro_namespace = DeclareLaunchArgument(
#         'attitude_duro_namespace',
#         default_value=[namespace,'/sensors/gps_1'])

#     arg_attitude_duro_parameter = DeclareLaunchArgument(
#         'attitude_duro_parameter',
#         default_value=PathJoinSubstitution([
#           FindPackageShare('mtu32_bringup'),
#           'config',
#           platform_model,
#           'duro_attitude.yaml'
#         ]))

#     arg_reference_duro_namespace = DeclareLaunchArgument(
#         'reference_duro_namespace',
#         default_value=[namespace,'/sensors/gps_2'])

#     arg_reference_duro_parameter = DeclareLaunchArgument(
#         'reference_duro_parameter',
#         default_value=PathJoinSubstitution([
#           FindPackageShare('mtu32_bringup'),
#           'config',
#           platform_model,
#           'duro_reference.yaml'
#         ]))

#     att_duro_node = Node(
#         package='swiftnav_ros2_driver',
#         executable='sbp-to-ros',
#         name='att_duro_node',
#         namespace=attitude_duro_namespace,
#         remappings=[
#             # ('/tf','/j100_0921/tf'),
#             # ('/tf_static','/j100_0921/tf_static'),
#             ('navsatfix','fix'),
#         ],
#         output='screen',
#         parameters=[attitude_duro_parameter]
#     )

#     ref_duro_node = Node(
#         package='swiftnav_ros2_driver',
#         executable='sbp-to-ros',
#         name='ref_duro_node',
#         namespace=reference_duro_namespace,
#         remappings=[
#             # ('/tf','/j100_0921/tf'),
#             # ('/tf_static','/j100_0921/tf_static'),
#             ('navsatfix','fix'),
#         ],
#         output='screen',
#         parameters=[reference_duro_parameter]
#     )

#     # config = os.path.join(
#     #     get_package_share_directory('mtu32_bringup'),
#     #     'config',
#     #     'swift_nav.yaml'
#     #     )
#     # node = Node(
#     #     package='swiftnav_ros2_driver',
#     #     executable='sbp-to-ros',
#     #     namespace='/a300_00036/sensors/gps_0/',
#     #     parameters=[config]
#     # )

#     # launch_desc.add_action(node)
#     launch_desc.add_action(arg_setup_path)
#     launch_desc.add_action(arg_reference_duro_namespace) 
#     launch_desc.add_action(arg_reference_duro_parameter)
#     launch_desc.add_action(arg_attitude_duro_namespace)
#     launch_desc.add_action(arg_attitude_duro_parameter)
#     launch_desc.add_action(att_duro_node)
#     launch_desc.add_action(ref_duro_node)
#     return launch_desc


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
                namespace=f'/{namespace}/sensors/gps_2',
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
                package='dual_duro_heading',
                executable='heading_filter',
                name='dual_duro_heading_node',
                namespace=f'/{namespace}/sensors/gps_1',
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
                    ('gps/fix', f'/{namespace}/sensors/gps_2/fix'),          # SwiftNav global position topic
                    ('odometry/filtered', f'/{namespace}/odometry/global'),  # from EKF
                    ('imu', f'/{namespace}/sensors/gps_1/heading_imu'),     # Your computed RTK heading topic
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
                #     ('imu/data', f'/{namespace}/sensors/gps_1/heading_imu'),
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
    