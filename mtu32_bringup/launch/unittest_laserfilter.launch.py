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
    remappings_tf=[
                ('/tf','/a300_00036/tf'),
                ('/tf_static','/a300_00036/tf_static'),
            ]

    # laser_filter_node = Node(
    #     package="laser_filters",
    #     executable="scan_to_scan_filter_chain",
    #     namespace='/a300_00036/sensors/lidar2d_0',
    #     parameters=[
    #         PathJoinSubstitution(
    #             [get_package_share_directory("mtu32_bringup"), "config", "ust_box_filter.yaml"]
    #         )
    #     ],
    #     remappings= remappings_tf,        
    # )

    laser_filter_node = Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                output='screen',
                namespace='/a300_00036/sensors/lidar2d_0',
                parameters=[
                    PathJoinSubstitution(
                        # [get_package_share_directory("mtu32_bringup"), "config","ust_box_filter.yaml"]
                        [get_package_share_directory("mtu32_bringup"), "config", "a300", "hokuyo_lidar_filter.yaml"]
                    )
                ],
                remappings= remappings_tf,  
            )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(laser_filter_node)
    return ld
