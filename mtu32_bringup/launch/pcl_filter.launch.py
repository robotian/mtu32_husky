"""Crop box filter on the arm camera's point cloud (sensors/camera_1), in the arm's end-effector frame.

The namespace comes from the robot's robot.yaml (setup_path, default /etc/clearpath/), so the same file serves
every robot: <namespace>/crop_box_filter, <namespace>/sensors/camera_1/points -> .../cropped_points.
"""
import os

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path').perform(context)
    namespace = ClearpathConfig(read_yaml(os.path.join(setup_path, 'robot.yaml'))).system.namespace
    input_topic = LaunchConfiguration('input_topic').perform(context) or 'sensors/camera_1/points'
    output_topic = LaunchConfiguration('output_topic').perform(context) or 'sensors/camera_1/cropped_points'

    return [Node(
        package='pcl_ros',
        executable='filter_crop_box_node',
        name='crop_box_filter',
        namespace=namespace,
        output='screen',
        parameters=[{
            'min_x': -0.5,
            'max_x': -0.03,
            'min_y': -0.4,
            'max_y': 0.4,
            'min_z': 0.02,
            'max_z': 0.8,
            'negative': False,
            'input_frame': 'arm_0_end_effector_link',
            'output_frame': 'arm_0_end_effector_link'
        }],
        remappings=[
            ('input', input_topic),
            ('output', output_topic),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/', description='Clearpath setup path'),
        DeclareLaunchArgument(
            'input_topic', default_value='',
            description='Input sensor_msgs/msg/PointCloud2 topic (default <namespace>/sensors/camera_1/points)'),
        DeclareLaunchArgument(
            'output_topic', default_value='',
            description='Output sensor_msgs/msg/PointCloud2 topic (default <namespace>/sensors/camera_1/cropped_points)'),
        OpaqueFunction(function=launch_setup),
    ])
