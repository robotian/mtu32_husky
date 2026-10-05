"""The robot's own Intel RealSense D405 (sensors/camera_1), through clearpath_sensors' launch file.

Namespace and platform come from the robot's robot.yaml (setup_path, default /etc/clearpath/), so the same file
serves every robot: <namespace>/sensors/camera_1 with config/<platform>/intel_realsense_d405.yaml
(e.g. a300_00036 -> config/a300/, j100_0921 -> config/j100/).
"""
import os

from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path').perform(context)
    clearpath_config = ClearpathConfig(read_yaml(os.path.join(setup_path, 'robot.yaml')))
    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    realsense_param = os.path.join(
        get_package_share_directory('mtu32_bringup'), 'config', platform_model, 'intel_realsense_d405.yaml')
    if not os.path.isfile(realsense_param):
        raise RuntimeError(f'no RealSense config for platform {platform_model}: {realsense_param}')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('clearpath_sensors'), 'launch', 'intel_realsense.launch.py'])),
        launch_arguments=[
            ('parameters', realsense_param),
            ('namespace', f'{namespace}/sensors/camera_1'),
            ('robot_namespace', namespace),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/', description='Clearpath setup path'),
        OpaqueFunction(function=launch_setup),
    ])
