from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')

    param_config = os.path.join(
        get_package_share_directory('mtu32_bringup'), 'config','j100', 'camera_1.yaml')

    # Declare launch files
    launch_file_intel_realsense = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'intel_realsense.launch.py'])

    # Include launch files
    launch_intel_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_intel_realsense]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    param_config
                )
                ,
                (
                    'namespace'
                    ,
                    'j100_0921/sensors/camera_1'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    'j100_0921'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_intel_realsense)
    return ld