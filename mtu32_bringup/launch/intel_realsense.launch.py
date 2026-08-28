import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare("clearpath_sensors")
    # mtu32_bringup_pkg_prefix = FindPackageShare('mtu32_bringup')
    
    realsense_param = os.path.join(
        get_package_share_directory("mtu32_bringup"), "config", "a300", "intel_realsense_d405.yaml"
    )

    # Declare launch files
    launch_file_intel_realsense = PathJoinSubstitution([pkg_clearpath_sensors, "launch", "intel_realsense.launch.py"])

    # Include launch files
    launch_intel_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_intel_realsense]),
        launch_arguments=[
            ("parameters", realsense_param),
            ("namespace", "a300_00036/sensors/camera_1"),
            ("robot_namespace", "a300_00036"),
        ],
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_intel_realsense)
    return ld
