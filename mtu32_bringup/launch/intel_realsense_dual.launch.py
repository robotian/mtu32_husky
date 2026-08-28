import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

local_parameters = [
    # Camera 0
    {"name": "camera_name1", "default": "camera_0", "description": "camera unique name"},
    {"name": "camera_namespace1", "default": "a300_00036/sensors/camera_0", "description": "camera1 namespace"},
    {"name": "serial_no1", "default": "838212070468", "description": "choose device by serial number"},
    {"name": "device_type1", "default": "d435", "description": "choose device by type"},
    {"name": "enable_color1", "default": "true", "description": "enable color stream"},
    {"name": "rgb_camera.color_profile1", "default": "848.480,30", "description": "color stream profile"},
    {"name": "enable_depth1", "default": "true", "description": "enable depth stream"},
    {"name": "depth_module.depth_profile1", "default": "848,480,15", "description": "depth stream profile"},
    {"name": "pointcloud.enable1", "default": "true", "description": "enable pointcloud"},
    # Camera 1
    {"name": "camera_name2", "default": "camera_1", "description": "camera unique name"},
    {"name": "camera_namespace2", "default": "a300_00036/sensors/camera_1", "description": "camera2 namespace"},
    {"name": "serial_no2", "default": "130322271563", "description": "choose device by serial number"},
    {"name": "device_type2", "default": "d405", "description": "choose device by type"},
    {"name": "enable_color2", "default": "true", "description": "enable color stream"},
    {"name": "rgb_camera.color_profile2", "default": "848.480,30", "description": "color stream profile"},
    {"name": "enable_depth2", "default": "true", "description": "enable depth stream"},
    {"name": "depth_module.depth_profile2", "default": "848,480,15", "description": "depth stream profile"},
    {"name": "pointcloud.enable2", "default": "true", "description": "enable pointcloud"},
]


def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare("clearpath_sensors")

    realsense_param = os.path.join(
        get_package_share_directory("mtu32_bringup"), "config", "a300", "intel_realsense_dual.yaml"
    )

    # Declare launch files
    launch_file_intel_realsense = PathJoinSubstitution([pkg_clearpath_sensors, "launch", "intel_realsense.launch.py"])

    # Include launch files
    launch_intel_realsense_camera_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_intel_realsense]),
        launch_arguments=[
            ("parameters", realsense_param),
            ("namespace", "a300_00036/sensors/camera_0"),
            ("robot_namespace", "a300_00036"),
        ],
    )

    launch_intel_realsense_camera_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_intel_realsense]),
        launch_arguments=[
            ("parameters", realsense_param),
            ("namespace", "a300_00036/sensors/camera_1"),
            ("robot_namespace", "a300_00036"),
        ],
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_intel_realsense_camera_0)
    ld.add_action(launch_intel_realsense_camera_1)
    return ld
