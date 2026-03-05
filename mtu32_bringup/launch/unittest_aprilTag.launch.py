import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Declare Arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Device ID for the camera'
    )

    # 2. Define the Config File Path
    config_path = os.path.join(
        get_package_share_directory('mtu32_bringup'),
        'config',
        'tags_36h11.yaml'
    )

    # 3. Create the Composable Node Container
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='/j100_0921',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='/j100_0921',
                remappings=[
                    ('image_rect', '/j100_0921/sensors/camera_0/color/image'),
                    ('/camera/camera_info', '/j100_0921/sensors/camera_0/color/image/camera_info'),
                    ('detections', '/j100_0921/sensors/camera_0/color/aprilTag_detections'),
                    ('/tf', '/j100_0921/tf'),
                ],
                parameters=[config_path],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        device_arg,
        apriltag_container
    ])