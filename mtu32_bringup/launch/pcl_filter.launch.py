import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_mtu32_bringup = get_package_share_directory('mtu32_bringup')
    config_file_path = os.path.join(pkg_mtu32_bringup, 'config', 'j100', 'realsense_point_cloud_filter.yaml')

    # Declare launch arguments for flexibility
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_path,
        description='Path to the filter YAML configuration file'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/j100_0921/sensors/camera_1/points',
        description='Input sensor_msgs/msg/PointCloud2 topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/j100_0921/sensors/camera_1/cropped_points',
        description='Output sensor_msgs/msg/PointCloud2 topic'
    )

    crop_box_node = Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name='crop_box_filter',
            namespace='j100_0921',
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
                ('input', '/j100_0921/sensors/camera_1/points'),
                ('output', '/j100_0921/sensors/camera_1/cropped_points'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        crop_box_node
    ])