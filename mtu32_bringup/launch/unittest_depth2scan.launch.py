# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Gary Liu */

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_config = os.path.join(
        get_package_share_directory('mtu32_bringup'), 'config','j100', 'depth2scan.yaml')

    remappings_tf=[
                ('/tf','/j100_0921/tf'),
                ('/tf_static',f'/j100_0921/tf_static'),
            ]
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            namespace='/j100_0921',
            remappings=remappings_tf +[('depth', '/j100_0921/sensors/camera_0/depth/image'),
                        ('depth_camera_info', '/j100_0921/sensors/camera_0/depth/camera_info'),
                        ('scan', '/j100_0921/sensors/camera_0/scan')],
            parameters=[param_config])
    ])
