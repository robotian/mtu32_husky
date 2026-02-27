# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
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

from launch_ros.actions import PushRosNamespace, SetRemap

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                          default_value='',
                          description='Override the default 2D laserscan topic'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),  # noqa: E501
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),
]


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_mtu_bringup = get_package_share_directory('mtu32_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    scan_topic = LaunchConfiguration('scan_topic')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    map_file = os.path.join(pkg_mtu_bringup, 'map', 'mocap_space1.yaml')    

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    # see if we've overridden the scan_topic
    eval_scan_topic = scan_topic.perform(context)
    if len(eval_scan_topic) == 0:
        eval_scan_topic = f'/{namespace}/sensors/lidar2d_0/scan'

    file_parameters = PathJoinSubstitution([
        pkg_mtu_bringup,
        'config',
        platform_model,
        'nav2.yaml'])

    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        param_rewrites={
            # the only *.topic parameters are scan.topic, so rewrite all of them to point to
            # our desired scan_topic
            'topic': eval_scan_topic,
        },
        convert_types=True
    )

    launch_nav2 = PathJoinSubstitution(
      [pkg_mtu_bringup, 'launch', 'navigation_launch.py'])

    launch_map_server = PathJoinSubstitution(
        [pkg_mtu_bringup, 'launch', 'map_server.launch.py'])


    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/' + namespace + '/odom',
                 '/' + namespace + '/platform/odom'),
        SetRemap('/tf', '/' + namespace + '/tf'),
        SetRemap('/tf_static', '/' + namespace + '/tf_static'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(launch_map_server),
        #     launch_arguments=[
        #         ('use_sim_time', use_sim_time),
        #         # ('params_file', rewritten_parameters),                
        #         ('namespace', namespace),
        #         ('autostart', autostart),
        #         ('use_lifecycle_manager', use_lifecycle_manager),
        #       ]
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', rewritten_parameters),
                ('use_composition', 'False'),
                ('namespace', namespace),
                ('map',map_file),
              ]
        ),
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld