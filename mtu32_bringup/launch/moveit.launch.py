#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
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

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch.event_handlers import OnProcessStart

def launch_setup(context, *args, **kwargs):
    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path_context = setup_path.perform(context)

    # Namespace
    namespace = ClearpathConfig(
        os.path.join(setup_path_context, 'robot.yaml')
    ).get_namespace()

    # Robot Description
    robot_description = {
        'robot_description': xacro.process_file(
            os.path.join(setup_path_context, 'robot.urdf.xacro')
        ).toxml()
    }

    # Semantic Robot Description
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            os.path.join(setup_path_context, 'robot.srdf')
        ).toxml()
    }

    # MoveIt Configuration
    # The generated moveit.yaml is scoped to <namespace>/move_group, so the servo
    # node does not receive its parameters. Extract the kinematics (IK solver) and
    # planning (joint limits) sections to pass to the servo node explicitly.
    moveit_yaml = os.path.join(setup_path_context, 'manipulators', 'config', 'moveit.yaml')
    moveit_params = read_yaml(moveit_yaml)
    moveit_params = moveit_params.get(namespace, moveit_params)
    moveit_params = moveit_params.get('move_group', moveit_params)
    moveit_params = moveit_params.get('ros__parameters', moveit_params)
    servo_moveit_params = {
        key: moveit_params[key]
        for key in ('robot_description_kinematics', 'robot_description_planning')
        if key in moveit_params
    }

    # Load Servo Configuration
    servo_yaml = os.path.join(
        get_package_share_directory('mtu32_bringup'),
        "config", "j100", "servo_config.yaml"
    )

    # MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        namespace=namespace,
        parameters=[
            servo_yaml,
            robot_description,
            robot_description_semantic,
            servo_moveit_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joint_states', 'platform/joint_states'),
        ],
        output="screen",
    )

    # Optional: Command Mode Activation Event Handler
    activate_twist_mode = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            f"/{namespace}/servo_node/switch_command_type",
            "moveit_msgs/srv/ServoCommandType",
            "{command_type: 1}"
        ],
        output="screen"
    )

    servo_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=servo_node,
            on_start=[activate_twist_mode],
        )
    )

    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='log',
            namespace=namespace,
            parameters=[
                moveit_yaml,
                robot_description,
                robot_description_semantic,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ]
        ),
        servo_node,
        servo_start_handler
    ]


def generate_launch_description():
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath setup path'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    )
    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
