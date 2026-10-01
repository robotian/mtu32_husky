import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from nav2_common.launch import RewrittenYaml
from launch.substitutions import PythonExpression

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
    DeclareLaunchArgument('use_mocap_fake_localizer', default_value='false',
                          choices=['true', 'false'],
                          description=''),
    DeclareLaunchArgument('moveit_delay', default_value='5.0',
                          description='Delay before starting MoveIt'),
    # DeclareLaunchArgument('use_composition_nav',
    #                        default_value='False',
    #                        description='Whether to use composed bringup',
    #                     )

]


def launch_setup(context, *args, **kwargs):
    pkg_mtu32_bringup = get_package_share_directory('mtu32_bringup')
    setup_path = LaunchConfiguration('setup_path')
    use_mocap_fake_localizer = LaunchConfiguration('use_mocap_fake_localizer')
    moveit_delay_val = float(LaunchConfiguration('moveit_delay').perform(context))

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    platform_model = clearpath_config.platform.get_platform_model()
    namespace = clearpath_config.system.namespace

    remappings_tf = [
        ('/tf', f'/{namespace}/tf'),
        ('/tf_static', f'/{namespace}/tf_static'),
    ]


    load_nodes = GroupAction(
        actions=[
            # No ekf_node here: in multirobot_sim the robot container runs the platform EKF as a boot service
            # (robot/bin/ekf, localization.yaml, publishes odom->base_link and platform/odom/filtered), as the
            # Clearpath platform service does on the real robot.

            # This sim's arm has no real ros2_control hardware interface (Isaac's own OmniGraph drives it
            # directly) -- without this, move_group/servo_node's moveit_simple_controller_manager has no
            # FollowJointTrajectory/GripperCommand action server to execute a planned trajectory against, and
            # every execute() call fails immediately with CONTROL_FAILED. Bridges both actions to the sim's own
            # arm_0/joint_command topic instead. See moveit_sim_bridge's own module docstring for what this
            # does and doesn't guarantee.
            Node(
                package="moveit_sim_bridge",
                executable="moveit_sim_bridge",
                name="moveit_sim_bridge",
                output='screen',
                namespace=namespace,
            ),
        ],
    )

    launch_file_bringup_main = PathJoinSubstitution([
        pkg_mtu32_bringup, 'launch', 'bringup_main.launch.py'
    ])
    bringup_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_bringup_main),
        launch_arguments=[
            ('setup_path', setup_path),
            ('scan_topic', LaunchConfiguration('scan_topic')),
            ('autostart', LaunchConfiguration('autostart')),
            ('use_lifecycle_manager', LaunchConfiguration('use_lifecycle_manager')),
            ('use_mocap_fake_localizer', use_mocap_fake_localizer),
            ('moveit_delay', LaunchConfiguration('moveit_delay')),
        ],
    )

    return [load_nodes, bringup_main]

    
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld