import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
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
    DeclareLaunchArgument('use_gps_localization', default_value='true',
                          choices=['true', 'false'],
                          description='Dual-GPS global localization (sim_swift_nav_dual.launch.py, publishes '
                                      'map->odom); skipped if robot.yaml has fewer than two GPS sensors'),
    DeclareLaunchArgument('use_nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Nav2 on a static map (bringup_nav2_map.launch.py, settings per robot from '
                                      'config/nav2_robots.yaml)'),
    DeclareLaunchArgument('nav2_map', default_value='',
                          description='Map yaml for Nav2, in mtu32_bringup/map or absolute (default: from '
                                      'nav2_robots.yaml)'),
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

    # Dual-GPS heading + navsat_transform + ekf_global_node: the map->odom TF that the map-based Nav2 launches
    # (e.g. bringup_nav2_map_a300.launch.py) wait for. It raises without two GPS sensors, which the generic
    # models (robot.<model>.yaml.tmpl) don't have, so it is skipped for them instead of failing the whole upstart.
    actions = [load_nodes, bringup_main]
    gps = clearpath_config.sensors.get_all_gps()
    if LaunchConfiguration('use_gps_localization').perform(context) == 'true':
        if len(gps) >= 2:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    pkg_mtu32_bringup, 'launch', 'sim_swift_nav_dual.launch.py'
                ])),
                launch_arguments=[('setup_path', setup_path)],
            ))
        else:
            actions.append(LogInfo(msg=f'{namespace}: {len(gps)} GPS sensor(s) in robot.yaml, '
                                       'skipping sim_swift_nav_dual (dual-GPS localization needs two)'))

    # Nav2 on a map (map_server + navigation servers, no AMCL): map->odom comes from the GPS localization above
    # or the mocap fake localizer; without either, Nav2's costmaps wait for it.
    if LaunchConfiguration('use_nav2').perform(context) == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                pkg_mtu32_bringup, 'launch', 'bringup_nav2_map.launch.py'
            ])),
            launch_arguments=[
                ('setup_path', setup_path),
                ('use_sim_time', LaunchConfiguration('use_sim_time')),
                ('map', LaunchConfiguration('nav2_map')),
            ],
        ))

    return actions

    
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld