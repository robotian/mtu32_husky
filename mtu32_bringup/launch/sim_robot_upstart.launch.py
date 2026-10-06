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
from launch_ros.actions import PushRosNamespace, SetParameter, SetRemap, Node
from nav2_common.launch import RewrittenYaml
from launch.substitutions import PythonExpression

ARGUMENTS = [
    # The multirobot_sim robot containers set USE_SIM_TIME=true when the simulator publishes /clock; unset elsewhere.
    DeclareLaunchArgument('use_sim_time', default_value=os.environ.get('USE_SIM_TIME', 'false'),
                          choices=['true', 'false'],
                          description='Use the simulator\'s /clock for every node (default: $USE_SIM_TIME or false)'),
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
    DeclareLaunchArgument('ref_source', default_value='',
                          choices=['', 'auto', 'ref', 'gps', 'external'],
                          description="ref_localizer's map -> odom source (empty = config/ref_localization.yaml's: "
                                      "auto = the sim's ref_pose, else GPS)"),
    DeclareLaunchArgument('ref_anchor', default_value='',
                          choices=['', 'fixed', 'start', 'external'],
                          description="where ref_localizer puts map in ref_frame (empty = the config's: fixed)"),
    DeclareLaunchArgument('moveit_delay', default_value='5.0',
                          description='Delay before starting MoveIt'),
    DeclareLaunchArgument('use_gps_localization', default_value='true',
                          choices=['true', 'false'],
                          description='Dual-GPS global localization (sim_swift_nav_dual.launch.py, publishes '
                                      'map->odom); skipped if robot.yaml has fewer than two GPS sensors'),
    DeclareLaunchArgument('use_nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Nav2 on a static map (bringup_main starts bringup_nav2_map.launch.py, settings '
                                      'per robot from config/nav2_robots.yaml)'),
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
            # the sim publishes ref_pose (SIM_REF_POSE) itself: no Motive client
            ('use_natnet', 'false'),
            ('ref_source', LaunchConfiguration('ref_source')),
            ('ref_anchor', LaunchConfiguration('ref_anchor')),
            ('moveit_delay', LaunchConfiguration('moveit_delay')),
            # no clearpath-manipulators service in the sim: bringup_main's MoveIt is the only one
            ('moveit', 'true'),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            # Nav2 on a map (bringup_nav2_map.launch.py) is started by bringup_main, as on a real robot
            ('use_nav2', LaunchConfiguration('use_nav2')),
            ('nav2_map', LaunchConfiguration('nav2_map')),
        ],
    )

    # Dual-GPS heading + navsat_transform + ekf_global_node: odometry/global, which bringup_main's ref_localizer
    # turns into map->odom when GPS is its source (the EKF's own TF is off). It raises without two GPS sensors, which the generic
    # models (robot.<model>.yaml.tmpl) don't have, so it is skipped for them instead of failing the whole upstart.
    # use_sim_time for every node started below, including those of the included launch files that don't pass it
    # themselves (a node's own parameters/params file still override it; Nav2's are rewritten in bringup_nav2_map,
    # which bringup_main starts).
    actions = [SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')), load_nodes, bringup_main]
    gps = clearpath_config.sensors.get_all_gps()
    if LaunchConfiguration('use_gps_localization').perform(context) == 'true':
        if len(gps) >= 2:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    pkg_mtu32_bringup, 'launch', 'sim_swift_nav_dual.launch.py'
                ])),
                launch_arguments=[('setup_path', setup_path), ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            ))
        else:
            actions.append(LogInfo(msg=f'{namespace}: {len(gps)} GPS sensor(s) in robot.yaml, '
                                       'skipping sim_swift_nav_dual (dual-GPS localization needs two)'))

    return actions

    
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld