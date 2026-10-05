import os

from ament_index_python.packages import get_package_share_directory
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
    TimerAction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from nav2_common.launch import RewrittenYaml

from clearpath_config.clearpath_config import ClearpathConfig

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
    # Global localization (config/ref_localization.yaml): ref_localizer is the only map -> odom publisher, from
    # the motion capture / simulator reference pose or from GPS (the GPS EKF's own TF is off).
    DeclareLaunchArgument('use_ref_localizer', default_value='true',
                          choices=['true', 'false'],
                          description='ref_localizer: map -> odom from the reference pose (mocap / sim) or GPS'),
    DeclareLaunchArgument('use_natnet', default_value='true',
                          choices=['true', 'false'],
                          description='natnet_ref_pose: the OptiTrack Motive client (false in the simulator, '
                                      'which publishes ref_pose itself)'),
    DeclareLaunchArgument('ref_source', default_value='',
                          choices=['', 'auto', 'ref', 'gps', 'external'],
                          description="ref_localizer's source (empty = config/ref_localization.yaml's)"),
    DeclareLaunchArgument('ref_anchor', default_value='',
                          choices=['', 'fixed', 'start', 'external'],
                          description="ref_localizer's anchor (empty = config/ref_localization.yaml's)"),
    # MoveIt: on a real robot whose robot.yaml has manipulators.moveit.enable: true, Clearpath's
    # clearpath-manipulators service already runs move_group, so a second one here must not start. The simulator
    # has no clearpath-manipulators service: sim_robot_upstart.launch.py passes moveit:=true.
    DeclareLaunchArgument('moveit', default_value='auto',
                          choices=['auto', 'true', 'false'],
                          description="start MoveIt here (moveit.launch.py: move_group + servo_node); auto = "
                                      "unless robot.yaml's manipulators.moveit.enable (Clearpath's own move_group)"),
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
    moveit_delay_val = float(LaunchConfiguration('moveit_delay').perform(context))

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    platform_model = clearpath_config.platform.get_platform_model()
    namespace = clearpath_config.system.namespace
    clearpath_moveit = bool(((config.get('manipulators') or {}).get('moveit') or {}).get('enable', False))
    moveit_arg = LaunchConfiguration('moveit').perform(context)
    start_moveit = moveit_arg == 'true' or (moveit_arg == 'auto' and not clearpath_moveit)

    remappings_tf = [
        ('/tf', f'/{namespace}/tf'),
        ('/tf_static', f'/{namespace}/tf_static'),
    ]

    # shared localization params, the web UI's rigid body assignments (keyed by /<ns>/natnet_ref_pose), then this
    # robot's own hand-written file (rigid body name, base_link offset), if any
    ref_params = [os.path.join(pkg_mtu32_bringup, 'config', 'ref_localization.yaml')]
    for name in ('assignments.yaml', f'{namespace}.yaml'):
        path = os.path.join(pkg_mtu32_bringup, 'config', 'ref_localization', name)
        if os.path.isfile(path):
            ref_params.append(path)
    ref_overrides = {k: v for k, v in (('source', LaunchConfiguration('ref_source').perform(context)),
                                       ('anchor', LaunchConfiguration('ref_anchor').perform(context))) if v}

    depth2scan_param_config = os.path.join(
        pkg_mtu32_bringup, 'config', f'{platform_model}', 'depth2scan.yaml'
    )

    aprilTag_config = os.path.join(
        pkg_mtu32_bringup, 'config', 'tags_36h11.yaml'
    )

    tf2pose_config = os.path.join(
        pkg_mtu32_bringup, 'config', f'{platform_model}', 'dock_pose_params.yaml'
    )

    launch_file_moveit = PathJoinSubstitution([
        pkg_mtu32_bringup, 'launch', 'moveit.launch.py'
    ])

    launch_file_pcl_filter = PathJoinSubstitution([
        pkg_mtu32_bringup, 'launch', 'pcl_filter.launch.py'
    ])


    launch_file_grid_cutter_filter = PathJoinSubstitution([
        get_package_share_directory('stow_arm_cpp'), 'launch', 'grid_cutter.launch.py'
    ])
    
    # Fixed trailing comma tuple bug
    use_sim_time = LaunchConfiguration('use_sim_time')
    moveit_node_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_moveit),
        launch_arguments=[('setup_path', setup_path), ('use_sim_time', use_sim_time)],
    )

    launch_file_cut_stem_gamepad_file = PathJoinSubstitution([
        get_package_share_directory('kinova_game_pad'), 'launch', 'cut_stem_gamepad.launch.py'
    ])

    load_nodes = GroupAction(        
        actions=[
            # every node below, also those whose launch file doesn't pass use_sim_time itself
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                output='screen',
                namespace=f'/{namespace}/sensors/lidar2d_0',
                parameters=[
                    PathJoinSubstitution(
                        [pkg_mtu32_bringup, "config", f'{platform_model}', "lidar_filter.yaml"]
                    )
                ],
                remappings=remappings_tf,  
            ),

            Node(
                package='mocap_fake_localizer',
                executable='ref_localizer.py',
                name='ref_localizer',
                output='screen',
                namespace=f'/{namespace}',
                parameters=ref_params + ([ref_overrides] if ref_overrides else []),
                remappings=remappings_tf,
                condition=IfCondition(LaunchConfiguration('use_ref_localizer')),
            ),

            Node(
                package='mocap_fake_localizer',
                executable='natnet_ref_pose.py',
                name='natnet_ref_pose',
                output='screen',
                namespace=f'/{namespace}',
                parameters=ref_params,
                remappings=remappings_tf,
                condition=IfCondition(LaunchConfiguration('use_natnet')),
            ),

            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depthimage_to_laserscan',
                namespace=f'/{namespace}',
                remappings=remappings_tf + [
                    ('depth', f'/{namespace}/sensors/camera_0/depth/image'),
                    ('depth_camera_info', f'/{namespace}/sensors/camera_0/depth/camera_info'),
                    ('scan', f'/{namespace}/sensors/camera_0/scan')
                ],
                parameters=[depth2scan_param_config]
            ),  

            Node(
                package='apriltag_ros',
                name='apriltag',
                executable='apriltag_node',
                namespace=f'/{namespace}',
                remappings=remappings_tf + [
                    ('image_rect', f'/{namespace}/sensors/camera_0/color/image'),
                    ('camera_info', f'/{namespace}/sensors/camera_0/color/image/camera_info'),
                    ('detections', f'/{namespace}/sensors/camera_0/color/aprilTag_detections'),
                ],
                parameters=[aprilTag_config],
            ),

            Node(
                package='docking_utils',
                name='tf2_pose_node',
                executable='tf2_pose_node',
                namespace=f'/{namespace}',
                parameters=[tf2pose_config],
                remappings=remappings_tf,
            ),  

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_pcl_filter),
                launch_arguments=[('setup_path', setup_path), ('use_sim_time', use_sim_time)],
            ),

            # TODO: the cutter stack below (grid cutter, pruner_server on /dev/ttyOpenCR, cut_stem gamepad, which
            # drives the arm through servo_node) is j100_0921's hardware -- only j100_0921 has the cutter -- but runs
            # on every robot, e.g. a300_00036 (2026-10-05: pruner_server can't open /dev/ttyOpenCR, gamepad under
            # /j100_0921/...). Gate it per robot (e.g. a `cutter` launch argument, default on for j100_0921 only),
            # keeping the sim's j100_0921 as is. MoveIt itself is gated by `moveit` (above).

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_grid_cutter_filter),
                launch_arguments=[('use_sim_time', use_sim_time)],
            ),

            LogInfo(msg=f'MoveIt: {"starting move_group + servo_node here" if start_moveit else "not started here"} '
                        f'(moveit:={moveit_arg}, robot.yaml manipulators.moveit.enable: {clearpath_moveit})'),
            TimerAction(
                period=moveit_delay_val,
                actions=[moveit_node_action],
                condition=IfCondition(str(start_moveit).lower()),
            ),

            Node(
                package='pruner_action_server',
                name='pruner_server',
                executable='pruner_server',
                namespace=f'/{namespace}'
            ),  

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_cut_stem_gamepad_file),
                launch_arguments=[('use_sim_time', use_sim_time)],
            ),                     
        ],
    )
    return [load_nodes]

    
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld