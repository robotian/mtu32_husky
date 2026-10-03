"""Nav2 on a static map, for every robot (generalises bringup_nav2_map_a300.launch.py).

Starts what bringup_nav2_map_a300.launch.py starts -- map_server (map_server_only.launch.py) and the navigation
servers (navigation.launch.py) in one nav2_container, no AMCL -- but takes everything robot-specific from the robot
itself instead of hardcoding the a300's:

* namespace and platform model from /etc/clearpath/robot.yaml;
* the Nav2 params file, the 2D scan source, the map and optional per-robot parameter overrides from
  config/nav2_robots.yaml (defaults -> platform -> robot namespace; see the comments there).

map->odom is not published here: as with the a300 launch, the robot's localization (GPS / mocap) must provide it.

    ros2 launch mtu32_bringup bringup_nav2_map.launch.py
    ros2 launch mtu32_bringup bringup_nav2_map.launch.py map:=mocap_space1.yaml scan_topic:=sensors/lidar2d_0/scan_filtered
"""
import copy
import os
import tempfile

import yaml
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
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

PKG = get_package_share_directory('mtu32_bringup')

ARGUMENTS = [
    DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/', description='Clearpath setup path'),
    DeclareLaunchArgument('profiles_file', default_value=os.path.join(PKG, 'config', 'nav2_robots.yaml'),
                          description='Per-robot Nav2 settings (params file, scan topic, map, overrides)'),
    DeclareLaunchArgument('params_file', default_value='',
                          description='Nav2 params (default: from profiles_file)'),
    DeclareLaunchArgument('scan_topic', default_value='',
                          description='2D scan topic, relative to the namespace or absolute (default: from profiles_file)'),
    DeclareLaunchArgument('map', default_value='',
                          description='Map yaml, in mtu32_bringup/map or absolute (default: from profiles_file)'),
    DeclareLaunchArgument('use_namespace', default_value='true',
                          description='Whether to apply a namespace to the navigation stack'),
    DeclareLaunchArgument('slam', default_value='False', description='Whether run a SLAM'),
    DeclareLaunchArgument('use_localization', default_value='True',
                          description='With slam:=True, whether to start SLAM'),
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
    DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
    DeclareLaunchArgument('use_composition', default_value='True', description='Whether to use composed bringup'),
    DeclareLaunchArgument('use_respawn', default_value='False',
                          description='Whether to respawn if a node crashes. Applied when composition is disabled.'),
    DeclareLaunchArgument('log_level', default_value='info', description='log level'),
]


def merge(base, override):
    """Deep-merge override into a copy of base: dicts key by key, anything else replaced."""
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def robot_profile(profiles_file, platform_model, namespace):
    with open(profiles_file) as f:
        profiles = yaml.safe_load(f) or {}
    profile = {}
    for layer in (profiles.get('defaults'),
                  (profiles.get('platforms') or {}).get(platform_model),
                  (profiles.get('robots') or {}).get(namespace)):
        profile = merge(profile, layer)
    return profile


def resolve_params_file(candidates, platform_model):
    if isinstance(candidates, str):
        candidates = [candidates]
    tried = []
    for name in candidates:
        path = name if os.path.isabs(name) else os.path.join(PKG, 'config', platform_model, name)
        if os.path.exists(path):
            return path
        tried.append(path)
    raise RuntimeError(f'bringup_nav2_map: no Nav2 params file for platform {platform_model!r}, tried {tried}')


def set_topics(node, topic):
    """Point every `topic:` key (costmap observation sources, collision monitor sources) at topic."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key == 'topic':
                node[key] = topic
            else:
                set_topics(value, topic)
    elif isinstance(node, list):
        for value in node:
            set_topics(value, topic)


def launch_setup(context, *args, **kwargs):
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    mtu_launch_dir = os.path.join(PKG, 'launch')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = read_yaml(os.path.join(arg('setup_path'), 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()
    sensors = config.get('sensors') or {}

    profile = robot_profile(arg('profiles_file'), platform_model, namespace)

    params_file = arg('params_file') or resolve_params_file(profile.get('params_file', 'nav2.yaml'), platform_model)

    scan_topic = arg('scan_topic') or profile.get('scan_topic', 'auto')
    if scan_topic == 'auto':
        scan_topic = 'sensors/lidar2d_0/scan_filtered' if sensors.get('lidar2d') else 'sensors/camera_0/scan'
    relative_topic = scan_topic.removeprefix(f'/{namespace}/')
    sensor_type = next((t for t in ('lidar2d', 'camera') if relative_topic.startswith(f'sensors/{t}_')), None)
    if not scan_topic.startswith('/'):
        scan_topic = f'/{namespace}/{scan_topic}'

    map_yaml_file = arg('map') or profile.get('map', '')
    if map_yaml_file and not os.path.isabs(map_yaml_file):
        map_yaml_file = os.path.join(PKG, 'map', map_yaml_file)

    with open(params_file) as f:
        params = yaml.safe_load(f)
    params = merge(params, profile.get('param_overrides'))
    set_topics(params, scan_topic)
    with tempfile.NamedTemporaryFile('w', prefix=f'{namespace}_', suffix='_nav2_map.yaml', delete=False) as f:
        yaml.safe_dump(params, f)
        params_file_out = f.name

    messages = [LogInfo(msg=f'bringup_nav2_map: {namespace} ({platform_model}) params {params_file}, '
                            f'scan {scan_topic}, map {map_yaml_file} -> {params_file_out}')]
    if sensor_type and not sensors.get(sensor_type):
        messages.append(LogInfo(msg=f'bringup_nav2_map: WARNING {scan_topic} needs a {sensor_type} sensor, '
                                    f'but robot.yaml has none; set scan_topic for {namespace} in nav2_robots.yaml'))

    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_localization = LaunchConfiguration('use_localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(source_file=params_file_out, root_key=namespace, param_rewrites={}, convert_types=True),
        allow_substs=True,
    )

    nav2_args = {
        'namespace': namespace,
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'params_file': params_file_out,
        'use_composition': use_composition,
        'use_respawn': use_respawn,
        'container_name': 'nav2_container',
    }

    bringup_cmd_group = GroupAction([
        PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
        SetRemap(f'/{namespace}/odom', f'/{namespace}/platform/odom'),
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'slam_launch.py')),
            condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_respawn': use_respawn,
                'params_file': params_file_out,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mtu_launch_dir, 'map_server_only.launch.py')),
            launch_arguments={**nav2_args, 'map': map_yaml_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mtu_launch_dir, 'navigation.launch.py')),
            launch_arguments=nav2_args.items(),
        ),
    ])

    return messages + [bringup_cmd_group]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function=launch_setup),
    ])
