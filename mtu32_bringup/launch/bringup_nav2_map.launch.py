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
import json
import math
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
    DeclareLaunchArgument('charger_docks', default_value='',
                          description="JSON with the charging stations ({\"chargers\": [{id, x, y, yaw_deg, tag}]}, the "
                                      "simulator's state.json): the docking server's docks become one per station "
                                      "(default: the params file's)"),
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


def dock_tag_id(platform_model):
    """The AprilTag id this platform docks on: its tf2_pose_node's target_frame (config/<platform>/
    dock_pose_params.yaml, which feeds detected_dock_pose) looked up in config/tags_36h11.yaml. None if unknown."""
    try:
        with open(os.path.join(PKG, 'config', platform_model, 'dock_pose_params.yaml')) as f:
            nodes = next(iter((yaml.safe_load(f) or {}).values()), {})
        frame = nodes['tf2_pose_node']['ros__parameters']['target_frame']
        with open(os.path.join(PKG, 'config', 'tags_36h11.yaml')) as f:
            tags = next(iter((yaml.safe_load(f) or {}).values()), {})['ros__parameters']['tag']
        return dict(zip(tags['frames'], tags['ids']))[frame]
    except (OSError, KeyError, TypeError, AttributeError, StopIteration, yaml.YAMLError):
        return None


def charger_docks(params, chargers_file, platform_model):
    """Replace the docking server's docks with one per charging station in chargers_file (the simulator's
    state.json: x, y, yaw_deg = the side its AprilTag faces, tag = the tag's centre, map frame). A dock's pose is
    where base_link ends up docked: the tag's position moved by the dock plugin's own external_detection_translation
    (as the docking server places it from a detection), heading into the station. The docks are checked against
    the battery (use_battery_status): the simulator's battery charges at a station. A dock is named by the station's
    dock_id (dock_station.dock_id, what a charging task's DockGoal.dock_id says) when the database has one, else
    charger_<n>. Only the stations whose tag
    this platform detects (dock_tag_id), if known. Returns a log line, or ''."""
    try:
        with open(chargers_file) as f:
            chargers = json.load(f).get('chargers') or []
    except (OSError, ValueError) as e:
        return f'bringup_nav2_map: no charger docks ({chargers_file}: {e})'
    server = (params.get('docking_server') or {}).get('ros__parameters')
    if not chargers or not server or not server.get('dock_plugins'):
        return ''
    tag_id = dock_tag_id(platform_model)
    if tag_id is not None:
        chargers = [c for c in chargers if c.get('apriltag_id') == tag_id]
        if not chargers:
            return f'bringup_nav2_map: no charger in {chargers_file} has tag {tag_id}: keeping the params file\'s docks'
    dock_type = server['dock_plugins'][0]
    plugin = server.setdefault(dock_type, {})
    tx = plugin.get('external_detection_translation_x', 0.0)
    ty = plugin.get('external_detection_translation_y', 0.0)
    server.pop('dock_database', None)  # it would win over the docks parameter
    server['docks'] = []
    for c in chargers:
        tag_x, tag_y = (c.get('tag') or [c['x'], c['y']])[:2]
        yaw = math.radians(c.get('yaw_deg', 0.0)) + math.pi  # facing the tag
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        x = tag_x + math.cos(yaw) * tx - math.sin(yaw) * ty
        y = tag_y + math.sin(yaw) * tx + math.cos(yaw) * ty
        name = c.get('dock_id') or c['id']  # dock_station.dock_id when the farm database names it, else charger_<n>
        server['docks'].append(name)
        server[name] = {'type': dock_type, 'frame': 'map', 'pose': [round(x, 3), round(y, 3), round(yaw, 4)]}
    plugin['use_battery_status'] = True
    return (f'bringup_nav2_map: docks from {chargers_file}: ' +
            ', '.join(f"{d} {server[d]['pose']}" for d in server['docks']) +
            f' ({dock_type}, use_battery_status{f", tag {tag_id}" if tag_id is not None else ""})')


def map_topics(node, fn):
    """Replace every `topic:` value (costmap observation sources, collision monitor sources) with fn(value)."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key == 'topic' and isinstance(value, str):
                node[key] = fn(value)
            else:
                map_topics(value, fn)
    elif isinstance(node, list):
        for value in node:
            map_topics(value, fn)


def in_namespace(topic, namespace):
    """The topic in this robot's namespace: relative -> absolute, another robot's /<ns>/sensors/... -> ours."""
    if not topic.startswith('/'):
        return f'/{namespace}/{topic}'
    parts = topic.split('/')
    if len(parts) > 3 and parts[2] == 'sensors':
        return '/'.join(['', namespace] + parts[2:])
    return topic


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
    if scan_topic != 'params':
        scan_topic = in_namespace(scan_topic, namespace)

    map_yaml_file = arg('map') or profile.get('map', '')
    if map_yaml_file and not os.path.isabs(map_yaml_file):
        map_yaml_file = os.path.join(PKG, 'map', map_yaml_file)

    with open(params_file) as f:
        params = yaml.safe_load(f)
    params = merge(params, profile.get('param_overrides'))
    # Some config/<platform>/nav2*.yaml hardcode use_sim_time: false for a server, which would override the launch
    # argument (navigation.launch.py's SetParameter only provides a default), so every node gets the argument's value.
    def set_use_sim_time(node):
        for key, value in node.items():
            if key == 'ros__parameters' and isinstance(value, dict):
                value['use_sim_time'] = arg('use_sim_time') == 'true'
            elif isinstance(value, dict):  # costmaps are nested one level deeper (local_costmap: local_costmap:)
                set_use_sim_time(value)

    set_use_sim_time(params)
    docks_msg = charger_docks(params, arg('charger_docks'), platform_model) if arg('charger_docks') else ''
    if scan_topic == 'params':
        map_topics(params, lambda topic: in_namespace(topic, namespace))
    else:
        map_topics(params, lambda topic: scan_topic)
    topics = set()
    map_topics(params, lambda topic: topics.add(topic) or topic)
    # A fixed path, like moveit.launch.py's /tmp/moveit_sim_execution_<ns>.yaml: husky_operations_manager reads the
    # docking_server block from it (the docks Nav2 really runs with, e.g. the simulator's chargers).
    params_file_out = os.path.join(tempfile.gettempdir(), f'{namespace}_nav2_map.yaml')
    with open(params_file_out, 'w') as f:
        yaml.safe_dump(params, f)

    messages = [LogInfo(msg=f'bringup_nav2_map: {namespace} ({platform_model}) params {params_file}, '
                            f'scan {", ".join(sorted(topics))}, map {map_yaml_file} -> {params_file_out}')]
    if docks_msg:
        messages.append(LogInfo(msg=docks_msg))
    for topic in sorted(topics):
        relative_topic = topic.removeprefix(f'/{namespace}/')
        sensor_type = next((t for t in ('lidar2d', 'camera') if relative_topic.startswith(f'sensors/{t}_')), None)
        if sensor_type and not sensors.get(sensor_type):
            messages.append(LogInfo(msg=f'bringup_nav2_map: WARNING {topic} needs a {sensor_type} sensor, '
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
