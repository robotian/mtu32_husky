"""Nav2 against the Isaac Sim robots (multirobot_sim), one instance per robot container.

Reuses the real robot's per-platform Nav2 parameters (config/<platform>/nav2.yaml) and navigation_launch.py, and only
patches what the simulator does differently:

* map->odom: by default nothing publishes it, and the sim's odom is ground truth (no drift), so it is a static
  identity (the robot's map frame origin is where it spawned). With gps:=true, sim_swift_nav_dual.launch.py's
  ekf_global_node publishes it (GPS + dual-antenna heading, the real outdoor flow) and the default params become
  config/<platform>/nav2_mapping.yaml when the platform has one (as bringup_nav2_mapping_j100.launch.py);
* odometry comes straight from platform/odom instead of the EKF's platform/odom/filtered (overridable: odom_topic);
* use_sim_time defaults to $USE_SIM_TIME (the robot container sets it when the sim publishes /clock) and is written
  into every node's parameters (config/<platform>/nav2*.yaml hardcode false for some servers);
* no map_server: the static layer is dropped from both costmaps and the global one becomes a fixed 100 x 100 m
  unknown-space grid centred on the map origin (the same approach as the real robots' nav2_mapless.yaml).

Needs a 2D scan: a200_0284 / a200_0333 / a300_00036 publish sensors/lidar2d_0/scan. That raw scan contains hits on the
robot's own body, which the collision monitor would read as an obstacle inside the footprint and refuse to move, and
which the costmaps would mark as lethal; so a laser_filters box filter sized to the nav2 footprint (plus 2 cm) runs on
it and Nav2 is pointed at sensors/lidar2d_0/scan_filtered (skipped if scan_topic is given). The Jackals have no simulated 2D
lidar; pass scan_topic:=/j100_0921/sensors/camera_0/scan (depthimage_to_laserscan, started by sim_robot_upstart) or
similar.

    ros2 launch mtu32_bringup sim_nav2.launch.py            # odometry only
    ros2 launch mtu32_bringup sim_swift_nav_dual.launch.py  # or: GPS localization first,
    ros2 launch mtu32_bringup sim_nav2.launch.py gps:=true  #     then Nav2 on it
    ros2 action send_goal /$ROBOT_NAMESPACE/navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

ARGUMENTS = [
    DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/', description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic', default_value='',
                          description='Use this 2D laserscan topic as is (default: self-filter sensors/lidar2d_0/scan)'),
    DeclareLaunchArgument('odom_topic', default_value='platform/odom', description='Odometry topic (relative to the namespace)'),
    DeclareLaunchArgument('params_file', default_value='',
                          description='Nav2 params (default: config/<platform>/nav2.yaml of mtu32_bringup)'),
    DeclareLaunchArgument('gps', default_value='false',
                          description='map->odom comes from sim_swift_nav_dual.launch.py (no static map->odom)'),
    DeclareLaunchArgument('use_sim_time', default_value=os.environ.get('USE_SIM_TIME', 'false'),
                          choices=['true', 'false'], description="the simulator's /clock (default: $USE_SIM_TIME)"),
    DeclareLaunchArgument('autostart', default_value='true'),
    DeclareLaunchArgument('log_level', default_value='info'),
]


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('mtu32_bringup')
    setup_path = LaunchConfiguration('setup_path').perform(context)

    clearpath_config = ClearpathConfig(read_yaml(os.path.join(setup_path, 'robot.yaml')))
    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    gps = LaunchConfiguration('gps').perform(context).lower() in ('true', '1')
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(pkg, 'config', f'{platform_model}', 'nav2.yaml')
        mapping = os.path.join(pkg, 'config', f'{platform_model}', 'nav2_mapping.yaml')
        if gps and os.path.exists(mapping):
            params_file = mapping

    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    filter_scan = not scan_topic
    if filter_scan:
        scan_topic = 'sensors/lidar2d_0/scan_filtered'
    odom_topic = LaunchConfiguration('odom_topic').perform(context)

    with open(params_file) as f:
        params = yaml.safe_load(f)

    def patch(node):
        if isinstance(node, dict):
            if 'odom_topic' in node:
                node['odom_topic'] = odom_topic
            if scan_topic and 'topic' in node:
                node['topic'] = scan_topic
            for v in node.values():
                patch(v)
        elif isinstance(node, list):
            for v in node:
                patch(v)

    patch(params)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    def set_use_sim_time(node):
        for key, value in node.items():
            if key == 'ros__parameters' and isinstance(value, dict):
                value['use_sim_time'] = use_sim_time == 'true'
            elif isinstance(value, dict):  # costmaps are nested one level deeper (local_costmap: local_costmap:)
                set_use_sim_time(value)

    set_use_sim_time(params)

    for costmap in ('local_costmap', 'global_costmap'):
        p = params[costmap][costmap]['ros__parameters']
        p['plugins'] = [pl for pl in p['plugins'] if pl != 'static_layer']
    # The sim is slower than the real robot (the in-place turn onto the goal yaw is also no "movement"), so give the
    # progress checker more time before it aborts and triggers recoveries.
    params['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance'] = 30.0
    g = params['global_costmap']['global_costmap']['ros__parameters']
    g.update(rolling_window=False, width=100, height=100, origin_x=-50.0, origin_y=-50.0, resolution=0.05)

    nodes = [SetParameter('use_sim_time', use_sim_time)]
    if filter_scan:
        footprint = yaml.safe_load(params['local_costmap']['local_costmap']['ros__parameters']['footprint'])
        xs, ys = [pt[0] for pt in footprint], [pt[1] for pt in footprint]
        margin = 0.02
        chain = {'/**': {'ros__parameters': {'filter1': {
            'name': 'self_filter',
            'type': 'laser_filters/LaserScanBoxFilter',
            'params': {'box_frame': 'base_link', 'invert': False,
                       'min_x': min(xs) - margin, 'max_x': max(xs) + margin,
                       'min_y': min(ys) - margin, 'max_y': max(ys) + margin,
                       'min_z': -1.0, 'max_z': 2.0}}}}}
        with tempfile.NamedTemporaryFile('w', suffix='_scan_filter_sim.yaml', delete=False) as f:
            yaml.safe_dump(chain, f)
        nodes.append(Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            namespace=f'/{namespace}/sensors/lidar2d_0',
            parameters=[f.name],
            # deeper namespace than the robot's: a relative 'tf' would resolve to .../sensors/lidar2d_0/tf
            remappings=[('/tf', f'/{namespace}/tf'), ('/tf_static', f'/{namespace}/tf_static')],
        ))

    with tempfile.NamedTemporaryFile('w', suffix='_nav2_sim.yaml', delete=False) as f:
        yaml.safe_dump(params, f)
        params = f.name

    if not gps:
        nodes.append(Node(
            package='tf2_ros',
            namespace=namespace,
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ))

    return nodes + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': LaunchConfiguration('autostart').perform(context),
                'params_file': params,
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': LaunchConfiguration('log_level').perform(context),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
