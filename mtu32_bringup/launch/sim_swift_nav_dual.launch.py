"""Sim twin of swift_nav_dual.launch.py: dual SwiftNav Duro RTK heading + GPS global localization (multirobot_sim).

Same node graph as on the real robot, with the two sbp-to-ros drivers replaced by the simulator:

* reference Duro  -> the sim publishes its NavSatFix on sensors/gps_<ref>/fix (what ref_duro_node's navsatfix
                     publisher remapped to 'fix' gives on the robot); exact position, no noise, covariance
                     'unknown' (zeros).
* attitude Duro   -> duro_sim baseline_node, named att_duro_node in sensors/gps_<att>, publishes 'baseline'
                     (attitude antenna relative to the reference antenna, fixed RTK) from the two simulated fixes.
* dual_duro_heading heading_filter, navsat_transform and ekf_global_node: the real nodes, with the real parameters
  from config/<platform>/dual_duro_heading.yaml (config/j100/ if the platform has none).

Antennas: the robot's GPS sensors in robot.yaml order, first = attitude (left antenna), second = reference (right
antenna): a200/a300 gps_0 / gps_1, Jackals gps_1 / gps_2. That is the right->left baseline the heading filter assumes.

Patched for the sim: the datum is the sim's GPS origin (setup_scene.py GPS_READ_SCRIPT; sim world X/Y = East/North),
so the map frame is the Isaac world frame; ekf_global_node's imu0 and the heading filter's frame follow the antenna
names above. ekf_global_node publishes map->odom, so start Nav2 with sim_nav2.launch.py gps:=true (no static map->odom):

    ros2 launch mtu32_bringup sim_swift_nav_dual.launch.py
    ros2 launch mtu32_bringup sim_nav2.launch.py gps:=true
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

# setup_scene.py GPS_READ_SCRIPT _ORIGIN_LAT/_ORIGIN_LON: lat/lon of the Isaac world origin
SIM_DATUM = '47.1211, -88.5455, 0.0'

ARGUMENTS = [
    DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/', description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value=os.environ.get('USE_SIM_TIME', 'false'),
                          choices=['true', 'false'], description="the simulator's /clock (default: $USE_SIM_TIME)"),
    DeclareLaunchArgument('datum', default_value=SIM_DATUM,
                          description="navsat_transform datum 'lat, lon, yaw'; empty = keep the params file's"),
    DeclareLaunchArgument('params_file', default_value='',
                          description='default: config/<platform>/dual_duro_heading.yaml (or j100) of mtu32_bringup'),
]


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('mtu32_bringup')
    setup_path = LaunchConfiguration('setup_path').perform(context)
    clearpath_config = ClearpathConfig(read_yaml(os.path.join(setup_path, 'robot.yaml')))
    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    gps = [g.name for g in clearpath_config.sensors.get_all_gps()]
    if len(gps) < 2:
        raise RuntimeError(f'{namespace}: dual-antenna heading needs two GPS sensors in robot.yaml, found {gps}')
    att, ref = gps[0], gps[1]

    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(pkg, 'config', platform_model, 'dual_duro_heading.yaml')
        if not os.path.exists(params_file):
            params_file = os.path.join(pkg, 'config', 'j100', 'dual_duro_heading.yaml')
    with open(params_file) as f:
        params = yaml.safe_load(f)

    navsat = params['/**/navsat_transform']['ros__parameters']
    datum = LaunchConfiguration('datum').perform(context)
    if datum:
        navsat['datum'] = [float(v) for v in datum.split(',')]
    ekf = params['/**/ekf_global_node']['ros__parameters']
    ekf['imu0'] = f'sensors/{att}/heading_imu'
    with tempfile.NamedTemporaryFile('w', suffix='_dual_duro_sim.yaml', delete=False) as f:
        yaml.safe_dump({'/**/navsat_transform': params['/**/navsat_transform'],
                        '/**/ekf_global_node': params['/**/ekf_global_node']}, f)
        params_sim = f.name

    remappings_tf = [('/tf', f'/{namespace}/tf'), ('/tf_static', f'/{namespace}/tf_static')]

    return [
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        Node(
            package='duro_sim',
            executable='baseline_node',
            name='att_duro_node',
            namespace=f'/{namespace}/sensors/{att}',
            output='screen',
            parameters=[{
                'frame_name': f'{att}_link',
                'rover_fix_topic': 'fix',
                'base_fix_topic': f'/{namespace}/sensors/{ref}/fix',
            }],
        ),
        Node(
            package='dual_duro_heading',
            executable='heading_filter',
            name='dual_duro_heading_node',
            namespace=f'/{namespace}/sensors/{att}',
            output='screen',
            parameters=[{'frame_id': f'{ref}_link'}],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            namespace=f'/{namespace}',
            remappings=remappings_tf + [
                ('gps/fix', f'/{namespace}/sensors/{ref}/fix'),
                ('odometry/filtered', f'/{namespace}/odometry/global'),
                ('imu', f'/{namespace}/sensors/{att}/heading_imu'),
                ('datum', f'/{namespace}/navsat_transform/datum'),
                ('fromLL', f'/{namespace}/navsat_transform/fromLL'),
                ('fromLLArray', f'/{namespace}/navsat_transform/fromLLArray'),
            ],
            parameters=[params_sim],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            namespace=f'/{namespace}',
            remappings=remappings_tf + [
                ('odometry/filtered', 'odometry/global'),
                ('set_pose', 'ekf_global_node/set_pose'),
                ('enable', 'ekf_global_node/enable'),
                ('reset', 'ekf_global_node/reset'),
                ('toggle', 'ekf_global_node/toggle'),
                ('/diagnostics', 'diagnostics'),
            ],
            parameters=[params_sim],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
