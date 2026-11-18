"""Stretch driver + lidar + AMCL localization + Nav2 (Regulated Pure Pursuit) + RViz.

    ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py

Then set the robot's starting pose in RViz ("2D Pose Estimate") and send goals
with go_to_goal.py.

Based on parcs_stretch_nav2_WVH (navigation.launch.py, bringup_launch.py and
navigation_launch.py), copied here so mini_golf does not depend on that repo
on the robot. Differences from the starter:
  * Nav2's velocity commands are remapped cmd_vel -> /stretch/cmd_vel, the
    topic the Stretch driver listens on (commented out in the starter).
  * Params: config/nav2_params.yaml (Regulated Pure Pursuit, see that file).
  * No joystick teleop and no SLAM option.
  * use_sim:=true runs against the MuJoCo simulator instead (testing only; needs
    the parcs_stretch_nav2 package from the starter repo, see sim_params()).
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WVH_MAP = os.path.join(NAV_DIR, 'maps', 'WVH_edit.yaml')
SIM_MAP = os.path.join(NAV_DIR, 'maps', 'sim_open_world_table.yaml')
KEEPOUT_MAP = os.path.join(NAV_DIR, 'maps', 'WVH_keepout.yaml')
PARAMS = os.path.join(NAV_DIR, 'config', 'nav2_params.yaml')

# Nav2 node name -> package, as in the starter's navigation_launch.py.
NAV2_NODES = {
    'controller_server': 'nav2_controller',
    'planner_server': 'nav2_planner',
    'behavior_server': 'nav2_behaviors',
    'bt_navigator': 'nav2_bt_navigator',
    'waypoint_follower': 'nav2_waypoint_follower',
}


def include(package, launch_file, args=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(args or {}).items())


def sim_params(params_file):
    """Simulator-only changes to the params, written to a temporary file.

    Same changes as the starter's use_sim:=true (see its SIM_CHANGES.md): the
    simulator publishes /scan_filtered, spawns the robot at (0, 0, 0), has
    near-perfect odometry, a much slower base, and no depth camera, and the WVH
    keepout zones don't apply to the simulator's map.
    """
    with open(params_file) as f:
        params = yaml.safe_load(f)
    amcl = params['amcl']['ros__parameters']
    amcl['scan_topic'] = '/scan_filtered'
    amcl['initial_pose'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    for alpha in ('alpha1', 'alpha2', 'alpha3', 'alpha4'):
        amcl[alpha] = 0.01
    progress = params['controller_server']['ros__parameters']['progress_checker']
    progress['required_movement_radius'] = 0.1
    progress['movement_time_allowance'] = 60.0
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap['plugins'] = [p for p in costmap['plugins'] if p != 'stvl_layer']
        costmap.pop('stvl_layer', None)
        costmap.pop('filters', None)
        costmap.pop('keepout_filter', None)
        costmap['obstacle_layer']['scan']['topic'] = '/scan_filtered'
    out = tempfile.NamedTemporaryFile('w', prefix='mini_golf_nav2_sim_', suffix='.yaml',
                                      delete=False)
    with out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name


def launch_setup(context):
    use_sim = LaunchConfiguration('use_sim').perform(context) == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    actions = []
    if use_sim:
        if map_file == WVH_MAP:
            map_file = SIM_MAP
        params_file = sim_params(params_file)
        actions.append(include('parcs_stretch_nav2', 'sim_driver.launch.py', {
            'mode': 'navigation', 'broadcast_odom_tf': 'True',
            'publish_static_map_odom': 'false',  # AMCL provides map->odom
            'use_mujoco_viewer': LaunchConfiguration('use_mujoco_viewer')}))
    else:
        actions.append(include('stretch_core', 'stretch_driver.launch.py', {
            'mode': 'navigation', 'broadcast_odom_tf': 'True'}))
        actions.append(include('stretch_core', 'rplidar.launch.py'))

    # Localization: map_server + AMCL (from the installed nav2_bringup).
    actions.append(include('nav2_bringup', 'localization_launch.py', {
        'map': map_file, 'params_file': params_file,
        'use_sim_time': 'false', 'autostart': 'true'}))

    # Navigation nodes, as in the starter's navigation_launch.py.
    remaps = [('cmd_vel', '/stretch/cmd_vel')]
    nav_nodes = list(NAV2_NODES)
    if not use_sim:
        # WVH keepout zones (the costmaps' keepout_filter reads this mask).
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='keepout_mask_server',
            output='screen', parameters=[params_file, {'yaml_filename': KEEPOUT_MAP}],
            remappings=[('/map', '/keepout_mask')]))
        actions.append(Node(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server', output='screen',
            parameters=[params_file, {'type': 0, 'filter_info_topic': '/costmap_filter_info',
                                      'mask_topic': '/keepout_mask',
                                      'base': 0.0, 'multiplier': 1.0}]))
        nav_nodes = ['keepout_mask_server', 'costmap_filter_info_server'] + nav_nodes
    for name, package in NAV2_NODES.items():
        actions.append(Node(package=package, executable=name, name=name, output='screen',
                            parameters=[params_file], remappings=remaps))
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': nav_nodes, 'bond_timeout': 10.0}]))

    if use_rviz:
        actions.append(include('nav2_bringup', 'rviz_launch.py'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=WVH_MAP,
                              description='Map yaml file (default: WVH)'),
        DeclareLaunchArgument('params_file', default_value=PARAMS,
                              description='Nav2 params file'),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim', default_value='false', choices=['true', 'false'],
                              description='Run against the MuJoCo simulator (testing only)'),
        DeclareLaunchArgument('use_mujoco_viewer', default_value='false',
                              choices=['true', 'false'],
                              description='(use_sim only) open the MuJoCo viewer'),
        OpaqueFunction(function=launch_setup),
    ])
