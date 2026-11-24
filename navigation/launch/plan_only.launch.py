"""Nav2 path planning only, on the WVH map: no robot, no simulator, nothing moves.

    ros2 launch ~/mini_golf/navigation/launch/plan_only.launch.py

Then ask for paths with plan_path.py or run_planning_tests.py. Do not run this
at the same time as mini_golf_nav.launch.py (both publish /map and TF).

Starts the same map, keepout zones and planner (NavFn + global costmap) as
mini_golf_nav.launch.py, with the same params file except:
  * the spatio-temporal voxel layer (head depth camera) is removed, as there is
    no camera here (and the plugin isn't installed on every machine);
  * the robot's pose is not from a driver/AMCL: map->odom is fixed (identity)
    and plan_path.py publishes odom->base_link at the start pose.
The lidar (base_link->laser) is placed as in the Stretch 3 URDF so fake scans
from plan_path.py are seen exactly as real ones would be.
"""
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WVH_MAP = os.path.join(NAV_DIR, 'maps', 'WVH_edit.yaml')
KEEPOUT_MAP = os.path.join(NAV_DIR, 'maps', 'WVH_keepout.yaml')
PARAMS = os.path.join(NAV_DIR, 'config', 'nav2_params.yaml')


def without_depth_camera(params_file):
    """Copy of the params with the depth-camera voxel layer removed (temp file)."""
    with open(params_file) as f:
        params = yaml.safe_load(f)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap['plugins'] = [p for p in costmap['plugins'] if p != 'stvl_layer']
        costmap.pop('stvl_layer', None)
    out = tempfile.NamedTemporaryFile('w', prefix='mini_golf_plan_only_', suffix='.yaml',
                                      delete=False)
    with out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name


def launch_setup(context):
    map_file = LaunchConfiguration('map').perform(context)
    params_file = without_depth_camera(LaunchConfiguration('params_file').perform(context))
    nodes = ['map_server', 'keepout_mask_server', 'costmap_filter_info_server', 'planner_server']
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params_file, {'yaml_filename': map_file}]),
        Node(package='nav2_map_server', executable='map_server', name='keepout_mask_server',
             output='screen', parameters=[params_file, {'yaml_filename': KEEPOUT_MAP}],
             remappings=[('/map', '/keepout_mask')]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', output='screen',
             parameters=[params_file, {'type': 0, 'filter_info_topic': '/costmap_filter_info',
                                       'mask_topic': '/keepout_mask',
                                       'base': 0.0, 'multiplier': 1.0}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planning', output='screen',
             parameters=[{'autostart': True, 'node_names': nodes, 'bond_timeout': 10.0}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', '0.004', '--z', '0.1664', '--yaw', '3.1416',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=WVH_MAP, description='Map yaml file'),
        DeclareLaunchArgument('params_file', default_value=PARAMS, description='Nav2 params'),
        OpaqueFunction(function=launch_setup),
    ])
