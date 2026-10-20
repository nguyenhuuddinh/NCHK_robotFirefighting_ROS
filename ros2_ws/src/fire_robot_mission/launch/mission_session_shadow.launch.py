"""Minimal M3 session runtime: only the supervisor, no hidden command owners."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _runtime(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('use_sim_time') != 'false':
        raise RuntimeError('M3 source proof requires use_sim_time=false')
    path = Path(value('shadow_budget_file'))
    if not path.is_absolute() or not path.parent.is_dir() or path.is_symlink():
        raise RuntimeError('Choose an absolute non-symlink budget file in an existing directory')
    init = value('initialize_shadow_budget')
    if init not in ('true', 'false'):
        raise RuntimeError('initialize_shadow_budget must be true or false')
    if not path.is_file() and init != 'true':
        raise RuntimeError('Missing budget needs explicit first-use initialization')
    params = value('params_file')
    if not os.path.isfile(params):
        raise RuntimeError('mission params file missing')
    return [Node(
        package='fire_robot_mission', executable='fire_mission_supervisor',
        name='fire_mission_supervisor', output='screen',
        parameters=[params, {
            'use_sim_time': False,
            'enable_fire_session_observer': True,
            'enable_close_owner_collection': True,
            'enable_global_stop_collection': True,
            'enable_shadow_session_cleanup': True,
            'require_through_status': True,
            'close_owner_manifest': 'io_nav_static_no_owners_v1',
            'shadow_budget_file': str(path),
            'initialize_shadow_budget': ParameterValue(
                LaunchConfiguration('initialize_shadow_budget'), value_type=bool),
        }])]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('fire_robot_mission'),
                           'config', 'mission_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['false']),
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('shadow_budget_file', description='Persistent shadow reservation file'),
        DeclareLaunchArgument('initialize_shadow_budget', default_value='false', choices=['false', 'true']),
        OpaqueFunction(function=_runtime),
    ])
