"""Optional one-shot-per-goal body rotation; default is disabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_params(context):
    path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(path):
        raise RuntimeError(f'mission params file does not exist: {path}')
    return []


def generate_launch_description():
    """Launch only the live-alignment owner, never Nav2 or any actuator."""
    package_dir = get_package_share_directory('fire_robot_mission')
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_dir, 'config', 'mission_params.yaml'))
    enabled = DeclareLaunchArgument(
        'enable_live_alignment', default_value='false',
        description='Opt in to one rotation per goal UUID; separate '
                    'operator heartbeat is still required')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    node = Node(
        package='fire_robot_mission',
        executable='live_alignment_takeover',
        name='live_alignment_takeover',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'enable_live_alignment': ParameterValue(
                LaunchConfiguration('enable_live_alignment'),
                value_type=bool),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'require_fire_session_context': ParameterValue(
                LaunchConfiguration('require_fire_session_context'), value_type=bool),
            'enable_close_owner_observation': ParameterValue(
                LaunchConfiguration('enable_close_owner_observation'), value_type=bool),
        }],
    )
    return LaunchDescription([
        params_file, enabled, use_sim_time,
        DeclareLaunchArgument('require_fire_session_context', default_value='false'),
        DeclareLaunchArgument('enable_close_owner_observation', default_value='false'),
        OpaqueFunction(function=_validate_params), node,
    ])
