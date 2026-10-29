"""Start the goal-specific Nav2 cancel guard, disarmed by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_params(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(params_file):
        raise RuntimeError(
            f'mission params file does not exist: {params_file}')
    return []


def generate_launch_description():
    """Require explicit opt-in plus operator heartbeat to cancel."""
    package_dir = get_package_share_directory('fire_robot_mission')
    default_params = os.path.join(
        package_dir, 'config', 'mission_params.yaml')
    params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    enabled = DeclareLaunchArgument(
        'enable_real_cancel', default_value='false',
        description='Opt in to goal-specific CancelGoal service calls')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    guard = Node(
        package='fire_robot_mission',
        executable='nav2_cancel_guard',
        name='nav2_cancel_guard',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enable_real_cancel': ParameterValue(
                    LaunchConfiguration('enable_real_cancel'),
                    value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool),
            },
        ],
    )
    return LaunchDescription([
        params_file, enabled, use_sim_time,
        OpaqueFunction(function=_validate_params), guard,
    ])
