"""Optional real Nav2 stop gateway. Defaults to no robot command output."""

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
    """Start only the gateway; launch Nav2 shadow and mission separately."""
    package_dir = get_package_share_directory('fire_robot_mission')
    default_params = os.path.join(
        package_dir, 'config', 'mission_params.yaml')
    params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    enabled = DeclareLaunchArgument(
        'enable_live_output', default_value='false',
        description='Opt in to real /cmd_vel_raw output; operator arm still '
                    'required')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    gateway = Node(
        package='fire_robot_mission',
        executable='live_nav_stop_gateway',
        name='live_nav_stop_gateway',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enable_live_output': ParameterValue(
                    LaunchConfiguration('enable_live_output'),
                    value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool),
            },
        ],
    )
    return LaunchDescription([
        params_file, enabled, use_sim_time,
        OpaqueFunction(function=_validate_params), gateway,
    ])
