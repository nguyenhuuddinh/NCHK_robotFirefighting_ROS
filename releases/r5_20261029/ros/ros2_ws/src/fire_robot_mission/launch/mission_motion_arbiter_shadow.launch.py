"""Launch supervisor, handoff gate and isolated motion arbiter shadow."""

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
    """Create a laptop-only, shadow-safe arbiter launch description."""
    package_dir = get_package_share_directory('fire_robot_mission')
    default_params = os.path.join(
        package_dir, 'config', 'mission_params.yaml')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Absolute path to mission shadow parameters',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time; false for the physical robot',
    )
    proof_arg = DeclareLaunchArgument(
        'require_live_cancel_proof',
        default_value='false',
        description='Require gateway release and exact-goal cancel proof '
                    'for shadow handoff; does not enable motor output',
    )
    common_parameters = [
        LaunchConfiguration('params_file'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    nodes = [
        Node(
            package='fire_robot_mission',
            executable='fire_mission_supervisor',
            name='fire_mission_supervisor',
            output='screen',
            parameters=common_parameters,
        ),
        Node(
            package='fire_robot_mission',
            executable='alignment_handoff_shadow',
            name='alignment_handoff_shadow',
            output='screen',
            parameters=common_parameters + [{
                'require_live_cancel_proof': ParameterValue(
                    LaunchConfiguration('require_live_cancel_proof'),
                    value_type=bool),
            }],
        ),
        Node(
            package='fire_robot_mission',
            executable='motion_arbiter_shadow',
            name='motion_arbiter_shadow',
            output='screen',
            parameters=common_parameters,
        ),
        Node(
            package='fire_robot_mission',
            executable='nav2_cancel_intent_shadow',
            name='nav2_cancel_intent_shadow',
            output='screen',
            parameters=common_parameters,
        ),
    ]
    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        proof_arg,
        OpaqueFunction(function=_validate_params),
        *nodes,
    ])
