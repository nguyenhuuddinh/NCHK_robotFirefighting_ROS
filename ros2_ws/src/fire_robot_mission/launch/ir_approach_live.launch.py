"""Separate opt-in one-shot IR jog; does not launch Nav2 or a pump."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_live_ir_jog', default_value='false'),
        DeclareLaunchArgument('allow_provisional_ir', default_value='false'),
        DeclareLaunchArgument('enable_reverse_jog', default_value='false'),
        DeclareLaunchArgument('integrated_handoff', default_value='false'),
        DeclareLaunchArgument('continuous_approach', default_value='false'),
        DeclareLaunchArgument('require_fire_session_context', default_value='false'),
        DeclareLaunchArgument('enable_close_owner_observation', default_value='false'),
        DeclareLaunchArgument('speed_m_s', default_value='0.05'),
        DeclareLaunchArgument('max_total_approach_m', default_value='2.0'),
        Node(
            package='fire_robot_mission',
            executable='ir_approach_live',
            name='ir_approach_live',
            output='screen',
            parameters=[{
                'enable_live_ir_jog': ParameterValue(
                    LaunchConfiguration('enable_live_ir_jog'),
                    value_type=bool),
                'allow_provisional_ir': ParameterValue(
                    LaunchConfiguration('allow_provisional_ir'),
                    value_type=bool),
                'enable_reverse_jog': ParameterValue(
                    LaunchConfiguration('enable_reverse_jog'),
                    value_type=bool),
                'integrated_handoff': ParameterValue(
                    LaunchConfiguration('integrated_handoff'),
                    value_type=bool),
                'continuous_approach': ParameterValue(
                    LaunchConfiguration('continuous_approach'),
                    value_type=bool),
                'require_fire_session_context': ParameterValue(
                    LaunchConfiguration('require_fire_session_context'), value_type=bool),
                'enable_close_owner_observation': ParameterValue(
                    LaunchConfiguration('enable_close_owner_observation'), value_type=bool),
                'speed_m_s': ParameterValue(
                    LaunchConfiguration('speed_m_s'), value_type=float),
                'max_total_approach_m': ParameterValue(
                    LaunchConfiguration('max_total_approach_m'),
                    value_type=float),
            }],
        ),
    ])
