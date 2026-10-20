"""Start a disabled-by-default, one-pulse-per-goal pump controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_pump_demo', default_value='false'),
        DeclareLaunchArgument('allow_provisional_ir', default_value='false'),
        DeclareLaunchArgument('pulse_s', default_value='5.0'),
        DeclareLaunchArgument('release_pump_on_terminal', default_value='false'),
        DeclareLaunchArgument('require_fire_session_context', default_value='false'),
        DeclareLaunchArgument('enable_close_owner_observation', default_value='false'),
        Node(
            package='fire_robot_mission',
            executable='pump_demo_controller',
            name='pump_demo_controller',
            output='screen',
            parameters=[{
                'enable_pump_demo': LaunchConfiguration('enable_pump_demo'),
                'allow_provisional_ir': LaunchConfiguration(
                    'allow_provisional_ir'),
                'pulse_s': ParameterValue(
                    LaunchConfiguration('pulse_s'), value_type=float),
                'release_pump_on_terminal': ParameterValue(
                    LaunchConfiguration('release_pump_on_terminal'), value_type=bool),
                'require_fire_session_context': ParameterValue(
                    LaunchConfiguration('require_fire_session_context'), value_type=bool),
                'enable_close_owner_observation': ParameterValue(
                    LaunchConfiguration('enable_close_owner_observation'), value_type=bool),
            }],
        ),
    ])
