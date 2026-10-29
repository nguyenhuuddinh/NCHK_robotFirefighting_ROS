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
            }],
        ),
    ])
