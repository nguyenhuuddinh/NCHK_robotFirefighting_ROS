"""Read-only post-spray evaluation for diagnostics and a future dashboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('settle_s', default_value='1.0'),
        DeclareLaunchArgument('confirm_s', default_value='1.0'),
        DeclareLaunchArgument('max_pulses', default_value='3'),
        Node(
            package='fire_robot_mission', executable='post_spray_review',
            name='post_spray_review', output='screen', parameters=[{
                'settle_s': ParameterValue(
                    LaunchConfiguration('settle_s'), value_type=float),
                'confirm_s': ParameterValue(
                    LaunchConfiguration('confirm_s'), value_type=float),
                'max_pulses': ParameterValue(
                    LaunchConfiguration('max_pulses'), value_type=int),
            }],
        ),
    ])
