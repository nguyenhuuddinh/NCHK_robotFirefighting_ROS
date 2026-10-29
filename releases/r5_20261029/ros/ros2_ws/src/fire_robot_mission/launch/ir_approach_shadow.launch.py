"""Launch a telemetry-only approach preview; never a motion owner."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fire_robot_mission',
            executable='ir_approach_shadow',
            name='ir_approach_shadow',
            output='screen',
        ),
    ])
