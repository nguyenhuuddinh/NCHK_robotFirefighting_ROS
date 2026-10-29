"""Run only the observe-only IR standoff preview on the laptop."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    params_default = os.path.join(
        get_package_share_directory('fire_robot_mission'),
        'config', 'ir_range_preview.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='fire_robot_mission',
            executable='ir_range_preview',
            name='ir_range_preview',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
