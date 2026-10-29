"""R5 session/UI observer only; does not start robot components or live arms."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('fire_robot_mission')) / 'config' / 'operational_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('show_gui', default_value='false'),
        Node(package='fire_robot_mission', executable='fire_robot_operator',
             name='fire_robot_operator', output='screen', parameters=[config, {
                 'show_gui': ParameterValue(LaunchConfiguration('show_gui'), value_type=bool),
             }]),
    ])
