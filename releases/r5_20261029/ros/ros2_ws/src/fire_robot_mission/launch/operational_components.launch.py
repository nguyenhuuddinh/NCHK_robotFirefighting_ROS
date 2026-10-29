"""R5.1 installed components, observe-only. Not a robot session launcher.

Live opt-in is deliberately absent until the operational owner/BOOT/handback
contracts have been connected. No process is started by offline launch tests.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('fire_robot_mission')) / 'config'
    operational = str(config / 'operational_params.yaml')
    return LaunchDescription([
        Node(package='fire_robot_mission', executable='ir_range_preview',
             name='ir_range_preview', output='screen',
             parameters=[str(config / 'ir_range_preview.yaml'), {'use_sim_time': False}]),
        Node(package='fire_robot_mission', executable='ir_approach_live',
             name='ir_approach_live', output='screen', parameters=[operational]),
        Node(package='fire_robot_mission', executable='fire_pulse_controller',
             name='fire_pulse_controller', output='screen', parameters=[operational]),
    ])
