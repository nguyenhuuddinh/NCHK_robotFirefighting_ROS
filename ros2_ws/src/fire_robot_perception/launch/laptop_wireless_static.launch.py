"""Laptop perception and RViz for the Pi wireless static profile.

Navigation and mission nodes belong to the Pi profile. This launch does not
open a command publisher or arm any actuator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = get_package_share_directory('fire_robot_perception')
    nav2_share = get_package_share_directory('nav2_bringup')
    perception_launch = os.path.join(
        perception_share, 'launch', 'perception.launch.py')
    rviz_config = os.path.join(
        nav2_share, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'model_path',
            default_value=(
                '/media/huudinh/New Volume/esp32/'
                'Fire_smoke_person_detection/best_final.pt'),
            description='YOLO model path on laptop'),
        DeclareLaunchArgument(
            'python_executable',
            default_value='/home/huudinh/.venvs/fire_robot_yolo/bin/python',
            description='YOLO Python on laptop'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'model_path': LaunchConfiguration('model_path'),
                'python_executable': LaunchConfiguration(
                    'python_executable'),
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2',
            name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
