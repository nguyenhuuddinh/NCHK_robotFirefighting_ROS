"""Light laptop perception profile for the unarmed R2 static comparison.

YOLO detections stay enabled. RViz, annotated images and optional statistics
are disabled. This profile does not launch Nav2 or mission nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('fire_robot_perception')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=(
            '/media/huudinh/New Volume/esp32/'
            'Fire_smoke_person_detection/best_final.pt')),
        DeclareLaunchArgument('python_executable', default_value=(
            '/home/huudinh/.venvs/fire_robot_yolo/bin/python')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'laptop_wireless_static.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'model_path': LaunchConfiguration('model_path'),
                'python_executable': LaunchConfiguration('python_executable'),
                'rviz': 'false',
                'enable_diagnostics': 'false',
                'publish_annotated': 'false',
                'use_alignment_observer': 'false',
            }.items(),
        ),
    ])
