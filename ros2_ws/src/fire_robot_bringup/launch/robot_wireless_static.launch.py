"""Pi wireless integration profile with all motion outputs disarmed.

This is a source gate for relocating Nav2 and mission to the Pi. The normal
robot launch still owns USB, sensors, TF and the safety command gate. Nav2 is
forced onto its isolated shadow output. The safety gate's input is moved to a
static-only topic so an unrelated /cmd_vel_raw publisher cannot move the robot.
Never run it beside another robot/Nav2 launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, **arguments):
    path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file)
    # Child launches reuse generic names such as cmd_vel_output_topic. Scoping
    # prevents robot.launch.py's real output argument from leaking into the
    # Nav2 shadow wrapper, which intentionally rejects a real output override.
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )], scoped=True)


def generate_launch_description():
    navigation_share = get_package_share_directory('fire_robot_navigation')
    map_path = os.path.join(
        navigation_share, 'maps', 'fire_room_9_10_2026.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_path,
                              description='Absolute path to map YAML on Pi'),
        DeclareLaunchArgument('camera_fps', default_value='10',
                              choices=['5', '10', '15', '20', '25', '30']),
        DeclareLaunchArgument('camera_max_bytes_per_sec',
                              default_value='750000'),
        _include(
            'fire_robot_bringup', 'robot.launch.py',
            use_sim_time=use_sim_time,
            use_camera='true',
            camera_backend='native_mjpeg',
            camera_fps=LaunchConfiguration('camera_fps'),
            camera_max_bytes_per_sec=LaunchConfiguration(
                'camera_max_bytes_per_sec'),
            cmd_vel_input_topic='/fire_robot_static/cmd_vel_raw',
        ),
        _include(
            'fire_robot_navigation', 'nav2_motion_shadow.launch.py',
            use_sim_time=use_sim_time,
            map=LaunchConfiguration('map'),
            rviz='false',
        ),
        _include(
            'fire_robot_mission',
            'mission_motion_arbiter_shadow.launch.py',
            use_sim_time=use_sim_time,
            require_live_cancel_proof='true',
        ),
        _include('fire_robot_mission', 'ir_range_preview.launch.py'),
        _include(
            'fire_robot_mission', 'mission_live_nav_stop.launch.py',
            use_sim_time=use_sim_time,
            enable_live_output='false',
        ),
        _include(
            'fire_robot_mission', 'nav2_cancel_guard.launch.py',
            use_sim_time=use_sim_time,
            enable_real_cancel='false',
        ),
        _include(
            'fire_robot_mission', 'mission_live_alignment.launch.py',
            use_sim_time=use_sim_time,
            enable_live_alignment='false',
        ),
        _include(
            'fire_robot_mission', 'ir_approach_live.launch.py',
            enable_live_ir_jog='false',
        ),
        _include('fire_robot_mission', 'post_spray_review.launch.py'),
    ])
