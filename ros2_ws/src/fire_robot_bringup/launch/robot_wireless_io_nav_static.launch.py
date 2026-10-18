"""Unarmed Pi I/O + Nav2 profile for isolating mission startup load.

This profile keeps camera, LiDAR, serial, TF and the safety gate. It omits
mission nodes and rosbridge. Nav2 output is shadow-only; the safety gate listens
on a separate static topic. It is a QA profile, not an autonomous mission.
Never run it beside another robot or Nav2 launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, *, scoped=True, **arguments):
    path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file)
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )], scoped=scoped)


def generate_launch_description():
    navigation_share = get_package_share_directory('fire_robot_navigation')
    map_path = os.path.join(
        navigation_share, 'maps', 'fire_room_9_10_2026.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_path),
        DeclareLaunchArgument('camera_fps', default_value='10',
                              choices=['5', '10', '15', '20', '25', '30']),
        DeclareLaunchArgument('camera_max_bytes_per_sec',
                              default_value='750000'),
        DeclareLaunchArgument('usb_recovery_capture_s', default_value='0',
                              description='ESP USB recovery capture seconds (0-180)'),
        DeclareLaunchArgument('usb_fault_trace_events', default_value='0',
                              description='Pi USB fault trace count (0-16)'),
        DeclareLaunchArgument('serial_port', default_value=(
            '/dev/serial/by-id/'
            'usb-Espressif_Systems_Freenove_ESP32-S3_WROOM_N8R2_'
            '_8MB_Flash___2MB_PSRAM__28372F1F73C8-if00')),
        _include(
            'fire_robot_bringup', 'robot.launch.py',
            use_sim_time=use_sim_time,
            use_camera='true',
            camera_backend='native_mjpeg',
            camera_fps=LaunchConfiguration('camera_fps'),
            camera_max_bytes_per_sec=LaunchConfiguration(
                'camera_max_bytes_per_sec'),
            serial_port=LaunchConfiguration('serial_port'),
            cmd_vel_input_topic='/fire_robot_static/cmd_vel_raw',
            enable_diagnostics='false',
            usb_recovery_capture_s=LaunchConfiguration(
                'usb_recovery_capture_s'),
            usb_fault_trace_events=LaunchConfiguration(
                'usb_fault_trace_events'),
            use_dashboard='false',
        ),
        # Delayed Nav2 TimerAction nodes need these launch configurations to
        # remain visible after the include returns (same as the G2 profile).
        _include(
            'fire_robot_navigation', 'nav2_motion_shadow.launch.py',
            scoped=False,
            use_sim_time=use_sim_time,
            map=LaunchConfiguration('map'),
            rviz='false',
        ),
    ])
