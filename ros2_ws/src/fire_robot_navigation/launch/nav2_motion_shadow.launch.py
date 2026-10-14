"""Run Nav2 with every final velocity output isolated from the robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SHADOW_CMD_VEL_TOPIC = '/fire_mission/nav2_cmd_vel_shadow'


def _include_nav2_shadow(context):
    """Reject unsafe overrides and construct the isolated Nav2 include."""
    requested_output = context.launch_configurations.get(
        'cmd_vel_output_topic')
    if requested_output not in (None, SHADOW_CMD_VEL_TOPIC):
        raise RuntimeError(
            'nav2_motion_shadow.launch.py does not permit overriding the '
            f'isolated output topic: {requested_output}')

    package_dir = get_package_share_directory('fire_robot_navigation')
    nav2_launch = os.path.join(package_dir, 'launch', 'nav2.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'rviz': LaunchConfiguration('rviz'),
            'cmd_vel_output_topic': SHADOW_CMD_VEL_TOPIC,
        }.items(),
    )]


def generate_launch_description():
    """Include the normal Nav2 stack while forcing a shadow-only output."""
    package_dir = get_package_share_directory('fire_robot_navigation')
    default_params = os.path.join(
        package_dir, 'config', 'nav2_params.yaml')

    arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'map', description='Absolute path to the map YAML file'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Absolute path to the Nav2 parameter file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically start the Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz for safe shadow navigation testing'),
    ]

    # Construct the include only after checking the launch context. Keeping it
    # behind OpaqueFunction also prevents the child output argument from being
    # presented as a top-level wrapper argument by ``ros2 launch --show-args``.
    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=_include_nav2_shadow),
    ])
