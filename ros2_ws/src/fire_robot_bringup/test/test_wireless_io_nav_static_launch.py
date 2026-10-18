"""Source gate for the separate, unarmed I/O + Nav2 R2 profile."""

import importlib.util
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    PopLaunchConfigurations)
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions


LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'robot_wireless_io_nav_static.launch.py'


def _profile():
    spec = importlib.util.spec_from_file_location('io_nav_static', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: f'/tmp/{package}'
    return module.generate_launch_description()


def _include(group):
    return next(action for action in group.get_sub_entities()
                if isinstance(action, IncludeLaunchDescription))


def _path(include):
    substitutions = include.launch_description_source._LaunchDescriptionSource__location
    return perform_substitutions(LaunchContext(), substitutions)


def test_io_nav_profile_is_unarmed_and_omits_mission():
    entities = _profile().entities
    groups = [item for item in entities if isinstance(item, GroupAction)]
    args = {item.name: item for item in entities
            if isinstance(item, DeclareLaunchArgument)}
    assert len(groups) == 2
    robot, nav2 = (_include(group) for group in groups)
    assert _path(robot).endswith('/robot.launch.py')
    assert _path(nav2).endswith('/nav2_motion_shadow.launch.py')
    robot_args = dict(robot.launch_arguments)
    nav_args = dict(nav2.launch_arguments)
    assert robot_args['cmd_vel_input_topic'] == '/fire_robot_static/cmd_vel_raw'
    assert robot_args['enable_diagnostics'] == 'false'
    assert robot_args['use_dashboard'] == 'false'
    assert nav_args['rviz'] == 'false'
    assert args['camera_fps'].default_value[0].text == '10'
    assert not any(isinstance(action, PopLaunchConfigurations)
                   for action in groups[-1].get_sub_entities())
