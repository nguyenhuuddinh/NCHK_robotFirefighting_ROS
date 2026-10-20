"""Source contract for the unarmed Pi wireless integration profile."""

import importlib.util
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    PopLaunchConfigurations)
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'robot_wireless_static.launch.py'


def _module():
    spec = importlib.util.spec_from_file_location('wireless_static', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: f'/tmp/{name}'
    return module


def _text(value):
    if isinstance(value, LaunchConfiguration):
        return f'<{_text(value.variable_name)}>'
    if isinstance(value, (tuple, list)):
        return ''.join(_text(part) for part in value)
    if hasattr(value, 'text'):
        return value.text
    return str(value)


def _includes(entities):
    groups = [entity for entity in entities
              if isinstance(entity, GroupAction)]
    includes = [child for group in groups
                for child in group.get_sub_entities()
                if isinstance(child, IncludeLaunchDescription)]
    assert len(groups) == len(includes)
    return includes


def _source_path(include):
    """Resolve the launch source without opening its mocked child file."""
    source = include.launch_description_source
    substitutions = source._LaunchDescriptionSource__location
    return perform_substitutions(LaunchContext(), substitutions)


def test_pi_profile_includes_one_robot_and_one_shadow_nav2():
    module = _module()
    entities = module.generate_launch_description().entities
    args = {entity.name: entity for entity in entities
            if isinstance(entity, DeclareLaunchArgument)}
    includes = _includes(entities)
    files = [_source_path(entity) for entity in includes]

    assert len(includes) == 9
    assert sum('/robot.launch.py' in path for path in files) == 1
    assert sum('/nav2_motion_shadow.launch.py' in path
               for path in files) == 1
    assert all('/perception.launch.py' not in path for path in files)
    assert args['camera_fps'].default_value[0].text == '10'
    assert args['map'].default_value[0].text.endswith(
        '/maps/fire_room_9_10_2026.yaml')


def test_pi_profile_never_enables_motion_cancel_or_ir_jog():
    module = _module()
    includes = _includes(module.generate_launch_description().entities)
    by_file = {}
    for entity in includes:
        location = _source_path(entity)
        for name in ('robot.launch.py', 'nav2_motion_shadow.launch.py',
                     'mission_live_nav_stop.launch.py',
                     'nav2_cancel_guard.launch.py',
                     'mission_live_alignment.launch.py',
                     'ir_approach_live.launch.py'):
            if '/' + name in location:
                by_file[name] = {
                    key: _text(value)
                    for key, value in entity.launch_arguments}

    assert by_file['robot.launch.py']['camera_fps'] == '<camera_fps>'
    assert by_file['robot.launch.py']['cmd_vel_input_topic'] == (
        '/fire_robot_static/cmd_vel_raw')
    assert by_file['nav2_motion_shadow.launch.py']['rviz'] == 'false'
    assert by_file['mission_live_nav_stop.launch.py'][
        'enable_live_output'] == 'false'
    assert by_file['nav2_cancel_guard.launch.py'][
        'enable_real_cancel'] == 'false'
    assert by_file['mission_live_alignment.launch.py'][
        'enable_live_alignment'] == 'false'
    assert by_file['ir_approach_live.launch.py'][
        'enable_live_ir_jog'] == 'false'


def test_delayed_nav2_nodes_keep_params_scope_and_shadow_output():
    """TimerAction nodes must retain launch configs after the include returns."""
    module = _module()
    groups = [entity for entity in module.generate_launch_description().entities
              if isinstance(entity, GroupAction)]
    nav2 = next(group for group in groups
                if any('/nav2_motion_shadow.launch.py' in _source_path(child)
                       for child in group.get_sub_entities()
                       if isinstance(child, IncludeLaunchDescription)))
    assert groups[-1] is nav2
    assert not any(isinstance(action, PopLaunchConfigurations)
                   for action in nav2.get_sub_entities())
    include = next(child for child in nav2.get_sub_entities()
                   if isinstance(child, IncludeLaunchDescription))
    assert dict(include.launch_arguments)['rviz'] == 'false'
