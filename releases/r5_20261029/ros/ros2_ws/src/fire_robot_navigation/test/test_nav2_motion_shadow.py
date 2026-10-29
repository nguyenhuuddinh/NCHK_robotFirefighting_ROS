"""Safety tests for the Nav2 mission-shadow launch wrapper."""

import importlib.util
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'nav2_motion_shadow.launch.py'

spec = importlib.util.spec_from_file_location(
    'nav2_motion_shadow_launch', LAUNCH_FILE)
nav2_motion_shadow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav2_motion_shadow)


def _canonical_text(value):
    if hasattr(value, 'text'):
        return str(value.text)
    if isinstance(value, (list, tuple)):
        return ''.join(_canonical_text(item) for item in value)
    return str(value)


def test_shadow_wrapper_forces_isolated_output_without_override_argument():
    launch_description = nav2_motion_shadow.generate_launch_description()
    declarations = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    includes = [
        entity for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    opaque_functions = [
        entity for entity in launch_description.entities
        if isinstance(entity, OpaqueFunction)
    ]

    assert {declaration.name for declaration in declarations} == {
        'use_sim_time', 'map', 'params_file', 'autostart', 'rviz'}
    assert includes == []
    assert len(opaque_functions) == 1
    assert opaque_functions[0]._OpaqueFunction__function is (
        nav2_motion_shadow._include_nav2_shadow)

    context = LaunchContext()
    context.launch_configurations.update({
        'use_sim_time': 'false',
        'map': '/tmp/map.yaml',
        'params_file': '/tmp/nav2_params.yaml',
        'autostart': 'true',
        'rviz': 'false',
    })
    includes = nav2_motion_shadow._include_nav2_shadow(context)
    assert len(includes) == 1
    launch_arguments = {
        name: value for name, value in includes[0].launch_arguments
    }
    assert _canonical_text(
        launch_arguments['cmd_vel_output_topic']) == (
            '/fire_mission/nav2_cmd_vel_shadow')
    assert 'cmd_vel_output_topic' not in {
        declaration.name for declaration in declarations}


def test_shadow_wrapper_rejects_any_real_output_override():
    context = LaunchContext()
    context.launch_configurations['cmd_vel_output_topic'] = '/cmd_vel_raw'
    try:
        nav2_motion_shadow._include_nav2_shadow(context)
    except RuntimeError as exc:
        assert 'does not permit overriding' in str(exc)
    else:
        raise AssertionError('unsafe output override must fail closed')


def test_shadow_wrapper_source_never_mentions_real_command_topics():
    source = LAUNCH_FILE.read_text()
    assert '/fire_mission/nav2_cmd_vel_shadow' in source
    assert '/cmd_vel_raw' not in source
    assert "DeclareLaunchArgument(\n            'cmd_vel_output_topic'" not in source
