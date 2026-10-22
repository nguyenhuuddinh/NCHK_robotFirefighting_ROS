"""Offline transport/launch contracts: no participant, socket, ROS init or USB."""
import importlib.util
from pathlib import Path
import runpy
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

from fire_robot_bringup.camera_transport import camera_process_environment

PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE/'config/native_camera_wifi_nonblocking.xml'


def actions(name):
    spec = importlib.util.spec_from_file_location('camera_transport_' + name, PACKAGE/'launch'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(PACKAGE)
    return module.generate_launch_description().entities


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def test_system_keeps_existing_environment_even_with_missing_profile():
    inherited = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                 'FASTRTPS_DEFAULT_PROFILES_FILE': '/custom.xml'}
    before = inherited.copy()
    assert camera_process_environment('ASYNCHRONOUS', 'system', '/missing', inherited) == {
        'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS'}
    assert inherited == before


@pytest.mark.parametrize('variable,value', [
    ('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ('FASTRTPS_DEFAULT_PROFILES_FILE', '/existing.xml'),
    ('FASTDDS_DEFAULT_PROFILES_FILE', '/existing.xml'),
    ('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
])
def test_opt_in_rejects_conflict_without_altering_parent(variable, value):
    inherited = {variable: value}
    with pytest.raises(ValueError, match=variable if variable != 'RMW_IMPLEMENTATION' else 'rmw_fastrtps_cpp'):
        camera_process_environment('ASYNCHRONOUS', 'wifi_nonblocking', PROFILE, inherited)
    assert inherited == {variable: value}


def test_missing_installed_profile_is_explicit():
    with pytest.raises(FileNotFoundError, match='Missing native camera DDS profile'):
        camera_process_environment('ASYNCHRONOUS', 'wifi_nonblocking', '/missing.xml', {})


@pytest.mark.parametrize('mode,transport', [('TYPO','system'), ('ASYNCHRONOUS','typo')])
def test_unknown_selection_fails(mode, transport):
    with pytest.raises(ValueError):
        camera_process_environment(mode, transport, PROFILE, {})


def test_opt_in_reaches_only_enabled_native_child(monkeypatch):
    for name in ('RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        monkeypatch.delenv(name, raising=False)
    context = LaunchContext()
    parent_before = dict(context.environment)
    context.launch_configurations.update(use_camera='true', camera_backend='native_mjpeg',
        native_camera_transport='wifi_nonblocking', native_camera_publication_mode='ASYNCHRONOUS')
    entities = actions('sensors.launch.py')
    assert not any(isinstance(a, SetEnvironmentVariable) for a in entities)
    action = next(a for a in entities if isinstance(a, OpaqueFunction))
    assert action.condition.evaluate(context)
    child = action.execute(context)[0]
    assert child.node_executable == 'native_mjpeg_camera'
    assert {resolve(context,k):resolve(context,v) for k,v in child.additional_env} == {
        'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(PROFILE.resolve())}
    assert all(not n.additional_env for n in entities if isinstance(n, Node))
    assert dict(context.environment) == parent_before
    context.launch_configurations['camera_backend'] = 'usb_cam'
    assert not action.condition.evaluate(context)
    context.launch_configurations.update(use_camera='false', camera_backend='native_mjpeg')
    assert not action.condition.evaluate(context)


@pytest.mark.parametrize('transport', ['system', 'wifi_nonblocking'])
def test_master_forwards_opt_in_and_defaults_to_unchanged_transport(transport):
    context = LaunchContext()
    context.launch_configurations['native_camera_transport'] = transport
    for name in ('robot.launch.py', 'sensors.launch.py'):
        declaration = next(a for a in actions(name) if isinstance(a, DeclareLaunchArgument)
                           and a.name == 'native_camera_transport')
        assert resolve(context, declaration.default_value) == 'system'
        assert set(declaration.choices) == {'system', 'wifi_nonblocking'}
    forwarded = next(dict(a.launch_arguments) for a in actions('robot.launch.py')
                     if isinstance(a, IncludeLaunchDescription)
                     and 'native_camera_transport' in dict(a.launch_arguments))
    assert resolve(context, forwarded['native_camera_transport']) == transport


def test_setup_installs_the_profile(monkeypatch):
    monkeypatch.chdir(PACKAGE)
    with patch('setuptools.setup') as setup:
        runpy.run_path(str(PACKAGE/'setup.py'), run_name='__camera_setup_test__')
    installed = dict(setup.call_args.kwargs['data_files'])
    assert 'config/native_camera_wifi_nonblocking.xml' in installed['share/fire_robot_bringup/config']
