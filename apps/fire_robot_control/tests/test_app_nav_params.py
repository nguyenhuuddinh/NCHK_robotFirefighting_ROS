"""App inflation integration: real YAML/launch rewriting, no robot or ROS nodes."""
from copy import deepcopy
import hashlib
from pathlib import Path
import runpy
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml

from services.app_nav_params import write_params
from services.candidate_build import build
from services.desktop_hooks import DesktopHooks
from services.packet_archive import source_packet

APP = Path(__file__).resolve().parents[1]
SOURCE = APP.parents[1] / 'releases/r5_20261029/ros'
RELATIVE_PARAMS = 'ros2_ws/src/fire_robot_navigation/config/nav2_params.yaml'


@pytest.fixture
def candidate(tmp_path):
    folder = tmp_path / 'app candidate with spaces'
    build(SOURCE, APP, folder)
    return folder


def test_only_inflation_radii_change_and_parent_remains_intact(candidate):
    parent = yaml.safe_load((SOURCE / RELATIVE_PARAMS).read_text())
    actual = yaml.safe_load((candidate / 'desktop/generated/app_nav2_params.yaml').read_text())
    expected = deepcopy(parent)
    for scope in ('local_costmap', 'global_costmap'):
        node = expected[scope][scope]['ros__parameters']
        assert node['inflation_layer']['inflation_radius'] == .55
        node['inflation_layer']['inflation_radius'] = .30
    # Whole-tree equality includes footprint/padding, RPP, collision detection,
    # speed, plugin/QoS/topic configuration, and every unrelated Nav2 parameter.
    assert actual == expected
    assert (candidate / RELATIVE_PARAMS).read_bytes() == (SOURCE / RELATIVE_PARAMS).read_bytes()
    for line in (SOURCE / 'operational_packet.sha256').read_text().splitlines():
        digest, name = line.split('  ', 1)
        assert hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() == digest


@pytest.mark.parametrize('mode', ['gate2', 'observe'])
def test_hook_passes_app_params_without_changing_r5_map_or_launch(mode, candidate):
    hooks = DesktopHooks.__new__(DesktopHooks)
    hooks.app = candidate / 'desktop'
    hooks.operator = SimpleNamespace(command=['app-operator'])
    hooks.runtime, hooks.out = candidate / 'runtime', candidate / 'session'
    hooks.binding = SimpleNamespace(session='session')
    original = {
        'nav': ['ros2', 'launch', '--noninteractive', 'fire_robot_navigation',
                'nav2_composed_shadow.launch.py', 'map:=/tmp/map with spaces.yaml',
                'rviz:=false', 'use_sim_time:=false', 'refresh_bt_action_status:=true'],
        'yolo': ['ros2', 'launch', '--noninteractive', 'fire_robot_perception', 'perception.launch.py'],
        'mission': ['ros2', 'launch', '--noninteractive', 'fire_robot_mission',
                    'operational_stack_' + mode + '.launch.py'],
        'operator': ['r5-operator'], 'rviz': ['rviz2'],
    }
    before = deepcopy(original)
    with patch('services.app_model.validate_model', return_value=Path('/tmp/best_minireplay.pt')):
        result = hooks.commands(original)
    assert original == before
    args = result['nav'][result['nav'].index('--') + 1:]
    assert args[:len(original['nav'])] == original['nav']
    assert [a for a in args if a.startswith('params_file:=')] == [
        'params_file:=' + str(candidate / 'desktop/generated/app_nav2_params.yaml')]
    assert args[-2:] == ['sigterm_timeout:=8.0', 'sigkill_timeout:=3.0']
    if mode == 'observe':
        assert result['mission'] == original['mission']


def test_humble_launch_rewrite_keeps_both_child_costmaps(candidate, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch-logs'))
    from launch import LaunchContext
    functions = runpy.run_path(str(SOURCE / 'ros2_ws/src/fire_robot_navigation/launch/nav2_composed_shadow.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': str(candidate / 'desktop/generated/app_nav2_params.yaml'),
        'map': str(tmp_path / 'selected map.yaml'), 'use_sim_time': 'false',
    })
    rewritten_path = Path(functions['configured_parameters']().perform(context))
    try:
        params = yaml.safe_load(rewritten_path.read_text())
        for scope in ('local_costmap', 'global_costmap'):
            node = params[scope][scope]['ros__parameters']
            assert node['inflation_layer']['inflation_radius'] == .30
            assert node['footprint_padding'] == .015
            assert node['use_sim_time'] is False
        assert params['map_server']['ros__parameters']['yaml_filename'] == str(tmp_path / 'selected map.yaml')
        controller = params['controller_server']['ros__parameters']['FollowPath']
        assert controller['inflation_cost_scaling_factor'] == params['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['cost_scaling_factor']
        assert controller['cost_scaling_dist'] <= .30
        assert controller['use_collision_detection'] is True
    finally:
        rewritten_path.unlink()


def test_generated_config_and_tuning_are_manifest_pinned(candidate):
    _, entries = source_packet(candidate)
    assert 'desktop/config/nav2_tuning.yaml' in entries
    assert 'desktop/generated/app_nav2_params.yaml' in entries
    path = candidate / 'desktop/generated/app_nav2_params.yaml'
    path.write_text(path.read_text() + '\n# tampered\n')
    with pytest.raises(ValueError):
        source_packet(candidate)


@pytest.mark.parametrize('invalid', [.20, .299, float('nan'), True])
def test_tuning_cannot_cut_into_footprint_or_rpp_slow_zone(tmp_path, invalid):
    tuning = tmp_path / 'tuning.yaml'
    tuning.write_text(yaml.safe_dump({scope: {'inflation_radius': invalid}
                                    for scope in ('local_costmap', 'global_costmap')}))
    output = tmp_path / 'params.yaml'
    with pytest.raises(ValueError, match='APP_NAV_INFLATION_TOO_SMALL_OR_INVALID'):
        write_params(SOURCE / RELATIVE_PARAMS, tuning, output)
    assert not output.exists()


def test_tuning_cannot_override_footprint(tmp_path):
    tuning = tmp_path / 'tuning.yaml'
    tuning.write_text(yaml.safe_dump({scope: {'inflation_radius': .40, 'footprint': '[]'}
                                    for scope in ('local_costmap', 'global_costmap')}))
    with pytest.raises(ValueError, match='APP_NAV_TUNING_INVALID'):
        write_params(SOURCE / RELATIVE_PARAMS, tuning, tmp_path / 'params.yaml')
