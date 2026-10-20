"""Minimal session launch refuses unconfigured state and has no live owners."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location('m3_mission_launch', ROOT / 'launch/mission_session_shadow.launch.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(tmp_path, **changes):
    result = LaunchContext()
    result.launch_configurations.update({
        'use_sim_time': 'false', 'params_file': str(ROOT / 'config/mission_params.yaml'),
        'shadow_budget_file': str(tmp_path / 'new.json'), 'initialize_shadow_budget': 'true',
        **changes})
    return result


def test_minimal_profile_creates_only_supervisor_and_explicit_no_owner_roster(tmp_path):
    mod = module()
    nodes = mod._runtime(context(tmp_path))
    assert len(nodes) == 1 and isinstance(nodes[0], Node)
    assert nodes[0].node_executable == 'fire_mission_supervisor'
    text = (ROOT / 'launch/mission_session_shadow.launch.py').read_text()
    assert "'close_owner_manifest': 'io_nav_static_no_owners_v1'" in text
    assert "'require_through_status': True" in text
    assert "'enable_shadow_session_cleanup': True" in text
    assert 'live_alignment_takeover' not in text and 'pump_demo_controller' not in text
    assert 'ActionClient' not in text


@pytest.mark.parametrize('changes', [
    {'use_sim_time': 'true'}, {'initialize_shadow_budget': 'yes'},
    {'initialize_shadow_budget': 'false'}, {'shadow_budget_file': 'relative.json'},
    {'params_file': '/missing/m3_params.yaml'},
])
def test_invalid_profile_rejected_before_constructing_node(tmp_path, changes):
    with pytest.raises(RuntimeError):
        module()._runtime(context(tmp_path, **changes))


def test_new_budget_initialization_is_never_default():
    args = {item.name: item for item in module().generate_launch_description().entities
            if isinstance(item, DeclareLaunchArgument)}
    assert args['initialize_shadow_budget'].default_value[0].text == 'false'
    assert args['shadow_budget_file'].default_value is None
